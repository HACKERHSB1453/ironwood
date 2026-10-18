# SPDX-License-Identifier: GPL-3.0-or-later
"""B5: headless Python API. Bracket scenario (sketch with w/h -> extrude -> fillet two edges -> through hole),
edit of a parameter, save/load round trip, and error cases. Run with PYTHONPATH pointing at the built module."""

import math
import os
import shutil
import tempfile
import unittest

import ironwood as iw

T = 10.0          # extrude height (mm)
R = 3.0           # fillet radius (mm)
HOLE_D = 6.0      # through hole diameter (mm)


def fillet_removed(length):
    """Volume removed from a box by a constant fillet of radius R on one edge of the given length."""
    return (1.0 - math.pi / 4.0) * R * R * length


def hole_removed(depth):
    return math.pi * (HOLE_D / 2.0) ** 2 * depth


def bracket_doc(w=40.0, h=20.0):
    """Sketch rectangle driven by parameters w and h -> extrude t -> fillet two vertical corners -> through hole."""
    d = iw.Document(name="bracket")
    d.add_parameter("w", f"{w} mm")
    d.add_parameter("h", f"{h} mm")
    d.add_parameter("t", f"{T} mm")
    d.add_parameter("r", f"{R} mm")

    sk = iw.Sketch("XY")
    bottom, right, top, left = sk.rect((0, 0), (w, h))
    sk.dimension("distance_x", [(bottom, "start"), (bottom, "end")], "w")
    sk.dimension("distance_y", [(right, "start"), (right, "end")], "h")
    report = sk.solve(d)
    assert report.ok, report.message

    sketch_feature = d.add_sketch(sk)
    extrude = d.extrude(sketch_feature, distance="t")
    body = extrude.body

    # Vertical corner edges at (0, 0) and (w, 0): centroid z = T/2, length T, on the y = 0 side.
    corners = body.edges(lambda e: abs(e.centroid[1]) < 1e-6 and abs(e.centroid[2] - T / 2) < 1e-6
                         and abs(e.length - T) < 1e-6)
    assert len(corners) == 2, len(corners)
    fillet = d.fillet(corners, radius="r")

    # Top face: planar, normal +Z, at z = T.
    tops = body.faces(lambda f: f.kind == "plane" and abs(f.normal[2] - 1.0) < 1e-9 and abs(f.centroid[2] - T) < 1e-6)
    assert len(tops) == 1, len(tops)
    top_face = tops[0]
    u, v = top_face.local((w / 2, h / 2, T))
    hole = d.hole(top_face, diameter=f"{HOLE_D} mm", through=True, at=(u, v))
    return d, sketch_feature, extrude, fillet, hole, body


class BracketScenario(unittest.TestCase):
    def test_volumes_after_each_step(self):
        d, _, extrude, fillet, hole, body = bracket_doc()
        ev = d.evaluate()
        self.assertTrue(ev.ok, [s.error for s in ev.statuses if s.state == "failed"])
        self.assertAlmostEqual(body.volume, 40 * 20 * T - 2 * fillet_removed(T) - hole_removed(T), places=2)
        # 4 sides + top + bottom, 2 fillet faces, 1 through-hole wall.
        self.assertEqual(body.face_count, 9)

    def test_timeline_order_and_kinds(self):
        d, _, _, _, _, _ = bracket_doc()
        self.assertTrue(d.evaluate().ok)
        kinds = [f.kind for f in d.timeline]
        self.assertEqual(kinds, ["sketch", "extrude", "fillet", "hole"])
        self.assertTrue(all(f.status == "ok" for f in d.timeline))

    def test_edit_width_changes_volume_and_fillets_survive(self):
        d, _, _, fillet, hole, body = bracket_doc()
        before = body.volume
        d.set_parameter("w", "50 mm")
        ev = d.evaluate()
        self.assertTrue(ev.ok, [s.error for s in ev.statuses if s.state == "failed"])
        self.assertTrue(all(f.status == "ok" for f in d.timeline))
        self.assertEqual(fillet.status, "ok")
        after = body.volume
        self.assertAlmostEqual(after - before, 20 * T * 10.0, places=2)  # +10 mm of width over 20 x 10 mm section
        self.assertAlmostEqual(after, 50 * 20 * T - 2 * fillet_removed(T) - hole_removed(T), places=2)

    def test_undo_restores_previous_volume(self):
        d, _, _, _, _, body = bracket_doc()
        v0 = body.volume
        d.set_parameter("w", "50 mm")
        self.assertNotAlmostEqual(body.volume, v0, places=2)
        d.undo()
        self.assertAlmostEqual(body.volume, v0, places=6)

    def test_suppress_hole_adds_material_back(self):
        d, _, _, _, hole, body = bracket_doc()
        with_hole = body.volume
        d.suppress(hole)
        self.assertAlmostEqual(body.volume - with_hole, hole_removed(T), places=2)

    def test_save_load_round_trip(self):
        d, _, _, _, _, body = bracket_doc()
        volume = body.volume
        tmp = tempfile.mkdtemp(prefix="iw_py_")
        try:
            path = os.path.join(tmp, "bracket.iwdoc")
            d.save(path)
            self.assertTrue(d.is_clean)
            loaded = iw.Document.open(path)
            self.assertEqual(loaded.name, "bracket")
            self.assertEqual([f.kind for f in loaded.timeline], ["sketch", "extrude", "fillet", "hole"])
            self.assertAlmostEqual(loaded.bodies[0].volume, volume, places=6)
            self.assertEqual({p.name for p in loaded.parameters}, {"w", "h", "t", "r"})
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_export_and_reimport_step(self):
        d, _, _, _, _, body = bracket_doc()
        tmp = tempfile.mkdtemp(prefix="iw_py_")
        try:
            path = os.path.join(tmp, "bracket.step")
            d.export(path)
            solids = iw.import_step(path)
            self.assertEqual(len(solids), 1)
            self.assertAlmostEqual(solids[0].volume, body.volume, places=3)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class ErrorCases(unittest.TestCase):
    def test_bad_expression_raises_parse_error(self):
        d = iw.Document()
        with self.assertRaises(iw.ParseError) as cm:
            d.add_parameter("bad", "1 mm +")
        self.assertIsInstance(cm.exception, iw.IronwoodError)
        self.assertTrue(str(cm.exception))

    def test_duplicate_parameter_raises_invalid_argument(self):
        d = iw.Document()
        d.add_parameter("a", "1 mm")
        with self.assertRaises(iw.InvalidArgumentError):
            d.add_parameter("a", "2 mm")

    def test_unknown_parameter_in_set_raises_not_found(self):
        d = iw.Document()
        with self.assertRaises(iw.NotFoundError):
            d.set_parameter("missing", "1 mm")

    def test_unknown_constraint_kind_raises_invalid_argument(self):
        sk = iw.Sketch()
        line = sk.line((0, 0), (10, 0))
        with self.assertRaises(iw.InvalidArgumentError):
            sk.constrain("wobbly", [line])

    def test_degenerate_line_raises_invalid_argument(self):
        sk = iw.Sketch()
        with self.assertRaises(iw.InvalidArgumentError):
            sk.line((1, 1), (1, 1))

    def test_extrude_with_several_profiles_needs_a_name(self):
        d = iw.Document()
        sk = iw.Sketch()
        sk.rect((0, 0), (10, 10))
        sk.rect((20, 0), (30, 10))
        f = d.add_sketch(sk)
        with self.assertRaises(iw.InvalidArgumentError):
            d.extrude(f, distance="5 mm")

    def test_missing_profile_name_raises_not_found(self):
        d = iw.Document()
        sk = iw.Sketch()
        sk.rect((0, 0), (10, 10))
        f = d.add_sketch(sk)
        with self.assertRaises(iw.NotFoundError):
            d.extrude(f, profile="e99.0", distance="5 mm")

    def test_failed_feature_is_reported_not_raised(self):
        d = iw.Document()
        d.add_parameter("w", "10 mm")
        sk = iw.Sketch()
        sk.rect((0, 0), (10, 10))
        f = d.add_sketch(sk)
        ext = d.extrude(f, distance="nope * 2")
        ev = d.evaluate()
        self.assertFalse(ev.ok)
        self.assertEqual(ext.status, "failed")
        with self.assertRaises(iw.IronwoodError):
            ev.raise_for_errors()

    def test_body_of_failed_feature_raises(self):
        d = iw.Document()
        sk = iw.Sketch()
        sk.rect((0, 0), (10, 10))
        f = d.add_sketch(sk)
        ext = d.extrude(f, distance="missing_param")
        d.evaluate()
        with self.assertRaises(iw.NotFoundError):
            _ = ext.body.volume

    def test_fillet_needs_edges_of_one_body(self):
        d = iw.Document()
        with self.assertRaises(iw.InvalidArgumentError):
            d.fillet([], radius="1 mm")

    def test_undo_with_nothing_to_undo_raises(self):
        d = iw.Document()
        with self.assertRaises(iw.IronwoodError):
            d.undo()

    def test_open_missing_file_raises(self):
        with self.assertRaises(iw.IronwoodError):
            iw.Document.open("/nonexistent/ironwood/doc")


def block(d, w, h, depth, name="block", operation="new_body", body=None, origin=(0, 0)):
    """Extrude a w x h rectangle at `origin` (sketch on XY) by `depth`; returns (sketch feature, extrude feature)."""
    sk = iw.Sketch("XY")
    lines = sk.rect(origin, (origin[0] + w, origin[1] + h))
    f = d.add_sketch(sk)
    return lines, d.extrude(f, distance=f"{depth} mm", operation=operation, body=body)


class OtherFeatures(unittest.TestCase):
    def test_area_bounding_box_and_face_count_of_a_block(self):
        d = iw.Document()
        _, ext = block(d, 40, 20, 10)
        self.assertTrue(d.evaluate().ok)
        body = ext.body
        self.assertAlmostEqual(body.area, 2 * (40 * 20 + 40 * 10 + 20 * 10), places=6)
        lo, hi = body.bounding_box
        self.assertEqual([round(x, 6) for x in lo], [0, 0, 0])
        self.assertEqual([round(x, 6) for x in hi], [40, 20, 10])
        self.assertEqual(body.face_count, 6)
        self.assertEqual(body.edge_count, 12)
        self.assertEqual(body.vertex_count, 8)
        cx, cy, cz = body.centre_of_mass
        self.assertAlmostEqual(cx, 20, places=6)
        self.assertAlmostEqual(cy, 10, places=6)
        self.assertAlmostEqual(cz, 5, places=6)

    def test_chamfer_removes_a_triangular_prism_per_edge(self):
        d = iw.Document()
        _, ext = block(d, 40, 20, 10)
        body = ext.body
        corner = body.edges(lambda e: abs(e.centroid[0]) < 1e-6 and abs(e.centroid[1]) < 1e-6 and abs(e.length - 10) < 1e-6)
        self.assertEqual(len(corner), 1)
        d.chamfer(corner, distance="1 mm")
        self.assertTrue(d.evaluate().ok)
        self.assertAlmostEqual(body.volume, 8000 - 0.5 * 1 * 1 * 10, places=3)

    def test_revolve_of_a_rectangle_about_its_side_is_a_cylinder(self):
        d = iw.Document()
        sk = iw.Sketch("XY")
        _, _, _, left = sk.rect((0, 0), (10, 20))
        f = d.add_sketch(sk)
        rev = d.revolve(f, left)
        self.assertTrue(d.evaluate().ok, [s.error for s in d.evaluate().statuses if s.state == "failed"])
        self.assertAlmostEqual(rev.body.volume, math.pi * 10 * 10 * 20, places=1)

    def test_move_translates_centre_of_mass_and_keeps_volume(self):
        d = iw.Document()
        _, ext = block(d, 40, 20, 10)
        body = ext.body
        v0, c0 = body.volume, body.centre_of_mass
        d.move(body, translate=(5, 0, 0))
        self.assertTrue(d.evaluate().ok)
        self.assertAlmostEqual(body.volume, v0, places=6)
        self.assertAlmostEqual(body.centre_of_mass[0], c0[0] + 5, places=6)

    def test_cut_removes_overlap_from_existing_body(self):
        d = iw.Document()
        _, ext = block(d, 40, 20, 10)
        body = ext.body
        block(d, 10, 10, 10, operation="cut", body=body)
        self.assertTrue(d.evaluate().ok)
        self.assertAlmostEqual(body.volume, 8000 - 1000, places=3)

    def test_offset_plane_from_top_face_is_accepted(self):
        d = iw.Document()
        _, ext = block(d, 40, 20, 10)
        top = ext.body.faces(lambda f: f.kind == "plane" and abs(f.normal[2] - 1) < 1e-9)[0]
        plane = d.offset_plane(top, "5 mm")
        ev = d.evaluate()
        self.assertTrue(ev.ok, [s.error for s in ev.statuses if s.state == "failed"])
        self.assertEqual(plane.kind, "plane")
        self.assertIsNone(plane.body)

    def test_stl_and_3mf_export_write_files(self):
        d = iw.Document()
        _, ext = block(d, 40, 20, 10)
        tmp = tempfile.mkdtemp(prefix="iw_py_")
        try:
            stl = os.path.join(tmp, "b.stl")
            mf = os.path.join(tmp, "b.3mf")
            d.export(stl, bodies=[ext.body])
            d.export(mf, bodies=[ext.body])
            self.assertGreater(os.path.getsize(stl), 0)
            self.assertGreater(os.path.getsize(mf), 0)
            solid = iw.import_stl(stl)
            self.assertAlmostEqual(solid.volume, 8000, delta=1.0)
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    def test_sketch_on_a_face_follows_the_face(self):
        d = iw.Document()
        _, ext = block(d, 40, 20, 10)
        top = ext.body.faces(lambda f: f.kind == "plane" and abs(f.normal[2] - 1) < 1e-9)[0]
        sk = iw.Sketch.on_face(top)
        sk.circle((20, 10), 3)
        sf = d.add_sketch(sk)
        boss = d.extrude(sf, distance="5 mm", operation="join", body=ext.body)
        self.assertTrue(d.evaluate().ok, [s.error for s in d.evaluate().statuses if s.state == "failed"])
        self.assertAlmostEqual(ext.body.volume, 8000 + math.pi * 9 * 5, places=2)
        self.assertEqual(boss.kind, "extrude")


class Repr(unittest.TestCase):
    def test_reprs_are_informative(self):
        d, sketch_feature, _, _, _, body = bracket_doc()
        self.assertIn("Document", repr(d))
        self.assertIn("sketch", repr(sketch_feature))
        self.assertIn("Body", repr(body))
        self.assertIn("Parameter", repr(d.parameters[0]))


if __name__ == "__main__":
    unittest.main()
