# SPDX-License-Identifier: GPL-3.0-or-later
"""C1: end-to-end part scenarios through the headless Python API.

(a) mounting bracket: L-profile sketch driven by t, w, h -> extrude -> two counterbore holes -> fillet of the inner
    bend edge -> edit t, w, h and the hole diameter one at a time. Every feature stays Ok, the volume matches the
    analytic value after every edit, and the fillet and holes keep their references (face/edge counts unchanged).
(b) flange: revolved ring profile driven by R, rin, tf -> six holes added one by one (no pattern feature) ->
    chamfer of the outer top edge -> edit the outer radius R. Same checks.
(c) save/load round trip of both documents (edits after reload still resolve), and STEP export re-imported with a
    matching volume.

All volumes are analytic (mm^3). Run through the ctest `python_api` target or with PYTHONPATH on the built module.
"""

import math
import os
import shutil
import tempfile
import unittest

import ironwood as iw

D = 20.0           # bracket extrude depth (mm); a constant, not a parameter
CB_D = 8.0         # counterbore diameter (mm)
CB_DEPTH = 3.0     # counterbore depth (mm)
FILLET_R = 2.0     # inner bend fillet radius (mm)
HOLE_XY = [(20.0, 5.0), (30.0, 5.0)]   # bracket hole centres on the top face (absolute, inside the bottom leg)


def bracket_area(w, h, t):
    """L-profile area: bottom leg w x t plus the vertical leg t x (h - t)."""
    return w * t + t * (h - t)


def bracket_volume(w, h, t, hd):
    """Extruded L, minus two counterbore holes, plus the concave inner-bend fillet (adds (1 - pi/4) r^2 per length)."""
    cb = math.pi * (CB_D / 2) ** 2 * CB_DEPTH + math.pi * (hd / 2) ** 2 * (D - CB_DEPTH)
    fillet_add = (1.0 - math.pi / 4.0) * FILLET_R ** 2 * D
    return bracket_area(w, h, t) * D - len(HOLE_XY) * cb + fillet_add


def build_bracket(w=40.0, h=60.0, t=10.0, hd=4.0):
    d = iw.Document(name="bracket")
    d.add_parameter("w", f"{w} mm")
    d.add_parameter("h", f"{h} mm")
    d.add_parameter("t", f"{t} mm")
    d.add_parameter("hd", f"{hd} mm")

    sk = iw.Sketch("XY")
    pts = [(0, 0), (w, 0), (w, t), (t, t), (t, h), (0, h)]
    b, r, i1, i2, top, left = [sk.line(pts[i], pts[(i + 1) % 6]) for i in range(6)]
    for a, c in [(b, r), (r, i1), (i1, i2), (i2, top), (top, left), (left, b)]:
        sk.constrain("coincident", [(a, "end"), (c, "start")])
    for line, kind in [(b, "horizontal"), (r, "vertical"), (i1, "horizontal"), (i2, "vertical"), (top, "horizontal"),
                       (left, "vertical")]:
        sk.constrain(kind, [line])
    # Pin the outer corner to the sketch origin (construction point): without it the solver may translate the L when a
    # dimension changes, which moves the holes relative to the material.
    origin = sk.point((0, 0), construction=True)
    sk.constrain("fix", [origin])
    sk.constrain("coincident", [origin, (b, "start")])
    # Four independent dimensions for the L (the top and left legs follow from closure).
    sk.dimension("distance_x", [(b, "start"), (b, "end")], "w")
    sk.dimension("distance_y", [(r, "start"), (r, "end")], "t")
    sk.dimension("distance_x", [(i1, "end"), (i1, "start")], "w - t")
    sk.dimension("distance_y", [(i2, "start"), (i2, "end")], "h - t")
    report = sk.solve(d)
    assert report.ok, report.message

    sketch_feature = d.add_sketch(sk)
    extrude = d.extrude(sketch_feature, distance=f"{D} mm")
    d.evaluate()
    body = extrude.body

    # Inner bend: the concave vertical edge at (t, t), running the full depth.
    bend = body.edges(lambda e: abs(e.centroid[0] - t) < 1e-6 and abs(e.centroid[1] - t) < 1e-6
                      and abs(e.length - D) < 1e-6)
    assert len(bend) == 1, len(bend)
    fillet = d.fillet(bend, radius=f"{FILLET_R} mm")
    d.evaluate()

    holes = []
    for x, y in HOLE_XY:
        tops = body.faces(lambda f: f.kind == "plane" and abs(f.normal[2] - 1.0) < 1e-9 and abs(f.centroid[2] - D) < 1e-6)
        assert len(tops) == 1, len(tops)
        face = tops[0]
        hole = d.hole(face, diameter="hd", through=True, kind="counterbore", at=face.local((x, y, D)),
                      counterbore_diameter=f"{CB_D} mm", counterbore_depth=f"{CB_DEPTH} mm")
        d.evaluate()
        holes.append(hole)
    return d, extrude, fillet, holes, body


def failed_states(ev):
    return [(s.feature_id, s.error) for s in ev.statuses if s.state != "ok"]


class MountingBracket(unittest.TestCase):
    def assertAllOk(self, d):
        ev = d.evaluate()
        self.assertTrue(ev.ok, failed_states(ev))
        self.assertTrue(all(f.status == "ok" for f in d.timeline), [(f.kind, f.status) for f in d.timeline])

    def test_build_is_ok_and_matches_analytic_volume(self):
        d, extrude, fillet, holes, body = build_bracket()
        self.assertAllOk(d)
        self.assertEqual([f.kind for f in d.timeline], ["sketch", "extrude", "fillet", "hole", "hole"])
        self.assertAlmostEqual(body.volume, bracket_volume(40, 60, 10, 4), places=2)
        # Faces: 6 L sides, top, bottom = 8; fillet adds 1; each counterbore adds 3 (floor, cb wall, bore wall).
        self.assertEqual(body.face_count, 8 + 1 + 3 * len(HOLE_XY))

    def test_edits_one_by_one_keep_features_ok_and_volumes_exact(self):
        d, extrude, fillet, holes, body = build_bracket()
        w, h, t, hd = 40.0, 60.0, 10.0, 4.0
        faces0 = body.face_count
        edge_count0 = body.edge_count

        steps = [("t", 12.0), ("w", 50.0), ("h", 70.0), ("hd", 5.0)]
        for name, value in steps:
            if name == "t":
                t = value
            elif name == "w":
                w = value
            elif name == "h":
                h = value
            else:
                hd = value
            d.set_parameter(name, f"{value} mm")
            self.assertAllOk(d)
            self.assertAlmostEqual(body.volume, bracket_volume(w, h, t, hd), places=2, msg=f"after {name}={value}")
            # Topology is unchanged by a dimension edit: fillet and holes still reference the same geometry.
            self.assertEqual(body.face_count, faces0, msg=f"face count after {name}")
            self.assertEqual(body.edge_count, edge_count0, msg=f"edge count after {name}")
            self.assertEqual(fillet.status, "ok")
            self.assertTrue(all(hole.status == "ok" for hole in holes))

    def test_each_edit_changes_volume_by_its_analytic_delta(self):
        d, _, _, _, body = build_bracket()
        v = body.volume
        d.set_parameter("w", "50 mm")
        self.assertAllOk(d)
        # Widening the bottom leg by 10 mm adds 10 mm x t (10 mm) x D of material; holes and fillet are unaffected.
        self.assertAlmostEqual(body.volume - v, 10.0 * 10.0 * D, places=2)
        v = body.volume
        d.set_parameter("t", "12 mm")
        self.assertAllOk(d)
        self.assertAlmostEqual(body.volume, bracket_volume(50, 60, 12, 4), places=2)
        v = body.volume
        d.set_parameter("hd", "6 mm")
        self.assertAllOk(d)
        # Only the through-bore section of each hole grows: pi * ((6/2)^2 - (4/2)^2) * (D - CB_DEPTH) per hole.
        expected = -len(HOLE_XY) * math.pi * (9.0 - 4.0) * (D - CB_DEPTH)
        self.assertAlmostEqual(body.volume - v, expected, places=2)

    def test_undo_back_to_start_restores_volume(self):
        d, _, _, _, body = build_bracket()
        v0 = body.volume
        for name, value in [("t", "12 mm"), ("w", "50 mm"), ("h", "70 mm"), ("hd", "5 mm")]:
            d.set_parameter(name, value)
        for _ in range(4):
            d.undo()
        self.assertAllOk(d)
        self.assertAlmostEqual(body.volume, v0, places=6)

    def test_suppress_fillet_then_unsuppress_restores_volume(self):
        d, _, fillet, _, body = build_bracket()
        v0 = body.volume
        d.suppress(fillet)
        ev = d.evaluate()
        self.assertTrue(ev.ok, failed_states(ev))
        self.assertEqual(fillet.status, "suppressed")
        self.assertTrue(all(f.status == "ok" for f in d.timeline if f.kind != "fillet"))
        self.assertAlmostEqual(body.volume, v0 - (1.0 - math.pi / 4.0) * FILLET_R ** 2 * D, places=2)
        d.suppress(fillet, False)
        self.assertAllOk(d)
        self.assertAlmostEqual(body.volume, v0, places=2)


def flange_volume(R, tf, rin, hd, c, n_holes=6):
    """Ring pi (R^2 - rin^2) tf, minus the holes (through), minus the outer top chamfer pi c^2 (R - c/3)."""
    ring = math.pi * (R * R - rin * rin) * tf
    holes = n_holes * math.pi * (hd / 2) ** 2 * tf
    chamfer = math.pi * c * c * (R - c / 3.0)
    return ring - holes - chamfer


HOLE_CIRCLE_R = 30.0
CHAMFER = 1.0


def build_flange(R=40.0, rin=10.0, tf=8.0, hd=6.0):
    d = iw.Document(name="flange")
    d.add_parameter("R", f"{R} mm")
    d.add_parameter("rin", f"{rin} mm")
    d.add_parameter("tf", f"{tf} mm")
    d.add_parameter("hd", f"{hd} mm")

    # Profile in XY, revolved about a construction axis along sketch Y (world Y).
    sk = iw.Sketch("XY")
    # Anchors: the axis starts at the fixed origin and the profile's inner bottom corner is fixed at (rin, 0), so the
    # solver cannot translate the profile or the axis when a dimension changes.
    origin = sk.point((0, 0), construction=True)
    sk.constrain("fix", [origin])
    axis = sk.line((0, 0), (0, tf + 4), construction=True)
    sk.constrain("coincident", [origin, (axis, "start")])
    sk.constrain("vertical", [axis])
    anchor = sk.point((rin, 0), construction=True)
    sk.constrain("fix", [anchor])
    bot = sk.line((rin, 0), (R, 0))
    sk.constrain("coincident", [anchor, (bot, "start")])
    right = sk.line((R, 0), (R, tf))
    top = sk.line((R, tf), (rin, tf))
    left = sk.line((rin, tf), (rin, 0))
    for a, c in [(bot, right), (right, top), (top, left), (left, bot)]:
        sk.constrain("coincident", [(a, "end"), (c, "start")])
    sk.constrain("horizontal", [bot])
    sk.constrain("vertical", [right])
    sk.constrain("horizontal", [top])
    sk.constrain("vertical", [left])
    sk.dimension("distance_x", [(bot, "start"), (bot, "end")], "R - rin")
    sk.dimension("distance_y", [(right, "start"), (right, "end")], "tf")
    report = sk.solve(d)
    assert report.ok, report.message

    sketch_feature = d.add_sketch(sk)
    rev = d.revolve(sketch_feature, axis)
    d.evaluate()
    body = rev.body

    holes = []
    for k in range(6):
        d.evaluate()
        tops = body.faces(lambda f: f.kind == "plane" and abs(f.normal[1] - 1.0) < 1e-9 and abs(f.centroid[1] - tf) < 1e-6)
        assert len(tops) == 1, len(tops)
        face = tops[0]
        ang = math.radians(60.0 * k)
        at = face.local((HOLE_CIRCLE_R * math.cos(ang), tf, HOLE_CIRCLE_R * math.sin(ang)))
        holes.append(d.hole(face, diameter="hd", through=True, at=at))
    d.evaluate()

    # Outer top circular edge: at y = tf, length 2 pi R (the inner circle is 2 pi rin, the bottom edges are at y = 0).
    outer = body.edges(lambda e: abs(e.centroid[1] - tf) < 1e-6 and abs(e.length - 2 * math.pi * R) < 1e-4)
    assert len(outer) == 1, len(outer)
    chamfer = d.chamfer(outer, distance=f"{CHAMFER} mm")
    d.evaluate()
    return d, rev, holes, chamfer, body


class Flange(unittest.TestCase):
    def assertAllOk(self, d):
        ev = d.evaluate()
        self.assertTrue(ev.ok, failed_states(ev))
        self.assertTrue(all(f.status == "ok" for f in d.timeline), [(f.kind, f.status) for f in d.timeline])

    def test_build_is_ok_and_matches_analytic_volume(self):
        d, rev, holes, chamfer, body = build_flange()
        self.assertAllOk(d)
        self.assertEqual([f.kind for f in d.timeline], ["sketch", "revolve"] + ["hole"] * 6 + ["chamfer"])
        self.assertAlmostEqual(body.volume, flange_volume(40, 8, 10, 6, CHAMFER), places=1)

    def test_edit_outer_radius_keeps_holes_and_chamfer(self):
        d, rev, holes, chamfer, body = build_flange()
        faces0, edges0 = body.face_count, body.edge_count
        for R in (45.0, 36.0, 50.0):
            d.set_parameter("R", f"{R} mm")
            self.assertAllOk(d)
            self.assertAlmostEqual(body.volume, flange_volume(R, 8, 10, 6, CHAMFER), places=1, msg=f"R={R}")
            self.assertEqual(body.face_count, faces0)
            self.assertEqual(body.edge_count, edges0)
            self.assertTrue(all(h.status == "ok" for h in holes))
            self.assertEqual(chamfer.status, "ok")

    def test_edit_hole_diameter_one_hole_at_a_time(self):
        d, rev, holes, chamfer, body = build_flange()
        self.assertAllOk(d)
        d.set_parameter("hd", "8 mm")
        self.assertAllOk(d)
        self.assertAlmostEqual(body.volume, flange_volume(40, 8, 10, 8, CHAMFER), places=1)


class SaveLoadAndExport(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="iw_e2e_")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_bracket_round_trip_then_edit_after_load(self):
        d, _, _, _, body = build_bracket()
        path = os.path.join(self.tmp, "bracket.iwdoc")
        d.save(path)
        loaded = iw.Document.open(path)
        ev = loaded.evaluate()
        self.assertTrue(ev.ok, failed_states(ev))
        self.assertEqual([f.kind for f in loaded.timeline], ["sketch", "extrude", "fillet", "hole", "hole"])
        self.assertAlmostEqual(loaded.bodies[0].volume, body.volume, places=6)
        # References survive the round trip: an edit after reload still lands on the same features.
        lbody = loaded.bodies[0]
        loaded.set_parameter("t", "12 mm")
        self.assertTrue(loaded.evaluate().ok)
        self.assertTrue(all(f.status == "ok" for f in loaded.timeline))
        self.assertAlmostEqual(lbody.volume, bracket_volume(40, 60, 12, 4), places=2)

    def test_flange_round_trip_then_edit_after_load(self):
        d, _, _, _, body = build_flange()
        path = os.path.join(self.tmp, "flange.iwdoc")
        d.save(path)
        loaded = iw.Document.open(path)
        ev = loaded.evaluate()
        self.assertTrue(ev.ok, failed_states(ev))
        self.assertEqual([f.kind for f in loaded.timeline], ["sketch", "revolve"] + ["hole"] * 6 + ["chamfer"])
        self.assertAlmostEqual(loaded.bodies[0].volume, body.volume, places=6)
        lbody = loaded.bodies[0]
        loaded.set_parameter("R", "45 mm")
        self.assertTrue(loaded.evaluate().ok)
        self.assertAlmostEqual(lbody.volume, flange_volume(45, 8, 10, 6, CHAMFER), places=1)

    def test_bracket_step_export_reimports_with_matching_volume(self):
        d, _, _, _, body = build_bracket()
        path = os.path.join(self.tmp, "bracket.step")
        d.export(path)
        solids = iw.import_step(path)
        self.assertEqual(len(solids), 1)
        self.assertAlmostEqual(solids[0].volume, body.volume, places=3)

    def test_flange_step_export_reimports_with_matching_volume(self):
        d, _, _, _, body = build_flange()
        path = os.path.join(self.tmp, "flange.step")
        d.export(path)
        solids = iw.import_step(path)
        self.assertEqual(len(solids), 1)
        self.assertAlmostEqual(solids[0].volume, body.volume, places=3)


if __name__ == "__main__":
    unittest.main()
