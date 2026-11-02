# SPDX-License-Identifier: GPL-3.0-or-later
"""F31: Document.import_step adds an Import feature to the timeline (one body per solid, stable names, save/load,
missing-file error), edge filters by curve type (line / circle / ...), and the construction planes / axes / points
that the core offers beyond offset_plane. Run with PYTHONPATH pointing at the built module."""

import math
import os
import shutil
import tempfile
import unittest

import ironwood as iw


def block(d, w, h, t, origin=(0.0, 0.0)):
    sk = iw.Sketch("XY")
    sk.rect(origin, (origin[0] + w, origin[1] + h))
    sf = d.add_sketch(sk)
    return sf, d.extrude(sf, distance=f"{t} mm")


def puck(d, r, t, centre=(0.0, 0.0)):
    sk = iw.Sketch("XY")
    sk.circle(centre, r)
    sf = d.add_sketch(sk)
    return sf, d.extrude(sf, distance=f"{t} mm")


def failed(ev):
    return [(s.feature_id, s.error) for s in ev.statuses if s.state == "failed"]


class ImportStepIntoDocument(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="iw_py_import_")
        # Source file: a 40 x 20 x 10 block and a separate r=5 puck, exported as two bodies.
        src = iw.Document()
        _, self.block_ext = block(src, 40, 20, 10)
        _, self.puck_ext = puck(src, 5, 8, centre=(100.0, 0.0))
        self.assertTrue(src.evaluate().ok)
        self.block_volume = self.block_ext.body.volume
        self.puck_volume = self.puck_ext.body.volume
        self.path = os.path.join(self.tmp, "parts.step")
        src.export(self.path)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_import_adds_a_timeline_feature_with_one_body_per_solid(self):
        d = iw.Document()
        imp = d.import_step(self.path)
        self.assertEqual(imp.kind, "import")
        self.assertEqual([f.kind for f in d.timeline], ["import"])
        ev = d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        bodies = imp.bodies
        self.assertEqual(len(bodies), 2)
        self.assertEqual(imp.body.id, bodies[0].id)
        volumes = sorted(b.volume for b in bodies)
        self.assertAlmostEqual(volumes[0], self.puck_volume, places=3)
        self.assertAlmostEqual(volumes[1], self.block_volume, places=3)
        self.assertEqual(len(d.bodies), 2)

    def test_standalone_import_step_is_unchanged(self):
        solids = iw.import_step(self.path)
        self.assertEqual(len(solids), 2)

    def test_imported_body_takes_downstream_features_and_survives_save_load(self):
        d = iw.Document()
        imp = d.import_step(self.path)
        block_body = max(imp.bodies, key=lambda b: b.volume)
        vertical = block_body.edges(lambda e: e.kind == "line" and abs(abs(e.direction[2]) - 1) < 1e-9)
        self.assertEqual(len(vertical), 4)
        d.fillet(vertical[:1], radius=2)
        ev = d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        filleted = block_body.volume
        self.assertLess(filleted, self.block_volume)
        out = os.path.join(self.tmp, "doc")
        d.save(out)
        loaded = iw.Document.open(out)
        self.assertEqual([f.kind for f in loaded.timeline], ["import", "fillet"])
        ev = loaded.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        self.assertAlmostEqual(sum(b.volume for b in loaded.bodies), filleted + self.puck_volume, places=3)

    def test_missing_file_reports_a_clear_error(self):
        with self.assertRaises(iw.IoError):
            iw.Document().import_step(os.path.join(self.tmp, "nope.step"))
        d = iw.Document()
        imp = d.import_step(self.path)
        os.remove(self.path)
        ev = d.evaluate()
        self.assertFalse(ev.ok)
        self.assertEqual(imp.status, "failed")
        self.assertIn("parts.step", imp.error)
        self.assertIn("missing", imp.error)
        with self.assertRaises(iw.IoError):
            ev.raise_for_errors()


class EdgeTypeFilters(unittest.TestCase):
    def test_block_edges_are_lines(self):
        d = iw.Document()
        _, ext = block(d, 40, 20, 10)
        body = ext.body
        self.assertEqual(len(body.edges("line")), 12)
        self.assertEqual(body.edges("circle"), [])
        e = body.edges("line")[0]
        self.assertEqual(e.kind, "line")
        self.assertAlmostEqual(math.hypot(*e.direction), 1.0, places=12)
        self.assertIsNone(e.radius)
        self.assertIsNone(e.center)

    def test_puck_has_circles_with_centre_axis_radius(self):
        d = iw.Document()
        _, ext = puck(d, 6, 4, centre=(10.0, -5.0))
        circles = ext.body.edges("circle")
        self.assertEqual(len(circles), 2)
        for c in circles:
            self.assertTrue(c.closed)
            self.assertAlmostEqual(c.radius, 6, places=9)
            self.assertAlmostEqual(c.center[0], 10, places=9)
            self.assertAlmostEqual(c.center[1], -5, places=9)
            self.assertAlmostEqual(abs(c.axis[2]), 1, places=12)
            self.assertIsNone(c.direction)
        self.assertEqual({round(c.center[2]) for c in circles}, {0, 4})
        both = ext.body.edges("line|circle")
        self.assertEqual(len(both), len(ext.body.edges()))
        # Callable filters still work and can combine type and size.
        self.assertEqual(len(ext.body.edges(lambda e: e.kind == "circle" and e.center[2] > 2)), 1)

    def test_unknown_kind_is_rejected(self):
        d = iw.Document()
        _, ext = block(d, 10, 10, 10)
        with self.assertRaises(iw.InvalidArgumentError):
            ext.body.edges("arc")
        with self.assertRaises(iw.InvalidArgumentError):
            ext.body.faces("sphere")
        self.assertEqual(len(ext.body.faces("plane")), 6)


class ConstructionKinds(unittest.TestCase):
    def setUp(self):
        self.d = iw.Document()
        _, self.ext = block(self.d, 40, 20, 10)
        self.body = self.ext.body

    def face(self, axis, value):
        return self.body.faces(lambda f: f.kind == "plane" and abs(abs(f.normal[axis]) - 1) < 1e-9
                               and abs(f.centroid[axis] - value) < 1e-6)[0]

    def test_midplane_between_two_faces(self):
        mid = self.d.midplane(self.face(0, 0), self.face(0, 40))
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        g = mid.construction
        self.assertEqual(g["kind"], "plane")
        self.assertAlmostEqual(abs(g["normal"][0]), 1, places=9)
        self.assertAlmostEqual(g["origin"][0], 20, places=6)

    def test_plane_through_three_points(self):
        p = self.d.plane_through_points((0, 0, 5), (1, 0, 5), ("0 mm", "1 mm", "5 mm"))
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        g = p.construction
        self.assertAlmostEqual(g["normal"][2], 1, places=9)
        self.assertAlmostEqual(g["origin"][2], 5, places=9)

    def test_axes_and_angle_plane(self):
        ax = self.d.axis_through_points((0, 0, 0), (0, 0, 10))
        edge = self.body.edges(lambda e: e.kind == "line" and abs(abs(e.direction[1]) - 1) < 1e-9)[0]
        along = self.d.axis_along_edge(edge)
        angled = self.d.plane_at_angle(self.face(0, 0), ax, 90)
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        self.assertEqual(ax.construction["kind"], "axis")
        self.assertAlmostEqual(abs(ax.construction["direction"][2]), 1, places=9)
        self.assertAlmostEqual(abs(along.construction["direction"][1]), 1, places=9)
        # The YZ-facing side plane turned 90 degrees about Z faces along Y.
        self.assertAlmostEqual(abs(angled.construction["normal"][1]), 1, places=9)

    def test_axis_through_cylinder_and_points(self):
        _, p = puck(self.d, 3, 5, centre=(70.0, 0.0))
        cyl = p.body.faces("cylinder")[0]
        ax = self.d.axis_through_cylinder(cyl)
        centre = self.d.point_at_circle_centre(p.body.edges("circle")[0])
        corner = self.body.vertices(lambda v: max(abs(c) for c in v.position) < 1e-9)[0]
        at_vertex = self.d.point_at_vertex(corner)
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        self.assertAlmostEqual(ax.construction["origin"][0], 70, places=6)
        self.assertAlmostEqual(abs(ax.construction["direction"][2]), 1, places=9)
        self.assertEqual(centre.construction["kind"], "point")
        self.assertAlmostEqual(centre.construction["origin"][0], 70, places=6)
        for c in at_vertex.construction["origin"]:
            self.assertAlmostEqual(c, 0.0, places=9)
        self.assertEqual(len(self.body.vertices()), 8)

    def test_non_construction_feature_has_no_construction_geometry(self):
        self.assertIsNone(self.ext.construction)

    def test_offset_plane_rejects_a_non_planar_face(self):
        _, p = puck(self.d, 3, 5, centre=(70.0, 0.0))
        with self.assertRaises(iw.InvalidArgumentError):
            self.d.offset_plane(p.body.faces("cylinder")[0], 5)


if __name__ == "__main__":
    unittest.main()
