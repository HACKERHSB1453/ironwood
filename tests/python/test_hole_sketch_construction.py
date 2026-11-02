# SPDX-License-Identifier: GPL-3.0-or-later
"""F26 bindings: Document.hole_from_sketch (holes at sketch points), construction plane_tangent, axis_two_planes and
point_edge_plane, and the Sketch.ORIGIN / X_AXIS / Y_AXIS reference ids. Run with PYTHONPATH pointing at the built
module."""

import math
import os
import shutil
import tempfile
import unittest

import ironwood as iw

BLOCK = 40.0 * 20.0 * 10.0


def failed(ev):
    return [(s.feature_id, s.error) for s in ev.statuses if s.state == "failed"]


def block(d, w=40.0, h=20.0, t=10.0):
    """A w x h x t block on XY from (0, 0, 0)."""
    sk = iw.Sketch("XY")
    sk.rect((0, 0), (w, h))
    return d.extrude(d.add_sketch(sk), distance=f"{t} mm")


def puck(d, r=5.0, t=8.0):
    sk = iw.Sketch("XY")
    sk.circle((0, 0), r)
    return d.extrude(d.add_sketch(sk), distance=f"{t} mm")


def close(a, b, tol=1e-6):
    return all(abs(x - y) < tol for x, y in zip(a, b))


def face_with_normal(body, n, kind="plane"):
    return body.faces(lambda f: f.kind == kind and close(f.normal, n, 1e-9))[0]


class SketchReferenceIds(unittest.TestCase):
    def test_reference_ids_are_distinct_ints_usable_in_constraints(self):
        ids = {iw.Sketch.ORIGIN, iw.Sketch.X_AXIS, iw.Sketch.Y_AXIS}
        self.assertEqual(len(ids), 3)
        sk = iw.Sketch("XY")
        p = sk.point((3, 4))
        sk.dimension("distance_x", [iw.Sketch.ORIGIN, p], 12)
        sk.dimension("distance_y", [iw.Sketch.ORIGIN, p], 7)
        rep = sk.solve()
        self.assertTrue(rep.ok, rep.message)
        self.assertEqual(rep.dof, 0)


class HoleFromSketch(unittest.TestCase):
    def setUp(self):
        self.d = iw.Document()
        self.ext = block(self.d)
        self.body = self.ext.body

    def hole_sketch(self, z, pts, plane_normal=(0, 0, 1)):
        sk = iw.Sketch("XY", origin=(0, 0, z)) if plane_normal == (0, 0, 1) else iw.Sketch(
            origin=(0, 0, z), normal=plane_normal, x_dir=(1, 0, 0))
        ids = [sk.point(p) for p in pts]
        return sk, ids

    def test_through_holes_from_points_on_the_top_plane_drill_down(self):
        sk, ids = self.hole_sketch(10.0, [(10, 10), (30, 10)])
        sf = self.d.add_sketch(sk)
        hole = self.d.hole_from_sketch(sf, ids, diameter=4, through=True)
        self.assertEqual(hole.kind, "hole")
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        self.assertAlmostEqual(self.body.volume, BLOCK - 2 * math.pi * 4.0 * 10.0, places=3)
        centres = sorted(round(e.center[0], 6) for e in self.body.edges("circle"))
        self.assertEqual(centres, [10.0, 10.0, 30.0, 30.0])  # entry + exit circle per hole

    def test_bottom_plane_drills_up_and_blind_depth_is_from_the_sketch_plane(self):
        sk, ids = self.hole_sketch(0.0, [(20, 10)])
        sf = self.d.add_sketch(sk)
        self.d.hole_from_sketch(sf, ids, diameter="4 mm", depth="6 mm", drill_angle="180 deg")
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        self.assertAlmostEqual(self.body.volume, BLOCK - math.pi * 4.0 * 6.0, places=3)
        # The hole floor is at z = 6 (drilled up from the bottom face).
        floor = self.body.faces(lambda f: f.kind == "plane" and abs(f.centroid[2] - 6.0) < 1e-6)
        self.assertEqual(len(floor), 1)

    def test_flip_reverses_the_drill_direction(self):
        # A plane at z = 10 with the body below: the default drills down; flipped it drills up, away from the body.
        sk, ids = self.hole_sketch(10.0, [(20, 10)])
        sf = self.d.add_sketch(sk)
        hole = self.d.hole_from_sketch(sf, ids, diameter=4, depth=5, flip=True, drill_angle="180 deg")
        ev = self.d.evaluate()
        self.assertEqual(hole.status, "ok", failed(ev))
        # The 5 mm bore now lies above the part and cuts nothing; only the kernel's entry-side clearance of
        # max(1 mm, diameter) = 4 mm (documented on HoleSketchPlacement) reaches down into the block.
        self.assertAlmostEqual(self.body.volume, BLOCK - math.pi * 4.0 * 4.0, places=3)

    def test_circle_centre_and_origin_are_accepted(self):
        sk = iw.Sketch("XY", origin=(0, 0, 10))
        c = sk.circle((30, 10), 1.0, construction=True)
        sf = self.d.add_sketch(sk)
        # The origin (0, 0) is a block corner: a quarter of that hole lies in the material.
        self.d.hole_from_sketch(sf, [c, iw.Sketch.ORIGIN], diameter=4, through=True)
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        expected = BLOCK - math.pi * 4.0 * 10.0 - 0.25 * math.pi * 4.0 * 10.0
        self.assertAlmostEqual(self.body.volume, expected, places=3)

    def test_counterbore_kind_and_thread_attribute(self):
        sk, ids = self.hole_sketch(10.0, [(20, 10)])
        sf = self.d.add_sketch(sk)
        self.d.hole_from_sketch(sf, ids, diameter=4, through=True, kind="counterbore", counterbore_diameter=8,
                                counterbore_depth=3, thread="M4x0.7")
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        expected = BLOCK - math.pi * 4.0 * 7.0 - math.pi * 16.0 * 3.0
        self.assertAlmostEqual(self.body.volume, expected, places=3)

    def test_editing_the_sketch_moves_the_holes(self):
        sk = iw.Sketch("XY", origin=(0, 0, 10))
        p = sk.point((10, 10))
        dx = sk.dimension("distance_x", [iw.Sketch.ORIGIN, p], "px")
        sk.dimension("distance_y", [iw.Sketch.ORIGIN, p], 10)
        self.assertGreater(dx, 0)
        self.d.add_parameter("px", "10 mm")
        self.assertTrue(sk.solve(self.d).ok)
        sf = self.d.add_sketch(sk)
        hole = self.d.hole_from_sketch(sf, [p], diameter=4, through=True)
        self.assertTrue(self.d.evaluate().ok)
        self.assertTrue(all(abs(e.center[0] - 10.0) < 1e-6 for e in self.body.edges("circle")))
        self.d.set_parameter("px", "25 mm")
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        self.assertEqual(hole.status, "ok")
        self.assertTrue(all(abs(e.center[0] - 25.0) < 1e-6 for e in self.body.edges("circle")))
        self.assertAlmostEqual(self.body.volume, BLOCK - math.pi * 4.0 * 10.0, places=3)

    def test_missing_point_is_skipped_and_all_missing_fails(self):
        sk, ids = self.hole_sketch(10.0, [(10, 10), (30, 10)])
        sf = self.d.add_sketch(sk)
        hole = self.d.hole_from_sketch(sf, ids, diameter=4, through=True)
        self.assertTrue(self.d.evaluate().ok)
        edited = sf.sketch
        edited.remove(ids[1])
        self.d.edit_sketch(sf, edited)
        ev = self.d.evaluate()
        self.assertEqual(hole.status, "ok", failed(ev))
        self.assertAlmostEqual(self.body.volume, BLOCK - math.pi * 4.0 * 10.0, places=3)
        edited.remove(ids[0])
        self.d.edit_sketch(sf, edited)
        self.d.evaluate()
        self.assertEqual(hole.status, "failed")

    def test_argument_errors(self):
        sk, ids = self.hole_sketch(10.0, [(10, 10)])
        sf = self.d.add_sketch(sk)
        with self.assertRaises(iw.InvalidArgumentError):
            self.d.hole_from_sketch(sf, ids, diameter=4)  # neither depth nor through
        with self.assertRaises(iw.InvalidArgumentError):
            self.d.hole_from_sketch(sf, [], diameter=4, through=True)
        with self.assertRaises(iw.InvalidArgumentError):
            self.d.hole_from_sketch(self.ext, ids, diameter=4, through=True)  # not a sketch
        with self.assertRaises(iw.InvalidArgumentError):
            self.d.hole_from_sketch(sf, ids, diameter=4, through=True, kind="tapped")

    def test_body_is_required_when_the_document_has_several(self):
        sk = iw.Sketch("XY")
        sk.rect((100, 0), (110, 10))
        other = self.d.extrude(self.d.add_sketch(sk), distance=5).body
        hs, ids = self.hole_sketch(10.0, [(20, 10)])
        sf = self.d.add_sketch(hs)
        with self.assertRaises(iw.InvalidArgumentError):
            self.d.hole_from_sketch(sf, ids, diameter=4, through=True)
        self.d.hole_from_sketch(sf, ids, diameter=4, through=True, body=self.body)
        self.assertTrue(self.d.evaluate().ok)
        self.assertAlmostEqual(self.body.volume, BLOCK - math.pi * 4.0 * 10.0, places=3)
        self.assertAlmostEqual(other.volume, 500.0, places=3)

    def test_save_load_round_trip_then_edit(self):
        tmp = tempfile.mkdtemp(prefix="iw_py_hole_sketch_")
        try:
            self.d.add_parameter("px", "10 mm")
            sk = iw.Sketch("XY", origin=(0, 0, 10))
            p = sk.point((10, 10))
            sk.dimension("distance_x", [iw.Sketch.ORIGIN, p], "px")
            sk.dimension("distance_y", [iw.Sketch.ORIGIN, p], 10)
            self.assertTrue(sk.solve(self.d).ok)
            self.d.hole_from_sketch(self.d.add_sketch(sk), [p], diameter=4, through=True)
            self.assertTrue(self.d.evaluate().ok)
            path = os.path.join(tmp, "holes.iwdoc")
            self.d.save(path)
            loaded = iw.Document.open(path)
            ev = loaded.evaluate()
            self.assertTrue(ev.ok, failed(ev))
            self.assertEqual([f.kind for f in loaded.timeline], ["sketch", "extrude", "sketch", "hole"])
            lbody = loaded.bodies[0]
            self.assertAlmostEqual(lbody.volume, BLOCK - math.pi * 4.0 * 10.0, places=3)
            loaded.set_parameter("px", "30 mm")
            self.assertTrue(loaded.evaluate().ok)
            self.assertTrue(all(abs(e.center[0] - 30.0) < 1e-6 for e in lbody.edges("circle")))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TangentPlane(unittest.TestCase):
    def setUp(self):
        self.d = iw.Document()
        self.body = puck(self.d).body
        self.side = self.body.faces("cylinder")[0]

    def test_zero_and_ninety_degrees_touch_plus_x_and_plus_y(self):
        p0 = self.d.plane_tangent(self.side, 0)
        p90 = self.d.plane_tangent(self.side, "90 deg")
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        g0, g90 = p0.construction, p90.construction
        self.assertEqual(g0["kind"], "plane")
        self.assertTrue(close(g0["normal"], (1, 0, 0)), g0)
        self.assertTrue(close(g0["origin"][:2], (5, 0)), g0)
        self.assertTrue(close(g90["normal"], (0, 1, 0)), g90)
        self.assertTrue(close(g90["origin"][:2], (0, 5)), g90)
        self.assertAlmostEqual(abs(g0["x_dir"][2]), 1.0)  # x_dir runs along the ruling (the Z axis)

    def test_angle_is_an_expression_driven_by_parameters(self):
        self.d.add_parameter("a", "180 deg")
        pl = self.d.plane_tangent(self.side, "a")
        self.assertTrue(self.d.evaluate().ok)
        self.assertTrue(close(pl.construction["normal"], (-1, 0, 0)))
        self.d.set_parameter("a", "270 deg")
        self.assertTrue(self.d.evaluate().ok)
        self.assertTrue(close(pl.construction["normal"], (0, -1, 0)))

    def test_sketch_on_tangent_plane_feeds_downstream(self):
        pl = self.d.plane_tangent(self.side, 0)
        self.assertTrue(self.d.evaluate().ok)
        self.d.offset_plane(pl, 2)
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        self.assertTrue(close(self.d.timeline[-1].construction["origin"][:2], (7, 0)))

    def test_planar_face_is_rejected(self):
        top = face_with_normal(self.body, (0, 0, 1))
        with self.assertRaises(iw.InvalidArgumentError):
            self.d.plane_tangent(top, 0)


class TwoPlaneAxisAndEdgePlanePoint(unittest.TestCase):
    def setUp(self):
        self.d = iw.Document()
        self.body = block(self.d).body
        self.top = face_with_normal(self.body, (0, 0, 1))
        self.right = face_with_normal(self.body, (1, 0, 0))
        self.bottom = face_with_normal(self.body, (0, 0, -1))

    def test_axis_from_two_faces_is_their_common_edge_line(self):
        ax = self.d.axis_two_planes(self.top, self.right)
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        g = ax.construction
        self.assertEqual(g["kind"], "axis")
        # top normal (0,0,1) x right normal (1,0,0) = (0,1,0); origin = the point of x=40, z=10 nearest the world origin.
        self.assertTrue(close(g["direction"], (0, 1, 0)), g)
        self.assertTrue(close(g["origin"], (40, 0, 10)), g)

    def test_axis_from_a_construction_plane_and_a_face(self):
        mid = self.d.offset_plane(self.bottom, 4)  # parallel to the bottom face, 4 mm along its normal
        ax = self.d.axis_two_planes(mid, self.right)
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        g = ax.construction
        self.assertAlmostEqual(abs(g["direction"][1]), 1.0)
        self.assertAlmostEqual(g["origin"][0], 40.0)
        self.assertAlmostEqual(g["origin"][2], mid.construction["origin"][2])

    def test_parallel_planes_fail_the_feature(self):
        ax = self.d.axis_two_planes(self.top, self.bottom)
        ev = self.d.evaluate()
        self.assertFalse(ev.ok)
        self.assertEqual(ax.status, "failed")
        self.assertIn("parallel", ax.error)

    def test_point_where_a_vertical_edge_crosses_a_plane(self):
        plane = self.d.plane_through_points((0, 0, 3), (1, 0, 3), (0, 1, 3))
        edge = self.body.edges(lambda e: e.kind == "line" and close(e.centroid, (40, 20, 5)))[0]
        pt = self.d.point_edge_plane(edge, plane)
        ev = self.d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        g = pt.construction
        self.assertEqual(g["kind"], "point")
        self.assertTrue(close(g["origin"], (40, 20, 3)), g)

    def test_point_with_a_face_as_plane_and_no_crossing_fails(self):
        edge = self.body.edges(lambda e: e.kind == "line" and close(e.centroid, (20, 0, 10)))[0]  # top front edge
        ok = self.d.point_edge_plane(self.body.edges(lambda e: close(e.centroid, (40, 0, 5)))[0], self.top)
        touch = self.d.point_edge_plane(edge, self.right)  # edge along X ends at x = 40: touches the plane at its end
        miss = self.d.point_edge_plane(self.body.edges(lambda e: close(e.centroid, (0, 0, 5)))[0],
                                       self.d.offset_plane(self.top, 5))  # z = 15: beyond the 0..10 edge
        self.d.evaluate()
        self.assertEqual(ok.status, "ok")
        self.assertTrue(close(ok.construction["origin"], (40, 0, 10)))
        self.assertEqual(touch.status, "ok")  # an end-point touch counts as a crossing
        self.assertTrue(close(touch.construction["origin"], (40, 0, 10)))
        self.assertEqual(miss.status, "failed")
        self.assertIsNone(miss.construction)

    def test_plane_argument_must_be_planar(self):
        p = iw.Document()
        body = puck(p).body
        edge = body.edges("circle")[0]
        with self.assertRaises(iw.InvalidArgumentError):
            p.point_edge_plane(edge, body.faces("cylinder")[0])
        with self.assertRaises(iw.InvalidArgumentError):
            p.axis_two_planes(body.faces("cylinder")[0], body.faces("plane")[0])


if __name__ == "__main__":
    unittest.main()
