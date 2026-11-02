# SPDX-License-Identifier: GPL-3.0-or-later
"""P1a C1 end-to-end scenario, scripted through the Python API: a mounting plate bracket.

One fully constrained sketch (anchored to the sketch ORIGIN) holds the w x h outline and two hole-centre points whose
positions are dimensioned with parameters -> extrude by t -> fillet the four vertical corners (r) -> one Hole feature
"from sketch points" (through, diameter hd) -> edit the early parameters (w, h, t, e) one at a time and together.
After every edit the fillet and the hole feature must stay Ok, the topology (face / edge counts) must be unchanged,
the holes must follow their dimensioned points, and the volume must match the analytic value.

test_e2e_parts.py covers the same chain with face-placed holes; this file covers placement from sketch points (F26),
which is how C1 is specified."""

import math
import os
import shutil
import tempfile
import unittest

import ironwood as iw

ORIGIN = iw.Sketch.ORIGIN


def plate_volume(w, h, t, r, hd, holes=2):
    """w x h x t plate, four vertical corners rounded with r (each removes (1 - pi/4) r^2 t), minus through holes."""
    corners = 4 * (1.0 - math.pi / 4.0) * r * r * t
    return w * h * t - corners - holes * math.pi * (hd / 2.0) ** 2 * t


def failed(ev):
    return [(s.feature_id, s.error) for s in ev.statuses if s.state == "failed"]


def build(w=60.0, h=30.0, t=8.0, r=4.0, hd=5.0, e=12.0):
    d = iw.Document(name="c1_bracket")
    for name, value in (("w", w), ("h", h), ("t", t), ("r", r), ("hd", hd), ("e", e)):
        d.add_parameter(name, f"{value} mm")

    sk = iw.Sketch("XY")
    bottom, right, top, left = sk.rect((0, 0), (w, h))
    sk.constrain("coincident", [ORIGIN, (bottom, "start")])          # anchor: no drift on edits (F46)
    sk.dimension("distance_x", [(bottom, "start"), (bottom, "end")], "w")
    sk.dimension("distance_y", [(right, "start"), (right, "end")], "h")
    # Hole centres: e in from the left and right edges, on the mid-height line.
    p1 = sk.point((e, h / 2))
    p2 = sk.point((w - e, h / 2))
    sk.dimension("distance_x", [ORIGIN, p1], "e")
    sk.dimension("distance_y", [ORIGIN, p1], "h / 2")
    sk.dimension("distance_x", [ORIGIN, p2], "w - e")
    sk.dimension("distance_y", [ORIGIN, p2], "h / 2")
    rep = sk.solve(d)
    assert rep.ok, rep.message
    assert rep.dof == 0, f"sketch must be fully constrained, dof={rep.dof}"
    assert len(sk.profile_names()) == 1, sk.profile_names()

    sketch = d.add_sketch(sk)
    extrude = d.extrude(sketch, distance="t")
    body = extrude.body
    ev = d.evaluate()
    assert ev.ok, failed(ev)

    vertical = body.edges(lambda ed: ed.kind == "line" and abs(abs(ed.direction[2]) - 1.0) < 1e-9)
    assert len(vertical) == 4, vertical
    fillet = d.fillet(vertical, radius="r")
    # The sketch plane (z = 0) is the bottom face and the body lies on its +normal side: the holes drill up through it.
    holes = d.hole_from_sketch(sketch, [p1, p2], diameter="hd", through=True)
    return d, sketch, extrude, fillet, holes, body


class C1Bracket(unittest.TestCase):
    def assert_state(self, d, extrude, fillet, holes, body, w, h, t, r, hd, e, msg=""):
        ev = d.evaluate()
        self.assertTrue(ev.ok, f"{msg}: {failed(ev)}")
        for f in (extrude, fillet, holes):
            self.assertEqual(f.status, "ok", f"{msg}: {f.kind} {f.error}")
        self.assertAlmostEqual(body.volume, plate_volume(w, h, t, r, hd), places=2, msg=msg)
        lo, hi = body.bounding_box
        self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip(lo, (0, 0, 0))), f"{msg}: {lo}")
        self.assertTrue(all(abs(a - b) < 1e-6 for a, b in zip(hi, (w, h, t))), f"{msg}: {hi}")
        # Hole circles (an entry and an exit circle each) sit on the dimensioned points.
        hole_circles = body.edges(lambda ed: ed.kind == "circle" and abs(ed.radius - hd / 2) < 1e-6)
        self.assertEqual(len(hole_circles), 4, msg)
        centres = sorted((round(c.center[0], 6), round(c.center[1], 6)) for c in hole_circles)
        self.assertEqual(centres, sorted([(e, h / 2)] * 2 + [(w - e, h / 2)] * 2), msg)
        # Fillet arcs (radius r) are still present on all four corners (top and bottom).
        self.assertEqual(len(body.edges(lambda ed: ed.kind == "circle" and abs(ed.radius - r) < 1e-6)), 8, msg)

    def test_build_is_ok_and_matches_analytic_values(self):
        d, sketch, extrude, fillet, holes, body = build()
        self.assertEqual([f.kind for f in d.timeline], ["sketch", "extrude", "fillet", "hole"])
        self.assert_state(d, extrude, fillet, holes, body, 60, 30, 8, 4, 5, 12, "initial")
        # 6 plate faces + 4 fillet faces + 1 bore per hole.
        self.assertEqual(body.face_count, 12)

    def test_edit_early_parameters_one_at_a_time_keeps_fillet_and_holes(self):
        d, sketch, extrude, fillet, holes, body = build()
        faces, edges = body.face_count, body.edge_count
        vals = dict(w=60.0, h=30.0, t=8.0, r=4.0, hd=5.0, e=12.0)
        for name, value in (("w", 80.0), ("h", 40.0), ("t", 12.0), ("e", 15.0), ("w", 50.0), ("hd", 6.0), ("r", 2.5)):
            d.set_parameter(name, f"{value} mm")
            vals[name] = value
            self.assert_state(d, extrude, fillet, holes, body, msg=f"after {name}={value}", **vals)
            self.assertEqual((body.face_count, body.edge_count), (faces, edges), f"after {name}={value}")

    def test_edit_several_parameters_at_once(self):
        d, sketch, extrude, fillet, holes, body = build()
        d.set_parameter("w", "100 mm")
        d.set_parameter("h", "25 mm")
        d.set_parameter("t", "5 mm")
        self.assert_state(d, extrude, fillet, holes, body, 100, 25, 5, 4, 5, 12, "multi-edit")

    def test_undo_redo_of_an_early_edit(self):
        d, sketch, extrude, fillet, holes, body = build()
        self.assertEqual(sketch.sketch.dimension_count, 6)
        d.set_parameter("e", "20 mm")
        self.assert_state(d, extrude, fillet, holes, body, 60, 30, 8, 4, 5, 20, "e=20")
        d.undo()
        self.assert_state(d, extrude, fillet, holes, body, 60, 30, 8, 4, 5, 12, "undo")
        d.redo()
        self.assert_state(d, extrude, fillet, holes, body, 60, 30, 8, 4, 5, 20, "redo")

    def test_points_moved_off_the_plate_cut_nothing_then_recover(self):
        # e = -20 puts both points outside the plate (x = -20 and x = 80). The hole feature stays Ok but removes no
        # material (current behaviour: no "hole misses the body" warning). Restoring e
        # brings the holes back on the same feature.
        d, sketch, extrude, fillet, holes, body = build()
        d.set_parameter("e", "-20 mm")
        ev = d.evaluate()
        self.assertTrue(ev.ok, failed(ev))
        self.assertAlmostEqual(body.volume, plate_volume(60, 30, 8, 4, 5, holes=0), places=2)
        self.assertEqual(len(body.edges(lambda ed: ed.kind == "circle" and abs(ed.radius - 2.5) < 1e-6)), 0)
        d.set_parameter("e", "12 mm")
        self.assert_state(d, extrude, fillet, holes, body, 60, 30, 8, 4, 5, 12, "restored")

    def test_save_load_then_edit(self):
        tmp = tempfile.mkdtemp(prefix="iw_py_c1_")
        try:
            d, *_ = build()
            path = os.path.join(tmp, "c1.iwdoc")
            d.save(path)
            loaded = iw.Document.open(path)
            self.assertEqual([f.kind for f in loaded.timeline], ["sketch", "extrude", "fillet", "hole"])
            _, extrude, fillet, holes = loaded.timeline
            body = extrude.body
            self.assert_state(loaded, extrude, fillet, holes, body, 60, 30, 8, 4, 5, 12, "loaded")
            loaded.set_parameter("w", "90 mm")
            self.assert_state(loaded, extrude, fillet, holes, body, 90, 30, 8, 4, 5, 12, "loaded + w=90")
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    unittest.main()
