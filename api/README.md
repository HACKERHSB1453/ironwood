# ironwood (Python API)

Headless Python bindings for Ironwood. Build the module with the normal CMake build (target `ironwood_py`); the
file is `build/<dir>/api/ironwood.<abi>.so`. Put that directory on `PYTHONPATH`.

Lengths are mm and angles are degrees. Pass numbers, or expression strings such as `"w * 2"` or `"2 in"`.
Errors raise `ironwood.IronwoodError` subclasses (`ParseError`, `InvalidArgumentError`, `NotFoundError`,
`KernelError`, ...) with the C++ message.

## Example: a bracket

```python
import ironwood as iw

d = iw.Document(name="bracket")
d.add_parameter("w", "40 mm")
d.add_parameter("h", "20 mm")
d.add_parameter("t", "10 mm")
d.add_parameter("r", "3 mm")

sk = iw.Sketch("XY")
bottom, right, top, left = sk.rect((0, 0), (40, 20))
sk.dimension("distance_x", [(bottom, "start"), (bottom, "end")], "w")
sk.dimension("distance_y", [(right, "start"), (right, "end")], "h")
assert sk.solve(d).ok

body = d.extrude(d.add_sketch(sk), distance="t").body

corners = body.edges(lambda e: abs(e.centroid[1]) < 1e-6 and abs(e.centroid[2] - 5) < 1e-6 and abs(e.length - 10) < 1e-6)
d.fillet(corners, radius="r")

top_face = body.faces(lambda f: f.kind == "plane" and abs(f.normal[2] - 1) < 1e-9 and abs(f.centroid[2] - 10) < 1e-6)[0]
d.hole(top_face, diameter="6 mm", through=True, at=top_face.local((20, 10, 10)))

print(d.evaluate().ok, body.volume)   # all features ok, volume in mm^3
d.set_parameter("w", "50 mm")          # edits flow through the timeline
print(body.volume)
d.save("/tmp/bracket.iwdoc")           # or any directory path
d.export("/tmp/bracket.step")          # .step / .stl (one body) / .3mf
```

## Main objects

- `Document`: `add_parameter`, `set_parameter`, `parameters`, `add_sketch`, `extrude`, `revolve`, `fillet`,
  `chamfer`, `hole` (on a face at `at=(u, v)`), `hole_from_sketch` (see below), `move`, construction geometry
  (`offset_plane`, `midplane`, `plane_through_points`, `plane_at_angle`, `plane_tangent(face, angle)` on a
  cylinder/cone, `axis_through_cylinder`, `axis_through_points`, `axis_along_edge`, `axis_two_planes(a, b)`,
  `point_at_vertex`, `point_at_circle_centre`, `point_edge_plane(edge, plane)`; plane inputs are planar `FaceRef`s
  or Plane features; read back with `Feature.construction`), `import_step` (an Import feature with one body
  per solid, `Feature.bodies`; the STEP file is re-read on recompute and must stay available), `suppress`, `delete`,
  `rollback`, `undo`/`redo`, `timeline`, `evaluate()`, `bodies`, `export`, `save`, `Document.open`.
- `Sketch`: `line`, `rect`, `circle`, `arc`, `point`, `constrain`, `dimension`, `solve` (returns `SolveReport`
  with DOF and conflicting record ids), `profile_names`. `Sketch.on_face(face)` sketches on a planar face. `Sketch.ORIGIN`,
  `Sketch.X_AXIS`, `Sketch.Y_AXIS` are the ids of the fixed reference geometry every sketch holds (anchor dimensions
  and constraints to them so a sketch does not drift when its dimensions change).
- `Body`: `volume`, `area`, `centre_of_mass`, `bounding_box`, `face_count`, `edge_count`, `vertex_count`,
  `faces(filter)`, `edges(filter)`, `vertices(filter)`. Filters are callables over `FaceRef` / `EdgeRef` /
  `VertexRef`, or (faces, edges) a kind string: `body.edges("circle")`, `body.edges("line|circle")`,
  `body.faces("plane")`. `EdgeRef.kind` is `line`, `circle`, `ellipse`, `bspline` or `other`; lines have
  `direction`, circles / ellipses `center`, `axis`, `radius` (ellipses also `minor_radius`).
- `iw.import_step(path)` / `iw.import_stl(path)` return standalone `Solid` objects (not on the timeline); use
  `Document.import_step(path)` to import onto the timeline.

## Holes from sketch points

```python
sk = iw.Sketch.on_face(top_face)            # or any sketch whose plane lies on / in the body
p1, p2 = sk.point((10, 10)), sk.point((30, 10))
holes = d.hole_from_sketch(d.add_sketch(sk), [p1, p2], diameter="hd", through=True)   # or depth="8 mm"
```

One Hole feature, one hole per point (point ids, circle / arc ids for their centre, or `Sketch.ORIGIN`). Points are
bound by id, so editing the sketch or its parameters moves the holes; a deleted point is skipped with a warning. The
holes run along the sketch normal into the body (the side the body lies on; `flip=True` reverses), depth is measured
from the sketch plane. `body=` is needed only when the document has more than one body. `kind`, `drill_angle`,
counterbore / countersink sizes and `thread` work as for `hole`.

## Tests

`tests/python/test_*.py` (unittest; the ctest `python_api` runs it). Run manually with
`PYTHONPATH=build/<dir>/api python3 -m unittest discover -s tests/python -t tests/python`.
