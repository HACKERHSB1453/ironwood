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
  `chamfer`, `hole`, `move`, `offset_plane`, `suppress`, `delete`, `rollback`, `undo`/`redo`, `timeline`,
  `evaluate()`, `bodies`, `export`, `save`, `Document.open`.
- `Sketch`: `line`, `rect`, `circle`, `arc`, `point`, `constrain`, `dimension`, `solve` (returns `SolveReport`
  with DOF and conflicting record ids), `profile_names`. `Sketch.on_face(face)` sketches on a planar face.
- `Body`: `volume`, `area`, `centre_of_mass`, `bounding_box`, `face_count`, `edge_count`, `vertex_count`,
  `faces(filter)`, `edges(filter)`. Filters are callables over `FaceRef` / `EdgeRef`.
- `iw.import_step(path)` / `iw.import_stl(path)` return standalone `Solid` objects (not on the timeline).

## Tests

`tests/python/test_ironwood.py` (unittest; the ctest `python_api` runs it). Run manually with
`PYTHONPATH=build/<dir>/api python3 -m unittest discover -s tests/python -t tests/python`.
