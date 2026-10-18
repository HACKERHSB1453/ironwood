# Benchmarks

`ironwood-bench` (source: `bench/bench.cpp`) prints one JSON document to stdout. Baselines are committed under
`bench/baselines/`, one file per date and machine.

## How to run

Use a Release build and run one benchmark process at a time, with the machine otherwise idle (timings are wall-clock
medians; parallel builds or GPU work skew them).

```bash
cmake --preset release -B build/bench
cmake --build build/bench -j 4 --target ironwood-bench
build/bench/ironwood-bench > bench/baselines/<date>_<machine>.json
```

Progress and diagnostics go to stderr. The process exits non-zero if any feature in a timeline fails, so a broken build
cannot produce a plausible-looking baseline. Each timing is the median of 3 repetitions. `n` is the count of work
items covered (see the table).

**Validation is on.** Boolean, fillet and chamfer results are checked (BRepCheck, volume and bounds invariants,
F38/F39) on every operation, and there is no switch for it in the kernel. Every number below includes that cost. F43
records the price: cold recompute is roughly 80% slower than without validation.

## What each benchmark measures

| Name | What it does | `n` |
|---|---|---|
| `step_import` | Imports a 12-part STEP fixture (8 boxes, 4 cylinders) written by the kernel's own exporter. | nodes |
| `evaluate_cold_50` | Cold evaluation of a 50-feature timeline of overlapping prism joins (fresh evaluator). | features computed |
| `tessellate_body_{coarse,fine}` | Tessellates the evaluated timeline body at deflection 0.5 / 0.05 mm (planar, so deflection barely matters). | triangles |
| `tessellate_cylinder_{coarse,fine}` | Same, on a curved cylinder (r 10, h 20), where deflection matters. | triangles |
| `tessellate_step_{coarse,fine}` | Tessellates the 12-part STEP fixture. | triangles |
| `reevaluate_edit_50` | Warm evaluator: edits feature 50, re-evaluates. Shows cache reach for a late edit. | features recomputed |
| `reevaluate_edit_1` | Warm evaluator: edits feature 1, re-evaluates everything after it. | features recomputed |
| `step_import_200` | Imports a generated 200-part assembly (100 boxes, 100 cylinders, about 47k STEP entities). | nodes |
| `tessellate_step200_{coarse,fine}` | Tessellates all parts of the 200-part assembly. | triangles |
| `sketch_solve_200` | `solveSketch` on a 200-entity sketch (200 lines, each H or V with a 10 mm distance dimension, initial geometry about 0.4 mm off so the solver iterates). Fresh copy per repetition; only the solve is timed. | entities |
| `fillet_box_12_edges` | Constant 2 mm fillet of all 12 edges of a 40 x 30 x 20 mm box (kernel op). | faces after |
| `hole_box_counterbore` | Through counterbore hole (d 6, cbore d 10 x 3 deep) on the top face of the same box (kernel op). | faces after |
| `realistic_100_cold` | Cold evaluation of a 100-feature realistic part: base block, four corner fillets, 49 sketch-profile pocket cuts and 49 holes. | features computed |
| `realistic_edit_early_pd` | Warm: edits `pd` (pocket depth, used by every pocket), so the recompute starts at feature 3. | features recomputed |
| `realistic_edit_late_hdl` | Warm: edits `hdl` (diameter of the last hole only). | features recomputed |

The realistic part is built in two passes: a scripted evaluation first collects the face and edge names it needs, then
the timed passes reuse the same feature list.

## Current numbers

Machine: Intel Core i7-14700KF (28 logical cores), NVIDIA GeForce RTX 5080 (Vulkan name recorded). Release build.
Baseline file: `bench/baselines/2026-10-19_i7-14700KF_rtx5080.json`.

| Benchmark | ms (median) | n |
|---|---:|---:|
| step_import | 14.7 | 13 |
| evaluate_cold_50 | 989.9 | 50 |
| tessellate_body_coarse | 9.1 | 412 |
| tessellate_body_fine | 7.9 | 412 |
| tessellate_cylinder_coarse | 0.34 | 100 |
| tessellate_cylinder_fine | 0.47 | 176 |
| tessellate_step_coarse | 3.05 | 496 |
| tessellate_step_fine | 3.05 | 532 |
| reevaluate_edit_50 | 36.0 | 1 |
| reevaluate_edit_1 | 1030.3 | 50 |
| step_import_200 | 217.5 | 201 |
| tessellate_step200_coarse | 40.7 | 11200 |
| tessellate_step200_fine | 44.1 | 12388 |
| sketch_solve_200 | 8.6 | 200 |
| fillet_box_12_edges | 4.7 | 26 |
| hole_box_counterbore | 2.7 | 9 |
| realistic_100_cold | 4100.0 | 100 |
| realistic_edit_early_pd | 4031.1 | 98 |
| realistic_edit_late_hdl | 84.8 | 1 |

A second run on the same machine differed by under 3% for every kernel and evaluation benchmark (the sub-millisecond
tessellation timings varied by up to 24%).

Read the numbers with these in mind:

- Cold evaluation costs about 20 ms per boolean feature (50 features in 990 ms), mostly the fuse; validation (F43) is
  included in that figure.
- A late edit is about 28x cheaper than an early one on the 50-feature timeline (36 ms against 1030 ms), because
  prefix reuse skips everything before the edit. The realistic part shows the same pattern (85 ms against 4031 ms).
- The 200-part STEP import takes about 0.2 s; tessellation of the same assembly takes about 40 ms at coarse
  deflection.
