# P1a task list — "first usable modeller"

Goal (ARCHITECTURE §9): a user models real parts end-to-end — sketch with constraints → extrude/revolve →
fillet/chamfer/hole → edit anything in the timeline — in a Fusion-style UI. Exit gate: naming fuzzer passes;
bench baselines recorded. **Routing (user decision): every task starts on Haiku; a task that fails its
acceptance is re-run on Sonnet.** Same rules as P0: file ownership, own `build/<TASK>` dir, `-Werror`,
offscreen-only GUI tests, never open windows on the user's desktop, no commits.

Fusion behaviour references are from public docs/knowledge; anything uncertain is marked [VERIFY].

## Cross-module contracts (decided by the supervisor — implement exactly)
- **Sketch module `src/sketch/` (new, `iw_sketch`)**: depends on core + kernel only. A `Sketch` lives on a
  plane (`origin, normal, xDir` in mm). Entities: Point, Line, Arc, Circle, (Spline later). Every entity has a
  stable `SketchEntityId` (uint64, never reused) and a `construction` flag. Constraints and dimensions refer to
  entity ids + sub-point role (`Start, End, Center, Mid`). Dimensions hold an **expression string** (A1).
- **Sketch naming:** profile regions get names derived from their *bounding entity ids* (sorted) — not from
  region index. Projected/derived geometry keeps the source ElementRef. This feeds ElementName sources so a
  sketch edit that keeps entity ids keeps downstream face names.
- **Features `src/features/` (new, `iw_features`)**: real modelling features move here (ExtrudeFeature migrates
  from recompute in B1). Each feature: params struct (JSON via nlohmann, privately), `explicitRefs` as
  `naming::ElementRef`s, `readsBodies/writesBodies`, registered in `FeatureRegistry`.
- **Expressions (A1)** are evaluated at feature compute time against the document parameter table; the
  parameter table is part of the stamp chain (a parameter edit invalidates exactly the features using it —
  add the used-parameter hash to each feature's paramHash).
- **Commands (B4)**: every user operation is a `ui::Command` (id like `"extrude"`, label, icon, shortcut,
  `start(Context)` opening a dialog) registered in a `CommandRegistry`; toolbar, S-box, marking menu and
  shortcuts all read the registry. Fusion default shortcuts (E extrude, R rect, C circle, L line, D dimension,
  F fillet, H hole, M move, Q press-pull, X construction, S S-box, I measure, P project, T trim, O offset)
  [VERIFY].

## Wave A — independent foundations (start now, parallel)
| Task | Owns | Builds | Acceptance |
|---|---|---|---|
| **A1 Expressions & parameters** | `src/core/expr.*`, `src/core/params.*`, `tests/test_expr.cpp` | Unit-aware expression parser/evaluator: `+ - * / ^`, parens, unary minus, numbers with units (mm cm m in ft deg rad), functions (sin cos tan asin acos atan sqrt abs min max floor ceil round pi), parameter references; dimension checking (length vs angle vs unitless); `ParameterTable` (user params with name/expr/unit/comment/favourite; dependency graph; cycle detection; evaluation order) | `2 in + 5 mm` = 55.8 mm; `d1*2` with d1=10 mm → 20 mm; cycle → error; unit mismatch (`10 mm + 5 deg`) → error; 1000 random expressions round-trip print/parse; renaming a param updates referencing expressions |
| **A2 Sketch entities & profiles** | `src/sketch/` (except `solver*`), `tests/test_sketch.cpp` | `Sketch` model (entities, construction flag, ids), plane transforms 2D↔3D, **profile detection**: planar graph of non-construction curves → closed regions incl. nested holes (even-odd), region area/centroid, region naming per contract; conversion region → `kernel::Shape` face | Rectangle → 1 profile; rect with inner circle → 2 profiles (ring + disc); two overlapping circles → 3 regions; open chain → 0 profiles; region names stable when an unrelated entity is added; area within 1e-9 relative |
| **A3 Constraint solver** | `src/sketch/solver*`, `third_party/planegcs/`, `tests/test_solver.cpp` | Vendor FreeCAD **planegcs** (LGPL-2.1+, keep headers & attribution, add to THIRD_PARTY.md; note any patches in `third_party/planegcs/PATCHES.md`); adapter `SketchSolver` mapping our constraints (coincident, horizontal, vertical, parallel, perpendicular, tangent, equal, concentric, midpoint, collinear, symmetric, fix, distance, distanceX/Y, radius, diameter, angle) ↔ GCS; drag solving (move a point under constraints); DOF count; **conflict & redundancy diagnostics** naming the offending constraints | Fully constrained rectangle reports DOF 0; dragging a corner of an under-constrained rect keeps H/V; conflicting dims → reports exactly the conflicting set; redundant H on an already-H line reported; 1000-entity grid sketch solves < 50 ms after a single-dim edit (release) |
| **A4 Kernel ops** | `src/kernel/*` (additive only), `tests/test_kernel.cpp` | `revolve(face, axis, angle)`, `filletEdges(shape, edgeIdx[], radius)` (constant), `chamferEdges(shape, edgeIdx[], d1, d2|angle)`, `hole(shape, placement, spec{simple/counterbore/countersink, dia, depth|through, cbore/csink params, drill-point angle})`, `transform(shape, matrix)` (move), `intersectWith`, `planeFrom*`/`axisFrom*` geometry helpers; all with ShapeHistory | Volumes analytic within 1e-6; fillet of box edge → expected volume & 1 new face named via history; chamfer likewise; through-hole removes exact volume; history maps every output face; deterministic over 20 runs; failing fillet (radius too big) → `KernelFailure` with message, never crash |
| **A5 Cleanups & export** | `src/io/document_io.cpp` (fsync), `src/naming/resolve.hpp` (comment), `src/core/job.hpp` (R5), `src/recompute/timeline.hpp` (includes), `src/core/undo.hpp` (noexcept), `src/kernel/exchange.*` (3MF export, additive), `apps/ironwood-cli/*`, `tests/test_cli.cpp` (new) | F3 fsync before rename; F13 items; **3MF export** (lib3mf is NOT installed → write the 3MF zip/XML ourselves; minimal core spec, mm units, per-body objects + colours); F8: CLI ctest with generated fixture, GPU name in bench via Vulkan, curved body in tess bench | 3MF opens in a validator-free check: unzip + XML schema-shaped assertions (vertex/triangle counts match tessellation); CLI test asserts printed counts; all previous tests green |
| **A6 Naming fuzzer (the P1a gate)** | `fuzz/` (new, own CMake target `ironwood-fuzz-naming` + ctest smoke run), `tests/test_fuzz_smoke.cpp` | Generator of random timelines from existing features (extrude new/join now; picks up revolve/fillet/chamfer as they land via the registry), random mutations (dimension edits, suppress/unsuppress, insert/delete feature, reorder where legal), and a resolver check: every ElementRef that resolved before a mutation either resolves to a geometrically consistent element after, or reports NotFound/Ambiguous **with a warning** — never a silent wrong pick (detect via geometric signature comparison) | Smoke: 200 scenarios in < 60 s in ctest; nightly mode N=5000; report with minimal reproducer on failure; seeds reproducible |

## Wave B — features & UI (after A1–A4)
| Task | Builds |
|---|---|
| **B1 Features** (may split B1a/B1b) | `src/features/`: SketchFeature (holds Sketch + solved state; outputs profiles + named sketch curves), Extrude from sketch profiles (one side / two sides / symmetric; distance / to object [P1b]; taper; operations new body / join / cut / intersect / new component[P1b]), Revolve, Fillet (const), Chamfer, Hole (cosmetic thread attribute), Move/Copy body, construction Plane (offset, angle, midplane, 3 points, tangent) / Axis / Point features; all expression-driven; migrate ExtrudeFeature; registry + io round-trip for each |
| **B2 Timeline UI** | Bottom timeline dock: feature icons in order, rollback marker drag (live re-evaluate, cache hits), double-click → edit dialog, right-click suppress/delete/rename, error/warning badges (red/yellow), hover highlights produced geometry |
| **B3a Sketch interaction** | Create sketch on plane/planar face, look-at animation, sketch palette, tools: line (+tangent-arc drag), 2-pt/centre rectangle, centre circle, 3-pt arc, point, construction toggle (X), trim, offset; inference/snapping glyphs (endpoint, midpoint, centre, on-curve, H/V, perpendicular, tangent); auto-constraints while drawing; Finish Sketch |
| **B3b Constraints & dimensions UI** | Constraint toolbar + glyphs on canvas; Dimension tool (D) with inline expression editor; fully-constrained colouring (blue under / black full) [VERIFY colours]; conflict dialog from A3 diagnostics; drag-solve at 60 Hz (16 ms budget) |
| **B4 Command framework & shell** | CommandRegistry, Fusion-style tabbed toolbar (SOLID / SURFACE… tabs, only P1a commands active), right-docked command dialogs with live preview (LatestWins jobs), distance-arrow & angle manipulators, S-box command palette, marking menu (right-click radial), keyboard shortcuts, Undo/Redo (UndoStack), Parameters dialog (A1 table: add/edit/favourites) |
| **B5 Python bindings** | `api/` pybind11 module `ironwood`: open/import/export, timeline add/edit/evaluate, parameters, sketches (entities + constraints), headless only; pytest suite |
| **B6 Inspect** | Measure (point/edge/face distance, angle, area, length, position, between bodies), Properties (volume, area, mass via density, centre of mass), section analysis (single plane, cap colour) |

## Wave C — integration & gate
C1 end-to-end scenario tests (scripted via B5 Python: model a bracket with sketch → extrude → fillet → hole,
edit an early dimension, verify downstream fillet/hole survive) · C2 naming fuzzer gate at N=5000 green ·
C3 bench additions (sketch solve, fillet, hole, 100-feature real part) + baselines committed to
`bench/baselines/` · C4 flock review of P1a diff + triage · C5 user hands-on test on the real display.
