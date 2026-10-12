# Ironwood — Architecture Plan v0.2

v0.2 = v0.1 revised after two independent reviews (Sonnet 5.5, Haiku 5.5). Changes are summarised in
REVIEW_LOG.md. Goal: an open-source, Linux-first, Fusion-parity CAD/CAM/CAE app (FEATURE_INVENTORY.md) that is
**faster than Fusion where it matters** through async design, incremental recompute, multithreading and GPU use.
Dev machine: Arch Linux, RTX 5080 (16 GB). Must also run on AMD/Intel GPUs.

## 1. Guiding principles
1. **Clean-room.** Behaviour is cloned from public docs + observation of the running app. No Autodesk code,
   binaries, assets, or decompiled logic. Fusion's own executables are **never** disassembled (§8).
2. **Don't rebuild solved problems.** Mature open-source cores for kernel, solver, meshing, rendering, EDA.
   Our value = Fusion's workflow + a fast architecture around them.
3. **The UI thread never blocks.** Every geometry op is an async, cancellable job.
4. **Recompute the minimum, reuse everything.** The main speed win is prefix reuse + caching, not parallelism.
5. **Measure before optimising.** No speed claim without a measured Fusion baseline on a defined model (§10).
   Speculative GPU work waits for profiler evidence.
6. **Headless-first core.** All modelling works without a GUI → testable, scriptable, CLI batch mode.

## 2. Technology stack
| Layer | Choice | Link mode / license | Notes |
|---|---|---|---|
| Language | **C++23** (std::expected etc.; GCC 16 / Clang 23 verified) core; Python 3 bindings (pybind11) from P1a | — | Supersedes the earlier C++20 choice (2026-10-12) |
| Build | CMake + Ninja + ccache; Arch system packages first, vcpkg for gaps | — | |
| Geometry kernel | **OCCT 7.9.x** (8.x when released & verified) behind a *thin* `kernel/` facade | LGPL-2.1 + OCCT exception, dynamic | Facade = isolation + instrumentation, **not** a promise of a swappable kernel |
| Fast mesh booleans | **Manifold** | Apache-2.0 | Build against oneTBB (one threading runtime) |
| Sketch solver | **planegcs** (extracted from FreeCAD) | LGPL-2.1+ | Budget extra work for redundancy/conflict diagnostics. SolveSpace solver **rejected** (GPL-3) |
| Threading | **oneTBB** | Apache-2.0 | |
| Viewport | **Qt 6 RHI (Vulkan backend)** + thin `render/` layer we own | Qt LGPL-3, dynamic | v0.1's hand-rolled Vulkan renderer dropped for P0; raw Vulkan only where profiling proves RHI is the bottleneck. No mesh shaders/meshlets until measured need |
| UI | **Qt 6 Widgets**, custom style (Fusion-like layout), i18n via `tr()` from day one | LGPL-3, dynamic | Avoid GPL-only Qt modules (Charts, etc.) |
| Profiling | **Tracy** (CPU zones + GPU timestamps) from P0 | BSD-3 | |
| Subdivision (Form) | OpenSubdiv (CPU/compute eval; verify GPU backend story) | Apache-2.0 (modified) | P6 |
| Photoreal render | Cycles standalone | Apache-2.0 | OptiX/CUDA runtimes are **user-installed**, not bundled; budget for tracking Blender upstream |
| FE meshing | **Netgen** in-process | LGPL-2.1 | Gmsh only as external process (license exception not relied on) |
| FEA solve | CPU sparse direct first (CHOLMOD / MUMPS); CalculiX as external-process reference | CalculiX GPL-2+, out-of-process | GPU solvers only for >~1M DOF after measurement; AMGX (Apache-2.0) ok; **cuDSS proprietary → optional, user-installed** |
| CAM geometry | Clipper2 (**Boost Software License 1.0**), OpenCAMLib (LGPL-2.1), own heightfield/dexel stock | | |
| Post processors | QuickJS (MIT) running **our own documented post API** | | Decided early because it shapes CAM output design (§7) |
| File I/O | OCCT STEP/IGES (XDE: names, colours, assemblies), own STL/3MF/OBJ/SVG/DXF | | DWG via **LibreDWG as a separate converter process only** (GPL-3; in-process `dlopen` would make the app GPL) |
| Electronics | KiCad bridge (files + STEP + IDX) | | Not rebuilding EDA |
| Video export | NVENC, VAAPI, and software (SVT-AV1/x264 via ffmpeg) as **equal first-class** paths | | |
| Tests | GoogleTest, pytest, golden-geometry corpus, naming-mutation fuzzer, perf CI | | |

### License — DECIDED 2026-10-12: **GPL-3.0-or-later** (user decision; replaced an earlier AGPL choice)
Free for everyone to use, modify and share; anyone distributing it or a modified version must publish the
source under GPL-3 → no closed forks. Add-ons may be sold (Blender-market model); in-process add-ons must be
GPL-compatible. Known gap accepted: a modified version run purely as an online service need not publish
source (AGPL would close that). No CLA for now — add one before the first outside contribution if license
flexibility is wanted later. Deps compatible as linked in the table above.
Every dependency is recorded in `THIRD_PARTY.md` with its link mode (dynamic / static / out-of-process).

## 3. Module layout
```
ironwood/
  core/       document, object model, units, expression language, undo (command pattern), events, jobs
  kernel/     thin OCCT facade: shapes, booleans, fillets, healing, tolerances, canonical shape hash, history extraction
  naming/     persistent topological naming (P0 data model, P1a+ implementation)  ← highest-risk module
  sketch/     2D/3D sketch entities, planegcs adapter, inference/snapping, profile detection, sketch-level naming
  features/   one class per feature: compute(inputs, refs, params) -> FeatureResult
  recompute/  timeline evaluator, dependency graph, dirty propagation, TBB scheduler, result cache
  assembly/   components, occurrences, joints, kinematics, contact, BOM
  tess/       parallel tessellation, crack-free LOD, mesh cache
  render/     RHI renderer, picking, analysis shaders, section caps, line/edge rendering
  ui/         Qt shell: workspaces, browser, timeline, dialogs, manipulators, marking menu, S-box, ViewCube
  io/         STEP/IGES/STL/3MF/OBJ/DXF/SVG/native; dwg via external converter
  mesh/ form/ sheetmetal/ drawing/ pmi/ cam/ sim/ generative/ renderpt/ anim/ ecad-bridge/
  api/        pybind11 bindings, add-in loader, CLI (`ironwood --headless`)
  tests/ bench/ corpus/ fuzz/
```
Dependency direction: `ui → everything`; `features → kernel, naming, sketch`; `kernel → OCCT only`.
Workspaces never depend on each other.

## 4. Document model & recompute engine
**Timeline semantics come first (fixes v0.1 flaw).** Fusion features are *order-dependent*: a feature sees the
model state at its timeline position, not only its explicit references. Two features that never reference each
other can still interact (both modify "the current body"). Therefore:
- Each feature declares **explicit refs** (sketch, edges, faces) **and implicit state deps** (the bodies it
  reads/writes, the active component, parameter values). The evaluator derives the dependency graph from both.
- Parallelism exists only between features whose read/write body-sets and components are disjoint
  (independent bodies, separate components, pattern instance tessellation). Most real histories are a linear
  chain on one body → **expect little DAG parallelism; the win is prefix reuse.**
- **Prefix reuse / rollback cache:** results for every timeline position are cached; editing feature N reuses
  1..N-1 untouched; rollback-marker moves and undo/redo are cache hits.
- **Direct-modelling mode** ("do not capture design history") and **base features** are first-class: the
  evaluator handles a document with no timeline, and islands inside one. **Configurations** select a different
  parameter/suppression vector → separate cache namespace per configuration.

**Cache keys.** OCCT `TopoDS_Shape` hashes are pointer-based, so content addressing needs our own
**canonical shape hash** (sorted topology + quantised geometry + tolerance). Initially use **version stamps**
(feature-id + input-stamp chain + param hash) — cheap and exact; add canonical hashes only where needed
(persisted cache, dedup). Persisted caches are optional, versioned by OCCT version, and stripped by
"save compact".

**Determinism.** Same inputs must give the same topology and face order, otherwise caching, naming and golden
tests break. Rule: any OCCT parallel mode feeding *referenced* results must pass a determinism check
(N repeated runs → identical canonical hash) in CI; otherwise it runs serial.

**Thread safety.** "Pure function" is a goal, not a fact for OCCT. P0 delivers an **OCCT thread-safety matrix**
(per API used: safe / safe-with-copy / main-thread-only). Global state (`Precision`, message printers,
`BRepTools::Clean`, shared `BRep_Builder` edits) is isolated; inputs are deep-copied when forking jobs that
mutate.

**Expression language.** Unit-aware expressions (`2 in + 5 mm`, functions, params referencing params,
favourites, comments) are a core module from P1a — every dimension field uses it.

## 5. Topological naming — funded research track, starts in P0
- **P0:** stable-ID data model in the document format: every face/edge/vertex reference is stored as a
  *name*, never an index. Nothing ships that can't be migrated later.
- **Names** derive from generation history (feature id + op + source names) using OCCT history
  (`BRepTools_History`, Generated/Modified/Deleted). **Per-feature history coverage table**: booleans are
  good; fillet, shell, draft, offset, sweep have partial/inconsistent history → per-feature naming adapters.
- **One-to-many / many-to-one policy** for split and merged faces, with a user-visible disambiguation rule.
- **Sketch-level naming** (curves and profiles across trims, constraint edits) — as hard as BRep naming.
- **Geometric fallback** (surface type, normal, centroid, adjacency signature) is **conservative**: any
  non-history rebinding raises a visible warning (Fusion-style yellow/red timeline icons) and is logged.
- Study FreeCAD 1.x's element map (LGPL). If we take code, it's a deliberate LGPL derivative and attributed;
  clean-room applies to Autodesk only.
- **Gate:** an automated mutation fuzzer (random dim edits, feature insert/delete/suppress, reorders) over a
  corpus of models; **hundreds+ scenarios** with diffing of resolved references must pass before P1a ships.

## 6. Performance techniques (honest version)
| Area | Technique | Expectation |
|---|---|---|
| Responsiveness | async jobs, cancellation, latest-wins speculative previews | Big, certain win |
| Edits late in timeline | prefix reuse cache | Big win; editing feature #1 ≈ full recompute (same as any history CAD) |
| Repeated geometry | instanced rendering of patterns / repeated components; tessellate once | Big win for transforms; *join/cut* patterns still need real booleans |
| Tessellation | per-face parallel (`BRepMesh_IncrementalMesh` parallel), crack-free LOD | Good win |
| Booleans | OCCT parallel boolean stages (intersection) where deterministic; batch pattern booleans | Moderate |
| Fillets / shells / drafts | **serial in OCCT** — hidden behind async + preview, parallel only across independent bodies | Latency hiding, not speedup |
| Picking | GPU ID buffer + screen-space radius search for edges/vertices + CPU BVH fallback, async readback ring | Low-latency, ~1 frame |
| Analysis (zebra, curvature, draft) | shaders; CPU reference implementation for equivalence tests | Big win |
| Drawings | parallel exact HLR **per view**; incremental update. (GPU depth pre-pass only after profiling) | Win scales with view count |
| CAM | parallel op queue; GPU heightfield/dexel for 3-axis roughing & stock sim | Big for raster-bound parts; linking/adaptive planning stays CPU |
| FEA | local CPU sparse direct (no cloud queue); GPU only for very large models | Win = no queue |
| Render / generative | local GPU (Cycles; SIMP topology opt) | Win = no queue/credits; manufacturable smoothing is the hard part |

## 7. Workspace notes
- **Sketch:** solve only dirty constraint clusters; latency budget 16 ms per drag frame on 1,000-entity sketch.
- **Assemblies:** own kinematic solver (reduced coordinates); contact via FCL (BSD). Lightweight/display-only
  component loading for large assemblies.
- **Drawings:** associative view graph; cached HLR per (model stamp, view params).
- **CAM:** *our own* post API (documented, JS). Compatibility with user-owned `.cps` posts is a separate,
  later investigation; no Autodesk post files shipped. CAM safety (gouge/collision checks, conservative defaults)
  precedes any real-machine test.
- **Form:** OpenSubdiv modelling; Exit-Form → B-spline conversion is research-grade (regular regions exact
  bicubic; extraordinary points approximated). Patent check before mimicking T-spline-specific behaviour.

## 8. Interoperability & REA's role (tightened)
- **Migration path #1:** STEP export from Fusion → Ironwood import (names, colours, assemblies). Ships in P0/P1a.
- **`.f3d` research (P8):** *file-level* analysis of **the user's own exported `.f3d`/`.f3z` files only** (zip
  structure, JSON/XML metadata, documented or observable container formats), for interoperability (EU Software
  Directive Art. 6). **Fusion's executables/libraries are not disassembled or decompiled.** Geometry blobs in
  ShapeManager formats may prove infeasible — accepted. Legal review before any of this is published.
- **Parasolid `.x_t` / ACIS `.sat`:** only if implementable from **publicly published format specifications**;
  otherwise out of scope (STEP covers the need).
- **Behaviour study:** observe a live Fusion (user's install) for UI behaviour and benchmark timings.
  Observations become notes and tests, never code.

## 9. Phased roadmap
| Phase | Scope | Exit criteria |
|---|---|---|
| **P0 Foundations** | repo/CI, license + THIRD_PARTY.md, kernel facade + thread-safety matrix, stable-ID naming data model, doc/undo, timeline evaluator skeleton proven on *extrude only*, RHI viewport (ViewCube, nav, picking), STEP/STL import, Tracy, test+bench harness, CLI headless runner | STEP assembly import → orbit smoothly (cold import time reported, not promised); determinism CI green |
| **P1a First usable modeller** | sketch + core constraints/dims + inference, extrude/revolve, constant fillet/chamfer, hole (cosmetic threads), move, construction planes/axes/points, parameters + expressions, timeline (edit, rollback, suppress), measure, STEP/STL/3MF export, Python bindings, minimal Fusion-style shell | Real parts modelled end-to-end; naming fuzzer gate passes; baseline benchmarks recorded |
| **P1b Daily driver** | shell, pattern, mirror, combine, press-pull, split body, appearance, section analysis, properties, full shortcut set, marking menu, S-box, DXF/SVG sketch import | User can stop using Fusion for typical part work |
| **P2 Advanced modelling** | timeline reorder, sweep/loft/rib/web/emboss/draft/replace/offset face, variable & G2 fillets, modelled threads + thread library, surface toolset, configurations, 3D sketch, analyses, direct-modelling mode polish, versions | Surface-heavy consumer-product model reproducible |
| **P3 Assemblies** | components, joints, motion, contact, BOM, linked designs, fastener generator, lightweight loading | 500-part assembly with joints & motion |
| **P4 Docs & fab** | drawings, sheet metal, plastic rules, PMI (STEP AP242) | Fab drawing + flat-pattern DXF of a sheet-metal part |
| **P5a CAM 2D/3-axis** | setups, tools, 2D ops, 3-axis ops, drilling, stock sim, collision/gouge checks, own posts | User-verified G-code on a real machine after sim |
| **P5b CAM advanced** | 3+2/multi-axis, turning, probing, fabrication (laser/waterjet/plasma, nesting) | per-op verified on machine |
| **P6a Mesh** | mesh repair/remesh/convert | STL from scanner repaired → solid |
| **P6b Render & animation** | Cycles render, exploded views, video export | Turntable render + exploded video of a P3 assembly |
| **P6c Form** | SubD modelling + BRep conversion (research) | Freeform part converted, filleted and exported |
| **P7 Sim & generative** | static/modal/thermal FEA, topology optimisation | NAFEMS benchmarks + analytic solutions within stated tolerance |
| **P8 Ecosystem** | Python add-ins + Fusion-API-shaped shim, KiCad bridge, .f3d research, collaboration | Ported user script runs; KiCad board round-trips |
Out of scope: additive/slicing CAM, PLM/Manage, McMaster/TraceParts content, commercial CAD-format import
(SolidWorks/Inventor/CATIA/JT), Autodesk Assistant parity (maybe later as local-LLM plugin).

## 10. Performance measurement protocol (replaces v0.1 ratio targets)
1. Define a **benchmark set**: ~10 models (simple part, 100-feature part, pattern-heavy part, surface part,
   200- & 2,000-component assemblies with stated triangle counts, sheet-metal part, CAM part, drawing with 6 views).
2. Record **Fusion baselines** on the user's machine (Proton or Windows): open, recompute after early/late edit,
   orbit fps, drawing generation, toolpath generation. Store in `bench/baselines/`.
3. Ironwood targets are **absolute numbers on those models**, checked in CI with regression budgets.
4. Budgets also cover: interaction latency (sketch drag ≤ 16 ms/frame, fillet preview first result ≤ 300 ms
   or progress shown), memory (RAM + 16 GB VRAM ceilings for the big assembly), file size, save/autosave time,
   cold start.
5. GPU-vs-CPU equivalence tests for every shader/compute path.

## 11. How it gets built
- Spec-first per module: design note + interface header + tests before implementation.
- Build path is the user's decision (aviary default vs Haiku-first subagents per routing preference). Either way:
  small, interface-bounded tasks; the core (kernel facade, recompute, naming) gets the strongest model and the
  closest supervision. The local 35B/40K-slot executor is unsuitable for core C++ — that is a schedule risk to
  plan around, not ignore.
- Every change: builds, unit tests, golden-geometry corpus, naming fuzzer (sampled), perf CI.

## 12. Top risks
1. Topological naming. 2. OCCT fillet/shell robustness vs ShapeManager. 3. Scope — strict phase gates; the
P1a/P1b wedge must be useful on its own. 4. UI feel — needs the user's eye. 5. CAM = physical safety.
6. Legal: clean-room discipline, trademarks, license consistency. 7. Contributor sustainability / bus factor.
8. OCCT determinism and thread safety.

## 13. Open decisions for the user
1. ~~License~~ → GPL-3.0-or-later. 2. Project name. 3. Build path. 4. Fusion available for baselines?
5. After P1b: default order or CAM earlier. 6. Public repo now or after P1a.
