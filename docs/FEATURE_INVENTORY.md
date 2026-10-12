# Ironwood — Feature Inventory (Fusion parity spec)

> Working codename "Ironwood" — placeholder, rename freely. Never ship with Autodesk names, logos, icons.
> **Source of truth:** Autodesk's *public* documentation, release notes (through Sept 2026), and observed
> behaviour of the app. **Clean-room rule:** no Autodesk code, binaries, icons, or decompiled logic enters
> this repo. We clone *what it does*, never *how its code does it*.
>
> Status: v0.2 (revised after Sonnet/Haiku review, see REVIEW_LOG.md). Written from general product
> knowledge + Autodesk's 2026 monthly "what's new" posts (sources at the bottom). Items dated "(Mon 2026)" are
> taken from those posts' summaries; exact command names/options still need confirming in the live app.
> Items tagged **[VERIFY]** are uncertain and must be confirmed against a live Fusion install before they
> become requirements. **Next pass:** add frequency-of-use + acceptance criteria for every P1a/P1b item.

**Phase tags** (see ARCHITECTURE.md §9): `P0` foundations · `P1` = P1a/P1b (exact split is in ARCHITECTURE
§9, which wins on conflict) · `P2` advanced modelling · `P3` assemblies · `P4` drawings/sheet metal/PMI ·
`P5` CAM (5a 2D/3-axis, 5b advanced) · `P6` mesh/render/animation/form · `P7` simulation/generative ·
`P8` electronics/collab/API/f3d.
**Perf tags:** `⚡GPU` = GPU-accelerable win · `⚡MT` = multithreading win · `⚡INC` = incremental/caching win.

---

## 0. Application shell (all workspaces)

| ID | Feature | Phase | Notes |
|---|---|---|---|
| APP-01 | Workspace switcher (Design, Generative, Render, Animation, Simulation, Manufacture, Drawing, Electronics) | P0 | Toolbar swaps per workspace |
| APP-02 | Tabbed toolbar per workspace (e.g. Design: Solid / Surface / Mesh / Form / Sheet Metal / Plastic / Utilities / Manage) | P1 | |
| APP-03 | Browser tree (document settings, named views, origin, analysis, joint origins, joints, sketches, bodies, components, construction) with visibility toggles, isolate, activate | P1 | |
| APP-04 | Parametric timeline (bottom): feature icons, drag-to-reorder, rollback marker, suppress, edit feature, groups, capture position toggle, error/warning badges | P1 | ⚡INC |
| APP-05 | ViewCube (orthographic faces/edges/corners, home view, orbit arrows) | P0 | |
| APP-06 | Navigation bar: orbit / free orbit / pan / zoom / fit / look-at; display settings (visual style, environment, effects, object visibility, camera ortho/perspective, ground plane, grid) | P0 | |
| APP-07 | Mouse nav presets (Fusion, Alias, Inventor, SolidWorks, Tinkercad) + zoom direction/invert + orbit around cursor | P0 | |
| APP-08 | Marking menu (right-click radial + overflow list), context-aware | P1 | |
| APP-09 | "S" shortcut box: searchable command palette + pinned shortcuts | P1 | |
| APP-10 | Command dialogs (right-docked, OK/Cancel, live preview, inputs: selection, distance manipulator arrows, angle rings, dropdowns, expression fields) | P1 | Preview must be async ⚡MT |
| APP-11 | On-canvas manipulators (distance arrow, angle ring, point drag, triad move) with numeric heads-up display | P1 | |
| APP-12 | Selection: window/crossing/freeform/paint select, selection filters, selection priority (body/face/edge/component), select-through, selection sets | P1 | ⚡GPU ID-buffer picking |
| APP-13 | Undo/redo (unlimited, per document) | P0 | |
| APP-14 | Units (mm, cm, m, in, ft) per document + default; display precision | P0 | |
| APP-15 | Preferences (general, design, manufacture, graphics, API, network, units, preview features) | P1 | |
| APP-16 | Keyboard shortcuts — Fusion defaults + user-remappable (E extrude, Q press-pull, F fillet, L line, C circle, R rect, D dimension, X construction toggle, P project, J joint, M move, H hole, S shortcuts, I measure, T trim, O offset…) [VERIFY full list] | P1 | |
| APP-17 | Data panel (projects, folders, versions, thumbnails) — local-first in Ironwood | P1 | |
| APP-18 | Version history, milestones, restore/promote version | P2 | local git-like store |
| APP-19 | Home tab with recent docs, search by property (Jan 2026) | P2 | |
| APP-20 | Notifications / job status (background jobs: CAM, render, sim) | P2 | |
| APP-21 | Text commands console | P8 | |
| APP-22 | Assistant (natural-language → command, Mar/Apr 2026) | P8 | optional, local LLM hook |
| APP-23 | Multiple documents in tabs; document recovery / autosave | P1 | |
| APP-24 | Dark/light themes, UI scaling (HiDPI), Linux Wayland + X11 | P0 | |

## 1. Design workspace

### 1.1 Sketch
| ID | Feature | Phase | Notes |
|---|---|---|---|
| SK-01 | Create sketch on plane/planar face; sketch palette (options: look at, slice, profile/points/dimensions/constraints visibility, 3D sketch toggle, show projected geometry) | P1 | |
| SK-02 | Line (with tangent arc drag), 2-point/3-point/centre rectangle, centre/2-pt/3-pt/2-tangent/3-tangent circle | P1 | |
| SK-03 | Arcs: 3-point, centre-point, tangent | P1 | |
| SK-04 | Polygon (circumscribed, inscribed, edge), ellipse, slots (centre-to-centre, overall, centre point, 3-point arc, centre-point arc) | P1 | |
| SK-05 | Splines: fit-point, control-point; spline handles, curvature display | P1 | |
| SK-06 | Conic curve, point, text (with fonts, along path, fit), mirror, circular pattern, rectangular pattern | P1 | |
| SK-07 | Project / Include (geometry, intersect, project to surface, include 3D geometry, cut edges) | P1 | |
| SK-08 | Modify: fillet, chamfer (sketch), trim, extend, break, offset (with chain), scale, move/copy, sketch scale, change parameters | P1 | |
| SK-09 | Construction toggle, centreline | P1 | |
| SK-10 | Constraints: horizontal/vertical, coincident, tangent, equal, parallel, perpendicular, fix/unfix, midpoint, concentric, collinear, symmetry, curvature (G2), smooth | P1 | |
| SK-11 | Auto-constraints while drawing (inference) + inference glyphs | P1 | |
| SK-12 | Dimensions: linear, aligned, angular, radial/diameter, ellipse axis, driven (reference) dimensions, expression-driven | P1 | |
| SK-13 | Fully-constrained state colouring (blue/black) + "show constraints", over-constraint conflict dialog | P1 | |
| SK-14 | Profile detection (closed regions, nested), multi-profile selection | P1 | |
| SK-15 | 3D sketch (points/lines/splines in space) | P2 | |
| SK-16 | Import SVG / DXF into sketch; insert canvas (image); calibrate | P1 (DXF/SVG) P2 (canvas) | |
| SK-17 | Sketch doctor / fix open profiles, overlapping curves | P2 | |
| SK-18 | Isoparametric curves (Mar 2026), intersection curve, project to surface | P2 | |
| SK-19 | Constraint solver performance on 1000+ entity sketches | P1 | ⚡INC (partitioned solve) |

### 1.2 Solid — Create
| ID | Feature | Phase |
|---|---|---|
| SO-01 | Extrude (one side/two sides/symmetric; distance/to object/through all/to; taper; start offset; operations new body/join/cut/intersect/new component; thin extrude) | P1 |
| SO-02 | Revolve (full/angle/to; operations as above) | P1 |
| SO-03 | Sweep (single path, path+guide rail, path+guide surface; twist, taper, orientation) | P2 |
| SO-04 | Loft (rails, centreline, tangent/curvature/point end conditions, closed) | P2 |
| SO-05 | Rib, Web, Emboss (emboss/deboss onto curved faces) | P2 |
| SO-06 | Hole (simple/counterbore/countersink; thread; tapped; drill point; placement single/multi/at point/from sketch points), Hole recognition | P1 |
| SO-07 | Thread: cosmetic (P1a); modelled + standards library ISO/ANSI/NPT + hub-based threads library (Jul 2026) | P1a / P2 |
| SO-08 | Box, Cylinder, Sphere, Torus, Coil, Pipe | P1 |
| SO-09 | Pattern: rectangular, circular, on path, (bodies/faces/features/components), suppress instances, pattern-of-pattern | P1 ⚡MT |
| SO-10 | Mirror (features/bodies/components/faces) | P1 |
| SO-11 | Thicken (surface → solid) | P2 |
| SO-12 | Boundary fill (cells from intersecting bodies/surfaces) | P2 |
| SO-13 | Create Base Feature (non-parametric island in timeline) | P2 |
| SO-14 | Derive (link geometry/params from another design) | P3 |
| SO-15 | Create Form / Create Mesh / Create PCB entry points | P6/P8 |

### 1.3 Solid — Modify
| ID | Feature | Phase |
|---|---|---|
| MO-01 | Press Pull (context: offset face/fillet/extrude) | P1 |
| MO-02 | Fillet: constant, variable radius, chord length; G1/G2 (tangent/curvature continuous); rule fillet; full round fillet; setback corners; tangent chain | P1a (constant) P2 (variable, G2, rule, full round) |
| MO-03 | Chamfer: equal distance, two distances, distance+angle; corner type | P1 |
| MO-04 | Shell (inside/outside/both, per-face thickness) | P1 |
| MO-05 | Draft (fixed plane / parting line) | P2 |
| MO-06 | Scale (uniform/non-uniform) | P1 |
| MO-07 | Combine (join/cut/intersect, keep tools) | P1 ⚡MT |
| MO-08 | Offset Face, Replace Face, Split Face, Split Body, Silhouette Split | P2 |
| MO-09 | Move/Copy (free move, translate, rotate, point-to-point, point-to-position) for bodies/components/faces | P1 |
| MO-10 | Align | P1 |
| MO-11 | Delete (faces with heal) / Remove | P1 |
| MO-12 | Physical Material, Appearance (drag-drop library, per-face override), Manage Materials | P1 (appearance) P2 (physical) |
| MO-13 | Change Parameters dialog: user params, model params, expressions, units, favourites, comments, import/export CSV | P1 |
| MO-14 | Compute All | P1 |
| MO-15 | Configurations (table-driven variants: params, suppression, appearance; configured rows) | P2 |

### 1.4 Surface
| ID | Feature | Phase |
|---|---|---|
| SU-01 | Extrude/Revolve/Sweep/Loft as surfaces; Patch (with continuity); Ruled; Offset; Extend; Trim; Untrim; Stitch; Unstitch; Reverse normal | P2 |
| SU-02 | Fill/Patch (multi-edge, G0/G1/G2), Merge, Split; any newer surface "blend"-type tool [VERIFY] | P2 |
| SU-03 | Surface-to-solid via stitch; thicken | P2 |

### 1.5 Mesh workspace (Design → Mesh tab)
| ID | Feature | Phase |
|---|---|---|
| ME-01 | Insert mesh (STL/OBJ/3MF), tessellate body to mesh, display modes | P2 ⚡GPU |
| ME-02 | Repair (close holes, stitch, remove), remesh, reduce, smooth, plane cut, reverse normals, separate, combine, shell | P6 ⚡GPU ⚡MT |
| ME-03 | Convert mesh → BRep (faceted, prismatic, organic) | P6 |
| ME-04 | Mesh section sketch, generate face groups | P6 |
| ME-05 | Export STL/3MF/OBJ (with refinement settings), 3D print utility | P1 |

### 1.6 Form workspace (T-Spline-like freeform)
| ID | Feature | Phase |
|---|---|---|
| FO-01 | Primitives (box, plane, cylinder, sphere, torus, quadball, pipe, face), Extrude/Sweep/Loft/Revolve as Form | P6 |
| FO-02 | Edit Form (translate/rotate/scale on vertices/edges/faces; soft modification), Insert Edge, Subdivide, Insert/Delete point, Merge/Bridge/Fill hole, Weld/Unweld, Crease/Uncrease, Bevel, Slide Edge, Pull to target, Flatten, Straighten, Match, Interpolate, Thicken, Freeze | P6 |
| FO-03 | Symmetry (mirror/circular internal, clear), Box/Smooth display mode toggle | P6 |
| FO-04 | Utilities: Make uniform, Convert (quad mesh → form), Repair body, Enable better performance | P6 |
| FO-05 | Exit Form → BRep NURBS conversion | P6 (hard, see ARCH §6) |

### 1.7 Sheet Metal
| ID | Feature | Phase |
|---|---|---|
| SM-01 | Sheet metal rules (thickness, K-factor, bend radius, relief shapes, miter gaps), rule library | P4 |
| SM-02 | Flange (with height/position/bend options, multi-edge); contour flange, lofted flange, hem, tab, closed corner, bend relief options [VERIFY each] | P4 |
| SM-03 | Bend, Unfold/Refold, Flat Pattern (+ DXF export of flat), Convert to sheet metal | P4 |
| SM-04 | Join by bend (Sep 2026), sheet metal fillet & chamfer (Sep 2026), rip, corner relief | P4 |

### 1.8 Plastic (design rules)
| ID | Feature | Phase |
|---|---|---|
| PL-01 | Plastic rules, Boss, Snap fit, Lip/Groove, Rest, Web [VERIFY which live on Plastic vs Solid tab] | P4 |

### 1.9 Construct
| ID | Feature | Phase |
|---|---|---|
| CO-01 | Planes: offset, angle, tangent, midplane, through two edges, through three points, tangent to face at point, along path | P1 |
| CO-02 | Axes: through cylinder/cone/torus, perpendicular at point, two planes, two points, edge, normal to face at point | P1 |
| CO-03 | Points: vertex, two edges, three planes, circle/sphere centre, edge+plane, along path | P1 |
| CO-04 | Construction geometry enhancements (Jul 2026) [VERIFY] | P2 |

### 1.10 Inspect
| ID | Feature | Phase |
|---|---|---|
| IN-01 | Measure (distance, angle, area, position; precision; select-through), Interference, Curvature comb, Zebra, Draft analysis, Curvature map, Accessibility, Minimum radius, Isocurve analysis | P1 (measure) P2 (rest) ⚡GPU (zebra/curvature/draft are shader-based) |
| IN-02 | Section analysis (plane, flip, hatch, cap colour), multiple sections | P1 ⚡GPU |
| IN-03 | Center of mass, Properties (mass, volume, area, density, inertia) | P1 |
| IN-04 | Display component colours | P1 |

### 1.11 Insert
| ID | Feature | Phase |
|---|---|---|
| IS-01 | Insert into current design (other design as linked/external component), Insert derive | P3 |
| IS-02 | ~~Insert McMaster-Carr / TraceParts~~ — **dropped** (3rd-party content); users import vendor STEP files | — |
| IS-03 | Insert mesh, SVG, DXF, Canvas, Decal | P1/P2 |
| IS-04 | Insert manufacturer part / content center fasteners (standard bolt/nut/washer generator) | P3 |

### 1.12 Assemble (components & joints)
| ID | Feature | Phase |
|---|---|---|
| AS-01 | Components (create/activate, new component from bodies, external/linked components, break link, ground/unground, rigid groups) | P3 |
| AS-02 | Joint types: rigid, revolute, slider, cylindrical, pin-slot, planar, ball; joint origins (simple/between two faces), snap points | P3 |
| AS-03 | As-built joint, joint limits, motion links, drive joints, animate joint/model, motion study | P3 |
| AS-04 | Contact sets / enable contact | P3 |
| AS-05 | Tangent relationship | P3 |
| AS-06 | BOM (live update Mar 2026), component properties (part number, description, material) | P3 |
| AS-07 | Large assembly performance (thousands of components) | P3 ⚡GPU instancing ⚡MT |

### 1.13 Manage / Utilities / Tools
| ID | Feature | Phase |
|---|---|---|
| UT-01 | Add-ins / Scripts manager (Python), API docs | P8 (P2 internal Python console) |
| UT-02 | 3D Print (export with mesh refinement, send to slicer) | P1 |
| UT-03 | Attributes, electronic part linking | P8 |
| UT-04 | Fusion Manage / PLM, cost roll-up | out of scope (note) |

## 2. Generative Design workspace (P7)
GD-01 Define study (preserve geometry, obstacle geometry, starting shape, symmetry) · GD-02 Loads & constraints
(structural, modal) · GD-03 Objectives (min mass / max stiffness), safety factor · GD-04 Manufacturing methods
(unrestricted, additive with overhang, milling 2.5/3/5-axis, 2-axis cutting, die casting) · GD-05 Materials ·
GD-06 Pre-check, generate (local GPU ⚡GPU — Fusion uses cloud credits), explore outcomes, compare, export to design.

## 3. Render workspace (P6)
RE-01 Scene settings (environment library, HDRI, background, ground plane/reflections, camera, depth of field,
aspect), Scene Lights (Sep 2026) · RE-02 Appearance editor (physically based, texture mapping: automatic/planar/
box/spherical/cylindrical) · RE-03 Decals · RE-04 In-canvas render (progressive) ⚡GPU · RE-05 Final render
(local GPU — Fusion queues cloud), render gallery, turntable · RE-06 Render quality presets.

## 4. Animation workspace (P6)
AN-01 Storyboards · AN-02 Transform components (keyframes), auto-explode (one level / all levels), manual explode,
show/hide, callouts, view (camera) actions · AN-03 Trail lines · AN-04 Publish video ⚡GPU (NVENC).

## 5. Simulation workspace (P7)
SI-01 Study types: static stress, modal frequencies, thermal, thermal stress, structural buckling, nonlinear static,
event simulation (dynamic), quasi-static, shape optimisation, electronics cooling, injection moulding [VERIFY list]
· SI-02 Simplify environment (remove features, replace with primitives) · SI-03 Materials, loads (force, pressure,
moment, bearing, remote, gravity, bolt preload), constraints (fixed, pin, frictionless, prescribed displacement)
· SI-04 Contacts (bonded, separation, sliding, rough, offset bonded), automatic contacts · SI-05 Mesh settings
(size, local refinement, adaptive) ⚡MT · SI-06 Solve (local GPU/CPU — Fusion largely cloud) ⚡GPU ⚡MT ·
SI-07 Results (stress, displacement, safety factor, strain, reaction force; probes, animation, compare,
modernised results Mar 2026) ⚡GPU · SI-08 Study report export.

## 6. Manufacture workspace (P5) — biggest perf opportunity
| ID | Feature | Notes |
|---|---|---|
| CAM-01 | Setup (machine, WCS, stock: box/cylinder/tube/from solid/relative, model selection, fixtures) | |
| CAM-02 | Machine library & kinematics (3/4/5-axis, mill-turn), Machine Builder, machine simulation | |
| CAM-03 | Tool library (mills, drills, taps, turning inserts, holders), feeds & speeds presets | |
| CAM-04 | 2D: Face, 2D Adaptive, 2D Pocket, 2D Contour, Slot, Trace, Thread milling, Bore, Circular, Engrave, Chamfer | ⚡MT |
| CAM-05 | 3D: Adaptive clearing, Pocket clearing, Steep & Shallow, Scallop, Contour, Ramp, Horizontal, Parallel, Pencil, Spiral, Radial, Morphed spiral, Project, Flow, Blend, Morph, Rest machining | ⚡GPU (heightfield/dexel) ⚡MT |
| CAM-06 | Drilling (cycles: chip break, deep peck, tapping, boring, back-bore…), hole recognition | |
| CAM-07 | Multi-axis: swarf, multi-axis contour, rotary, tool orientation, 3+2 | |
| CAM-08 | Turning: face, profile roughing/finishing, groove, thread, part, chamfer, single groove | |
| CAM-09 | ~~Additive (FFF/SLA/DLP/MPBF/DED)~~ — **out of scope**; export to a slicer instead | — |
| CAM-10 | Probing, inspection, surface inspection, WCS probe | |
| CAM-11 | Fabrication: waterjet, laser, plasma cutting; nesting (2D) | ⚡MT |
| CAM-12 | Simulation with stock (material removal), collision/gouge check, machine sim | ⚡GPU (voxel/dexel) |
| CAM-13 | Post processing: post library, post properties, NC programs, G-code output, setup sheets | JS post engine (see ARCH §7) |
| CAM-14 | Templates, toolpath patterns, generate in background ("generate" queue) | ⚡MT (Fusion's actual concurrency: measure [VERIFY]) |

## 7. Drawing workspace (P4) — Fusion is notoriously slow here
DR-01 From design/animation; sheet sizes & standards (ISO/ASME), title blocks, borders, templates · DR-02 Views:
base, projected, section (full/half/offset/aligned), detail, break, break-out [VERIFY], exploded (from animation)
⚡MT ⚡GPU (hidden-line removal) · DR-03 Dimensions (linear, aligned, angular, radial, ordinate, baseline, chain,
arc length), auto-dimensions · DR-04 Annotations: text, leader, balloon, centre marks/lines, hole/thread notes,
surface texture, feature control frames (GD&T), datum identifiers, weld symbols · DR-05 Tables: parts list, hole
table, revision table, general table · DR-06 Sheets management, export PDF/DWG/DXF · DR-07 Associativity (update on
model change) ⚡INC.

## 8. PMI / Model-based definition (P4)
PMI-01 3D dimensions, GD&T feature control frames, datums, notes on the model; semantic PMI export (STEP AP242)
(Sep 2026 PMI updates [VERIFY]).

## 9. Electronics workspace (P8 — integrate, don't rebuild)
EL-01 Schematic, PCB 2D/3D, library editor, autorouter, DRC/ERC, ECAD-MCAD sync, Altium import (Jul 2026).
**Plan:** bridge to KiCad (open source, mature) via KiCad files + STEP/IDF/IDX sync rather than re-implementing EDA.

## 10. Data, collaboration, files (P1 → P8)
| ID | Feature | Phase |
|---|---|---|
| DA-01 | Native file format (open, documented, diffable) | P0 |
| DA-02 | Import/export: STEP (AP203/214/242), IGES, STL, 3MF, OBJ, DXF, SVG (P0–P1b); DWG via external LibreDWG converter, FBX/USD (P2+). Parasolid x_t / ACIS sat **only from publicly published specs**, else out. SolidWorks/Inventor/CATIA/JT native import **out of scope** | P0–P2+ |
| DA-03 | `.f3d` / `.f3z` import (Fusion archive) — interoperability research, see ARCH §8 | P8 |
| DA-04 | Linked designs, references, "update out-of-date" | P3 |
| DA-05 | Collaboration: sharing, comments, markup — local-first + optional self-hosted sync | P8 |
| DA-06 | Cloud credits features (render, sim, generative) → **all local, GPU** | P6/P7 |

## 11. API (P8)
API-01 Python API for scripts/add-ins (document, features, sketches, CAM, UI commands, events) ·
API-02 Optional compatibility shim mirroring the *public* Fusion API surface so existing user scripts port
easily [legal review — interfaces/functionality; we write our own implementation] · API-03 Command-line /
headless mode (batch export, regenerate, CAM post) — something Fusion lacks.

---

## 12. Cross-cutting features added in v0.2 (from review)
| ID | Feature | Phase |
|---|---|---|
| X-01 | Direct-modelling mode ("do not capture design history") + switching to/from history | P1b (basic) P2 (polish) |
| X-02 | Snapping & inference in 3D and sketch (endpoint, midpoint, centre, tangent, on-curve, intersection) | P1a |
| X-03 | Edit-in-place for components; associative project/include across components | P3 |
| X-04 | Pattern compute options (identical / adjust / optimised) [VERIFY names] | P1b |
| X-05 | Expression language: unit-aware (`2 in + 5 mm`), functions, param-to-param links, favourites | P1a |
| X-06 | Named views; display settings saved with document | P1b |
| X-07 | Section analysis stored in browser/timeline | P1b |
| X-08 | Thickness analysis (Inspect) | P2 |
| X-09 | Version compare (visual diff) | P2 |
| X-10 | Design templates | P2 |
| X-11 | Lightweight / display-only component loading for large assemblies [VERIFY Fusion term] | P3 |
| X-12 | Lattice / volumetric lattice body [VERIFY availability & workspace] | P6+ |
| X-13 | Internationalisation (Qt `tr()` from P0; Fusion ships many languages) + accessibility (keyboard-only, screen-reader labels) | P0 infra, ongoing |
| X-14 | Headless CLI batch mode (regenerate, export, post) — an Ironwood differentiator | P0 runner, P1a export |

## 13. Explicitly out of scope
Additive/slicing CAM (export to a slicer) · PLM / Fusion Manage / cost roll-ups · McMaster-Carr/TraceParts content
· native SolidWorks/Inventor/CATIA/JT import · Autodesk cloud-credit services (replaced by local compute) ·
Autodesk Assistant parity (maybe later as an optional local-LLM plugin).

## 14. Known Fusion slow spots → candidates for measurement
**No speed ratios are claimed until Fusion baselines are measured** (ARCHITECTURE §10). These are the
suspected slow spots and the technique we expect to help:
| Suspected slow in Fusion | Ironwood technique | Notes |
|---|---|---|
| Edits late in a long timeline; rollback/undo | prefix-reuse cache | Editing feature #1 ≈ full recompute in any history CAD |
| Patterns with hundreds of instances | instanced display, tessellate once, batched booleans | Join/cut patterns still need real booleans |
| Large assemblies (1000s of components) | instancing, lightweight loading, parallel tessellation | Unique parts: tessellation is the bottleneck |
| Complex fillets/shells | async + preview + cancel (kernel-bound) | Hides latency, doesn't speed the op |
| Drawing view generation | parallel exact HLR per view, incremental update | Scales with number of views |
| CAM toolpaths | parallel op queue; GPU heightfield for 3-axis roughing & sim | Linking/adaptive logic stays CPU |
| Cloud-queued sim/render/generative | local CPU/GPU compute | Win = no queue/credits |
| UI freezes during compute | everything async | Certain win |

---
### Sources (2026 release information)
- [Fusion 2026 release notes](https://help.autodesk.com/cloudhelp/ENU/Fusion-ReleaseNotes/files/FIC-REL-NOTES-INC.htm)
- [January 2026 update](https://www.autodesk.com/products/fusion-360/blog/january-2026-product-update-whats-new/)
- [March 2026 update](https://www.autodesk.com/products/fusion-360/blog/march-2026-product-update-whats-new/)
- [April 2026 update](https://www.autodesk.com/products/fusion-360/blog/april-2026-product-update-whats-new/)
- [July 2026 update](https://www.autodesk.com/products/fusion-360/blog/july-2026-major-product-update-whats-new/)
- [Fusion workspaces overview](https://help.autodesk.com/cloudhelp/ENU/Fusion-GetStarted/files/GS-WORKSPACES.htm)
- September 2026 items (Join by bend, sheet-metal fillet/chamfer, Scene Lights, PMI) come from a search summary of
  the September post; the page itself blocked automated access — confirm in-app.
