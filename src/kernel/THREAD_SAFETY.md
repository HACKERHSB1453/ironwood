<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# OCCT thread-safety matrix (P0 deliverable, ARCHITECTURE section 4)

Status legend: `UNVERIFIED` (default) -> `SAFE` | `SAFE-WITH-COPY` (deep-copy inputs first) | `MAIN-THREAD-ONLY`.
Each row must be backed by a stress test (N threads, repeated runs, identical canonical result) before its
status changes. Facade functions in `ops.hpp` / `exchange.hpp` inherit the weakest status of the APIs they use.

| OCCT API / class | Used by | Status | Evidence / notes |
|---|---|---|---|
| `BRepPrimAPI_MakeBox` / `MakeCylinder` | makeBox, makeCylinder | UNVERIFIED | |
| `BRepPrimAPI_MakePrism` | extrudeProfile | UNVERIFIED | |
| `BRepBuilderAPI_MakeFace/MakeWire/MakeEdge` | profile building | UNVERIFIED | |
| `BRepAlgoAPI_Fuse` / `Cut` / `Common` (+ `SetRunParallel`) | fuse, cut, common | UNVERIFIED | Implemented; run SERIAL (`SetRunParallel(false)`) + `SetNonDestructive(true)`; 100x repeat determinism test passes (single thread). No multi-thread stress test yet. Parallel mode not enabled. |
| `BRepTools_History` / `BRepAlgoAPI_BuilderAlgo::History()` | ShapeHistory extraction | UNVERIFIED | Implemented via `BRepBuilderAPI_MakeShape::Generated/Modified/IsDeleted` (BOP + MakePrism). Single-thread tests only. |
| `TopExp::MapShapes`, `TopExp_Explorer` | sub-shape ordering | UNVERIFIED | Used for sub-shape index tables, built once at wrap time and immutable afterwards. `Shape::subShapeCount/subShapeKey` only read those tables (+ `BRepGProp` on the shared shape); expected safe for concurrent const use, NOT stress-tested. |
| `BRepMesh_IncrementalMesh` (parallel flag) | tess, exportStl | UNVERIFIED | exportStl meshes a private `BRepBuilderAPI_Copy` (copyGeom=false, copyMesh=false) so the shared TShape is never mutated (SAFE-WITH-COPY pattern). Parallel meshing on; not stress-tested. |
| `BRep_Tool::Triangulation` | tess | UNVERIFIED | |
| `BRepBndLib::Add`, `Bnd_Box` | boundingBox | UNVERIFIED | `Shape::boundingBox` uses `AddOptimal` (no triangulation); read-only on shared shape; not stress-tested. |
| `BRepGProp::VolumeProperties/SurfaceProperties`, `GProp_GProps` | massProperties | UNVERIFIED | massProperties and subShapeKey; read-only; not stress-tested. |
| `STEPCAFControl_Reader` / `STEPCAFControl_Writer` | importStep, exportStep | UNVERIFIED | Implemented; every call re-sets `Interface_Static` units (xstep.cascade.unit / write.step.unit = MM); `XCAFApp_Application` singleton creates a document per call. Serialised by an internal mutex (`exchangeMutex()` in exchange.cpp, covers importStep/exportStep); callers may run on any thread, one at a time. |
| `XCAFApp_Application`, `TDocStd_Document`, `XCAFDoc_ShapeTool/ColorTool` | importStep | UNVERIFIED | |
| `RWStl::ReadFile`, `StlAPI_Writer` | importStl, exportStl | Serialised by the same internal mutex (exchange.cpp) | Implemented (import via RWStl, export via StlAPI_Writer on a copy). |
| `BRepBuilderAPI_Sewing` | importStl | UNVERIFIED | importStl sews per-facet triangle faces (tol 1e-6); closed result -> solid. |
| `BRepCheck_Analyzer`, `ShapeFix_*` | healing | UNVERIFIED | |
| `Precision::*`, `Message::DefaultMessenger()`, `Interface_Static` | global | UNVERIFIED | Global state: isolate / configure once at startup |
| `BRepTools::Clean` | tess cache | UNVERIFIED | Mutates shared TShape |
| `BRep_Builder` (editing shared shapes) | any | UNVERIFIED | Never edit a Shape visible to another thread |

## Notes added by the kernel implementation task
* `Shape` itself is immutable; its sub-shape index tables are built in `wrapShape()` and never mutated, so all
  `Shape` const members are expected to be safe for concurrent readers (UNVERIFIED: no stress test yet).
* Sub-shape ordering: `TopExp::MapShapes` (explicit DFS via `TopExp_Explorer`, first occurrence of TShape+Location,
  insertion order); documented at the top of `internal/shape_impl.hpp`.
* Booleans run serial and non-destructive; parallel BOP is intentionally off until a multi-thread determinism test exists.
