# planegcs (vendored)

2D geometric constraint solver from FreeCAD's Sketcher module, used by `iw_sketch_solver`
(`src/sketch/solver`).

- Origin: https://github.com/FreeCAD/FreeCAD
- Path: `src/Mod/Sketcher/App/planegcs/`
- Tag: `1.1.4` (latest stable FreeCAD release at vendoring time; `26.3rc1` is a release candidate and was not used)
- Commit: `4fd3bf320d9566a27e60069fc8387448aaa3a094`
- License: GNU Library General Public License, version 2 or later, per the file headers.
  `LICENSE-LGPL-2.1.txt` is the LGPL-2.1 text from the FreeCAD repository root (`LICENSE`).
  Compatible with Ironwood's GPL-3.0-or-later as a linked static library.
- Copyright headers are kept unchanged in every file.

Local modifications are listed in `PATCHES.md`. Sources are otherwise byte-identical to the tag.

`compat/` holds shims that replace FreeCAD-internal headers (`Base/Console.h`, `FCConfig.h`,
`planegcs_compat.h`). They are not part of upstream.

Build: `iw_planegcs` (in `src/sketch/solver/CMakeLists.txt`) compiles these sources with `-w` and
without Ironwood's warning flags. Eigen and Boost headers are included as system headers.
