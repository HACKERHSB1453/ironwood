// SPDX-License-Identifier: LGPL-2.0-or-later
// Ironwood compatibility shim for the vendored FreeCAD planegcs sources (see ../PATCHES.md).
// Replaces FreeCAD's Sketcher/SketcherGlobal.h: FreeCAD uses SketcherExport for Windows DLL
// visibility; a static Linux build needs no export macro.
#ifndef IRONWOOD_PLANEGCS_COMPAT_H
#define IRONWOOD_PLANEGCS_COMPAT_H

#define SketcherExport

#endif  // IRONWOOD_PLANEGCS_COMPAT_H
