# SPDX-License-Identifier: GPL-3.0-or-later
# kernel: thin OCCT facade. Links only OCCT + core. OCCT types never appear in public headers.

# iw_kernel_occt: INTERFACE carrier of the OCCT toolkit include directories and libraries for Ironwood modules that
# include the sanctioned accessor kernel/occt_access.hpp (tess, future features). Such modules link it PRIVATE
# instead of listing TK libraries themselves. Not for apps or UI.
add_library(iw_kernel_occt INTERFACE)
target_link_libraries(iw_kernel_occt INTERFACE
    TKernel TKMath TKBRep TKG2d TKG3d TKGeomBase TKGeomAlgo TKTopAlgo TKPrim TKBO TKBool
    TKShHealing TKFillet TKMesh TKDE TKDESTEP TKDESTL TKXSBase TKXCAF TKLCAF TKCAF TKCDF)

iw_add_module(kernel
    SOURCES shape.cpp ops.cpp exchange.cpp measure.cpp naming_support.cpp
    PUBLIC_LIBS iw_core
    PRIVATE_LIBS iw_kernel_occt)
