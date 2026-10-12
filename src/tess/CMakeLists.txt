# SPDX-License-Identifier: GPL-3.0-or-later
# tess: tessellation of kernel::Shape. OCCT meshing is reached through the kernel's sanctioned accessor
# (kernel/occt_access.hpp); the OCCT include dirs and toolkit libs come from the iw_kernel_occt INTERFACE target.
iw_add_module(tess
    SOURCES tessellate.cpp
    PUBLIC_LIBS iw_core iw_kernel
    PRIVATE_LIBS iw_kernel_occt)
