# SPDX-License-Identifier: GPL-3.0-or-later
iw_add_module(naming
    SOURCES element_name.cpp naming_map.cpp resolve.cpp
    PUBLIC_LIBS iw_core iw_kernel)
