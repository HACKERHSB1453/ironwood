# SPDX-License-Identifier: GPL-3.0-or-later
set(CMAKE_AUTOMOC ON)
iw_add_module(ui
    SOURCES main_window.cpp import_job.cpp
    PUBLIC_LIBS iw_core iw_kernel iw_naming iw_tess iw_recompute iw_io iw_render Qt6::Widgets)
