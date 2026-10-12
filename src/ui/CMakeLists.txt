# SPDX-License-Identifier: GPL-3.0-or-later
set(CMAKE_AUTOMOC ON)
iw_add_module(ui
    SOURCES main_window.cpp import_job.cpp
            command.cpp command_actions.cpp default_commands.cpp doc_state.cpp fuzzy.cpp
            command_dialog.cpp manipulators.cpp sbox.cpp marking_menu.cpp ribbon.cpp parameters_dialog.cpp
            sketch/session.cpp sketch/view_math.cpp sketch/sketch_overlay.cpp sketch/sketch_palette.cpp
            sketch/plane_chooser.cpp sketch/sketch_mode.cpp
    PUBLIC_LIBS iw_core iw_kernel iw_naming iw_tess iw_recompute iw_io iw_render iw_sketch iw_sketch_solver
                Qt6::Widgets)
