# SPDX-License-Identifier: GPL-3.0-or-later
set(CMAKE_AUTOMOC ON)
find_package(Qt6 6.5 REQUIRED COMPONENTS Svg)
# F25: original icon set (resources/icons/*.svg) compiled into the library. Rendered with QSvgRenderer (Qt6::Svg is
# linked), so icons never depend on a runtime imageformats plugin.
qt_add_resources(IW_UI_ICON_RES "${PROJECT_SOURCE_DIR}/resources/icons.qrc")
iw_add_module(ui
    SOURCES main_window.cpp import_job.cpp
            command.cpp command_actions.cpp default_commands.cpp fuzzy.cpp
            command_dialog.cpp manipulators.cpp sbox.cpp marking_menu.cpp ribbon.cpp parameters_dialog.cpp
            sketch/session.cpp sketch/view_math.cpp sketch/sketch_overlay.cpp sketch/sketch_palette.cpp
            sketch/plane_chooser.cpp sketch/sketch_mode.cpp icons.cpp
            timeline/doc_eval.cpp timeline/feature_info.cpp timeline/timeline_widget.cpp timeline/feature_dialogs.cpp
            inspect/measure_dialog.cpp inspect/properties_panel.cpp inspect/section_dialog.cpp inspect/inspect_commands.cpp
            ${IW_UI_ICON_RES}
    PUBLIC_LIBS iw_core iw_kernel iw_naming iw_tess iw_recompute iw_io iw_render iw_sketch iw_sketch_solver iw_features
                Qt6::Widgets Qt6::Svg)
