# SPDX-License-Identifier: GPL-3.0-or-later
# render: RHI viewport. Together with ui, the only module that links Qt Gui. QRhi lives in Qt private headers.
set(CMAKE_AUTOMOC ON)
find_package(Qt6 6.5 REQUIRED COMPONENTS ShaderTools)
iw_add_module(render
    SOURCES camera.cpp nav.cpp viewport.cpp vulkan_support.cpp scene_renderer.cpp viewcube_model.cpp
            viewcube_widget.cpp offscreen.cpp demo_scene.cpp
    PUBLIC_LIBS iw_core iw_tess Qt6::Widgets Qt6::Gui
    PRIVATE_LIBS Qt6::GuiPrivate)

# GLSL 440 -> .qsb (SPIR-V for Vulkan + GLSL 330 for the OpenGL fallback); embedded under :/iw/shaders/.
qt_add_shaders(iw_render iw_render_shaders
    PREFIX /iw/shaders
    BASE shaders
    GLSL "330"
    NOHLSL
    NOMSL
    FILES shaders/bg.vert shaders/bg.frag shaders/mesh.vert shaders/mesh.frag shaders/mesh_pick.frag
          shaders/edge.vert shaders/edge.frag shaders/edge_pick.frag)
