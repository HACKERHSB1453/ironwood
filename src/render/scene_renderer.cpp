// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/scene_renderer.hpp"

#include <QFile>
#include <QMatrix4x4>
#include <rhi/qshader.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <rhi/qrhi.h>
#include <unordered_map>

namespace iw::render {

namespace {

struct RhiDeleter {
    void operator()(QRhiResource* r) const {
        if (r) r->deleteLater();
    }
};
template <class T> using RhiPtr = std::unique_ptr<T, RhiDeleter>;

QShader loadShader(const char* name) {
    QFile f(QString(":/iw/shaders/%1.qsb").arg(name));
    if (!f.open(QIODevice::ReadOnly)) return {};
    return QShader::fromSerialized(f.readAll());
}

struct UboData {
    float mvp[16];
    float mv[16];
    float ids[4];
    float ids2[4];
    float color[4]; ///< rgb = body surface colour (mesh shader); rgba for overlays and the section cap.
    float clip[4];  ///< Section plane in object space: xyz normal, w offset; kept side dot >= 0.
    float sec[4];   ///< x = 1 when the section clip is active.
};
static_assert(sizeof(UboData) == 208);

/// The section plane (world space, unit normal) expressed in a body's object space:
/// n·(A p + t) + d = (Aᵀ n)·p + (n·t + d).
std::array<float, 4> objectClip(const SectionRequest& s, const Transform& t) {
    const auto& c = s.clipPlane;
    const double len = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    const double n[3] = {c[0] / len, c[1] / len, c[2] / len};
    const double d = c[3] / len;
    std::array<float, 4> out{};
    for (int j = 0; j < 3; ++j) {
        double v = 0;
        for (int i = 0; i < 3; ++i) v += t.m[static_cast<std::size_t>(i * 4 + j)] * n[i];
        out[static_cast<std::size_t>(j)] = static_cast<float>(v);
    }
    const double dO = n[0] * t.m[3] + n[1] * t.m[7] + n[2] * t.m[11] + d;
    out[3] = static_cast<float>(dO);
    return out;
}
bool sectionActive(const SectionRequest& s) {
    const auto& c = s.clipPlane;
    return s.enabled && std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]) > 1e-12;
}

/// Pipeline recipes. Vertex layouts: Mesh = pos,normal,faceId; Edge = pos,edgeId; Pos = pos only.
enum class Verts { None, Mesh, Edge, Pos };
enum class Stencil { Off, Mask, Cap };
struct PipeSpec {
    const QShader* vs = nullptr;
    const QShader* fs = nullptr;
    QRhiGraphicsPipeline::Topology topo = QRhiGraphicsPipeline::Triangles;
    Verts verts = Verts::None;
    QRhiRenderPassDescriptor* rpd = nullptr;
    int samples = 1;
    bool depth = false;       ///< depth test on
    bool depthWrite = true;   ///< only meaningful with depth
    bool bias = false;        ///< faces pushed back so edges on their boundary win
    bool blend = false;       ///< straight-alpha blending (overlays)
    bool colourWrite = true;
    Stencil stencil = Stencil::Off;
};

QMatrix4x4 fromRowMajor(const std::array<double, 16>& m) {
    float f[16];
    for (int i = 0; i < 16; ++i) f[i] = static_cast<float>(m[static_cast<std::size_t>(i)]);
    return QMatrix4x4(f);
}
/// Camera::projectionMatrix is column-major (m[col*4+row]).
QMatrix4x4 fromColumnMajor(const std::array<double, 16>& m) {
    float f[16];
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) f[r * 4 + c] = static_cast<float>(m[static_cast<std::size_t>(c * 4 + r)]);
    return QMatrix4x4(f);
}

/// GPU copy of one mesh: faces re-indexed so that every vertex belongs to exactly one B-rep face (needed for the
/// per-vertex face id), and edge polylines expanded to a line list.
struct GpuMesh {
    RhiPtr<QRhiBuffer> vbuf, ibuf, ebuf;
    std::uint32_t indexCount = 0, edgeVertexCount = 0;
};

struct GpuOverlay {
    RhiPtr<QRhiBuffer> vbuf;
    std::uint32_t vertexCount = 0;
};

/// Min/max of the visible bodies' world bounds. False when nothing visible has bounds.
bool visibleBounds(const std::vector<SceneBody>& scene, std::array<double, 3>& lo, std::array<double, 3>& hi) {
    bool any = false;
    for (const auto& b : scene) {
        if (!b.visible || !b.mesh || b.mesh->bounds.isEmpty()) continue;
        const auto& mb = b.mesh->bounds;
        for (int i = 0; i < 8; ++i) {
            const Vec3 p = b.transform.apply({(i & 1) ? mb.max.x : mb.min.x, (i & 2) ? mb.max.y : mb.min.y,
                                              (i & 4) ? mb.max.z : mb.min.z});
            const double c[3] = {p.x, p.y, p.z};
            for (std::size_t k = 0; k < 3; ++k) {
                if (!any) { lo[k] = hi[k] = c[k]; }
                lo[k] = std::min(lo[k], c[k]);
                hi[k] = std::max(hi[k], c[k]);
            }
            any = true;
        }
    }
    return any;
}

/// Two triangles (6 vertices, xyz) covering the section plane around the scene: the cap.
std::array<float, 18> capQuad(const SectionRequest& s, const std::array<double, 3>& lo,
                              const std::array<double, 3>& hi) {
    const auto& c = s.clipPlane;
    const double len = std::sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
    const double n[3] = {c[0] / len, c[1] / len, c[2] / len};
    const double d = c[3] / len;
    const double ctr[3] = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
    const double radius = 0.5 * std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                                          (hi[2] - lo[2]) * (hi[2] - lo[2]));
    const double dist = n[0] * ctr[0] + n[1] * ctr[1] + n[2] * ctr[2] + d;
    const double pc[3] = {ctr[0] - dist * n[0], ctr[1] - dist * n[1], ctr[2] - dist * n[2]};
    // in-plane basis
    const double a[3] = {std::abs(n[0]) < 0.9 ? 1.0 : 0.0, std::abs(n[0]) < 0.9 ? 0.0 : 1.0, 0.0};
    double u[3] = {n[1] * a[2] - n[2] * a[1], n[2] * a[0] - n[0] * a[2], n[0] * a[1] - n[1] * a[0]};
    const double ul = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    for (double& x : u) x /= ul;
    const double v[3] = {n[1] * u[2] - n[2] * u[1], n[2] * u[0] - n[0] * u[2], n[0] * u[1] - n[1] * u[0]};
    const double h = 2 * radius + 1;
    auto corner = [&](double su, double sv) {
        std::array<double, 3> p{};
        for (std::size_t k = 0; k < 3; ++k) p[k] = pc[k] + su * h * u[k] + sv * h * v[k];
        return p;
    };
    const auto p0 = corner(-1, -1), p1 = corner(1, -1), p2 = corner(1, 1), p3 = corner(-1, 1);
    std::array<float, 18> out{};
    std::size_t o = 0;
    for (const auto* p : {&p0, &p1, &p2, &p0, &p2, &p3})
        for (std::size_t k = 0; k < 3; ++k) out[o++] = static_cast<float>((*p)[k]);
    return out;
}

constexpr int kFaceStride = 7 * sizeof(float);
constexpr int kEdgeStride = 4 * sizeof(float);

struct CpuMesh {
    std::vector<float> verts;        // pos(3) normal(3) faceId(1)
    std::vector<std::uint32_t> idx;
    std::vector<float> edges;        // pos(3) edgeId(1) per line-list vertex
};

CpuMesh buildCpuMesh(const tess::MeshData& m) {
    CpuMesh out;
    const std::size_t nv = m.positions.size() / 3;
    // normals: use the mesh's, else accumulate area-weighted face normals
    std::vector<float> normals;
    if (m.normals.size() == m.positions.size()) normals = m.normals;
    else {
        normals.assign(m.positions.size(), 0.f);
        for (std::size_t t = 0; t + 2 < m.indices.size(); t += 3) {
            const auto a = m.indices[t], b = m.indices[t + 1], c = m.indices[t + 2];
            if (a >= nv || b >= nv || c >= nv) continue;
            const float* pa = &m.positions[a * 3];
            const float* pb = &m.positions[b * 3];
            const float* pc = &m.positions[c * 3];
            const float e1[3] = {pb[0] - pa[0], pb[1] - pa[1], pb[2] - pa[2]};
            const float e2[3] = {pc[0] - pa[0], pc[1] - pa[1], pc[2] - pa[2]};
            const float n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                                e1[0] * e2[1] - e1[1] * e2[0]};
            for (auto v : {a, b, c})
                for (int k = 0; k < 3; ++k) normals[v * 3 + static_cast<std::size_t>(k)] += n[k];
        }
        for (std::size_t v = 0; v < nv; ++v) {
            float* n = &normals[v * 3];
            const float l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (l > 0) { n[0] /= l; n[1] /= l; n[2] /= l; }
        }
    }

    std::vector<tess::MeshData::FaceRange> ranges = m.faceRanges;
    if (ranges.empty() && !m.indices.empty())
        ranges.push_back({0, static_cast<std::uint32_t>(m.indices.size())});

    std::vector<std::int64_t> owner(nv, -1);
    std::unordered_map<std::uint64_t, std::uint32_t> dup;
    std::vector<std::uint32_t> srcOf;  // new vertex -> source vertex (only duplicates are appended)
    std::vector<std::uint32_t> faceOf; // duplicates' face
    out.idx.reserve(m.indices.size());
    for (std::size_t f = 0; f < ranges.size(); ++f) {
        const auto r = ranges[f];
        if (r.indexCount == 0 || static_cast<std::size_t>(r.firstIndex) + r.indexCount > m.indices.size()) continue;
        for (std::uint32_t k = 0; k + 2 < r.indexCount; k += 3) {
            const std::uint32_t tri[3] = {m.indices[r.firstIndex + k], m.indices[r.firstIndex + k + 1],
                                          m.indices[r.firstIndex + k + 2]};
            if (tri[0] >= nv || tri[1] >= nv || tri[2] >= nv) continue;
            for (auto v : tri) {
                if (owner[v] < 0) owner[v] = static_cast<std::int64_t>(f);
                if (owner[v] == static_cast<std::int64_t>(f)) { out.idx.push_back(v); continue; }
                const std::uint64_t key = (static_cast<std::uint64_t>(v) << 32) | f;
                auto [it, fresh] = dup.try_emplace(key, 0u);
                if (fresh) {
                    it->second = static_cast<std::uint32_t>(nv + srcOf.size());
                    srcOf.push_back(v);
                    faceOf.push_back(static_cast<std::uint32_t>(f));
                }
                out.idx.push_back(it->second);
            }
        }
    }
    out.verts.resize((nv + srcOf.size()) * 7);
    auto put = [&](std::size_t dst, std::size_t src, float face) {
        float* o = &out.verts[dst * 7];
        std::memcpy(o, &m.positions[src * 3], 3 * sizeof(float));
        std::memcpy(o + 3, &normals[src * 3], 3 * sizeof(float));
        o[6] = face;
    };
    for (std::size_t v = 0; v < nv; ++v) put(v, v, owner[v] < 0 ? 0.f : static_cast<float>(owner[v]));
    for (std::size_t d = 0; d < srcOf.size(); ++d) put(nv + d, srcOf[d], static_cast<float>(faceOf[d]));

    for (std::size_t e = 0; e < m.edgePolylines.size(); ++e) {
        const auto& pts = m.edgePolylines[e].points;
        const std::size_t n = pts.size() / 3;
        for (std::size_t i = 0; i + 1 < n; ++i) {
            for (std::size_t k : {i, i + 1}) {
                out.edges.insert(out.edges.end(), {pts[k * 3], pts[k * 3 + 1], pts[k * 3 + 2], static_cast<float>(e)});
            }
        }
    }
    return out;
}

} // namespace

struct SceneRenderer::Impl {
    struct Slot {
        QRhiReadbackResult result;
        std::function<void(PickResult)> done;
        Camera camera;
        QSize px;
        int x = 0, y = 0;
        std::uint64_t generation = 0; ///< sceneGeneration at submit time.
        bool busy = false;
    };

    explicit Impl(QRhi* r) : rhi(r) {}

    QRhi* rhi;
    std::vector<SceneBody> scene;
    bool sceneDirty = true;
    std::vector<std::shared_ptr<GpuMesh>> gpu; // parallel to scene
    PickResult hover, selection;
    SectionRequest section;
    std::vector<OverlayFace> overlays;
    bool overlaysDirty = true;
    std::vector<GpuOverlay> overlayGpu; // parallel to overlays
    bool capReady = false;               // section cap quad written for this frame

    int stride = 0;
    std::size_t slotCount = 0; // N uniform slots per pass (bodies + overlays)
    RhiPtr<QRhiBuffer> ubuf;
    RhiPtr<QRhiBuffer> capVbuf;
    RhiPtr<QRhiShaderResourceBindings> srb;
    QShader bgVs, bgFs, meshVs, meshFs, meshPickFs, edgeVs, edgeFs, edgePickFs;
    QShader overlayVs, overlayFs, overlayPickFs, maskFs;
    bool shadersLoaded = false;

    // main-pass pipelines
    std::unique_ptr<QRhiRenderPassDescriptor, RhiDeleter> mainRpd;
    int mainSamples = 0;
    RhiPtr<QRhiGraphicsPipeline> bgPipe, meshPipe, edgePipe, overlayPipe, maskPipe, capPipe;

    // pick resources
    RhiPtr<QRhiTexture> pickTex;
    RhiPtr<QRhiRenderBuffer> pickDepth;
    RhiPtr<QRhiTextureRenderTarget> pickRt;
    std::unique_ptr<QRhiRenderPassDescriptor, RhiDeleter> pickRpd;
    RhiPtr<QRhiGraphicsPipeline> meshPickPipe, edgePickPipe, overlayPickPipe;
    std::array<std::unique_ptr<Slot>, 3> readbacks;
    std::uint64_t sceneGeneration = 0; ///< Bumped by setScene; stale pick readbacks are reported empty.

    bool ensureCommon();
    void ensureUbuf(std::size_t wanted);
    void ensureMainPipelines(QRhiRenderTarget* rt);
    bool ensurePick();
    void upload(QRhiResourceUpdateBatch* rub);
    void writeUniforms(QRhiResourceUpdateBatch* rub, const Camera& cam, QSize px, bool pick, int pickX, int pickY);
    void drawScene(QRhiCommandBuffer* cb, QSize px, bool pick);
    void drawSectionCap(QRhiCommandBuffer* cb, QSize px);
    void drawOverlays(QRhiCommandBuffer* cb, QSize px, bool pick);
    RhiPtr<QRhiGraphicsPipeline> makePipe(const PipeSpec& spec);
    void drawFinish(Slot&);
};

SceneRenderer::SceneRenderer(QRhi* rhi) : d_(std::make_unique<Impl>(rhi)) {}
SceneRenderer::~SceneRenderer() = default;
void SceneRenderer::setHover(const PickResult& r) { d_->hover = r; }
void SceneRenderer::setSelection(const PickResult& r) { d_->selection = r; }

void SceneRenderer::setScene(std::vector<SceneBody> bodies) {
    d_->scene = std::move(bodies);
    d_->sceneDirty = true;
    ++d_->sceneGeneration;
}

void SceneRenderer::setSection(const SectionRequest& s) { d_->section = s; }

void SceneRenderer::setOverlays(std::vector<OverlayFace> faces) {
    d_->overlays = std::move(faces);
    d_->overlaysDirty = true;
}

bool SceneRenderer::Impl::ensureCommon() {
    if (shadersLoaded) return true;
    bgVs = loadShader("bg.vert");
    bgFs = loadShader("bg.frag");
    meshVs = loadShader("mesh.vert");
    meshFs = loadShader("mesh.frag");
    meshPickFs = loadShader("mesh_pick.frag");
    edgeVs = loadShader("edge.vert");
    edgeFs = loadShader("edge.frag");
    edgePickFs = loadShader("edge_pick.frag");
    overlayVs = loadShader("overlay.vert");
    overlayFs = loadShader("overlay.frag");
    overlayPickFs = loadShader("overlay_pick.frag");
    maskFs = loadShader("mask.frag");
    shadersLoaded = bgVs.isValid() && bgFs.isValid() && meshVs.isValid() && meshFs.isValid() &&
                    meshPickFs.isValid() && edgeVs.isValid() && edgeFs.isValid() && edgePickFs.isValid() &&
                    overlayVs.isValid() && overlayFs.isValid() && overlayPickFs.isValid() && maskFs.isValid();
    if (!shadersLoaded) {
        qWarning("iw::render: embedded shaders missing or invalid (:/iw/shaders/*.qsb)");
        return false;
    }
    stride = rhi->ubufAligned(static_cast<int>(sizeof(UboData)));
    ensureUbuf(0);
    capVbuf.reset(rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer, 18 * sizeof(float)));
    capVbuf->create();
    return true;
}

void SceneRenderer::Impl::ensureUbuf(std::size_t wanted) {
    if (ubuf && wanted <= slotCount) return;
    slotCount = std::max<std::size_t>(wanted, 4);
    ubuf.reset(rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer,
                              static_cast<quint32>(stride) * static_cast<quint32>(2 * slotCount + 1)));
    ubuf->create();
    if (!srb) srb.reset(rhi->newShaderResourceBindings());
    srb->setBindings({QRhiShaderResourceBinding::uniformBufferWithDynamicOffset(
        0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubuf.get(),
        sizeof(UboData))});
    srb->create();
}

RhiPtr<QRhiGraphicsPipeline> SceneRenderer::Impl::makePipe(const PipeSpec& s) {
    RhiPtr<QRhiGraphicsPipeline> p(rhi->newGraphicsPipeline());
    p->setShaderStages({{QRhiShaderStage::Vertex, *s.vs}, {QRhiShaderStage::Fragment, *s.fs}});
    QRhiVertexInputLayout layout;
    switch (s.verts) {
    case Verts::Mesh:
        layout.setBindings({QRhiVertexInputBinding(static_cast<quint32>(kFaceStride))});
        layout.setAttributes({QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0),
                              QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3, 3 * sizeof(float)),
                              QRhiVertexInputAttribute(0, 2, QRhiVertexInputAttribute::Float, 6 * sizeof(float))});
        break;
    case Verts::Edge:
        layout.setBindings({QRhiVertexInputBinding(static_cast<quint32>(kEdgeStride))});
        layout.setAttributes({QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0),
                              QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float, 3 * sizeof(float))});
        break;
    case Verts::Pos:
        layout.setBindings({QRhiVertexInputBinding(static_cast<quint32>(3 * sizeof(float)))});
        layout.setAttributes({QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0)});
        break;
    case Verts::None: break;
    }
    p->setVertexInputLayout(layout);
    p->setTopology(s.topo);
    p->setCullMode(QRhiGraphicsPipeline::None);
    p->setDepthTest(s.depth);
    p->setDepthWrite(s.depth && s.depthWrite);
    p->setDepthOp(QRhiGraphicsPipeline::LessOrEqual);
    if (s.bias) { // faces are pushed back so edge lines on their boundary win the depth test
        p->setDepthBias(2);
        p->setSlopeScaledDepthBias(2.0f);
    }
    QRhiGraphicsPipeline::TargetBlend tb;
    tb.enable = s.blend;
    tb.srcColor = QRhiGraphicsPipeline::SrcAlpha;
    tb.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    tb.srcAlpha = QRhiGraphicsPipeline::One;
    tb.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    tb.colorWrite = s.colourWrite ? QRhiGraphicsPipeline::ColorMask(0xF) : QRhiGraphicsPipeline::ColorMask(0);
    p->setTargetBlends({tb});
    if (s.stencil != Stencil::Off) {
        // Mask: every surface toggles the parity of the stencil (the solid's interior at the cut plane is odd).
        // Cap: draw only where the parity is odd (stencil reference 0).
        QRhiGraphicsPipeline::StencilOpState st{};
        if (s.stencil == Stencil::Mask) {
            st.failOp = QRhiGraphicsPipeline::Keep;
            st.depthFailOp = QRhiGraphicsPipeline::Keep;
            st.passOp = QRhiGraphicsPipeline::Invert;
            st.compareOp = QRhiGraphicsPipeline::Always;
        } else {
            st.failOp = QRhiGraphicsPipeline::Keep;
            st.depthFailOp = QRhiGraphicsPipeline::Keep;
            st.passOp = QRhiGraphicsPipeline::Keep;
            st.compareOp = QRhiGraphicsPipeline::NotEqual;
        }
        p->setStencilTest(true);
        p->setStencilFront(st);
        p->setStencilBack(st);
    }
    p->setSampleCount(s.samples);
    p->setShaderResourceBindings(srb.get());
    p->setRenderPassDescriptor(s.rpd);
    if (!p->create()) {
        qWarning("iw::render: graphics pipeline creation failed");
        return nullptr;
    }
    return p;
}

void SceneRenderer::Impl::ensureMainPipelines(QRhiRenderTarget* rt) {
    auto* rpd = rt->renderPassDescriptor();
    if (mainRpd && meshPipe && mainRpd->isCompatible(rpd) && mainSamples == rt->sampleCount()) return;
    mainRpd.reset(rpd->newCompatibleRenderPassDescriptor());
    mainSamples = rt->sampleCount();
    auto spec = [&](const QShader* vs, const QShader* fs) {
        PipeSpec s;
        s.vs = vs;
        s.fs = fs;
        s.rpd = mainRpd.get();
        s.samples = mainSamples;
        return s;
    };
    PipeSpec bg = spec(&bgVs, &bgFs);
    bgPipe = makePipe(bg);
    PipeSpec mesh = spec(&meshVs, &meshFs);
    mesh.verts = Verts::Mesh;
    mesh.depth = true;
    mesh.bias = true;
    meshPipe = makePipe(mesh);
    PipeSpec edge = spec(&edgeVs, &edgeFs);
    edge.verts = Verts::Edge;
    edge.topo = QRhiGraphicsPipeline::Lines;
    edge.depth = true;
    edgePipe = makePipe(edge);
    PipeSpec overlay = spec(&overlayVs, &overlayFs);
    overlay.verts = Verts::Pos;
    overlay.depth = true;
    overlay.depthWrite = false;
    overlay.blend = true;
    overlayPipe = makePipe(overlay);
    // Stencil passes: the mask writes no colour and no depth; the cap is depth-tested against the kept geometry.
    PipeSpec mask = spec(&meshVs, &maskFs);
    mask.verts = Verts::Mesh;
    mask.colourWrite = false;
    mask.stencil = Stencil::Mask;
    maskPipe = makePipe(mask);
    PipeSpec cap = spec(&overlayVs, &overlayFs);
    cap.verts = Verts::Pos;
    cap.depth = true;
    cap.stencil = Stencil::Cap;
    capPipe = makePipe(cap);
}

bool SceneRenderer::Impl::ensurePick() {
    if (pickRt && meshPickPipe && edgePickPipe && overlayPickPipe) return true;
    const QSize sz(kPickWindow, kPickWindow);
    pickTex.reset(rhi->newTexture(QRhiTexture::RGBA32F, sz, 1,
                                  QRhiTexture::RenderTarget | QRhiTexture::UsedAsTransferSource));
    if (!pickTex->create()) return false;
    pickDepth.reset(rhi->newRenderBuffer(QRhiRenderBuffer::DepthStencil, sz, 1));
    if (!pickDepth->create()) return false;
    QRhiTextureRenderTargetDescription desc(QRhiColorAttachment(pickTex.get()));
    desc.setDepthStencilBuffer(pickDepth.get());
    pickRt.reset(rhi->newTextureRenderTarget(desc));
    pickRpd.reset(pickRt->newCompatibleRenderPassDescriptor());
    pickRt->setRenderPassDescriptor(pickRpd.get());
    if (!pickRt->create()) return false;
    PipeSpec mesh;
    mesh.vs = &meshVs;
    mesh.fs = &meshPickFs;
    mesh.verts = Verts::Mesh;
    mesh.rpd = pickRpd.get();
    mesh.depth = true;
    mesh.bias = true;
    meshPickPipe = makePipe(mesh);
    PipeSpec edge;
    edge.vs = &edgeVs;
    edge.fs = &edgePickFs;
    edge.verts = Verts::Edge;
    edge.topo = QRhiGraphicsPipeline::Lines;
    edge.rpd = pickRpd.get();
    edge.depth = true;
    edgePickPipe = makePipe(edge);
    PipeSpec overlay;
    overlay.vs = &overlayVs;
    overlay.fs = &overlayPickFs;
    overlay.verts = Verts::Pos;
    overlay.rpd = pickRpd.get();
    overlay.depth = true;
    overlay.depthWrite = false;
    overlayPickPipe = makePipe(overlay);
    return meshPickPipe && edgePickPipe && overlayPickPipe;
}

void SceneRenderer::Impl::upload(QRhiResourceUpdateBatch* rub) {
    ensureUbuf(scene.size() + overlays.size());
    if (overlaysDirty) {
        overlaysDirty = false;
        overlayGpu.clear();
        for (const auto& f : overlays) {
            GpuOverlay g;
            const std::size_t n = f.triangles.size() / 3 * 3; // whole triangles only
            if (n) {
                std::vector<float> v(n * 3);
                for (std::size_t i = 0; i < n; ++i) {
                    v[i * 3] = static_cast<float>(f.triangles[i].x);
                    v[i * 3 + 1] = static_cast<float>(f.triangles[i].y);
                    v[i * 3 + 2] = static_cast<float>(f.triangles[i].z);
                }
                g.vbuf.reset(rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer,
                                            static_cast<quint32>(v.size() * sizeof(float))));
                g.vbuf->create();
                rub->uploadStaticBuffer(g.vbuf.get(), v.data());
                g.vertexCount = static_cast<std::uint32_t>(n);
            }
            overlayGpu.push_back(std::move(g));
        }
    }
    if (!sceneDirty) return;
    sceneDirty = false;
    gpu.clear();
    std::unordered_map<const tess::MeshData*, std::shared_ptr<GpuMesh>> cache;
    for (const auto& body : scene) {
        std::shared_ptr<GpuMesh> g;
        if (body.mesh) {
            auto it = cache.find(body.mesh.get());
            if (it != cache.end()) g = it->second;
            else {
                g = std::make_shared<GpuMesh>();
                CpuMesh cpu = buildCpuMesh(*body.mesh);
                if (!cpu.idx.empty()) {
                    g->vbuf.reset(rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer,
                                                 static_cast<quint32>(cpu.verts.size() * sizeof(float))));
                    g->vbuf->create();
                    rub->uploadStaticBuffer(g->vbuf.get(), cpu.verts.data());
                    g->ibuf.reset(rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer,
                                                 static_cast<quint32>(cpu.idx.size() * sizeof(std::uint32_t))));
                    g->ibuf->create();
                    rub->uploadStaticBuffer(g->ibuf.get(), cpu.idx.data());
                    g->indexCount = static_cast<std::uint32_t>(cpu.idx.size());
                }
                if (!cpu.edges.empty()) {
                    g->ebuf.reset(rhi->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer,
                                                 static_cast<quint32>(cpu.edges.size() * sizeof(float))));
                    g->ebuf->create();
                    rub->uploadStaticBuffer(g->ebuf.get(), cpu.edges.data());
                    g->edgeVertexCount = static_cast<std::uint32_t>(cpu.edges.size() / 4);
                }
                cache.emplace(body.mesh.get(), g);
            }
        }
        gpu.push_back(std::move(g));
    }
}

void SceneRenderer::Impl::writeUniforms(QRhiResourceUpdateBatch* rub, const Camera& cam, QSize px, bool pick,
                                        int pickX, int pickY) {
    const double aspect = static_cast<double>(px.width()) / std::max(px.height(), 1);
    QMatrix4x4 proj = fromColumnMajor(cam.projectionMatrix(aspect));
    if (pick) {
        // Crop the frustum to the kPickWindow^2 pixel window centred on (pickX,pickY).
        const float sx = static_cast<float>(px.width()) / kPickWindow;
        const float sy = static_cast<float>(px.height()) / kPickWindow;
        const float cx = (static_cast<float>(pickX) + 0.5f) / px.width() * 2.f - 1.f;
        const float cy = 1.f - (static_cast<float>(pickY) + 0.5f) / px.height() * 2.f;
        QMatrix4x4 crop;
        crop.scale(sx, sy, 1.f);
        crop.translate(-cx, -cy, 0.f);
        proj = crop * proj;
    }
    proj = rhi->clipSpaceCorrMatrix() * proj;
    const QMatrix4x4 view = fromRowMajor(cam.viewMatrix().m);
    const QMatrix4x4 viewProj = proj * view;
    const bool clipOn = sectionActive(section);

    auto fill = [&](UboData& u, const QMatrix4x4& mvp, const QMatrix4x4& mv) {
        std::memcpy(u.mvp, mvp.constData(), sizeof(u.mvp));
        std::memcpy(u.mv, mv.constData(), sizeof(u.mv));
    };
    const std::size_t base = pick ? 1 + slotCount : 1;
    const std::size_t n = scene.size();
    if (!pick) {
        // slot 0: background (ids2.y only) and the section cap quad (viewProj, capColour)
        UboData bg{};
        fill(bg, viewProj, view);
        for (int c = 0; c < 4; ++c) bg.color[c] = section.capColour[static_cast<std::size_t>(c)];
        bg.ids2[1] = rhi->isYUpInNDC() ? 1.f : -1.f;
        rub->updateDynamicBuffer(ubuf.get(), 0, sizeof(UboData), &bg);
        capReady = false;
        if (clipOn) {
            std::array<double, 3> lo{}, hi{};
            if (visibleBounds(scene, lo, hi)) {
                const auto quad = capQuad(section, lo, hi);
                rub->updateDynamicBuffer(capVbuf.get(), 0, sizeof(quad), quad.data());
                capReady = true;
            }
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        const auto& b = scene[i];
        const QMatrix4x4 model = fromRowMajor(b.transform.m);
        UboData u{};
        fill(u, viewProj * model, view * model);
        const bool h = hover.bodyIndex && *hover.bodyIndex == i;
        const bool s = selection.bodyIndex && *selection.bodyIndex == i;
        for (int c = 0; c < 3; ++c) u.color[c] = b.color[static_cast<std::size_t>(c)];
        u.color[3] = 1.f;
        // Whole-body highlight when the selection names a body but no face/edge.
        u.ids2[2] = s && !selection.faceIndex && !selection.edgeIndex ? 1.f : 0.f;
        u.ids[0] = static_cast<float>(i + 1);
        u.ids[1] = h && hover.faceIndex ? static_cast<float>(*hover.faceIndex) : -1.f;
        u.ids[2] = h && hover.edgeIndex ? static_cast<float>(*hover.edgeIndex) : -1.f;
        u.ids[3] = s && selection.faceIndex ? static_cast<float>(*selection.faceIndex) : -1.f;
        u.ids2[0] = s && selection.edgeIndex ? static_cast<float>(*selection.edgeIndex) : -1.f;
        if (clipOn) {
            const auto oc = objectClip(section, b.transform);
            for (int c = 0; c < 4; ++c) u.clip[c] = oc[static_cast<std::size_t>(c)];
            u.sec[0] = 1.f;
        }
        const std::size_t slot = base + i;
        rub->updateDynamicBuffer(ubuf.get(), static_cast<quint32>(slot) * static_cast<quint32>(stride),
                                 sizeof(UboData), &u);
    }
    for (std::size_t k = 0; k < overlays.size(); ++k) {
        UboData u{};
        fill(u, viewProj, view);   // overlay vertices are world space
        for (int c = 0; c < 4; ++c) u.color[c] = overlays[k].rgba[static_cast<std::size_t>(c)];
        u.ids[0] = static_cast<float>(k + 1);
        const std::size_t slot = base + n + k;
        rub->updateDynamicBuffer(ubuf.get(), static_cast<quint32>(slot) * static_cast<quint32>(stride),
                                 sizeof(UboData), &u);
    }
}

void SceneRenderer::Impl::drawScene(QRhiCommandBuffer* cb, QSize px, bool pick) {
    // setViewport must come AFTER setGraphicsPipeline: that is when QRhi also (re)sets the scissor.
    const QRhiViewport vp(0, 0, static_cast<float>(px.width()), static_cast<float>(px.height()));
    auto* meshP = pick ? meshPickPipe.get() : meshPipe.get();
    auto* edgeP = pick ? edgePickPipe.get() : edgePipe.get();
    const std::size_t base = pick ? 1 + slotCount : 1;
    for (std::size_t i = 0; i < scene.size() && i < gpu.size(); ++i) {
        if (!scene[i].visible || !gpu[i]) continue;
        const std::size_t slot = base + i;
        const QRhiCommandBuffer::DynamicOffset off{0, static_cast<quint32>(slot) * static_cast<quint32>(stride)};
        const auto& g = *gpu[i];
        if (g.indexCount && meshP) {
            cb->setGraphicsPipeline(meshP);
            cb->setViewport(vp);
            cb->setShaderResources(srb.get(), 1, &off);
            const QRhiCommandBuffer::VertexInput vi(g.vbuf.get(), 0);
            cb->setVertexInput(0, 1, &vi, g.ibuf.get(), 0, QRhiCommandBuffer::IndexUInt32);
            cb->drawIndexed(g.indexCount);
        }
        if (g.edgeVertexCount && edgeP) {
            cb->setGraphicsPipeline(edgeP);
            cb->setViewport(vp);
            cb->setShaderResources(srb.get(), 1, &off);
            const QRhiCommandBuffer::VertexInput vi(g.ebuf.get(), 0);
            cb->setVertexInput(0, 1, &vi);
            cb->draw(g.edgeVertexCount);
        }
    }
}

void SceneRenderer::Impl::drawSectionCap(QRhiCommandBuffer* cb, QSize px) {
    if (!capReady || !maskPipe || !capPipe) return;
    const QRhiViewport vp(0, 0, static_cast<float>(px.width()), static_cast<float>(px.height()));
    // Parity mask: each visible body's faces, clipped like the rest of the scene, depth test off.
    const std::size_t base = 1;
    for (std::size_t i = 0; i < scene.size() && i < gpu.size(); ++i) {
        if (!scene[i].visible || !gpu[i] || !gpu[i]->indexCount) continue;
        const QRhiCommandBuffer::DynamicOffset off{0, static_cast<quint32>(base + i) * static_cast<quint32>(stride)};
        const auto& g = *gpu[i];
        cb->setGraphicsPipeline(maskPipe.get());
        cb->setViewport(vp);
        cb->setShaderResources(srb.get(), 1, &off);
        const QRhiCommandBuffer::VertexInput vi(g.vbuf.get(), 0);
        cb->setVertexInput(0, 1, &vi, g.ibuf.get(), 0, QRhiCommandBuffer::IndexUInt32);
        cb->drawIndexed(g.indexCount);
    }
    // Cap quad where the parity is odd (stencil != 0 with reference 0).
    const QRhiCommandBuffer::DynamicOffset off{0, 0};
    cb->setGraphicsPipeline(capPipe.get());
    cb->setViewport(vp);
    cb->setStencilRef(0);
    cb->setShaderResources(srb.get(), 1, &off);
    const QRhiCommandBuffer::VertexInput vi(capVbuf.get(), 0);
    cb->setVertexInput(0, 1, &vi);
    cb->draw(6);
}

void SceneRenderer::Impl::drawOverlays(QRhiCommandBuffer* cb, QSize px, bool pick) {
    const QRhiViewport vp(0, 0, static_cast<float>(px.width()), static_cast<float>(px.height()));
    auto* pipe = pick ? overlayPickPipe.get() : overlayPipe.get();
    if (!pipe) return;
    const std::size_t base = pick ? 1 + slotCount : 1;
    for (std::size_t k = 0; k < overlayGpu.size(); ++k) {
        const auto& g = overlayGpu[k];
        if (!g.vbuf || !g.vertexCount) continue;
        const QRhiCommandBuffer::DynamicOffset off{
            0, static_cast<quint32>(base + scene.size() + k) * static_cast<quint32>(stride)};
        cb->setGraphicsPipeline(pipe);
        cb->setViewport(vp);
        cb->setShaderResources(srb.get(), 1, &off);
        const QRhiCommandBuffer::VertexInput vi(g.vbuf.get(), 0);
        cb->setVertexInput(0, 1, &vi);
        cb->draw(g.vertexCount);
    }
}

void SceneRenderer::render(QRhiCommandBuffer* cb, QRhiRenderTarget* rt, const Camera& camera, QSize px) {
    auto& d = *d_;
    if (!d.ensureCommon()) return;
    auto* rub = d.rhi->nextResourceUpdateBatch();
    d.upload(rub);
    d.ensureMainPipelines(rt);
    d.writeUniforms(rub, camera, px, false, 0, 0);
    cb->beginPass(rt, QColor::fromRgbF(0.8f, 0.82f, 0.86f, 1.0f), {1.0f, 0}, rub);
    if (d.bgPipe) {
        cb->setGraphicsPipeline(d.bgPipe.get());
        cb->setViewport(QRhiViewport(0, 0, static_cast<float>(px.width()), static_cast<float>(px.height())));
        const QRhiCommandBuffer::DynamicOffset off{0, 0};
        cb->setShaderResources(d.srb.get(), 1, &off);
        cb->draw(3);
    }
    d.drawScene(cb, px, false);
    d.drawSectionCap(cb, px);
    d.drawOverlays(cb, px, false);
    cb->endPass();
}

bool SceneRenderer::canSubmitPick() const {
    for (const auto& s : d_->readbacks)
        if (!s || !s->busy) return true;
    return false;
}

bool SceneRenderer::submitPick(QRhiCommandBuffer* cb, const Camera& camera, QSize px, int x, int y,
                               std::function<void(PickResult)> done) {
    auto& d = *d_;
    if (!d.ensureCommon() || !d.ensurePick()) return false;
    Impl::Slot* slot = nullptr;
    for (auto& s : d.readbacks) {
        if (!s) s = std::make_unique<Impl::Slot>();
        if (!s->busy) { slot = s.get(); break; }
    }
    if (!slot) return false;
    slot->busy = true;
    slot->generation = d.sceneGeneration;
    slot->done = std::move(done);
    slot->camera = camera;
    slot->camera.setViewportSize(px.width(), px.height());
    slot->px = px;
    slot->x = x;
    slot->y = y;
    slot->result = QRhiReadbackResult();
    slot->result.completed = [this, slot] {
        Impl& dd = *d_;
        PickResult out;
        const auto& r = slot->result;
        const std::size_t want = static_cast<std::size_t>(kPickWindow) * kPickWindow * 4 * sizeof(float);
        // A readback taken before the last setScene refers to bodies that are no longer shown: report nothing.
        const bool current = slot->generation == dd.sceneGeneration;
        if (current && r.format == QRhiTexture::RGBA32F && static_cast<std::size_t>(r.data.size()) >= want &&
            r.pixelSize.width() == kPickWindow) {
            const float* src = reinterpret_cast<const float*>(r.data.constData());
            std::vector<float> flipped;
            if (dd.rhi->isYUpInFramebuffer()) { // bottom-left origin readback: make the top row first
                flipped.resize(static_cast<std::size_t>(kPickWindow) * kPickWindow * 4);
                for (int row = 0; row < kPickWindow; ++row)
                    std::memcpy(&flipped[static_cast<std::size_t>(row) * kPickWindow * 4],
                                src + static_cast<std::size_t>(kPickWindow - 1 - row) * kPickWindow * 4,
                                kPickWindow * 4 * sizeof(float));
                src = flipped.data();
            }
            PickDecode dec = decodePickWindow(src, kPickWindow, kEdgePickRadius);
            out = dec.result;
            if (dec.hit) {
                const int fx = slot->x + dec.px - kPickWindow / 2, fy = slot->y + dec.py - kPickWindow / 2;
                const double ndcX = (fx + 0.5) / slot->px.width() * 2 - 1;
                const double ndcY = 1 - (fy + 0.5) / slot->px.height() * 2;
                const Ray ray = slot->camera.rayThrough(ndcX, ndcY);
                const double c = dot(ray.dir, slot->camera.forward());
                if (c > 1e-9) out.point = ray.origin + ray.dir * (static_cast<double>(dec.depth) / c);
            }
        }
        auto cb2 = std::move(slot->done);
        slot->done = nullptr;
        slot->busy = false;
        if (cb2) cb2(out);
    };
    auto* rub = d.rhi->nextResourceUpdateBatch();
    d.upload(rub);
    d.writeUniforms(rub, slot->camera, px, true, x, y);
    cb->beginPass(d.pickRt.get(), QColor::fromRgbF(0.f, 0.f, 0.f, 0.f), {1.0f, 0}, rub);
    d.drawScene(cb, QSize(kPickWindow, kPickWindow), true);
    d.drawOverlays(cb, QSize(kPickWindow, kPickWindow), true);
    auto* rb = d.rhi->nextResourceUpdateBatch();
    rb->readBackTexture(QRhiReadbackDescription(d.pickTex.get()), &slot->result);
    cb->endPass(rb);
    return true;
}

PickDecode decodePickWindow(const float* rgba, int window, double edgeRadius) {
    PickDecode out;
    const int c = window / 2;
    auto at = [&](int x, int y) { return rgba + (static_cast<std::size_t>(y) * static_cast<std::size_t>(window) + static_cast<std::size_t>(x)) * 4; };

    // Overlays (marker b = 2) win over bodies behind them: nearest overlay texel within the radius.
    const float* ov = nullptr;
    double ovD = 1e30;
    int ox = 0, oy = 0;
    for (int y = 0; y < window; ++y) {
        for (int x = 0; x < window; ++x) {
            const float* p = at(x, y);
            if (p[2] < 1.5f || p[2] > 2.5f) continue;
            const double dist = std::hypot(x - c, y - c);
            if (dist > edgeRadius || dist >= ovD - 1e-9) continue;
            ovD = dist;
            ov = p;
            ox = x;
            oy = y;
        }
    }
    if (ov) {
        out.hit = true;
        out.px = ox;
        out.py = oy;
        out.depth = ov[3];
        out.result.overlayIndex = static_cast<std::uint32_t>(std::lround(ov[1])) - 1u;
        return out;
    }

    const float* edge = nullptr;
    double bestD = 1e30;
    int ex = 0, ey = 0;
    for (int y = 0; y < window; ++y) {
        for (int x = 0; x < window; ++x) {
            const float* p = at(x, y);
            if (p[0] < 0.5f || p[2] < 0.5f) continue;
            const double dist = std::hypot(x - c, y - c);
            if (dist > edgeRadius) continue;
            if (dist < bestD - 1e-9 || (std::abs(dist - bestD) < 1e-9 && edge && p[3] < edge[3])) {
                bestD = dist;
                edge = p;
                ex = x;
                ey = y;
            }
        }
    }
    const float* mid = at(c, c);
    const bool midFace = mid[0] >= 0.5f && mid[2] < 0.5f;
    auto idx = [](float v) { return static_cast<std::uint32_t>(std::lround(v)) - 1u; };
    if (edge) {
        out.hit = true;
        out.px = ex;
        out.py = ey;
        out.depth = edge[3];
        out.result.bodyIndex = idx(edge[0]);
        out.result.edgeIndex = idx(edge[1]);
        if (midFace && idx(mid[0]) == *out.result.bodyIndex) out.result.faceIndex = idx(mid[1]);
    } else if (midFace) {
        out.hit = true;
        out.px = c;
        out.py = c;
        out.depth = mid[3];
        out.result.bodyIndex = idx(mid[0]);
        out.result.faceIndex = idx(mid[1]);
    }
    return out;
}

} // namespace iw::render
