// SPDX-License-Identifier: GPL-3.0-or-later
#include "render/scene_renderer.hpp"

#include <QFile>
#include <QMatrix4x4>
#include <rhi/qshader.h>
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
    float color[4]; ///< rgb = body surface colour (mesh shader), w unused.
};
static_assert(sizeof(UboData) == 176);

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

    int stride = 0;
    std::size_t slotCount = 0; // N bodies the uniform buffer is sized for
    RhiPtr<QRhiBuffer> ubuf;
    RhiPtr<QRhiShaderResourceBindings> srb;
    QShader bgVs, bgFs, meshVs, meshFs, meshPickFs, edgeVs, edgeFs, edgePickFs;
    bool shadersLoaded = false;

    // main-pass pipelines
    std::unique_ptr<QRhiRenderPassDescriptor, RhiDeleter> mainRpd;
    int mainSamples = 0;
    RhiPtr<QRhiGraphicsPipeline> bgPipe, meshPipe, edgePipe;

    // pick resources
    RhiPtr<QRhiTexture> pickTex;
    RhiPtr<QRhiRenderBuffer> pickDepth;
    RhiPtr<QRhiTextureRenderTarget> pickRt;
    std::unique_ptr<QRhiRenderPassDescriptor, RhiDeleter> pickRpd;
    RhiPtr<QRhiGraphicsPipeline> meshPickPipe, edgePickPipe;
    std::array<std::unique_ptr<Slot>, 3> readbacks;
    std::uint64_t sceneGeneration = 0; ///< Bumped by setScene; stale pick readbacks are reported empty.

    bool ensureCommon();
    void ensureUbuf(std::size_t bodies);
    void ensureMainPipelines(QRhiRenderTarget* rt);
    bool ensurePick();
    void upload(QRhiResourceUpdateBatch* rub);
    void writeUniforms(QRhiResourceUpdateBatch* rub, const Camera& cam, QSize px, bool pick, int pickX, int pickY);
    void drawScene(QRhiCommandBuffer* cb, QSize px, bool pick);
    RhiPtr<QRhiGraphicsPipeline> makePipe(const QShader& vs, const QShader& fs, QRhiGraphicsPipeline::Topology topo,
                                          int stride_, bool edge, QRhiRenderPassDescriptor* rpd, int samples,
                                          bool bias);
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
    shadersLoaded = bgVs.isValid() && bgFs.isValid() && meshVs.isValid() && meshFs.isValid() &&
                    meshPickFs.isValid() && edgeVs.isValid() && edgeFs.isValid() && edgePickFs.isValid();
    if (!shadersLoaded) {
        qWarning("iw::render: embedded shaders missing or invalid (:/iw/shaders/*.qsb)");
        return false;
    }
    stride = rhi->ubufAligned(static_cast<int>(sizeof(UboData)));
    ensureUbuf(0);
    return true;
}

void SceneRenderer::Impl::ensureUbuf(std::size_t bodies) {
    if (ubuf && bodies <= slotCount) return;
    slotCount = std::max<std::size_t>(bodies, 4);
    ubuf.reset(rhi->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer,
                              static_cast<quint32>(stride) * static_cast<quint32>(2 * slotCount + 1)));
    ubuf->create();
    if (!srb) srb.reset(rhi->newShaderResourceBindings());
    srb->setBindings({QRhiShaderResourceBinding::uniformBufferWithDynamicOffset(
        0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubuf.get(),
        sizeof(UboData))});
    srb->create();
}

RhiPtr<QRhiGraphicsPipeline> SceneRenderer::Impl::makePipe(const QShader& vs, const QShader& fs,
                                                           QRhiGraphicsPipeline::Topology topo, int stride_, bool edge,
                                                           QRhiRenderPassDescriptor* rpd, int samples, bool bias) {
    RhiPtr<QRhiGraphicsPipeline> p(rhi->newGraphicsPipeline());
    p->setShaderStages({{QRhiShaderStage::Vertex, vs}, {QRhiShaderStage::Fragment, fs}});
    QRhiVertexInputLayout layout;
    if (stride_ > 0) {
        layout.setBindings({QRhiVertexInputBinding(static_cast<quint32>(stride_))});
        if (edge)
            layout.setAttributes({QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0),
                                  QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float, 3 * sizeof(float))});
        else
            layout.setAttributes({QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0),
                                  QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3, 3 * sizeof(float)),
                                  QRhiVertexInputAttribute(0, 2, QRhiVertexInputAttribute::Float, 6 * sizeof(float))});
    }
    p->setVertexInputLayout(layout);
    p->setTopology(topo);
    p->setCullMode(QRhiGraphicsPipeline::None);
    const bool depth = stride_ > 0;
    p->setDepthTest(depth);
    p->setDepthWrite(depth);
    p->setDepthOp(QRhiGraphicsPipeline::LessOrEqual);
    if (bias) { // faces are pushed back so edge lines on their boundary win the depth test
        p->setDepthBias(2);
        p->setSlopeScaledDepthBias(2.0f);
    }
    p->setSampleCount(samples);
    p->setShaderResourceBindings(srb.get());
    p->setRenderPassDescriptor(rpd);
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
    bgPipe = makePipe(bgVs, bgFs, QRhiGraphicsPipeline::Triangles, 0, false, mainRpd.get(), mainSamples, false);
    meshPipe = makePipe(meshVs, meshFs, QRhiGraphicsPipeline::Triangles, kFaceStride, false, mainRpd.get(),
                        mainSamples, true);
    edgePipe = makePipe(edgeVs, edgeFs, QRhiGraphicsPipeline::Lines, kEdgeStride, true, mainRpd.get(), mainSamples,
                        false);
}

bool SceneRenderer::Impl::ensurePick() {
    if (pickRt && meshPickPipe && edgePickPipe) return true;
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
    meshPickPipe = makePipe(meshVs, meshPickFs, QRhiGraphicsPipeline::Triangles, kFaceStride, false, pickRpd.get(), 1,
                            true);
    edgePickPipe = makePipe(edgeVs, edgePickFs, QRhiGraphicsPipeline::Lines, kEdgeStride, true, pickRpd.get(), 1,
                            false);
    return meshPickPipe && edgePickPipe;
}

void SceneRenderer::Impl::upload(QRhiResourceUpdateBatch* rub) {
    if (!sceneDirty) return;
    sceneDirty = false;
    gpu.clear();
    ensureUbuf(scene.size());
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

    auto fill = [&](UboData& u, const QMatrix4x4& mvp, const QMatrix4x4& mv) {
        std::memcpy(u.mvp, mvp.constData(), sizeof(u.mvp));
        std::memcpy(u.mv, mv.constData(), sizeof(u.mv));
    };
    const std::size_t n = scene.size();
    if (!pick) {
        UboData bg{};
        fill(bg, QMatrix4x4(), QMatrix4x4());
        bg.ids2[1] = rhi->isYUpInNDC() ? 1.f : -1.f;
        rub->updateDynamicBuffer(ubuf.get(), 0, sizeof(UboData), &bg);
    }
    for (std::size_t i = 0; i < n; ++i) {
        const auto& b = scene[i];
        const QMatrix4x4 model = fromRowMajor(b.transform.m);
        UboData u{};
        fill(u, proj * view * model, view * model);
        const bool h = hover.bodyIndex && *hover.bodyIndex == i;
        const bool s = selection.bodyIndex && *selection.bodyIndex == i;
        for (int c = 0; c < 3; ++c) u.color[c] = b.color[static_cast<std::size_t>(c)];
        // Whole-body highlight when the selection names a body but no face/edge.
        u.ids2[2] = s && !selection.faceIndex && !selection.edgeIndex ? 1.f : 0.f;
        u.ids[0] = static_cast<float>(i + 1);
        u.ids[1] = h && hover.faceIndex ? static_cast<float>(*hover.faceIndex) : -1.f;
        u.ids[2] = h && hover.edgeIndex ? static_cast<float>(*hover.edgeIndex) : -1.f;
        u.ids[3] = s && selection.faceIndex ? static_cast<float>(*selection.faceIndex) : -1.f;
        u.ids2[0] = s && selection.edgeIndex ? static_cast<float>(*selection.edgeIndex) : -1.f;
        const std::size_t slot = (pick ? 1 + slotCount : 1) + i;
        rub->updateDynamicBuffer(ubuf.get(), static_cast<quint32>(slot) * static_cast<quint32>(stride),
                                 sizeof(UboData), &u);
    }
}

void SceneRenderer::Impl::drawScene(QRhiCommandBuffer* cb, QSize px, bool pick) {
    // setViewport must come AFTER setGraphicsPipeline: that is when QRhi also (re)sets the scissor.
    const QRhiViewport vp(0, 0, static_cast<float>(px.width()), static_cast<float>(px.height()));
    auto* meshP = pick ? meshPickPipe.get() : meshPipe.get();
    auto* edgeP = pick ? edgePickPipe.get() : edgePipe.get();
    for (std::size_t i = 0; i < scene.size() && i < gpu.size(); ++i) {
        if (!scene[i].visible || !gpu[i]) continue;
        const std::size_t slot = (pick ? 1 + slotCount : 1) + i;
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
    auto* rb = d.rhi->nextResourceUpdateBatch();
    rb->readBackTexture(QRhiReadbackDescription(d.pickTex.get()), &slot->result);
    cb->endPass(rb);
    return true;
}

PickDecode decodePickWindow(const float* rgba, int window, double edgeRadius) {
    PickDecode out;
    const int c = window / 2;
    auto at = [&](int x, int y) { return rgba + (static_cast<std::size_t>(y) * static_cast<std::size_t>(window) + static_cast<std::size_t>(x)) * 4; };
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
