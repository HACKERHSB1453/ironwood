// SPDX-License-Identifier: GPL-3.0-or-later
#include "kernel/exchange.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

#include <BRepBuilderAPI_Copy.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeSolid.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <BRepGProp.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_Static.hxx>
#include <Message_ProgressRange.hxx>
#include <Poly_Triangulation.hxx>
#include <Quantity_Color.hxx>
#include <Quantity_ColorRGBA.hxx>
#include <RWStl.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <Standard_Failure.hxx>
#include <StlAPI_Writer.hxx>
#include <TCollection_AsciiString.hxx>
#include <TCollection_ExtendedString.hxx>
#include <TDF_Label.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDF_Tool.hxx>
#include <TColStd_SequenceOfAsciiString.hxx>
#include <UnitsMethods_LengthUnit.hxx>
#include <STEPControl_StepModelType.hxx>
#include <Message_ProgressScope.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_ColorType.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include "kernel/internal/progress.hpp"
#include "kernel/internal/shape_impl.hpp"

namespace iw::kernel {
namespace {

namespace fs = std::filesystem;

/// Serialises every OCCT XDE/STL entry point of this file: STEP readers/writers use global state (THREAD_SAFETY.md),
/// and import jobs run on JobSystem workers. Not re-entrant: the shape overload of exportStep delegates to the
/// assembly overload and therefore must not lock itself.
std::mutex& exchangeMutex() {
    static std::mutex m;
    return m;
}

Error occtError(ErrorCode code, const Standard_Failure& e, const char* what) {
    return Error{code, std::string(what) + ": " + (e.GetMessageString() ? e.GetMessageString() : "OCCT exception")};
}

std::string toUtf8(const TCollection_ExtendedString& s) { return std::string(TCollection_AsciiString(s).ToCString()); }

gp_Trsf toTrsf(const Transform& t) {
    gp_Trsf g;
    g.SetValues(t.m[0], t.m[1], t.m[2], t.m[3], t.m[4], t.m[5], t.m[6], t.m[7], t.m[8], t.m[9], t.m[10], t.m[11]);
    return g;
}

Transform fromTrsf(const gp_Trsf& g) {
    Transform t;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c) t.m[static_cast<std::size_t>(r * 4 + c)] = g.Value(r + 1, c + 1);
    return t;
}

Color fromRgba(const Quantity_ColorRGBA& c) {
    double r, g, b;
    c.GetRGB().Values(r, g, b, Quantity_TOC_sRGB);
    return {static_cast<float>(r), static_cast<float>(g), static_cast<float>(b), c.Alpha()};
}
Quantity_ColorRGBA toRgba(const Color& c) {
    return Quantity_ColorRGBA(Quantity_Color(c.r, c.g, c.b, Quantity_TOC_sRGB), c.a);
}

std::string normaliseUnit(const TCollection_AsciiString& u) {
    TCollection_AsciiString n = u;
    n.UpperCase();
    if (n == "MM" || n == "MILLIMETRE" || n == "MILLIMETER") return "mm";
    if (n == "M" || n == "METRE" || n == "METER") return "m";
    if (n == "CM" || n == "CENTIMETRE" || n == "CENTIMETER") return "cm";
    if (n == "INCH" || n == "IN") return "inch";
    if (n == "FT" || n == "FOOT") return "ft";
    if (n == "UM" || n == "MICROMETRE") return "um";
    return std::string(u.ToCString());
}

// ---------------------------------------------------------------------------------------------- STEP import

struct ImportCtx {
    Handle(XCAFDoc_ShapeTool) st;
    Handle(XCAFDoc_ColorTool) ct;
    ImportedAssembly* out = nullptr;
    std::map<std::string, Shape> shapeCache; // product label entry -> shared Shape (instances share the handle)
    int depth = 0;
};

std::string labelName(const TDF_Label& l) {
    Handle(TDataStd_Name) n;
    if (l.FindAttribute(TDataStd_Name::GetID(), n)) return toUtf8(n->Get());
    return {};
}

std::optional<Color> labelColor(ImportCtx& c, const TDF_Label& l) {
    Quantity_ColorRGBA col;
    for (auto type : {XCAFDoc_ColorSurf, XCAFDoc_ColorGen, XCAFDoc_ColorCurv})
        if (c.ct->GetColor(l, type, col)) return fromRgba(col);
    // Face-level colours only: accept when every coloured sub-shape agrees.
    TDF_LabelSequence subs;
    XCAFDoc_ShapeTool::GetSubShapes(l, subs);
    std::optional<Color> agreed;
    for (int i = 1; i <= subs.Length(); ++i) {
        if (!c.ct->GetColor(subs.Value(i), XCAFDoc_ColorSurf, col)) continue;
        const Color k = fromRgba(col);
        if (agreed && !(*agreed == k)) return std::nullopt;
        agreed = k;
    }
    return agreed;
}


Shape sharedShape(ImportCtx& c, const TDF_Label& proto) {
    TCollection_AsciiString entry;
    TDF_Tool::Entry(proto, entry);
    const std::string key = entry.ToCString();
    if (auto it = c.shapeCache.find(key); it != c.shapeCache.end()) return it->second;
    TopoDS_Shape sh = XCAFDoc_ShapeTool::GetShape(proto);
    Shape w = sh.IsNull() ? Shape{} : wrapShape(sh.Located(TopLoc_Location()));
    c.shapeCache.emplace(key, w);
    return w;
}

/// Appends the occurrence `label` (a free shape or a component reference) and its subtree; returns node index.
std::int32_t addImportNode(ImportCtx& c, const TDF_Label& label, std::int32_t parent) {
    if (++c.depth > 256) { --c.depth; return -1; }
    TDF_Label proto = label;
    const bool isRef = XCAFDoc_ShapeTool::IsReference(label);
    if (isRef) XCAFDoc_ShapeTool::GetReferredShape(label, proto);

    AssemblyNode node;
    node.name = labelName(label);
    if (node.name.empty()) node.name = labelName(proto);
    node.color = labelColor(c, label);
    if (!node.color) node.color = labelColor(c, proto);
    node.parent = parent;
    if (isRef) node.local = fromTrsf(XCAFDoc_ShapeTool::GetLocation(label).Transformation());
    else node.local = fromTrsf(XCAFDoc_ShapeTool::GetShape(label).Location().Transformation());

    const auto idx = static_cast<std::int32_t>(c.out->nodes.size());
    const bool assembly = XCAFDoc_ShapeTool::IsAssembly(proto);
    if (!assembly) node.shape = sharedShape(c, proto);
    c.out->nodes.push_back(std::move(node));
    if (assembly) {
        TDF_LabelSequence comps;
        XCAFDoc_ShapeTool::GetComponents(proto, comps);
        for (int i = 1; i <= comps.Length(); ++i) {
            const std::int32_t ch = addImportNode(c, comps.Value(i), idx);
            if (ch >= 0) c.out->nodes[static_cast<std::size_t>(idx)].children.push_back(static_cast<std::uint32_t>(ch));
        }
    }
    --c.depth;
    return idx;
}

void configureStepUnits() {
    Interface_Static::SetCVal("xstep.cascade.unit", "MM");
    Interface_Static::SetCVal("write.step.unit", "MM");
}

} // namespace

Result<ImportedAssembly> importStep(const fs::path& path, CancelToken cancel) {
    std::lock_guard lock(exchangeMutex());
    std::error_code ec;
    if (path.empty() || !fs::is_regular_file(path, ec))
        return makeError(ErrorCode::IoError, "importStep: cannot open '" + path.string() + "'");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "importStep cancelled");
    try {
        configureStepUnits();
        Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
        Handle(TDocStd_Document) doc;
        app->NewDocument("MDTV-XCAF", doc);

        STEPCAFControl_Reader reader;
        reader.SetColorMode(Standard_True);
        reader.SetNameMode(Standard_True);
        reader.SetLayerMode(Standard_False);
        reader.SetPropsMode(Standard_False);
        reader.SetGDTMode(Standard_False);
        reader.SetMatMode(Standard_False);
        const IFSelect_ReturnStatus st = reader.ReadFile(path.string().c_str());
        if (st == IFSelect_RetFail && !fs::exists(path))
            return makeError(ErrorCode::IoError, "importStep: cannot open file");
        if (st != IFSelect_RetDone)
            return makeError(ErrorCode::ParseError, "importStep: '" + path.string() + "' is not a readable STEP file");
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "importStep cancelled");

        ImportedAssembly result;
        result.lengthUnitInFile = "mm";
        {
            TColStd_SequenceOfAsciiString lens, angs, sas;
            reader.ChangeReader().FileUnits(lens, angs, sas);
            if (lens.Length() > 0) result.lengthUnitInFile = normaliseUnit(lens.Value(1));
        }

        Handle(Message_ProgressIndicator) pi = new CancelProgress(cancel);
        if (!reader.Transfer(doc, pi->Start())) {
            if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "importStep cancelled");
            return makeError(ErrorCode::ParseError, "importStep: STEP transfer produced no shapes");
        }
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "importStep cancelled");

        ImportCtx ctx;
        ctx.st = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        ctx.ct = XCAFDoc_DocumentTool::ColorTool(doc->Main());
        ctx.out = &result;
        TDF_LabelSequence frees;
        ctx.st->GetFreeShapes(frees);
        for (int i = 1; i <= frees.Length(); ++i) {
            const std::int32_t n = addImportNode(ctx, frees.Value(i), -1);
            if (n >= 0) result.roots.push_back(static_cast<std::uint32_t>(n));
        }
        if (result.nodes.empty()) return makeError(ErrorCode::ParseError, "importStep: file contains no shapes");
        app->Close(doc);
        return result;
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(ErrorCode::ParseError, e, "importStep"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("importStep: ") + e.what());
    }
}

// ---------------------------------------------------------------------------------------------- STEP export

namespace {

struct ExportCtx {
    Handle(XCAFDoc_ShapeTool) st;
    Handle(XCAFDoc_ColorTool) ct;
    const ImportedAssembly* in = nullptr;
    std::map<const void*, TDF_Label> protoOf; // Shape::Impl* -> prototype label (instances share products)
    std::vector<char> visiting;
};

void setName(const TDF_Label& l, const std::string& name) {
    if (!name.empty()) TDataStd_Name::Set(l, TCollection_ExtendedString(name.c_str(), true));
}

void setColor(ExportCtx& c, const TDF_Label& l, const std::optional<Color>& col) {
    if (col) c.ct->SetColor(l, toRgba(*col), XCAFDoc_ColorSurf);
}

/// Creates (or re-uses, for shared leaf shapes) the product label for node `i`.
Result<TDF_Label> buildProto(ExportCtx& c, std::uint32_t i, const gp_Trsf* rootTrsf) {
    if (i >= c.in->nodes.size()) return makeError(ErrorCode::InvalidArgument, "exportStep: node index out of range");
    if (c.visiting[i]) return makeError(ErrorCode::InvalidArgument, "exportStep: assembly contains a cycle");
    const AssemblyNode& n = c.in->nodes[i];
    if (n.children.empty()) {
        if (n.shape.isNull()) return makeError(ErrorCode::InvalidArgument, "exportStep: leaf node '" + n.name + "' has no shape");
        if (!rootTrsf) {
            if (auto it = c.protoOf.find(n.shape.impl()); it != c.protoOf.end()) return it->second;
        }
        TopoDS_Shape sh = occtShape(n.shape);
        if (rootTrsf && !(n.local == Transform{})) sh = sh.Moved(TopLoc_Location(*rootTrsf));
        TDF_Label l = c.st->AddShape(sh, Standard_False, Standard_False);
        setName(l, n.name);
        setColor(c, l, n.color);
        if (!rootTrsf) c.protoOf.emplace(n.shape.impl(), l);
        return l;
    }
    c.visiting[i] = 1;
    TDF_Label asmLabel = c.st->NewShape();
    setName(asmLabel, n.name);
    setColor(c, asmLabel, n.color);
    auto addChild = [&](std::uint32_t ci, const gp_Trsf& base) -> Result<void> {
        if (ci >= c.in->nodes.size()) return makeError(ErrorCode::InvalidArgument, "exportStep: child index out of range");
        const AssemblyNode& ch = c.in->nodes[ci];
        auto proto = buildProto(c, ci, nullptr);
        if (!proto) return std::unexpected(proto.error());
        gp_Trsf t = base;
        t.Multiply(toTrsf(ch.local));
        const bool ident = ch.local == Transform{} && !rootTrsf;
        TDF_Label comp = c.st->AddComponent(asmLabel, *proto, ident ? TopLoc_Location() : TopLoc_Location(t));
        if (comp.IsNull()) return makeError(ErrorCode::KernelFailure, "exportStep: could not add component '" + ch.name + "'");
        setName(comp, ch.name);
        setColor(c, comp, ch.color);
        return {};
    };
    gp_Trsf base;
    if (rootTrsf) base = *rootTrsf;
    if (!n.shape.isNull()) { // node with both geometry and children: geometry becomes an extra leaf component
        TDF_Label l = c.st->AddShape(occtShape(n.shape), Standard_False, Standard_False);
        setName(l, n.name);
        c.st->AddComponent(asmLabel, l, TopLoc_Location(base));
    }
    for (std::uint32_t ci : n.children) {
        auto r = addChild(ci, base);
        if (!r) { c.visiting[i] = 0; return std::unexpected(r.error()); }
    }
    c.visiting[i] = 0;
    return asmLabel;
}

} // namespace

Result<void> exportStep(const fs::path& path, const ImportedAssembly& assembly) {
    std::lock_guard lock(exchangeMutex());
    if (path.empty()) return makeError(ErrorCode::InvalidArgument, "exportStep: empty path");
    if (assembly.roots.empty()) return makeError(ErrorCode::InvalidArgument, "exportStep: assembly has no roots");
    try {
        configureStepUnits();
        Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
        Handle(TDocStd_Document) doc;
        app->NewDocument("MDTV-XCAF", doc);
        XCAFDoc_DocumentTool::SetLengthUnit(doc, 1.0, UnitsMethods_LengthUnit_Millimeter);

        ExportCtx ctx;
        ctx.st = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
        ctx.ct = XCAFDoc_DocumentTool::ColorTool(doc->Main());
        ctx.in = &assembly;
        ctx.visiting.assign(assembly.nodes.size(), 0);
        for (std::uint32_t r : assembly.roots) {
            if (r >= assembly.nodes.size()) return makeError(ErrorCode::InvalidArgument, "exportStep: root index out of range");
            const gp_Trsf t = toTrsf(assembly.nodes[r].local);
            auto lab = buildProto(ctx, r, &t);
            if (!lab) return std::unexpected(lab.error());
        }
        ctx.st->UpdateAssemblies();

        STEPCAFControl_Writer w;
        w.SetColorMode(Standard_True);
        w.SetNameMode(Standard_True);
        w.SetLayerMode(Standard_False);
        w.SetPropsMode(Standard_False);
        if (!w.Transfer(doc, STEPControl_AsIs)) return makeError(ErrorCode::KernelFailure, "exportStep: STEP transfer failed");
        if (w.Write(path.string().c_str()) != IFSelect_RetDone)
            return makeError(ErrorCode::IoError, "exportStep: cannot write '" + path.string() + "'");
        app->Close(doc);
        return {};
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(ErrorCode::KernelFailure, e, "exportStep"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("exportStep: ") + e.what());
    }
}

Result<void> exportStep(const fs::path& path, const Shape& shape) {
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "exportStep: null shape");
    ImportedAssembly a;
    AssemblyNode n;
    n.name = "Part";
    n.shape = shape;
    a.nodes.push_back(std::move(n));
    a.roots.push_back(0);
    return exportStep(path, a);
}

// ---------------------------------------------------------------------------------------------- STL

Result<Shape> importStl(const fs::path& path, CancelToken cancel) {
    std::lock_guard lock(exchangeMutex());
    std::error_code ec;
    if (path.empty() || !fs::is_regular_file(path, ec))
        return makeError(ErrorCode::IoError, "importStl: cannot open '" + path.string() + "'");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "importStl cancelled");
    try {
        Handle(Message_ProgressIndicator) pi = new CancelProgress(cancel);
        Handle(Poly_Triangulation) tri = RWStl::ReadFile(path.string().c_str(), pi->Start());
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "importStl cancelled");
        if (tri.IsNull() || tri->NbTriangles() == 0)
            return makeError(ErrorCode::ParseError, "importStl: '" + path.string() + "' is not a readable STL file");

        BRepBuilderAPI_Sewing sewing(1e-6);
        sewing.SetNonManifoldMode(Standard_False);
        int added = 0;
        for (int t = 1; t <= tri->NbTriangles(); ++t) {
            if ((t & 0x3ff) == 0 && cancel.cancelled()) return makeError(ErrorCode::Cancelled, "importStl cancelled");
            int a, b, c;
            tri->Triangle(t).Get(a, b, c);
            const gp_Pnt p1 = tri->Node(a), p2 = tri->Node(b), p3 = tri->Node(c);
            BRepBuilderAPI_MakePolygon poly(p1, p2, p3, Standard_True);
            if (!poly.IsDone()) continue; // degenerate facet
            BRepBuilderAPI_MakeFace face(poly.Wire(), Standard_True);
            if (!face.IsDone()) continue;
            sewing.Add(face.Face());
            ++added;
        }
        if (added == 0) return makeError(ErrorCode::ParseError, "importStl: no valid facets");
        sewing.Perform();
        TopoDS_Shape res = sewing.SewedShape();
        if (res.IsNull()) return makeError(ErrorCode::KernelFailure, "importStl: sewing failed");
        // Closed single shell -> solid (orientation fixed so the volume is positive).
        if (res.ShapeType() == TopAbs_SHELL && BRep_Tool::IsClosed(res)) {
            BRepBuilderAPI_MakeSolid ms(TopoDS::Shell(res));
            if (ms.IsDone()) {
                TopoDS_Shape solid = ms.Solid();
                GProp_GProps p;
                BRepGProp::VolumeProperties(solid, p);
                if (p.Mass() < 0) solid.Reverse();
                res = solid;
            }
        }
        return wrapShape(res);
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(ErrorCode::ParseError, e, "importStl"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("importStl: ") + e.what());
    }
}

Result<void> exportStl(const fs::path& path, const Shape& shape, const StlOptions& options, CancelToken cancel) {
    std::lock_guard lock(exchangeMutex());
    if (shape.isNull()) return makeError(ErrorCode::InvalidArgument, "exportStl: null shape");
    if (path.empty()) return makeError(ErrorCode::InvalidArgument, "exportStl: empty path");
    if (!(options.linearDeflection > 0) || !(options.angularDeflection > 0))
        return makeError(ErrorCode::InvalidArgument, "exportStl: deflections must be > 0");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "exportStl cancelled");
    try {
        // Meshing writes triangulations into the TShape; mesh a private copy so the immutable Shape stays untouched.
        BRepBuilderAPI_Copy copy(occtShape(shape), /*copyGeom=*/Standard_False, /*copyMesh=*/Standard_False);
        const TopoDS_Shape work = copy.Shape();
        Handle(Message_ProgressIndicator) pi = new CancelProgress(cancel);
        Message_ProgressScope scope(pi->Start(), "stl", 2);
        BRepMesh_IncrementalMesh mesh(work, options.linearDeflection, Standard_False, options.angularDeflection,
                                      /*isInParallel=*/Standard_True);
        mesh.Perform(scope.Next());
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "exportStl cancelled");
        StlAPI_Writer w;
        w.ASCIIMode() = !options.binary;
        if (!w.Write(work, path.string().c_str(), scope.Next())) {
            if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "exportStl cancelled");
            return makeError(ErrorCode::IoError, "exportStl: cannot write '" + path.string() + "'");
        }
        return {};
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(ErrorCode::KernelFailure, e, "exportStl"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("exportStl: ") + e.what());
    }
}

// ---- 3MF export ------------------------------------------------------------------------------------------------

namespace {

constexpr const char* kCoreNs = "http://schemas.microsoft.com/3dmanufacturing/core/2015/02";
constexpr const char* kMaterialNs = "http://schemas.microsoft.com/3dmanufacturing/material/2015/02";
constexpr const char* kModelPart = "3D/3dmodel.model";

std::uint32_t crc32Of(const std::string& data) {
    static const std::array<std::uint32_t, 256> table = [] {
        std::array<std::uint32_t, 256> t{};
        for (std::uint32_t i = 0; i < 256; ++i) {
            std::uint32_t c = i;
            for (int k = 0; k < 8; ++k) c = (c & 1u) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            t[i] = c;
        }
        return t;
    }();
    std::uint32_t crc = 0xFFFFFFFFu;
    for (const char ch : data) crc = table[(crc ^ static_cast<unsigned char>(ch)) & 0xFFu] ^ (crc >> 8);
    return crc ^ 0xFFFFFFFFu;
}

void put16(std::string& out, std::uint32_t v) {
    out.push_back(static_cast<char>(v & 0xFFu));
    out.push_back(static_cast<char>((v >> 8) & 0xFFu));
}
void put32(std::string& out, std::uint32_t v) {
    put16(out, v & 0xFFFFu);
    put16(out, v >> 16);
}

struct ZipEntry {
    std::string name;
    std::string data;
};

/// Minimal ZIP writer (PKWARE APPNOTE 6.3): STORED entries, CRC-32 per entry, UTF-8 names, no zip64 (a 3MF
/// package is far below 4 GiB). Local headers, then the central directory, then the end record.
std::string buildZip(const std::vector<ZipEntry>& entries) {
    constexpr std::uint32_t kDosTime = 0;
    constexpr std::uint32_t kDosDate = (0u << 9) | (1u << 5) | 1u; // 1980-01-01
    constexpr std::uint32_t kVersion = 20;                        // 2.0: needed for STORED
    constexpr std::uint32_t kFlagsUtf8 = 0x0800u;
    std::string out;
    std::string central;
    for (const ZipEntry& e : entries) {
        const auto offset = static_cast<std::uint32_t>(out.size());
        const std::uint32_t crc = crc32Of(e.data);
        const auto size = static_cast<std::uint32_t>(e.data.size());
        const auto nameLen = static_cast<std::uint32_t>(e.name.size());

        put32(out, 0x04034b50u);
        put16(out, kVersion);
        put16(out, kFlagsUtf8);
        put16(out, 0); // method: stored
        put16(out, kDosTime);
        put16(out, kDosDate);
        put32(out, crc);
        put32(out, size);
        put32(out, size);
        put16(out, nameLen);
        put16(out, 0); // extra length
        out += e.name;
        out += e.data;

        put32(central, 0x02014b50u);
        put16(central, kVersion); // made by
        put16(central, kVersion); // needed
        put16(central, kFlagsUtf8);
        put16(central, 0);
        put16(central, kDosTime);
        put16(central, kDosDate);
        put32(central, crc);
        put32(central, size);
        put32(central, size);
        put16(central, nameLen);
        put16(central, 0); // extra
        put16(central, 0); // comment
        put16(central, 0); // disk
        put16(central, 0); // internal attributes
        put32(central, 0); // external attributes
        put32(central, offset);
        central += e.name;
    }
    const auto cdOffset = static_cast<std::uint32_t>(out.size());
    const auto count = static_cast<std::uint32_t>(entries.size());
    out += central;
    put32(out, 0x06054b50u);
    put16(out, 0);
    put16(out, 0);
    put16(out, count);
    put16(out, count);
    put32(out, static_cast<std::uint32_t>(central.size()));
    put32(out, cdOffset);
    put16(out, 0); // comment length
    return out;
}

std::string xmlEscape(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    for (const char c : s) {
        switch (c) {
        case '&': out += "&amp;"; break;
        case '<': out += "&lt;"; break;
        case '>': out += "&gt;"; break;
        case '"': out += "&quot;"; break;
        case '\'': out += "&apos;"; break;
        default: out.push_back(c);
        }
    }
    return out;
}

std::string hexColor(const std::optional<Color>& color) {
    const Color c = color.value_or(Color{0.8f, 0.8f, 0.8f, 1.0f});
    auto channel = [](float v) {
        return static_cast<unsigned>(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
    };
    char buf[16];
    std::snprintf(buf, sizeof buf, "#%02X%02X%02XFF", channel(c.r), channel(c.g), channel(c.b));
    return buf;
}

/// Triangulated body in export coordinates (mm). Vertices are per face (not welded), like tess::tessellate.
struct BodyMesh {
    std::vector<double> xyz; ///< 3 values per vertex
    std::vector<std::array<std::uint32_t, 3>> triangles;
};

Result<BodyMesh> meshForExport(const Shape& shape, const StlOptions& options, const CancelToken& cancel) {
    // Meshing writes triangulations into the TShape; mesh a private copy so the immutable Shape stays untouched.
    BRepBuilderAPI_Copy copy(occtShape(shape), /*copyGeom=*/Standard_False, /*copyMesh=*/Standard_False);
    const TopoDS_Shape work = copy.Shape();
    BRepMesh_IncrementalMesh mesh(work, options.linearDeflection, Standard_False, options.angularDeflection,
                                  /*isInParallel=*/Standard_True);
    BodyMesh out;
    for (TopExp_Explorer ex(work, TopAbs_FACE); ex.More(); ex.Next()) {
        if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "export3mf cancelled");
        const TopoDS_Face face = TopoDS::Face(ex.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) return makeError(ErrorCode::KernelFailure, "export3mf: a face failed to mesh");

        // Same winding rule as tess::tessellate: a REVERSED face or a mirroring location flips the triangles.
        const gp_Trsf trsf = loc.Transformation();
        const bool flip = (face.Orientation() == TopAbs_REVERSED) != trsf.IsNegative();
        const auto base = static_cast<std::uint32_t>(out.xyz.size() / 3);
        for (Standard_Integer k = 1; k <= tri->NbNodes(); ++k) {
            const gp_Pnt p = tri->Node(k).Transformed(trsf);
            out.xyz.push_back(p.X());
            out.xyz.push_back(p.Y());
            out.xyz.push_back(p.Z());
        }
        for (Standard_Integer t = 1; t <= tri->NbTriangles(); ++t) {
            Standard_Integer a = 0, b = 0, c = 0;
            tri->Triangle(t).Get(a, b, c);
            if (flip) std::swap(b, c);
            out.triangles.push_back({base + static_cast<std::uint32_t>(a - 1), base + static_cast<std::uint32_t>(b - 1),
                                     base + static_cast<std::uint32_t>(c - 1)});
        }
    }
    if (out.triangles.empty()) return makeError(ErrorCode::KernelFailure, "export3mf: body has no triangles");
    return out;
}

/// 3D/3dmodel.model. Resource ids: 1 = basematerials group (one base per body, pindex = body index),
/// 2..N+1 = objects, build items reference the objects with identity transforms.
std::string buildModelXml(const std::vector<MeshBody>& bodies, const std::vector<BodyMesh>& meshes) {
    std::string x = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    x += std::string("<model unit=\"millimeter\" xml:lang=\"en-US\" xmlns=\"") + kCoreNs + "\" xmlns:m=\"" +
         kMaterialNs + "\">\n";
    x += " <resources>\n  <m:basematerials id=\"1\">\n";
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const std::string name = bodies[i].name.empty() ? "Body " + std::to_string(i + 1) : bodies[i].name;
        x += "   <m:base name=\"" + xmlEscape(name) + "\" displaycolor=\"" + hexColor(bodies[i].color) + "\"/>\n";
    }
    x += "  </m:basematerials>\n";

    char num[3][64];
    for (std::size_t i = 0; i < bodies.size(); ++i) {
        const std::string name = bodies[i].name.empty() ? "Body " + std::to_string(i + 1) : bodies[i].name;
        const BodyMesh& m = meshes[i];
        x += "  <object id=\"" + std::to_string(i + 2) + "\" name=\"" + xmlEscape(name) +
             "\" type=\"model\" pid=\"1\" pindex=\"" + std::to_string(i) + "\">\n   <mesh>\n    <vertices>\n";
        for (std::size_t v = 0; v + 2 < m.xyz.size(); v += 3) {
            std::snprintf(num[0], sizeof num[0], "%.9g", m.xyz[v]);
            std::snprintf(num[1], sizeof num[1], "%.9g", m.xyz[v + 1]);
            std::snprintf(num[2], sizeof num[2], "%.9g", m.xyz[v + 2]);
            x += std::string("     <vertex x=\"") + num[0] + "\" y=\"" + num[1] + "\" z=\"" + num[2] + "\"/>\n";
        }
        x += "    </vertices>\n    <triangles>\n";
        for (const auto& t : m.triangles)
            x += "     <triangle v1=\"" + std::to_string(t[0]) + "\" v2=\"" + std::to_string(t[1]) + "\" v3=\"" +
                 std::to_string(t[2]) + "\"/>\n";
        x += "    </triangles>\n   </mesh>\n  </object>\n";
    }
    x += " </resources>\n <build>\n";
    for (std::size_t i = 0; i < bodies.size(); ++i)
        x += "  <item objectid=\"" + std::to_string(i + 2) + "\"/>\n";
    x += " </build>\n</model>\n";
    return x;
}

constexpr const char* kContentTypesXml =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<Types xmlns=\"http://schemas.openxmlformats.org/package/2006/content-types\">"
    "<Default Extension=\"rels\" ContentType=\"application/vnd.openxmlformats-package.relationships+xml\"/>"
    "<Default Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\"/>"
    "</Types>\n";

constexpr const char* kRelsXml =
    "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    "<Relationships xmlns=\"http://schemas.openxmlformats.org/package/2006/relationships\">"
    "<Relationship Target=\"/3D/3dmodel.model\" Id=\"rel0\" "
    "Type=\"http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel\"/>"
    "</Relationships>\n";

} // namespace

Result<void> export3mf(const fs::path& path, const std::vector<MeshBody>& bodies, const StlOptions& options,
                       CancelToken cancel) {
    std::lock_guard lock(exchangeMutex());
    if (path.empty()) return makeError(ErrorCode::InvalidArgument, "export3mf: empty path");
    if (bodies.empty()) return makeError(ErrorCode::InvalidArgument, "export3mf: no bodies");
    for (const MeshBody& b : bodies)
        if (b.shape.isNull())
            return makeError(ErrorCode::InvalidArgument, "export3mf: null shape in body '" + b.name + "'");
    if (!(options.linearDeflection > 0) || !(options.angularDeflection > 0))
        return makeError(ErrorCode::InvalidArgument, "export3mf: deflections must be > 0");
    if (cancel.cancelled()) return makeError(ErrorCode::Cancelled, "export3mf cancelled");
    try {
        std::vector<BodyMesh> meshes;
        meshes.reserve(bodies.size());
        for (const MeshBody& b : bodies) {
            Result<BodyMesh> m = meshForExport(b.shape, options, cancel);
            if (!m) return std::unexpected(m.error());
            meshes.push_back(std::move(*m));
        }
        const std::string bytes = buildZip({
            {"[Content_Types].xml", kContentTypesXml},
            {"_rels/.rels", kRelsXml},
            {kModelPart, buildModelXml(bodies, meshes)},
        });
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        if (!out) return makeError(ErrorCode::IoError, "export3mf: cannot write '" + path.string() + "'");
        out.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
        out.close();
        if (!out) return makeError(ErrorCode::IoError, "export3mf: failed writing '" + path.string() + "'");
        return {};
    } catch (const Standard_Failure& e) {
        return std::unexpected(occtError(ErrorCode::KernelFailure, e, "export3mf"));
    } catch (const std::exception& e) {
        return makeError(ErrorCode::Internal, std::string("export3mf: ") + e.what());
    }
}

} // namespace iw::kernel
