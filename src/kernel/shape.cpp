// SPDX-License-Identifier: GPL-3.0-or-later
#include "kernel/shape.hpp"

#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GeomAbs_CurveType.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Lin.hxx>
#include <gp_Vec.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <algorithm>
#include <cmath>
#include <gp_Dir.hxx>
#include <Standard_Failure.hxx>
#include <Standard_Version.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopoDS_Shape.hxx>

#include "kernel/internal/shape_impl.hpp"
#include "kernel/occt_access.hpp"

namespace iw::kernel {

Shape::Shape() = default;
Shape::~Shape() = default;
Shape::Shape(const Shape&) = default;
Shape::Shape(Shape&&) noexcept = default;
Shape& Shape::operator=(const Shape&) = default;
Shape& Shape::operator=(Shape&&) noexcept = default;
Shape::Shape(std::shared_ptr<const Impl> impl) noexcept : impl_(std::move(impl)) {}

bool Shape::isNull() const noexcept { return !impl_ || impl_->shape.IsNull(); }

ShapeType Shape::type() const noexcept {
    if (isNull()) return ShapeType::Null;
    switch (impl_->shape.ShapeType()) {
    case TopAbs_COMPOUND: return ShapeType::Compound;
    case TopAbs_COMPSOLID: return ShapeType::CompSolid;
    case TopAbs_SOLID: return ShapeType::Solid;
    case TopAbs_SHELL: return ShapeType::Shell;
    case TopAbs_FACE: return ShapeType::Face;
    case TopAbs_WIRE: return ShapeType::Wire;
    case TopAbs_EDGE: return ShapeType::Edge;
    case TopAbs_VERTEX: return ShapeType::Vertex;
    default: return ShapeType::Null;
    }
}

ShapeStamp Shape::stamp() const noexcept { return impl_ ? impl_->stamp : ShapeStamp{}; }

Shape Shape::withStamp(ShapeStamp s) const {
    if (!impl_) return {};
    auto copy = std::make_shared<Impl>(*impl_);
    copy->stamp = s;
    return Shape(std::move(copy));
}

std::uint32_t Shape::subShapeCount(SubShapeKind kind) const {
    if (isNull()) return 0;
    return static_cast<std::uint32_t>(impl_->index->map(kind).Extent());
}

FaceGeometry Shape::faceGeometry(std::uint32_t index) const {
    FaceGeometry g;
    if (isNull() || index >= subShapeCount(SubShapeKind::Face)) return g;
    const TopoDS_Face face = TopoDS::Face(impl_->index->faces.FindKey(static_cast<int>(index) + 1));
    const bool reversed = face.Orientation() == TopAbs_REVERSED;
    const auto toVec = [](const gp_Dir& d) { return Vec3{d.X(), d.Y(), d.Z()}; };
    const auto toPnt = [](const gp_Pnt& p) { return Vec3{p.X(), p.Y(), p.Z()}; };
    try {
        BRepAdaptor_Surface surf(face);
        switch (surf.GetType()) {
        case GeomAbs_Plane: {
            const gp_Pln pl = surf.Plane();
            g.kind = FaceGeometry::Kind::Plane;
            g.point = toPnt(pl.Location());
            g.normal = toVec(pl.Axis().Direction());
            if (reversed) g.normal = g.normal * -1.0;
            break;
        }
        case GeomAbs_Cylinder: {
            const gp_Cylinder cy = surf.Cylinder();
            g.kind = FaceGeometry::Kind::Cylinder;
            g.point = toPnt(cy.Location());
            g.direction = toVec(cy.Axis().Direction());
            g.radius = cy.Radius();
            break;
        }
        case GeomAbs_Cone: {
            const gp_Cone co = surf.Cone();
            g.kind = FaceGeometry::Kind::Cone;
            g.point = toPnt(co.Location());
            g.direction = toVec(co.Axis().Direction());
            g.radius = co.RefRadius();
            break;
        }
        default:
            g.point = toPnt(surf.Value(surf.FirstUParameter(), surf.FirstVParameter()));
            break;
        }
    } catch (const Standard_Failure&) {
        return FaceGeometry{};
    }
    return g;
}

EdgeGeometry Shape::edgeGeometry(std::uint32_t index) const {
    EdgeGeometry g;
    if (isNull() || index >= subShapeCount(SubShapeKind::Edge)) return g;
    const auto toVec = [](const gp_Dir& d) { return Vec3{d.X(), d.Y(), d.Z()}; };
    const auto toPnt = [](const gp_Pnt& p) { return Vec3{p.X(), p.Y(), p.Z()}; };
    try {
        // Forward copy: the edge's own parameterisation (F42), independent of how the face that stored it first uses it.
        // BRepAdaptor_Curve applies the edge Location, so every value below is in world coordinates.
        const TopoDS_Edge edge = TopoDS::Edge(impl_->index->edges.FindKey(static_cast<int>(index) + 1).Oriented(TopAbs_FORWARD));
        if (BRep_Tool::Degenerated(edge) || !BRep_Tool::IsGeometric(edge)) {
            // F31 contract: kind Other, no point. F42: where the edge is (its vertex positions).
            const TopoDS_Vertex v0 = TopExp::FirstVertex(edge, /*CumOri=*/Standard_False);
            const TopoDS_Vertex v1 = TopExp::LastVertex(edge, /*CumOri=*/Standard_False);
            g.degenerate = true;
            if (!v0.IsNull()) g.start = toPnt(BRep_Tool::Pnt(v0));
            g.end = v1.IsNull() ? g.start : toPnt(BRep_Tool::Pnt(v1));
            return g;
        }
        BRepAdaptor_Curve curve(edge);
        const double u0 = curve.FirstParameter();
        const double u1 = curve.LastParameter();
        const auto tangentAt = [&](double u, Vec3& p) {
            gp_Pnt pt;
            gp_Vec d1;
            curve.D1(u, pt, d1);
            p = toPnt(pt);
            const double m = d1.Magnitude();
            return m > 1e-12 ? Vec3{d1.X() / m, d1.Y() / m, d1.Z() / m} : Vec3{};
        };
        g.startTangent = tangentAt(u0, g.start);
        g.endTangent = tangentAt(u1, g.end);
        g.point = g.start;
        g.closed = length(g.end - g.start) <= std::max(BRep_Tool::Tolerance(edge), 1e-7);
        g.length = GCPnts_AbscissaPoint::Length(curve);
        g.degenerate = g.length <= 1e-9;
        switch (curve.GetType()) {
        case GeomAbs_Line:
            g.kind = EdgeGeometry::Kind::Line;
            g.direction = toVec(curve.Line().Direction());
            break;
        case GeomAbs_Circle: {
            const gp_Circ c = curve.Circle();
            g.kind = EdgeGeometry::Kind::Circle;
            g.point = toPnt(c.Location());
            g.direction = toVec(c.Axis().Direction());
            g.radius = c.Radius();
            break;
        }
        case GeomAbs_Ellipse: {
            const gp_Elips e = curve.Ellipse();
            g.kind = EdgeGeometry::Kind::Ellipse;
            g.point = toPnt(e.Location());
            g.direction = toVec(e.Axis().Direction());
            g.radius = e.MajorRadius();
            g.minorRadius = e.MinorRadius();
            break;
        }
        case GeomAbs_BSplineCurve:
        case GeomAbs_BezierCurve: g.kind = EdgeGeometry::Kind::BSpline; break;
        default: break;
        }
    } catch (const Standard_Failure&) {
        return EdgeGeometry{};
    }
    return g;
}

std::vector<Shape> Shape::solids() const {
    std::vector<Shape> out;
    if (isNull()) return out;
    TopTools_IndexedMapOfShape map;
    TopExp::MapShapes(impl_->shape, TopAbs_SOLID, map);
    out.reserve(static_cast<std::size_t>(map.Extent()));
    for (int i = 1; i <= map.Extent(); ++i) out.push_back(wrapShape(map.FindKey(i)));
    return out;
}

namespace {

TopAbs_ShapeEnum occtKind(SubShapeKind k) {
    switch (k) {
    case SubShapeKind::Face: return TopAbs_FACE;
    case SubShapeKind::Edge: return TopAbs_EDGE;
    default: return TopAbs_VERTEX;
    }
}

/// Indices (in `to`) of the `toKind` sub-shapes contained in `sub`, sorted and unique.
std::vector<std::uint32_t> contained(const TopoDS_Shape& sub, TopAbs_ShapeEnum toKind, const TopTools_IndexedMapOfShape& to) {
    std::vector<std::uint32_t> out;
    for (TopExp_Explorer ex(sub, toKind); ex.More(); ex.Next()) {
        const int i = to.FindIndex(ex.Current());
        if (i > 0) out.push_back(static_cast<std::uint32_t>(i - 1));
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

} // namespace

std::vector<std::vector<std::uint32_t>> Shape::adjacency(SubShapeKind from, SubShapeKind to) const {
    if (isNull()) return {};
    const auto& fromMap = impl_->index->map(from);
    const auto& toMap = impl_->index->map(to);
    std::vector<std::vector<std::uint32_t>> out(static_cast<std::size_t>(fromMap.Extent()));
    try {
        if (from == to) {
            for (std::uint32_t i = 0; i < out.size(); ++i) out[i] = {i};
        } else if (static_cast<int>(from) < static_cast<int>(to)) { // higher dimension -> its boundary elements
            for (int i = 1; i <= fromMap.Extent(); ++i)
                out[static_cast<std::size_t>(i - 1)] = contained(fromMap.FindKey(i), occtKind(to), toMap);
        } else { // lower dimension -> the elements it bounds: invert the containment table
            for (int j = 1; j <= toMap.Extent(); ++j)
                for (std::uint32_t i : contained(toMap.FindKey(j), occtKind(from), fromMap))
                    out[i].push_back(static_cast<std::uint32_t>(j - 1)); // j ascending -> each list stays sorted/unique
        }
    } catch (const Standard_Failure&) {
        return {};
    }
    return out;
}

std::vector<std::uint32_t> Shape::edgeFaces(std::uint32_t edge) const {
    if (isNull() || edge >= subShapeCount(SubShapeKind::Edge)) return {};
    std::vector<std::uint32_t> out;
    try {
        const TopoDS_Shape& e = impl_->index->edges.FindKey(static_cast<int>(edge) + 1);
        const auto& faces = impl_->index->faces;
        for (int f = 1; f <= faces.Extent(); ++f)
            for (TopExp_Explorer ex(faces.FindKey(f), TopAbs_EDGE); ex.More(); ex.Next())
                if (ex.Current().IsSame(e)) {
                    out.push_back(static_cast<std::uint32_t>(f - 1));
                    break;
                }
    } catch (const Standard_Failure&) {
        return {};
    }
    return out;
}

std::vector<std::uint32_t> Shape::faceEdges(std::uint32_t face) const {
    if (isNull() || face >= subShapeCount(SubShapeKind::Face)) return {};
    try {
        return contained(impl_->index->faces.FindKey(static_cast<int>(face) + 1), TopAbs_EDGE, impl_->index->edges);
    } catch (const Standard_Failure&) {
        return {};
    }
}

std::vector<std::uint32_t> Shape::edgeVertices(std::uint32_t edge) const {
    if (isNull() || edge >= subShapeCount(SubShapeKind::Edge)) return {};
    std::vector<std::uint32_t> out;
    try {
        const TopoDS_Edge e = TopoDS::Edge(impl_->index->edges.FindKey(static_cast<int>(edge) + 1).Oriented(TopAbs_FORWARD));
        const auto& verts = impl_->index->vertices;
        for (const TopoDS_Vertex& v : {TopExp::FirstVertex(e, Standard_False), TopExp::LastVertex(e, Standard_False)}) {
            if (v.IsNull()) continue;
            const int i = verts.FindIndex(v);
            if (i > 0 && (out.empty() || out.front() != static_cast<std::uint32_t>(i - 1))) out.push_back(static_cast<std::uint32_t>(i - 1));
        }
    } catch (const Standard_Failure&) {
        return {};
    }
    return out;
}

std::vector<std::uint32_t> Shape::vertexEdges(std::uint32_t vertex) const {
    if (isNull() || vertex >= subShapeCount(SubShapeKind::Vertex)) return {};
    std::vector<std::uint32_t> out;
    try {
        const TopoDS_Shape& v = impl_->index->vertices.FindKey(static_cast<int>(vertex) + 1);
        const auto& edges = impl_->index->edges;
        for (int e = 1; e <= edges.Extent(); ++e)
            for (TopExp_Explorer ex(edges.FindKey(e), TopAbs_VERTEX); ex.More(); ex.Next())
                if (ex.Current().IsSame(v)) {
                    out.push_back(static_cast<std::uint32_t>(e - 1));
                    break;
                }
    } catch (const Standard_Failure&) {
        return {};
    }
    return out;
}

Box3 Shape::boundingBox() const {
    Box3 out;
    if (isNull()) return out;
    try {
        Bnd_Box box;
        // Tight box from the geometry (no triangulation, no tolerance inflation).
        BRepBndLib::AddOptimal(impl_->shape, box, /*useTriangulation=*/false, /*useShapeTolerance=*/false);
        if (box.IsVoid()) return out;
        const auto [x0, y0, z0, x1, y1, z1] = [&] {
            double a, b, c, d, e, f;
            box.Get(a, b, c, d, e, f);
            return std::array<double, 6>{a, b, c, d, e, f};
        }();
        out.extend({x0, y0, z0});
        out.extend({x1, y1, z1});
    } catch (const Standard_Failure&) {
        return Box3{};
    }
    return out;
}

std::array<double, 4> Shape::subShapeKey(SubShapeKind kind, std::size_t index) const {
    if (isNull()) return {};
    const auto& map = impl_->index->map(kind);
    if (index >= static_cast<std::size_t>(map.Extent())) return {};
    try {
        const TopoDS_Shape& sub = map.FindKey(static_cast<int>(index) + 1);
        gp_Pnt c;
        double measure = 0;
        if (kind == SubShapeKind::Vertex) {
            c = BRep_Tool::Pnt(TopoDS::Vertex(sub));
        } else {
            GProp_GProps props;
            if (kind == SubShapeKind::Face) BRepGProp::SurfaceProperties(sub, props);
            else BRepGProp::LinearProperties(sub, props);
            c = props.CentreOfMass();
            measure = props.Mass();
        }
        auto r = [](double v) {
            const double q = std::round(v * 1e9) / 1e9;
            return q == 0.0 ? 0.0 : q; // normalise -0
        };
        return {r(c.X()), r(c.Y()), r(c.Z()), r(measure)};
    } catch (const Standard_Failure&) {
        return {};
    }
}

bool Shape::isSame(const Shape& o) const noexcept {
    if (!impl_ || !o.impl_) return false;
    // IsEqual = same TShape + same Location + same Orientation (IsSame would ignore orientation).
    return impl_->shape.IsEqual(o.impl_->shape);
}

std::string occtVersion() { return OCC_VERSION_STRING_EXT; }

namespace occt {

const TopoDS_Shape& get(const Shape& s) {
    static const TopoDS_Shape kNull;
    return s.impl() ? s.impl()->shape : kNull;
}

Shape wrap(const TopoDS_Shape& s, ShapeStamp stamp) { return wrapShape(s, stamp); }

} // namespace occt

} // namespace iw::kernel
