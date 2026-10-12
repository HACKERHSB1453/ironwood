// SPDX-License-Identifier: GPL-3.0-or-later
#include "kernel/shape.hpp"

#include <Bnd_Box.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
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
