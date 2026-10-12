// SPDX-License-Identifier: GPL-3.0-or-later
// Region -> kernel face. OCCT is used here only; the public header stays OCCT-free.

#include <cmath>
#include <numbers>
#include <string>

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <Standard_Failure.hxx>
#include <ShapeFix_Face.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include "kernel/occt_access.hpp"
#include "sketch/profile.hpp"

namespace iw::sketch {

namespace {

gp_Pnt toPnt(const SketchPlane& plane, Point2 p) {
    const Vec3 w = plane.toWorld(p);
    return {w.x, w.y, w.z};
}

std::unexpected<Error> kernelFailure(const std::string& what) {
    return makeError(ErrorCode::KernelFailure, "profileFace: " + what);
}

Result<TopoDS_Wire> loopWire(const SketchPlane& plane, const ProfileLoop& loop) {
    const gp_Dir n(plane.normal.x, plane.normal.y, plane.normal.z);
    const gp_Dir x(plane.xDir.x, plane.xDir.y, plane.xDir.z);
    BRepBuilderAPI_MakeWire mw;
    for (const ProfileEdge& e : loop.edges) {
        TopoDS_Edge edge;
        if (!e.isArc) {
            BRepBuilderAPI_MakeEdge me(toPnt(plane, e.start), toPnt(plane, e.end));
            if (!me.IsDone()) return kernelFailure("line edge failed");
            edge = me.Edge();
        } else {
            const gp_Circ circ(gp_Ax2(toPnt(plane, e.center), n, x), e.radius);
            const double twoPi = 2.0 * std::numbers::pi;
            const double absSweep = std::abs(e.sweep);
            if (absSweep >= twoPi - 1e-9) {
                BRepBuilderAPI_MakeEdge me(circ);
                if (!me.IsDone()) return kernelFailure("circle edge failed");
                edge = e.sweep < 0 ? TopoDS::Edge(me.Edge().Reversed()) : me.Edge();
            } else if (e.sweep > 0) {
                // OCCT builds the counter-clockwise arc from the first point to the second.
                BRepBuilderAPI_MakeEdge me(circ, toPnt(plane, e.start), toPnt(plane, e.end));
                if (!me.IsDone()) return kernelFailure("arc edge failed");
                edge = me.Edge();
            } else {
                BRepBuilderAPI_MakeEdge me(circ, toPnt(plane, e.end), toPnt(plane, e.start));
                if (!me.IsDone()) return kernelFailure("arc edge failed");
                edge = TopoDS::Edge(me.Edge().Reversed());
            }
        }
        mw.Add(edge);
        if (!mw.IsDone()) return kernelFailure("loop is not a connected wire");
    }
    return mw.Wire();
}

} // namespace

Result<kernel::Shape> profileFace(const SketchPlane& plane, const SketchProfile& profile) {
    try {
        Result<TopoDS_Wire> outer = loopWire(plane, profile.outer);
        if (!outer) return std::unexpected(outer.error());
        const gp_Pnt o(plane.origin.x, plane.origin.y, plane.origin.z);
        const gp_Ax3 ax(o, gp_Dir(plane.normal.x, plane.normal.y, plane.normal.z),
                        gp_Dir(plane.xDir.x, plane.xDir.y, plane.xDir.z));
        BRepBuilderAPI_MakeFace mf(gp_Pln(ax), *outer, Standard_True);
        if (!mf.IsDone()) return kernelFailure("outer loop is not planar");
        for (const ProfileLoop& hole : profile.holes) {
            Result<TopoDS_Wire> hw = loopWire(plane, hole);
            if (!hw) return std::unexpected(hw.error());
            mf.Add(*hw);
        }
        ShapeFix_Face fix(mf.Face());
        fix.FixOrientation();
        fix.Perform();
        return kernel::occt::wrap(fix.Face());
    } catch (const Standard_Failure& f) {
        return kernelFailure(std::string("OCCT failure: ") + (f.GetMessageString() ? f.GetMessageString() : "unknown"));
    }
}

} // namespace iw::sketch
