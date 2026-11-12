//! Topology of the blend corner's End at a smooth vertex: the blend cut by the plane
//! through the vertex square to its edge.
//!
//! Upstream: none. The fork's `source/SMLib/src/SmBlendCornerEnd.cpp`
//! (tangency-labs/smlib `41a39ec`, smlib#124).
//!
//! | Rust                                         | Fork                                   |
//! |----------------------------------------------|----------------------------------------|
//! | [`FilletCorner::make_end_at_smooth_vertex`]  | `SmBlendCorner::MakeEndAtSmoothVertex` |
//!
//! Deviations: the caller resolves `GetFilletSolverOfEdgeuse` of the blended edge use
//! ([`EndAtSmoothTopologyEnv::solver_eu_solver`]). A missing solver, face or curve the
//! fork dereferences through `NER` is `SM_ERR_NULL_POINTER`; other missing topology is
//! an input error. The cap's plane is kept on the corner ([`FilletCorner::end_cap`],
//! `m_pEndCap`). No recursion (WR-697).

use super::{need, new_corner_vertex, radial};
use crate::blend::fillet_corner::p3::FilletCorner;
use crate::blend::fillet_executive::session::SolverObject;
use crate::blend::fillet_geom::p1::{
    CornerLookup, FilletBrep, FilletCornerId, FilletEdgeType, FilletSolverId, FilletVertexType,
};
use crate::error::{EntityRef, Result, SolidError};
use crate::geom::extent2d::Extent2d;
use crate::intersect::point_class::PointClassType;
use crate::kernel::Kernel;
use crate::math::{Vec2, Vec3};
use crate::nurbs::b_spline_surface_n_lib::p2::OrientType;
use crate::surface::plane::Plane;
use crate::surface::surface::Surface;
use crate::topology::edgeuse::p1 as eu1;
use crate::topology::{edge, vertex, Brep};

const OP: &str = "blend::fillet_corner::topology::end_smooth";

/// Actual objects `SmBlendCorner::MakeEndAtSmoothVertex` uses.
#[derive(Debug)]
pub struct EndAtSmoothTopologyEnv<'a> {
    /// The executive's kernel.
    pub k: &'a Kernel,
    /// The target whose edge uses the corner stores.
    pub target: &'a mut Brep,
    /// The executive's actual pseudo-B-rep.
    pub pseudo: &'a mut FilletBrep,
    /// This corner's identity in the executive's corner store.
    pub corner: FilletCornerId,
    /// The executive's solver store.
    pub solvers: &'a [Option<SolverObject>],
    /// `GetFilletSolverOfEdgeuse` of `(*m_pSolverEUs)[0]` (`None`: `NULL`).
    pub solver_eu_solver: Option<FilletSolverId>,
}

/// Not the fork's (#15212's sibling): the normal of the End's cap at `vertex`. The fork's is
/// the blended edge's `tangent` there. Where a face of the blend (`faces`) ends at the vertex in
/// an edge other than the blended edge `blended` and the edges it runs on into (a shallow
/// crease across the blended edge, its faces a few degrees apart), the cap is turned to hold
/// that edge's tangent: else the rail on that face, ending on a plane square to the blended
/// edge, crosses the edge and ends on the face beyond, which the blend does not touch
/// tangentially, and the faces the rails enclose are not closed. A face end whose faces meet
/// at the vertex within `crease_rad` is tangent, not a crease: the rail rolls over it (#2406),
/// and it is not held. The turn is taken only when it is under `tol_rad` (the corner's
/// tangency tolerance); one face edge is held, or two (the plane through both). Without such
/// an edge, the fork's normal.
///
/// # Errors
/// Those of the edge queries and evaluations.
#[allow(clippy::too_many_arguments)] // the cap's vertex, edge, faces and tolerances
fn cap_normal_through_face_ends(
    k: &Kernel,
    target: &mut Brep,
    vertex: crate::topology::VertexId,
    blended: crate::topology::EdgeId,
    faces: [crate::topology::FaceId; 2],
    tangent: Vec3,
    crease_rad: f64,
    tol_rad: f64,
) -> Result<Vec3> {
    // the tangents at the vertex of the face edges that are neither the blended edge nor one
    // it runs on into (an edge leaving the vertex along the blended edge's line)
    let along = 45.0_f64.to_radians().cos();
    let mut held: Vec<Vec3> = Vec::new();
    for face in faces {
        for e in vertex::edges(target, vertex, Some(face))? {
            if e == blended || !is_crease_at(k, target, vertex, e, crease_rad)? {
                continue;
            }
            let Some(curve) = edge::curve(target, e)? else {
                continue;
            };
            let ivl = edge::interval(target, e)?;
            let at = if edge::start_vertex(target, e)? == Some(vertex) {
                ivl.min()
            } else {
                ivl.max()
            };
            let Some(d) = curve.evaluate(k, at, 1, true, true)?.get(1).copied() else {
                continue;
            };
            let Ok(d) = d.unitize() else {
                continue;
            };
            if d.dot(tangent).abs() >= along {
                continue;
            }
            if !held.iter().any(|h| h.cross(d).length() <= 1e-9) {
                held.push(d);
            }
        }
    }
    let turned = match held.as_slice() {
        [d] => tangent - *d * tangent.dot(*d),
        [d0, d1] => d0.cross(*d1),
        _ => return Ok(tangent),
    };
    let Ok(mut turned) = turned.unitize() else {
        return Ok(tangent);
    };
    if turned.dot(tangent) < 0.0 {
        turned = -turned;
    }
    let angle = turned.angle_between(tangent)?;
    if angle > 1e-12 && angle <= tol_rad {
        Ok(turned)
    } else {
        Ok(tangent)
    }
}

/// Whether edge `e`'s two faces meet at `vertex` at an angle over `angle_rad`: each edge use's
/// binormal there is off square to the other face's normal by more than it. A lamina or
/// non-manifold edge is no crease.
///
/// # Errors
/// Those of the edge queries and the binormal evaluations.
fn is_crease_at(
    k: &Kernel,
    target: &mut Brep,
    vertex: crate::topology::VertexId,
    e: crate::topology::EdgeId,
    angle_rad: f64,
) -> Result<bool> {
    if !edge::is_manifold(target, e)? {
        return Ok(false);
    }
    let ivl = edge::interval(target, e)?;
    let at = if edge::start_vertex(target, e)? == Some(vertex) {
        ivl.min()
    } else {
        ivl.max()
    };
    let Some(eu0) = edge::primary_edgeuse(target, e)? else {
        return Ok(false);
    };
    let Some(eu_other) = eu1::radial(target, eu0)? else {
        return Ok(false);
    };
    let r0 = crate::topology::edgeuse::p2::binormal_ignoring_status(k, target, eu0, at, false)?;
    let r1 =
        crate::topology::edgeuse::p2::binormal_ignoring_status(k, target, eu_other, at, false)?;
    let (Ok(n0), Ok(n1), Ok(b0), Ok(b1)) = (
        r0.face_normal.unitize(),
        r1.face_normal.unitize(),
        r0.binormal.unitize(),
        r1.binormal.unitize(),
    ) else {
        return Ok(false);
    };
    let off = b1.dot(n0).abs().max(b0.dot(n1).abs());
    Ok(off.asin() > angle_rad)
}

impl FilletCorner {
    /// The fork's `SmBlendCorner::MakeEndAtSmoothVertex` (41a39ec): the End at a smooth
    /// vertex (`SM_BEND_AT_SMOOTH`). The blend is cut by the plane P through the vertex
    /// square to its edge there ([`FilletCorner::end_cap`], sized from the side faces'
    /// tight boxes). Each rail ends where it crosses P (`SM_FV_RAIL_X_EXTENDED_EDGEUSE`,
    /// found by `CalcRailIntEndSurface` on the cap); the end rib is the blend's
    /// intersection with P between the rails' ends (`SM_FE_FILLET_X_SIDE_FACE`); the
    /// cap's other two edges are the side faces' traces in P from the rails' ends to the
    /// vertex (`SM_FE_PRECOMPUTED`, `SM_FV_ON_VERTEX`; their curves `CalcEndCapEdges`).
    /// The corner is not blending.
    ///
    /// Allocations and corner-list insertions occur in the fork's order and remain
    /// visible if a later operation fails.
    ///
    /// # Errors
    /// `SM_ERR_NULL_POINTER` without solver, geometry, side faces or edge curve; `SM_ERR`
    /// for a rail index past 1; those of the evaluation, the unitizations, the tight
    /// boxes, the plane's NURB and the pseudo-topology operations.
    #[allow(clippy::too_many_lines)] // one fork routine
    pub fn make_end_at_smooth_vertex(&mut self, env: EndAtSmoothTopologyEnv<'_>) -> Result<()> {
        let EndAtSmoothTopologyEnv {
            k,
            target,
            pseudo,
            corner,
            solvers,
            solver_eu_solver,
        } = env;
        let nul = |what: &'static str| SolidError::blend_missing(OP, what);
        let fil_eu = need(
            self.solver_eus
                .as_deref()
                .and_then(|eus| eus.first())
                .copied(),
            "corner without blended edge use",
        )?;
        let solver = solver_eu_solver
            .and_then(|id| solvers.get(id.0 as usize))
            .and_then(Option::as_ref)
            .map(SolverObject::base)
            .ok_or_else(|| nul("the blended edge's solver"))?;
        let geom = solver.first_fillet_geom()?;
        let faces = [
            eu1::face(target, fil_eu)?.ok_or_else(|| nul("the blended edge use's face"))?,
            eu1::face(target, radial(target, fil_eu)?)?
                .ok_or_else(|| nul("the blended edge's other face"))?,
        ];
        let corner_vertex = need(self.vertex, "corner without vertex")?;

        // P: through the vertex, square to the edge
        let e = eu1::edge(target, fil_eu)?.ok_or_else(|| nul("the blended edge use's edge"))?;
        let ivl = edge::interval(target, e)?;
        let at_start = edge::start_vertex(target, e)? == Some(corner_vertex);
        let curve = edge::curve(target, e)?.ok_or_else(|| nul("the blended edge's curve"))?;
        let pv = curve.evaluate(
            k,
            if at_start { ivl.min() } else { ivl.max() },
            1,
            true,
            true,
        )?;
        let origin = vertex::point(target, corner_vertex)?;
        let tangent = pv
            .get(1)
            .copied()
            .ok_or_else(|| nul("the edge curve's tangent"))?
            .unitize()?;
        // the angle at which a face's end is a crease, not a tangent edge the rail rolls over:
        // the corner's 3D tolerance over the blend's radius
        let mut radius: f64 = 0.0;
        for slot in solver.surfaces.iter().flatten() {
            radius = radius.max(slot.surface.offset_distance().abs());
        }
        let crease_rad = if radius > 0.0 {
            self.approx_tol_3d / radius
        } else {
            self.tangency_tol_radians
        };
        let normal = if k.fixes.fillet_end_cap_holds_crease {
            cap_normal_through_face_ends(
                k,
                target,
                corner_vertex,
                e,
                faces,
                tangent,
                crease_rad,
                self.tangency_tol_radians,
            )?
        } else {
            tangent
        };
        let mut x = if normal.x.abs() < 0.9 {
            Vec3::new(1.0, 0.0, 0.0)
        } else {
            Vec3::new(0.0, 1.0, 0.0)
        };
        x = x - normal * x.dot(normal);
        x = x.unitize()?;
        let y = normal.cross(x).unitize()?;
        // P's extent: past every point of the side faces from the vertex (the blend lies
        // between them)
        let mut reach: f64 = 0.0;
        for &f in &faces {
            let bbox = crate::query::face::p3::calculate_tight_bounding_box(k, &mut *target, f)?;
            reach = reach.max(bbox.min().distance(origin));
            reach = reach.max(bbox.max().distance(origin));
        }
        reach *= 2.0;
        let uv_domain = Extent2d::from_bounds(-reach, -reach, 2.0 * reach, 2.0 * reach);
        let mut plane = Plane::new(origin, x, y, Vec2::new(1.0, 1.0), &uv_domain);
        plane.make_nurb()?;
        self.end_cap = Some(Surface::from(plane));

        // the rails' ends on P, and the vertex
        let mut ends = [None, None];
        for (ii, &face) in faces.iter().enumerate() {
            let fv = new_corner_vertex(k, pseudo, corner)?;
            let v = pseudo.fillet_vertex_mut(fv)?;
            v.set_point_class(PointClassType::Face, Some(EntityRef::Face(face)));
            v.set_fillet_vertex_type(FilletVertexType::RailXExtendedEdgeuse);
            self.vertices.push(fv);
            let rail = solver.find_index_of_rail_on_face(target, Some(face))?;
            if rail > 1 {
                return Err(SolidError::blend_failed(
                    OP,
                    "the blend has no rail on a face beside its edge",
                ));
            }
            pseudo.make_rail_edgeuse_on(
                geom,
                rail,
                Some(fv),
                OrientType::Unknown,
                None,
                CornerLookup {
                    this: Some((Some(corner), self)),
                    ..Default::default()
                },
                solver,
                target,
            )?;
            if let Some(slot) = ends.get_mut(ii) {
                *slot = Some(fv);
            }
        }
        let [Some(end0), Some(end1)] = ends else {
            return Err(SolidError::blend_failed(
                OP,
                "the rails' ends on the end cap were not made",
            ));
        };
        let on_vertex = new_corner_vertex(k, pseudo, corner)?;
        let v = pseudo.fillet_vertex_mut(on_vertex)?;
        v.set_point_class(
            PointClassType::Vertex,
            Some(EntityRef::Vertex(corner_vertex)),
        );
        v.set_fillet_vertex_type(FilletVertexType::OnVertex);
        self.vertices.push(on_vertex);

        // the end rib, then the cap's edges on the side faces: a loop of the cap
        let rib = pseudo.make_fillet_edge(Some(end0), Some(end1), Some(corner))?;
        pseudo
            .fillet_edge_mut(rib)?
            .set_fillet_edge_type(FilletEdgeType::FilletXSideFace);
        let prim = need(
            eu1::primary_edgeuse(pseudo.brep(), rib)?,
            "fillet edge without primary edge use",
        )?;
        pseudo.set_edgeuse_fillet_geom(prim, Some(geom))?;
        self.edges.push(rib);
        let [face0, face1] = faces;
        let on_second = pseudo.make_fillet_edge(Some(end1), Some(on_vertex), Some(corner))?;
        let fe = pseudo.fillet_edge_mut(on_second)?;
        fe.set_fillet_edge_type(FilletEdgeType::Precomputed);
        fe.set_original_face(Some(face1));
        self.edges.push(on_second);
        let on_first = pseudo.make_fillet_edge(Some(on_vertex), Some(end0), Some(corner))?;
        let fe = pseudo.fillet_edge_mut(on_first)?;
        fe.set_fillet_edge_type(FilletEdgeType::Precomputed);
        fe.set_original_face(Some(face0));
        self.edges.push(on_first);

        self.blending = false;
        Ok(())
    }
}
