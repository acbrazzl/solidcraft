//! #15212's sibling: a round on one piece of the edge of `round_across_crease.rs`, ending at the
//! shallow crease where the face beside it changes plane.
//!
//! The body is a block 6 long in x, 10 deep in y and 8 high. Its section in the plane y = 0 is
//! extruded along y. The bottom is flat from x = 0 to x = `CREASE` and rises at `TILT` degrees to
//! x = `LENGTH`, so the crease is the line x = `CREASE`, z = 0. The side face y = `DEPTH` is cut by
//! a plane turned `draft` degrees about z through (`CREASE`, `DEPTH`, 0). The bottom edge of that
//! side face is two edges meeting at the crease, each rounded at the same radius.
//!
//! The round ends at the crease vertex, where its edge runs on into the other piece (an End at a
//! smooth vertex), capped by a plane through the vertex. Square to the edge, that plane holds the
//! crease only while the side face is square to the bottom: drafted, the cap crossed the bottom
//! past the crease, the rail ended on the face beyond, and the trim leaked. The cap now holds the
//! crease, turned from square to the edge by under the tangency tolerance.

#![allow(clippy::disallowed_methods)] // closed-form oracle, not ops geometry
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // test

mod common;

use common::edge_at;
use solidcore_ops::blend::{blend, Blend, Corners};
use solidcore_ops::boolean::boolean;
use solidcore_ops::sweep::{extrude, Curve, DraftCorner};
use solidcore_ops::{make_box, session, Body, BooleanOp, CheckLevel};

const LENGTH: f64 = 6.0;
const CREASE: f64 = 0.8;
const HEIGHT: f64 = 8.0;
const DEPTH: f64 = 10.0;
const TILT: f64 = 3.0;
const RADIUS: f64 = 2.6;

/// SolidCraft's fix set: every fix but those SolidCraft's smlib backend holds back (`HELD_BACK`
/// in `crates/smlib/src/core.rs`), so the rounds are made as SolidCraft makes them.
fn fixes() {
    const HELD_BACK: [&str; 2] = [
        "fillet_refuse_rounds_breaking_through",
        "fillet_round_ends_on_outline",
    ];
    let on = solidcore::kernel::UpstreamFixes::NAMES
        .iter()
        .copied()
        .filter(|n| !HELD_BACK.contains(n));
    session::set_thread_fixes(on).unwrap();
}

/// The bottom's height over `x`.
fn bottom(x: f64) -> f64 {
    if x <= CREASE {
        0.0
    } else {
        (x - CREASE) * TILT.to_radians().tan()
    }
}

/// The block, its side face drafted `draft` degrees in plan.
fn block(draft: f64) -> Body {
    let line = |start: [f64; 3], end: [f64; 3]| Curve::Line { start, end };
    let points = [
        [0.0, 0.0, HEIGHT],
        [LENGTH, 0.0, HEIGHT],
        [LENGTH, 0.0, bottom(LENGTH)],
        [CREASE, 0.0, 0.0],
        [0.0, 0.0, 0.0],
    ];
    let profile: Vec<Curve> = points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .map(|(&from, &to)| line(from, to))
        .collect();
    let b = extrude(
        &profile,
        [0.0, 1.0, 0.0],
        DEPTH + 2.0,
        0.0,
        DraftCorner::default(),
        true,
    )
    .unwrap();
    // everything past the side face: a box turned about z, its face y = 0 through the crease
    let cut = make_box([-20.0, 0.0, -20.0], [40.0, 20.0, 40.0]).unwrap();
    let (c, s) = ((-draft).to_radians().cos(), (-draft).to_radians().sin());
    cut.transform(&[
        [c, s, 0.0, 0.0],
        [-s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [CREASE, DEPTH, 0.0, 1.0],
    ])
    .unwrap();
    boolean(b, cut, BooleanOp::Subtract).unwrap().unwrap()
}

/// The long piece of the side face's bottom edge alone, rounded `RADIUS`; the volume removed.
fn rounded(draft: f64) -> f64 {
    let b = block(draft);
    let before = b.volume(1e-9).unwrap();
    let side = |x: f64| DEPTH - (x - CREASE) * draft.to_radians().tan();
    let edge = edge_at(&b, [3.0, side(3.0), bottom(3.0)]);
    blend(&b, Blend::Round, &[edge], &[RADIUS], &[], Corners::Filled)
        .unwrap_or_else(|e| panic!("draft {draft}: {e:?}"));
    let report = b.check(CheckLevel::All).unwrap();
    assert!(report.is_valid(), "draft {draft}: {report}");
    before - b.volume(1e-9).unwrap()
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn unit(a: [f64; 3]) -> [f64; 3] {
    let n = dot(a, a).sqrt();
    [a[0] / n, a[1] / n, a[2] / n]
}

fn toward(a: [f64; 3], away_from: [f64; 3]) -> [f64; 3] {
    if dot(a, away_from) < 0.0 {
        a
    } else {
        [-a[0], -a[1], -a[2]]
    }
}

/// The volume the round removes: a straight prism of the corner's spandrel (the region between
/// the side face, the tilted piece of the bottom and the ball's circle), cut by the end face
/// x = `LENGTH` and by the cap, the plane through the crease vertex that holds the crease (along
/// y) and is otherwise square to the edge. Its volume is the spandrel's area times the length of
/// its centroid's line between the two planes. For a corner of angle `phi`, with `d` from the
/// corner to the tangent points and `a` half the circle's arc, the area is the kite's less the
/// sector, `d R - a R^2`, and the centroid lies on the bisector at the kite's moment less the
/// sector's over that area.
fn capped_volume(draft: f64) -> f64 {
    let (r, tilt, draft) = (RADIUS, TILT.to_radians(), draft.to_radians());
    let vertex = [CREASE, DEPTH, 0.0];
    // the faces' outward normals, the edge, and each face's way into the material across it
    let side = unit([draft.tan(), 1.0, 0.0]);
    let base = unit([tilt.tan(), 0.0, -1.0]);
    let mut t = unit(cross(side, base));
    if t[0] < 0.0 {
        t = [-t[0], -t[1], -t[2]];
    }
    let along_side = toward(unit(cross(t, side)), base);
    let along_base = toward(unit(cross(t, base)), side);
    let phi = dot(along_side, along_base).acos();
    let bisector = unit([
        along_side[0] + along_base[0],
        along_side[1] + along_base[1],
        along_side[2] + along_base[2],
    ]);
    let d = r / (phi / 2.0).tan();
    let to_centre = r / (phi / 2.0).sin();
    let half_arc = (std::f64::consts::PI - phi) / 2.0;
    let area = d * r - half_arc * r * r;
    let kite = d * r * (d * (phi / 2.0).cos() + to_centre) / 3.0;
    let sector = half_arc * r * r * (to_centre - 2.0 * r * half_arc.sin() / (3.0 * half_arc));
    let from_corner = (kite - sector) / area;
    let c = [
        vertex[0] + from_corner * bisector[0],
        vertex[1] + from_corner * bisector[1],
        vertex[2] + from_corner * bisector[2],
    ];
    // the cap: through the vertex, its normal the edge's without its component along the crease
    let cap = unit([t[0], 0.0, t[2]]);
    let rel = [c[0] - vertex[0], c[1] - vertex[1], c[2] - vertex[2]];
    let at_cap = -dot(rel, cap) / dot(t, cap);
    let at_end = (LENGTH - c[0]) / t[0];
    area * (at_end - at_cap)
}

/// The side face square to the bottom: the cap square to the edge holds the crease, as before.
#[test]
fn a_round_ending_at_a_shallow_crease_is_capped_through_it() {
    fixes();
    let removed = rounded(0.0);
    let want = capped_volume(0.0);
    assert!(
        (removed - want).abs() < 1e-6 * want,
        "removed {removed}, the capped prism {want}"
    );
}

/// The side face drafted 3 degrees: the cap turns to hold the crease, and the round builds.
#[test]
fn a_round_ending_at_a_crease_beside_a_drafted_side_is_capped_through_the_crease() {
    fixes();
    let removed = rounded(3.0);
    let want = capped_volume(3.0);
    assert!(
        (removed - want).abs() < 1e-6 * want,
        "removed {removed}, the capped prism {want}"
    );
}
