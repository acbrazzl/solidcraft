//! #15437 with the crease in the wall and the top drafted: a round on the long piece of an arm
//! top's edge along a wall, ending at the vertex where the wall turns by a few degrees.
//!
//! A boss 22 long in x and 16 high, its wall y = 10 for x >= `PIECE` and leaning `TILT` degrees
//! out (about z) for x < `PIECE`: a crease, the vertical line x = `PIECE`. An arm 12 x 6 joined
//! against the wall, its top drafted `DRAFT` degrees (rising with x from z = 8 at its end face
//! x = 0). The arm top meets the wall in the long piece, rounded, and a short piece across the
//! crease, left sharp: the round ends at the crease vertex, where its edge runs on into the short
//! piece, `TILT` degrees off.
//!
//! The square end cap crossed the wall's crease, the rail ended on the leaning piece and the
//! trim leaked. The cap now holds the crease: it is the plane x = `PIECE`. The round reaches the
//! end face x = 12, and its faces meet square (the top is drafted about y, the wall is vertical),
//! so it adds the corner's spandrel, `(1 - pi/4) R^2`, times the length between the two planes
//! along the edge, `(12 - PIECE) / cos(DRAFT)`.

#![allow(clippy::disallowed_methods)] // the invented body's corner points and closed form
#![allow(clippy::unwrap_used, clippy::expect_used, clippy::panic)] // test

mod common;

use common::edge_at;
use solidcore_ops::blend::{blend, Blend, Corners};
use solidcore_ops::boolean::boolean;
use solidcore_ops::sweep::{extrude, Curve, DraftCorner};
use solidcore_ops::{session, Body, BooleanOp, CheckLevel};

const TILT: f64 = 4.0;
const DRAFT: f64 = 2.5;
const PIECE: f64 = 2.2;
const RADIUS: f64 = 0.87;

/// SolidCraft's fix set (all but `HELD_BACK` in `crates/smlib/src/core.rs`), with or without
/// `fillet_end_cap_holds_crease`.
fn fixes(cap: bool) {
    const HELD_BACK: [&str; 2] = [
        "fillet_refuse_rounds_breaking_through",
        "fillet_round_ends_on_outline",
    ];
    let on = solidcore::kernel::UpstreamFixes::NAMES
        .iter()
        .copied()
        .filter(|n| !HELD_BACK.contains(n))
        .filter(|n| cap || *n != "fillet_end_cap_holds_crease");
    session::set_thread_fixes(on).unwrap();
}

/// The arm top's height over `x`.
fn top(x: f64) -> f64 {
    8.0 + x * DRAFT.to_radians().tan()
}

fn lines(points: &[[f64; 3]]) -> Vec<Curve> {
    points
        .iter()
        .zip(points.iter().cycle().skip(1))
        .map(|(&start, &end)| Curve::Line { start, end })
        .collect()
}

/// The boss with its creased wall and the drafted arm.
fn body() -> Body {
    let t = TILT.to_radians().tan();
    let boss = [
        [-10.0, 0.0, 0.0],
        [12.0, 0.0, 0.0],
        [12.0, 10.0, 0.0],
        [PIECE, 10.0, 0.0],
        [-10.0, 10.0 + (PIECE + 10.0) * t, 0.0],
    ];
    let boss = extrude(
        &lines(&boss),
        [0.0, 0.0, 1.0],
        16.0,
        0.0,
        DraftCorner::default(),
        true,
    )
    .unwrap();
    // the arm's side section in the plane y = 9 (inside the boss), extruded to y = 16
    let arm = [
        [0.0, 9.0, 0.0],
        [12.0, 9.0, 0.0],
        [12.0, 9.0, top(12.0)],
        [0.0, 9.0, top(0.0)],
    ];
    let arm = extrude(
        &lines(&arm),
        [0.0, 1.0, 0.0],
        7.0,
        0.0,
        DraftCorner::default(),
        true,
    )
    .unwrap();
    boolean(boss, arm, BooleanOp::Union).unwrap().unwrap()
}

/// The long piece rounded alone: the volume added, or the refusal.
fn rounded() -> solidcore_ops::Result<f64> {
    let b = body();
    let before = b.volume(1e-9).unwrap();
    let edge = edge_at(&b, [6.0, 10.0, top(6.0)]);
    blend(&b, Blend::Round, &[edge], &[RADIUS], &[], Corners::Filled)?;
    let report = b.check(CheckLevel::All).unwrap();
    assert!(report.is_valid(), "{report}");
    Ok(b.volume(1e-9).unwrap() - before)
}

#[test]
fn a_round_ending_at_a_wall_crease_under_a_drafted_top_is_capped_through_it() {
    fixes(true);
    let added = rounded().unwrap();
    let want = (1.0 - std::f64::consts::FRAC_PI_4) * RADIUS * RADIUS * (12.0 - PIECE)
        / DRAFT.to_radians().cos();
    assert!(
        (added - want).abs() < 1e-6 * want,
        "added {added}, the capped prism {want}"
    );
}

/// Without the cap through the crease the trim leaks and the round is refused.
#[test]
fn without_the_cap_through_the_crease_it_is_refused() {
    fixes(false);
    assert!(rounded().is_err(), "built without the fix");
}
