use serde_json::{Value, json};

use super::{BodyListParams, DescribeParams, ModelEdges, ModelFaces, ModelThreads, ModelThreadsParams};
use crate::Session;
use crate::schema::tests::{assert_keys, assert_strict_keeps_todays_errors, conforms};
use crate::schema::{ParamCheck, input_schema_for, output_schema_for};

#[test]
fn schemas_and_the_key_check_agree() {
    assert_keys(input_schema_for::<DescribeParams>(), &["command"]);
    assert_keys(input_schema_for::<BodyListParams>(), &["body"]);
    assert_keys(input_schema_for::<ModelThreadsParams>(), &[]);
}

#[test]
fn strict_keeps_todays_errors() {
    assert_strict_keeps_todays_errors(&[
        ("engine.describe", json!({"command": 5})),
        ("engine.describe", json!({"command": null})),
        ("engine.describe", json!({"command": "solid.extrud"})),
        ("model.edges", json!({})),
        ("model.edges", json!({"body": "Nope"})),
        ("model.faces", json!({})),
        ("model.faces", json!({"body": "Nope"})),
    ]);
}

/// #10942: a body's edges and faces and the design's threads reply as their output schemas say,
/// in Warn without warnings for good calls and with them for a misspelt key. A cylindrical face
/// carries `cylinder` and a folded sheet's face `sheet_face`; strict refuses a `body` that isn't
/// text, which the handlers read as no body.
#[test]
fn model_lists_reply_with_their_schemas() {
    let (edges, faces, threads) = (output_schema_for::<ModelEdges>(), output_schema_for::<ModelFaces>(), output_schema_for::<ModelThreads>());
    let mut s = Session { param_check: ParamCheck::Warn, ..Session::default() };
    let run = |s: &mut Session, id: &str, p: Value| s.execute(id, &p).unwrap_or_else(|e| panic!("{id} {p}: {e}"));
    let matches = |schema: &schemars::Schema, r: &Value, warned: bool| {
        let schema = schema.as_value();
        assert!(conforms(schema, schema, r), "{r} doesn't match {schema}");
        assert_eq!(r.get("param_warnings").is_some(), warned, "{r}");
    };
    run(&mut s, "solid.cylinder", json!({"radius": 4, "height": 20, "body_name": "Shaft"}));
    run(&mut s, "solid.thread", json!({"face": [4, 0, 10], "designation": "M8"}));
    run(&mut s, "sheet.manage_rules", json!({"name": "R1", "thickness": "1 mm", "bend_radius": "1 mm", "active": true}));
    run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Base"}));
    run(&mut s, "sketch.rectangle.two_point", json!({"p0": [20, 0], "p1": [60, 30]}));
    run(&mut s, "sketch.finish", json!({}));
    run(&mut s, "sheet.flange", json!({"sketch": "Base"}));
    let sheet = s.model.state().bodies.iter().map(|b| b.name.clone()).find(|n| n != "Shaft").unwrap_or_default();

    let r = run(&mut s, "model.edges", json!({"body": "Shaft"}));
    matches(&edges, &r, false);
    assert!(r["edges"].as_array().is_some_and(|e| !e.is_empty()), "{r}");
    let r = run(&mut s, "model.faces", json!({"body": "Shaft"}));
    matches(&faces, &r, false);
    assert!(r["faces"].as_array().into_iter().flatten().any(|f| f["cylinder"]["radius"].as_f64().is_some_and(|r| (r - 4.0).abs() < 1e-9)), "{r}");
    let r = run(&mut s, "model.faces", json!({"body": sheet}));
    matches(&faces, &r, false);
    assert!(r["faces"].as_array().into_iter().flatten().any(|f| f["sheet_face"].as_str().is_some_and(|n| n.starts_with("sheet:"))), "{r}");
    let r = run(&mut s, "model.threads", json!({}));
    matches(&threads, &r, false);
    assert_eq!((r["threads"][0]["designation"].as_str(), r["threads"][0]["hand"].as_str()), (Some("M8"), Some("right")), "{r}");

    matches(&edges, &run(&mut s, "model.edges", json!({"body": "Shaft", "bdy": 1})), true);
    matches(&faces, &run(&mut s, "model.faces", json!({"body": "Shaft", "bdy": 1})), true);
    matches(&threads, &run(&mut s, "model.threads", json!({"body": "Shaft"})), true);
    let mut strict = Session { param_check: ParamCheck::Strict, ..Session::default() };
    strict.execute("solid.cylinder", &json!({"radius": 4, "height": 20, "body_name": "Shaft"})).unwrap_or_else(|e| panic!("{e}"));
    for id in ["model.edges", "model.faces"] {
        let error = strict.execute(id, &json!({"body": 5})).err().map(|e| e.to_string()).unwrap_or_default();
        assert!(error.contains("`body` must be text"), "{id}: {error}");
    }
}

#[test]
fn describe_returns_both_schemas() {
    let mut s = Session::default();
    let d = s.execute("engine.describe", &json!({"command": "solid.box"})).unwrap_or_else(|e| panic!("{e}"));
    assert_eq!(d["id"], "solid.box");
    assert!(d["input"]["properties"].get("length").is_some());
    assert!(d["output"]["properties"].get("created_bodies").is_some());
    // Its own output schema lists the warnings warn mode can add.
    let own = s.execute("engine.describe", &json!({"command": "engine.describe"})).unwrap_or_else(|e| panic!("{e}"));
    assert!(own["output"]["properties"].get("param_warnings").is_some(), "{own}");
    let d = s.execute("engine.describe", &json!({"command": "solid.revolve"})).unwrap_or_else(|e| panic!("{e}"));
    assert!(d["input"].is_null() && d["output"].is_null(), "unconverted commands have no schemas yet");
    let e = s.execute("engine.describe", &json!({"command": "solid.extrud"})).err().map(|e| e.to_string());
    assert!(e.as_deref().is_some_and(|e| e.contains("did you mean `solid.extrude`")), "{e:?}");
}

/// `model.edges`' middle and length come from each edge's exact curve: on a cylinder of radius
/// 4 every edge round it has its middle on that circle and its length a whole turn or a half,
/// to 1e-9, where the polyline's middle lay inside the circle by up to the chord tolerance (so
/// as a pick it missed a curved edge by that much). The seam is a line of 20 with its middle at
/// half height.
#[test]
fn model_edges_reports_each_edges_exact_middle_and_length() {
    use std::f64::consts::PI;
    let mut s = Session::default();
    s.execute("solid.cylinder", &json!({"radius": 4, "height": 20, "body_name": "Shaft"})).unwrap_or_else(|e| panic!("{e}"));
    let r = s.execute("model.edges", &json!({"body": "Shaft"})).unwrap_or_else(|e| panic!("{e}"));
    let edges = r["edges"].as_array().cloned().unwrap_or_default();
    let v = |x: &Value| -> [f64; 3] { [x[0].as_f64().unwrap_or(f64::NAN), x[1].as_f64().unwrap_or(f64::NAN), x[2].as_f64().unwrap_or(f64::NAN)] };
    let (mut round, mut seams) = (0, 0);
    for e in &edges {
        let (mid, start, end, length) = (v(&e["mid"]), v(&e["start"]), v(&e["end"]), e["length"].as_f64().unwrap_or(f64::NAN));
        if (mid[2] - start[2]).abs() < 1e-9 && (start[2] - end[2]).abs() < 1e-9 {
            round += 1;
            assert!((mid[0].hypot(mid[1]) - 4.0).abs() < 1e-9, "middle {mid:?} off the r 4 circle: {e}");
            assert!((length - 8.0 * PI).abs() < 1e-9 || (length - 4.0 * PI).abs() < 1e-9, "length {length}: {e}");
        } else {
            seams += 1;
            assert!((length - 20.0).abs() < 1e-9 && (mid[2] - 10.0).abs() < 1e-9 && (mid[0].hypot(mid[1]) - 4.0).abs() < 1e-9, "{e}");
        }
    }
    assert!(round >= 2 && seams >= 1, "{r}");
}
