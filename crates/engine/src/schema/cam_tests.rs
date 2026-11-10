use serde_json::{Value, json};

use super::{
    CamContourParams, CamDeleteOpParams, CamDeleteOpReply, CamDrillParams, CamDrillReply, CamEngraveParams, CamEngraveReply, CamListParams,
    CamListReply, CamMillingReply, CamPocketParams, CamPostDefineParams, CamPostDefineReply, CamPostParams, CamPostReply, CamSetupParams,
    CamSetupReply, CamSimulateParams, CamSimulateReply, CamToolParams, CamToolReply, CamVCarveParams, CamVCarveReply,
};
use crate::Session;
use crate::schema::tests::{assert_keys, assert_strict_keeps_todays_errors, conforms};
use crate::schema::{ParamCheck, input_schema_for, output_schema_for};

/// A design with a sketch `Plate` (a 40 × 30 rectangle with a circle in it), a 6 mm and a 2 mm
/// flat end mill (1, 2), a drill (3), an 80° V-bit (14) and a setup `S` with its stock, at the
/// parameter-check level `level`.
fn workshop(level: ParamCheck) -> Session {
    let mut s = Session { param_check: level, ..Session::default() };
    for (id, p) in [
        ("sketch.create", json!({"plane": "XY", "name": "Plate"})),
        ("sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [40, 30]})),
        ("sketch.circle.center", json!({"center": [20, 15], "radius": 5})),
        ("sketch.finish", json!({})),
        ("cam.tool", json!({"number": 1, "diameter": 6, "flute_length": 20, "spindle_rpm": 12000, "feed": 1000, "plunge_feed": 300, "stepdown": 2})),
        ("cam.tool", json!({"number": 2, "diameter": 2, "flute_length": 8, "spindle_rpm": 18000, "feed": 500, "plunge_feed": 150, "stepdown": 1})),
        ("cam.tool", json!({"number": 3, "kind": "drill", "diameter": 5, "flute_length": 40, "spindle_rpm": 3000, "plunge_feed": 150})),
        (
            "cam.tool",
            json!({"number": 14, "kind": "vbit", "diameter": 5, "flute_length": 4.5, "tip_angle": 80, "spindle_rpm": 20000, "feed": 800, "plunge_feed": 300, "stepdown": 0.5}),
        ),
        ("cam.setup", json!({"name": "S", "stock": {"min": [-5, -5, -10], "max": [45, 35, 0]}})),
    ] {
        s.execute(id, &p).unwrap_or_else(|e| panic!("{id} {p}: {e}"));
    }
    s
}

/// `base` with `extra`'s keys.
fn with(base: &Value, extra: Value) -> Value {
    let mut p = base.clone();
    if let (Some(p), Some(extra)) = (p.as_object_mut(), extra.as_object()) {
        p.extend(extra.clone());
    }
    p
}

#[test]
fn schemas_and_the_key_check_agree() {
    assert_keys(
        input_schema_for::<CamToolParams>(),
        &[
            "delete",
            "description",
            "diameter",
            "feed",
            "flute_length",
            "flutes",
            "kind",
            "number",
            "plunge_feed",
            "spindle_rpm",
            "stepdown",
            "stepover",
            "taper_angle",
            "tip_angle",
            "tip_diameter",
        ],
    );
    assert_keys(input_schema_for::<CamSetupParams>(), &["delete", "name", "stock", "work_offset"]);
    assert_keys(
        input_schema_for::<CamContourParams>(),
        &[
            "at",
            "bottom",
            "clearance",
            "climb",
            "coolant",
            "cut",
            "dogbones",
            "feed",
            "holes",
            "name",
            "outer",
            "plunge_feed",
            "profiles",
            "ramp_angle",
            "regions",
            "rest_from",
            "retract",
            "rotate",
            "setup",
            "side",
            "sketch",
            "spindle_rpm",
            "stepdown",
            "stock_to_leave",
            "tabs",
            "tolerance",
            "tool",
            "top",
        ],
    );
    assert_keys(
        input_schema_for::<CamPocketParams>(),
        &[
            "at",
            "bottom",
            "clearance",
            "climb",
            "coolant",
            "cut",
            "dogbones",
            "feed",
            "holes",
            "name",
            "outer",
            "plunge_feed",
            "profiles",
            "ramp_angle",
            "regions",
            "rest_from",
            "retract",
            "rotate",
            "setup",
            "sketch",
            "spindle_rpm",
            "stepdown",
            "stepover",
            "stock_to_leave",
            "tolerance",
            "tool",
            "top",
        ],
    );
    assert_keys(
        input_schema_for::<CamEngraveParams>(),
        &[
            "at",
            "bottom",
            "clearance",
            "coolant",
            "curves",
            "cut",
            "feed",
            "name",
            "paths",
            "plunge_feed",
            "retract",
            "rotate",
            "setup",
            "sketch",
            "spindle_rpm",
            "stepdown",
            "tolerance",
            "tool",
            "top",
        ],
    );
    assert_keys(
        input_schema_for::<CamVCarveParams>(),
        &[
            "at",
            "clearance",
            "coolant",
            "cut",
            "feed",
            "holes",
            "max_depth",
            "name",
            "outer",
            "plunge_feed",
            "profiles",
            "regions",
            "retract",
            "rotate",
            "setup",
            "sketch",
            "spindle_rpm",
            "stepdown",
            "stepover",
            "tolerance",
            "tool",
            "top",
        ],
    );
    assert_keys(
        input_schema_for::<CamDrillParams>(),
        &[
            "at",
            "bottom",
            "clearance",
            "coolant",
            "cut",
            "depth",
            "dwell",
            "feed",
            "name",
            "peck",
            "plunge_feed",
            "points",
            "retract",
            "rotate",
            "setup",
            "sketch",
            "spindle_rpm",
            "stepdown",
            "tool",
            "top",
        ],
    );
    assert_keys(input_schema_for::<CamDeleteOpParams>(), &["name", "setup"]);
    assert_keys(input_schema_for::<CamPostParams>(), &["path", "post", "program", "setup", "verify"]);
    assert_keys(input_schema_for::<CamSimulateParams>(), &["operations", "path", "resolution", "setup", "stock"]);
    assert_keys(input_schema_for::<CamPostDefineParams>(), &["delete", "name", "post"]);
    assert_keys(input_schema_for::<CamListParams>(), &[]);
}

/// Calls that fail today get the same message in strict: the contract reads the keys a handler
/// checks with the handler's own reader, and the handler refuses the rest.
#[test]
fn strict_keeps_todays_errors() {
    assert_strict_keeps_todays_errors(&[
        ("cam.tool", json!({"number": 0, "diameter": 6, "flute_length": 20})),
        ("cam.tool", json!({"number": 1.5, "diameter": 6, "flute_length": 20})),
        ("cam.tool", json!({"number": "one", "diameter": 6, "flute_length": 20})),
        ("cam.tool", json!({"number": 1, "kind": "laser", "diameter": 6, "flute_length": 20})),
        ("cam.tool", json!({"number": 1, "diameter": 6})),
        ("cam.tool", json!({"number": 1, "kind": "vbit", "diameter": 6, "flute_length": 20})),
        ("cam.tool", json!({"number": 7, "delete": true})),
        ("cam.setup", json!({"name": " "})),
        ("cam.setup", json!({"name": "S", "work_offset": 7})),
        ("cam.setup", json!({"name": "S", "work_offset": null})),
        ("cam.setup", json!({"name": "S", "stock": {"min": [0, 0, 0]}})),
        ("cam.setup", json!({"name": "S", "stock": {"min": [0, 0, 0], "max": [1, 1, "high"]}})),
        ("cam.setup", json!({"name": "S", "stock": {"min": [0, 0, 0], "max": [0, 0, 0]}})),
        ("cam.setup", json!({"name": "Nope", "delete": true})),
        ("cam.delete_op", json!({"setup": "Nope", "name": "Cut"})),
        ("cam.post", json!({"setup": "Nope"})),
        ("cam.post.define", json!({"post": 5})),
        ("cam.post.define", json!({"post": {"name": "Mine"}})),
        ("cam.post.define", json!({"name": "Nope", "delete": true})),
    ]);
    // The operations need a tool and a setup before a handler looks at anything else.
    let contour = json!({"setup": "S", "name": "Edge", "tool": 1, "outer": [[0, 0], [40, 0], [40, 30], [0, 30]], "bottom": -2});
    let sketched = json!({"setup": "S", "name": "Edge", "tool": 1, "sketch": "Plate", "bottom": -2});
    let carve = json!({"setup": "S", "name": "Carve", "tool": 14, "outer": [[0, 20], [30, 20], [30, 21], [0, 21]], "stepover": 0.1});
    let drill = json!({"setup": "S", "name": "Holes", "tool": 3, "depth": 5});
    let engrave = json!({"setup": "S", "name": "Lines", "tool": 14, "bottom": -0.5});
    let cases = [
        ("cam.contour", with(&contour, json!({"tool": 0}))),
        ("cam.contour", with(&contour, json!({"tool": 99}))),
        ("cam.contour", with(&contour, json!({"setup": "Nope"}))),
        ("cam.contour", with(&contour, json!({"side": "up"}))),
        ("cam.contour", with(&contour, json!({"tabs": 5}))),
        ("cam.contour", with(&contour, json!({"outer": [[0, 0], [40, 0], "corner"]}))),
        ("cam.contour", with(&contour, json!({"outer": [[0, 0], [40, 0]]}))),
        ("cam.contour", with(&contour, json!({"holes": [[[1, 1], [2, 2]]]}))),
        ("cam.contour", with(&contour, json!({"holes": 5}))),
        ("cam.contour", with(&contour, json!({"regions": [{"holes": []}]}))),
        ("cam.contour", with(&contour, json!({"regions": [{"outer": [[0, 0], [1, 0], "corner"]}]}))),
        ("cam.contour", with(&contour, json!({"cut": {"heights": 1}}))),
        ("cam.contour", with(&contour, json!({"rest_from": "Nope"}))),
        ("cam.contour", with(&sketched, json!({"rotate": 45}))),
        ("cam.contour", with(&sketched, json!({"sketch": "Nope"}))),
        ("cam.pocket", with(&contour, json!({"tool": 14}))),
        ("cam.pocket", with(&contour, json!({"regions": [{"outer": [[0, 0], [1, 0]]}]}))),
        ("cam.vcarve", with(&carve, json!({"max_depth": -1}))),
        ("cam.vcarve", with(&carve, json!({"max_depth": null}))),
        ("cam.vcarve", with(&carve, json!({"tool": 1}))),
        ("cam.drill", with(&drill, json!({"points": [[1]]}))),
        ("cam.drill", with(&drill, json!({"points": [[1, "two"]]}))),
        ("cam.drill", with(&drill, json!({"points": [[0, 0]], "bottom": -5}))),
        ("cam.engrave", with(&engrave, json!({"paths": [[[0, 0], "end"]]}))),
        ("cam.engrave", with(&engrave, json!({"paths": [[[0, 0]]]}))),
        ("cam.engrave", with(&engrave, json!({"sketch": "Plate", "curves": 5}))),
        ("cam.engrave", with(&engrave, json!({"sketch": "Plate", "curves": "some"}))),
        ("cam.engrave", with(&engrave, json!({"sketch": "Plate", "curves": ["l1", 2]}))),
        ("cam.simulate", json!({"setup": "S", "stock": {"min": 1}, "path": "stock.png"})),
        ("cam.simulate", json!({"setup": "S", "stock": null, "path": "stock.png"})),
    ];
    for (id, p) in &cases {
        let error = |level: ParamCheck| workshop(level).execute(id, p).err().map(|e| e.to_string());
        let off = error(ParamCheck::Off);
        assert!(off.is_some(), "{id} {p} should fail today");
        assert_eq!(error(ParamCheck::Strict), off, "{id} {p}");
    }
}

/// Every manufacturing command replies as its output schema says, in Warn without warnings, in
/// each form it takes: regions from `outer`, from `regions` and from a sketch, a rest cut that
/// finds something left and one that finds nothing, curves from a sketch and from `paths`,
/// holes with their own bottom, a program returned and one written, and the deletions. A
/// misspelt key gets `param_warnings` in Warn, since the handlers ignore unknown keys, and is
/// refused in Strict.
#[test]
fn cam_commands_reply_with_their_schemas() {
    let mut s = workshop(ParamCheck::Warn);
    let run = |s: &mut Session, schema: schemars::Schema, id: &str, p: Value| {
        let r = s.execute(id, &p).unwrap_or_else(|e| panic!("{id} {p}: {e}"));
        let schema = schema.as_value();
        assert!(conforms(schema, schema, &r), "{id} {p}: {r} doesn't match {schema}");
        assert!(r.get("param_warnings").is_none(), "{id} {p}: {r}");
        r
    };
    let dir = std::env::temp_dir().join(format!("solidcraft-cam-schema-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let plate = json!([[0, 0], [40, 0], [40, 30], [0, 30]]);
    let square = json!([[0, 0], [20, 0], [20, 20], [0, 20]]);
    let disc: Vec<[f64; 2]> =
        (0..72).map(|i| f64::from(i) * std::f64::consts::TAU / 72.0).map(|a| [60.0 + 10.0 * a.cos(), 10.0 + 10.0 * a.sin()]).collect();

    let tool = || output_schema_for::<CamToolReply>();
    let made = run(
        &mut s,
        tool(),
        "cam.tool",
        json!({"number": 10, "kind": "tapered_ball", "diameter": 3.175, "flute_length": 15, "taper_angle": 5.05, "tip_diameter": 0.5,
            "description": "Tapered ball", "flutes": 2, "spindle_rpm": 18000, "feed": 600, "plunge_feed": 200, "stepdown": 1, "stepover": 0.2}),
    );
    assert_eq!(made["tool"]["kind"], "tapered_ball", "{made}");
    assert_eq!(run(&mut s, tool(), "cam.tool", json!({"number": 10, "delete": true}))["deleted"], 10);

    let setup = || output_schema_for::<CamSetupReply>();
    run(&mut s, setup(), "cam.setup", json!({"name": "Spare", "work_offset": 2, "stock": {"min": [0, 0, -5], "max": [10, 10, 0]}}));
    assert_eq!(run(&mut s, setup(), "cam.setup", json!({"name": "Spare", "stock": null}))["setup"]["stock"], Value::Null);
    assert_eq!(run(&mut s, setup(), "cam.setup", json!({"name": "Spare", "delete": true}))["deleted"], "Spare");

    let milling = || output_schema_for::<CamMillingReply>();
    let outside = run(
        &mut s,
        milling(),
        "cam.contour",
        json!({"setup": "S", "name": "Out", "tool": 1, "outer": plate, "holes": [[[18, 13], [22, 13], [22, 17], [18, 17]]], "side": "outside",
            "climb": false, "stock_to_leave": 0.1, "tolerance": 0.01, "ramp_angle": 3, "tabs": {"width": 6, "height": 1, "inner": false}, "dogbones": false,
            "top": 0, "bottom": -2, "retract": 2, "clearance": 10, "stepdown": 1, "feed": 900, "plunge_feed": 250, "spindle_rpm": 10000, "coolant": true}),
    );
    assert_eq!(outside["too_small"].as_array().map(Vec::len), Some(1), "the 4 mm hole is narrower than the 6 mm tool: {outside}");
    run(
        &mut s,
        milling(),
        "cam.contour",
        json!({"setup": "S", "name": "Two", "tool": 2, "regions": [{"outer": square, "holes": []}, {"outer": disc}], "tabs": true, "bottom": -4}),
    );
    run(
        &mut s,
        milling(),
        "cam.contour",
        json!({"setup": "S", "name": "Sketched", "tool": 2, "sketch": "Plate", "profiles": "all", "rotate": 90, "at": [100, 0], "side": "on", "bottom": -1}),
    );
    run(&mut s, milling(), "cam.pocket", json!({"setup": "S", "name": "Rough", "tool": 1, "outer": square, "stepover": 2.5, "bottom": -2}));
    let rest = run(
        &mut s,
        milling(),
        "cam.pocket",
        json!({"setup": "S", "name": "Corners", "tool": 2, "outer": square, "bottom": -2, "rest_from": "Rough"}),
    );
    assert!(rest["rest"]["cut_length"].as_f64().is_some_and(|l| l > 0.0), "{rest}");
    run(&mut s, milling(), "cam.pocket", json!({"setup": "S", "name": "Disc", "tool": 1, "outer": disc, "bottom": -2}));
    let none =
        run(&mut s, milling(), "cam.pocket", json!({"setup": "S", "name": "Disc rest", "tool": 2, "outer": disc, "bottom": -2, "rest_from": "Disc"}));
    assert_eq!(none["rest"]["nothing_left"], true, "{none}");
    let saved = run(&mut s, output_schema_for::<CamListReply>(), "cam.list", json!({}));
    let cut = saved["setups"][0]["operations"][0]["cut"].clone();
    run(&mut s, milling(), "cam.contour", json!({"setup": "S", "name": "Again", "tool": 1, "outer": plate, "cut": cut}));

    let engraved = || output_schema_for::<CamEngraveReply>();
    let paths = json!([[[0, 0], [10, 0], [10, 5]], [[20, 0], [30, 0], [30, 10], [20, 10], [20, 0]]]);
    assert_eq!(
        run(&mut s, engraved(), "cam.engrave", json!({"setup": "S", "name": "Zig", "tool": 14, "paths": paths, "bottom": -0.5, "tolerance": 0.01}))["chains"],
        2
    );
    run(
        &mut s,
        engraved(),
        "cam.engrave",
        json!({"setup": "S", "name": "Drawn", "tool": 14, "sketch": "Plate", "curves": "all", "rotate": 180, "at": [0, 50], "bottom": -0.5}),
    );

    let carved = || output_schema_for::<CamVCarveReply>();
    let stroke = run(
        &mut s,
        carved(),
        "cam.vcarve",
        json!({"setup": "S", "name": "Stroke", "tool": 14, "outer": [[0, 20], [30, 20], [30, 21], [0, 21]], "stepover": 0.1}),
    );
    assert!(stroke.get("flat_clearing").is_none(), "{stroke}");
    let held = run(
        &mut s,
        carved(),
        "cam.vcarve",
        json!({"setup": "S", "name": "Held", "tool": 14, "outer": [[0, 0], [10, 0], [10, 10], [0, 10]], "stepover": 0.5, "max_depth": 2}),
    );
    assert!(held["flat_clearing"]["wall_offset"].as_f64().is_some(), "{held}");

    let drilled = || output_schema_for::<CamDrillReply>();
    assert_eq!(
        run(
            &mut s,
            drilled(),
            "cam.drill",
            json!({"setup": "S", "name": "Dots", "tool": 14, "points": [[0, 0, -0.3], [5, 0, -0.8], [10, 0]], "depth": 0.5, "dwell": 0.1})
        )["holes"],
        3
    );
    run(&mut s, drilled(), "cam.drill", json!({"setup": "S", "name": "Centre", "tool": 3, "sketch": "Plate", "bottom": -8, "peck": 4}));

    let posted = || output_schema_for::<CamPostReply>();
    let program = run(&mut s, posted(), "cam.post", json!({"setup": "S", "post": "rs274", "program": "Plate"}));
    assert!(program["gcode"].as_str().is_some_and(|g| g.contains("M30")) && program["readback"]["within"] == true, "{program}");
    let file = dir.join("plate.nc");
    let written = run(&mut s, posted(), "cam.post", json!({"setup": "S", "verify": false, "path": file.to_string_lossy()}));
    assert!(written.get("gcode").is_none() && written.get("readback").is_none() && file.is_file(), "{written}");

    let picture = dir.join("stock.png");
    let simulated = run(
        &mut s,
        output_schema_for::<CamSimulateReply>(),
        "cam.simulate",
        json!({"setup": "S", "operations": ["Rough", "Stroke"], "resolution": 0.5, "stock": {"min": [-5, -5, -10], "max": [45, 35, 0]}, "path": picture.to_string_lossy()}),
    );
    assert_eq!(simulated["operations"].as_array().map(Vec::len), Some(2), "{simulated}");
    run(
        &mut s,
        output_schema_for::<CamSimulateReply>(),
        "cam.simulate",
        json!({"setup": "S", "operations": "Rough", "path": picture.to_string_lossy()}),
    );

    let defined = || output_schema_for::<CamPostDefineReply>();
    let mut post = serde_json::to_value(solidcraft_cam::Post::rs274()).unwrap();
    post["name"] = json!("Mine");
    assert_eq!(run(&mut s, defined(), "cam.post.define", json!({"post": post}))["post"], "Mine");
    let listed = run(&mut s, output_schema_for::<CamListReply>(), "cam.list", json!({}));
    assert_eq!(listed["posts"], json!(["Mine"]), "{listed}");
    assert_eq!(run(&mut s, defined(), "cam.post.define", json!({"name": "Mine", "delete": true}))["deleted"], "Mine");

    let deleted = run(&mut s, output_schema_for::<CamDeleteOpReply>(), "cam.delete_op", json!({"setup": "S", "name": "Again"}));
    assert_eq!(deleted, json!({"deleted": "Again", "setup": "S"}));

    // A misspelt key: a warning on a reply that still matches, and a refusal in Strict.
    let misspelt = json!({"setup": "S", "name": "Typo", "tool": 1, "outer": plate, "bottom": -1, "stepdwn": 1});
    let warned = s.execute("cam.contour", &misspelt).unwrap();
    let schema = milling();
    assert!(conforms(schema.as_value(), schema.as_value(), &warned), "{warned}");
    assert_eq!(warned["param_warnings"], json!(["unknown key `stepdwn`; did you mean `stepdown`?"]), "{warned}");
    let refused = workshop(ParamCheck::Strict).execute("cam.contour", &misspelt).unwrap_err().to_string();
    assert!(refused.starts_with("invalid parameters for `cam.contour`: unknown key `stepdwn`"), "{refused}");
    let _ = std::fs::remove_dir_all(dir);
}
