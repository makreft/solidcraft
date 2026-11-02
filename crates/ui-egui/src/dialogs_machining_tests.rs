//! The MANUFACTURE tab's dialogs build the parameters of their `cam.*` commands, and what they
//! build runs.
use serde_json::json;
use solidcraft_engine::{Sel, Session};

use super::*;
use crate::Services;
use crate::dialogs::{Dialog, apply_commands};

/// A 100 × 50 plate drawn on XY as the sketch `Plate`.
fn plate_app() -> SolveApp {
    let mut s = Session::default();
    for (id, p) in [
        ("sketch.create", json!({"plane": "XY", "name": "Plate"})),
        ("sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [100, 50]})),
        ("sketch.finish", json!({})),
    ] {
        s.execute(id, &p).unwrap();
    }
    SolveApp::new(s, Services::default())
}

fn form(app: &SolveApp, id: &str) -> Dialog {
    let d = Dialog::for_command(app, id).unwrap_or_else(|| panic!("{id} opens no dialog"));
    assert!(matches!(d.kind, Kind::Machining(_)), "{id}");
    d
}

/// Runs what the dialog's OK runs.
fn ok(app: &mut SolveApp, d: &Dialog) {
    for (id, p) in apply_commands(app, d).unwrap_or_else(|e| panic!("{e}")) {
        app.run(&id, p.clone()).unwrap_or_else(|e| panic!("{id} {p}: {e}"));
    }
}

fn tool_form(d: &mut Dialog) -> &mut ToolForm {
    match &mut d.kind {
        Kind::Machining(Mach::Tool(f)) => f,
        other => panic!("{other:?}"),
    }
}

/// A 6 mm cutter with its cutting data, typed as a person types them (a decimal comma, a unit).
fn add_tool(app: &mut SolveApp) {
    let mut d = form(app, "cam.tool");
    let f = tool_form(&mut d);
    assert_eq!(f.number, "1", "the first free number");
    f.diameter = "6 mm".into();
    f.flute_length = "22".into();
    f.spindle_rpm = "16000".into();
    f.feed = "1500".into();
    f.plunge_feed = "500".into();
    f.stepdown = "3,5".into();
    ok(app, &d);
}

#[test]
fn the_tool_dialog_makes_and_edits_a_tool() {
    let mut app = plate_app();
    add_tool(&mut app);
    let t = app.session.doc.cam.tool(1).cloned().unwrap();
    assert_eq!((t.diameter, t.flute_length, t.spindle_rpm, t.stepdown), (6.0, 22.0, Some(16000.0), Some(3.5)));

    // Picking the tool fills its fields; a change keeps the rest.
    let mut d = form(&app, "cam.tool");
    let f = tool_form(&mut d);
    assert_eq!(f.number, "2", "the next free number");
    f.existing = 1;
    load_tool(&app, f);
    assert_eq!((f.number.as_str(), f.diameter.as_str(), f.feed.as_str()), ("1", "6", "1500"));
    f.feed = "1800".into();
    ok(&mut app, &d);
    let t = app.session.doc.cam.tool(1).cloned().unwrap();
    assert_eq!((t.feed, t.plunge_feed, app.session.doc.cam.tools.len()), (Some(1800.0), Some(500.0), 1));

    // A diameter that is no value is an error in the dialog, not a tool.
    let mut d = form(&app, "cam.tool");
    tool_form(&mut d).diameter = "six".into();
    tool_form(&mut d).flute_length = "20".into();
    assert!(apply_commands(&app, &d).unwrap_err().contains("Diameter"));
}

#[test]
fn the_setup_dialog_sets_the_work_offset_and_stock() {
    let mut app = plate_app();
    let mut d = form(&app, "cam.setup");
    let Kind::Machining(Mach::Setup(f)) = &mut d.kind else { panic!() };
    assert_eq!(f.name, "Setup 1");
    f.offset = 1;
    f.stock = true;
    f.min = ["0".into(), "0".into(), "-18".into()];
    f.max = ["100".into(), "50".into(), "0".into()];
    ok(&mut app, &d);
    let s = app.session.doc.cam.setup("Setup 1").cloned().unwrap();
    assert_eq!((s.work_offset, s.stock.map(|b| b.min[2])), (2, Some(-18.0)));

    // Editing it: its values come back, and unticking Stock removes the stock.
    let mut d = form(&app, "cam.setup");
    let Kind::Machining(Mach::Setup(f)) = &mut d.kind else { panic!() };
    f.existing = 1;
    load_setup(&app, f);
    assert_eq!((f.offset, f.stock, f.max[0].as_str()), (1, true, "100"));
    f.stock = false;
    ok(&mut app, &d);
    let s = app.session.doc.cam.setup("Setup 1").cloned().unwrap();
    assert_eq!((s.work_offset, s.stock), (2, None));
}

#[test]
fn the_contour_and_pocket_dialogs_cut_the_picked_profiles() {
    let mut app = plate_app();
    add_tool(&mut app);
    app.run("cam.setup", json!({"name": "Top", "stock": {"min": [0, 0, -18], "max": [100, 50, 0]}})).unwrap();
    let sketch = app.session.doc.find_feature("Plate").unwrap().id;

    let mut d = form(&app, "cam.contour");
    assert!(apply_commands(&app, &d).unwrap_err().contains("profiles"), "nothing picked yet");
    d.inputs[0].items = vec![Sel::Profile { sketch, index: 0 }];
    let Kind::Machining(Mach::Op(f)) = &mut d.kind else { panic!() };
    assert_eq!(f.name, "Contour 1");
    f.bottom = "-18.3".into();
    f.tabs = true;
    f.dogbones = true;
    let (_, p) = apply_commands(&app, &d).unwrap().remove(0);
    assert_eq!((p["sketch"].as_u64(), p["profiles"].clone(), p["side"].clone()), (Some(sketch), json!([0]), json!("outside")));
    assert_eq!((p["bottom"].clone(), p["tabs"].clone(), p.get("top")), (json!(-18.3), json!(true), None), "the top is the stock's");
    ok(&mut app, &d);

    let mut d = form(&app, "cam.pocket");
    d.inputs[0].items = vec![Sel::Profile { sketch, index: 0 }];
    let Kind::Machining(Mach::Op(f)) = &mut d.kind else { panic!() };
    f.bottom = "-2".into();
    f.stepover = "2.5".into();
    f.ramp = "3".into();
    let (id, p) = apply_commands(&app, &d).unwrap().remove(0);
    assert_eq!((id.as_str(), p.get("side"), p["ramp_angle"].as_f64().map(|a| (a - 3.0).abs() < 1e-9)), ("cam.pocket", None, Some(true)));
    ok(&mut app, &d);
    let names: Vec<_> = app.session.doc.cam.setups[0].operations.iter().map(|o| o.name().to_string()).collect();
    assert_eq!(names, ["Contour 1", "Pocket 1"]);

    // Delete CAM Operation removes the one chosen.
    let mut d = form(&app, "cam.delete_op");
    let Kind::Machining(Mach::DeleteOp { op, .. }) = &mut d.kind else { panic!() };
    *op = 1;
    ok(&mut app, &d);
    assert_eq!(app.session.doc.cam.setups[0].operations.len(), 1);
}

#[test]
fn the_post_dialog_writes_the_program_beside_the_design() {
    let mut app = plate_app();
    add_tool(&mut app);
    app.run("cam.setup", json!({"name": "Top", "stock": {"min": [0, 0, -18], "max": [100, 50, 0]}})).unwrap();
    app.run("cam.contour", json!({"setup": "Top", "name": "Outline", "tool": 1, "sketch": "Plate", "bottom": -3})).unwrap();
    let dir = std::env::temp_dir().join(format!("solidcraft-cam-dialog-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    app.session.path = Some(dir.join("Plate.solidcraft").to_string_lossy().into_owned());

    let d = form(&app, "cam.post");
    let (_, p) = apply_commands(&app, &d).unwrap().remove(0);
    let file = dir.join("Top.nc");
    assert_eq!(p, json!({"setup": "Top", "path": file.to_string_lossy()}));
    ok(&mut app, &d);
    let text = std::fs::read_to_string(&file).unwrap();
    assert!(text.contains("G54") && text.contains("M30") && text.contains("Z-3"), "{text}");
    std::fs::remove_dir_all(&dir).ok();
}

/// Every CAM command on the MANUFACTURE tab opens its dialog from the ribbon, and a dialog with
/// no tool or setup yet says what to add first instead of failing on OK.
#[test]
fn every_manufacture_command_opens_its_dialog() {
    let app = plate_app();
    for id in ["cam.tool", "cam.setup", "cam.contour", "cam.pocket", "cam.delete_op", "cam.post"] {
        let d = form(&app, id);
        let Kind::Machining(m) = &d.kind else { panic!("{id}") };
        assert_eq!(m.command(), id);
    }
    let d = form(&app, "cam.contour");
    assert!(apply_commands(&app, &d).unwrap_err().contains("setup"));
}
