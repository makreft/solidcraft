//! The MANUFACTURE tab's dialogs: CAM Tool (`cam.tool`), CAM Setup (`cam.setup`), 2D Contour
//! (`cam.contour`), 2D Pocket (`cam.pocket`), Delete CAM Operation (`cam.delete_op`) and Post
//! Process (`cam.post`). Each builds its command's parameters. Lengths are typed as expressions
//! in the design's units (parameters allowed) and sent in millimetres; speeds, feeds and counts
//! are plain numbers. An empty optional field leaves the value to the tool or the setup, as the
//! command does.

use serde_json::{Map, Value, json};
use solidcraft_engine::doc::cam::CamOperation;
use solidcraft_engine::doc::expr;

use crate::SolveApp;
use crate::dialogs::{Kind, check, form_row};
use crate::selection::{PROFILES, SelInput, SelectionExt as _};
use crate::ui_kit::{StatusTone, choice_row, hint_row, status_line, text_row};

/// The tool kinds as `cam.tool` names them, and their labels.
const TOOL_KINDS: &[(&str, &str)] = &[
    ("flat_endmill", "Flat End Mill"),
    ("ball_endmill", "Ball End Mill"),
    ("drill", "Drill"),
    ("vbit", "V-Bit"),
    ("tapered_ball", "Tapered Ball End Mill"),
];
/// The sides a contour runs on, and their labels.
const SIDES: [(&str, &str); 3] = [("outside", "Outside"), ("inside", "Inside"), ("on", "On the Line")];
/// Work offsets 1…6 as the program writes them.
const OFFSETS: [&str; 6] = ["G54", "G55", "G56", "G57", "G58", "G59"];
/// Quarter turns of the profiles about the sketch origin.
const TURNS: [&str; 4] = ["0°", "90°", "180°", "270°"];

#[derive(Clone, Debug)]
pub enum Mach {
    Tool(ToolForm),
    Setup(SetupForm),
    Op(OpForm),
    DeleteOp { setup: usize, op: usize },
    Post(PostForm),
}

impl Mach {
    pub fn title(&self) -> &'static str {
        match self {
            Mach::Tool(_) => "CAM TOOL",
            Mach::Setup(_) => "CAM SETUP",
            Mach::Op(f) if f.pocket => "2D POCKET",
            Mach::Op(_) => "2D CONTOUR",
            Mach::DeleteOp { .. } => "DELETE CAM OPERATION",
            Mach::Post(_) => "POST PROCESS",
        }
    }

    pub fn command(&self) -> &'static str {
        match self {
            Mach::Tool(_) => "cam.tool",
            Mach::Setup(_) => "cam.setup",
            Mach::Op(f) if f.pocket => "cam.pocket",
            Mach::Op(_) => "cam.contour",
            Mach::DeleteOp { .. } => "cam.delete_op",
            Mach::Post(_) => "cam.post",
        }
    }
}

/// CAM Tool: a new tool, or one of the design's (`existing` counts from 1), with its cutting data.
#[derive(Clone, Debug, Default)]
pub struct ToolForm {
    pub existing: usize,
    loaded: usize,
    pub number: String,
    pub kind: usize,
    pub diameter: String,
    pub flute_length: String,
    pub flutes: String,
    pub spindle_rpm: String,
    pub feed: String,
    pub plunge_feed: String,
    pub stepdown: String,
    pub stepover: String,
    /// A V-bit's included tip angle and a tapered ball end mill's taper per side (degrees).
    pub tip_angle: String,
    pub taper_angle: String,
    /// The flat at a V-bit's tip, or a tapered ball end mill's ball.
    pub tip_diameter: String,
    pub description: String,
}

/// CAM Setup: a new setup, or one of the design's (`existing` counts from 1).
#[derive(Clone, Debug, Default)]
pub struct SetupForm {
    pub existing: usize,
    loaded: usize,
    pub name: String,
    pub offset: usize,
    pub stock: bool,
    pub min: [String; 3],
    pub max: [String; 3],
}

/// 2D Contour and 2D Pocket: the profiles are the dialog's selection input.
#[derive(Clone, Debug)]
pub struct OpForm {
    pub pocket: bool,
    pub setup: usize,
    pub tool: usize,
    pub name: String,
    pub side: usize,
    pub turn: usize,
    pub bottom: String,
    pub top: String,
    pub stepdown: String,
    pub stepover: String,
    pub leave: String,
    pub ramp: String,
    pub climb: bool,
    pub tabs: bool,
    pub tab_width: String,
    pub tab_height: String,
    pub tab_spacing: String,
    pub dogbones: bool,
    pub feed: String,
    pub plunge_feed: String,
    pub spindle_rpm: String,
}

/// Post Process: which setup, through which post, into which file.
#[derive(Clone, Debug, Default)]
pub struct PostForm {
    pub setup: usize,
    /// 0 is the built-in RS-274 post, then the posts saved with the design.
    pub post: usize,
    pub program: String,
    pub file: String,
}

fn tr(text: &str) -> &str {
    crate::i18n::tr_label(text)
}

/// The first name `base N` the design's names don't have yet.
fn free_name(base: &str, taken: &[&str]) -> String {
    (1..).map(|n| format!("{base} {n}")).find(|n| !taken.contains(&n.as_str())).unwrap_or_else(|| base.to_string())
}

/// A number as the field shows it.
fn shown(v: f64) -> String {
    let t = format!("{v:.4}");
    t.trim_end_matches('0').trim_end_matches('.').to_string()
}

fn shown_opt(v: Option<f64>) -> String {
    v.map(shown).unwrap_or_default()
}

/// The dialog for a `cam.*` command.
pub fn start(app: &SolveApp, id: &str) -> Option<(Kind, Vec<SelInput>)> {
    let cam = &app.session.doc.cam;
    let mach = match id {
        "cam.tool" => {
            let next = (1u32..).find(|n| cam.tool(*n).is_none()).unwrap_or(1);
            Mach::Tool(ToolForm { number: next.to_string(), flutes: "2".into(), ..ToolForm::default() })
        }
        "cam.setup" => {
            let taken: Vec<&str> = cam.setups.iter().map(|s| s.name.as_str()).collect();
            Mach::Setup(SetupForm { name: free_name(tr("Setup"), &taken), ..SetupForm::default() })
        }
        "cam.contour" | "cam.pocket" => {
            let pocket = id == "cam.pocket";
            let taken: Vec<&str> = cam.setups.iter().flat_map(|s| s.operations.iter().map(CamOperation::name)).collect();
            let name = free_name(tr(if pocket { "Pocket" } else { "Contour" }), &taken);
            let form = OpForm {
                pocket,
                setup: 0,
                tool: 0,
                name,
                side: 0,
                turn: 0,
                bottom: String::new(),
                top: String::new(),
                stepdown: String::new(),
                stepover: String::new(),
                leave: String::new(),
                ramp: String::new(),
                climb: true,
                tabs: false,
                tab_width: String::new(),
                tab_height: String::new(),
                tab_spacing: String::new(),
                dogbones: false,
                feed: String::new(),
                plunge_feed: String::new(),
                spindle_rpm: String::new(),
            };
            return Some((Kind::Machining(Mach::Op(form)), vec![SelInput::new("Profiles", PROFILES, true)]));
        }
        "cam.delete_op" => Mach::DeleteOp { setup: 0, op: 0 },
        "cam.post" => Mach::Post(PostForm::default()),
        _ => return None,
    };
    Some((Kind::Machining(mach), Vec::new()))
}

/// The dialog's rows (after any selection input); true when Enter was pressed in a value.
pub fn rows(app: &SolveApp, ui: &mut egui::Ui, m: &mut Mach) -> bool {
    let cam = &app.session.doc.cam;
    let tools: Vec<String> = cam.tools.iter().map(|t| tool_label(t.number, t.diameter, &t.description)).collect();
    let setups: Vec<&str> = cam.setups.iter().map(|s| s.name.as_str()).collect();
    let mut enter = false;
    match m {
        Mach::Tool(f) => {
            let mut options = vec![tr("New Tool")];
            options.extend(tools.iter().map(String::as_str));
            choice_row(ui, "Tool", "cam_tool_existing", &mut f.existing, &options);
            if f.existing != f.loaded {
                load_tool(app, f);
            }
            enter |= hint_row(ui, "Number", &mut f.number, "1…9999");
            let kinds: Vec<&str> = TOOL_KINDS.iter().map(|k| k.1).collect();
            choice_row(ui, "Type", "cam_tool_kind", &mut f.kind, &kinds);
            enter |= hint_row(ui, "Diameter", &mut f.diameter, "mm");
            enter |= hint_row(ui, "Flute Length", &mut f.flute_length, "mm");
            match TOOL_KINDS.get(f.kind).map(|k| k.0) {
                Some("vbit") => {
                    enter |= hint_row(ui, "Tip Angle", &mut f.tip_angle, "deg");
                    enter |= hint_row(ui, "Tip Diameter", &mut f.tip_diameter, "0 mm");
                }
                Some("tapered_ball") => {
                    enter |= hint_row(ui, "Taper Angle", &mut f.taper_angle, "deg");
                    enter |= hint_row(ui, "Tip Diameter", &mut f.tip_diameter, "mm");
                }
                _ => {}
            }
            enter |= hint_row(ui, "Flutes", &mut f.flutes, "2");
            enter |= hint_row(ui, "Spindle Speed", &mut f.spindle_rpm, "rev/min");
            enter |= hint_row(ui, "Feed", &mut f.feed, "mm/min");
            enter |= hint_row(ui, "Plunge Feed", &mut f.plunge_feed, "mm/min");
            enter |= hint_row(ui, "Stepdown", &mut f.stepdown, "mm");
            enter |= hint_row(ui, "Stepover", &mut f.stepover, "mm");
            enter |= text_row(ui, "Description", &mut f.description, "").0;
        }
        Mach::Setup(f) => {
            let mut options = vec![tr("New Setup")];
            options.extend(setups.iter().copied());
            choice_row(ui, "Setup", "cam_setup_existing", &mut f.existing, &options);
            if f.existing != f.loaded {
                load_setup(app, f);
            }
            if f.existing == 0 {
                enter |= text_row(ui, "Name", &mut f.name, "").0;
            }
            choice_row(ui, "Work Offset", "cam_setup_offset", &mut f.offset, &OFFSETS);
            form_row(ui, "Stock", |ui| check(ui, &mut f.stock));
            if f.stock {
                for (axis, (lo, hi)) in ["X", "Y", "Z"].iter().zip(f.min.iter_mut().zip(f.max.iter_mut())) {
                    enter |= hint_row(ui, &format!("{} {axis}", tr("Stock Min")), lo, "mm");
                    enter |= hint_row(ui, &format!("{} {axis}", tr("Stock Max")), hi, "mm");
                }
            }
        }
        Mach::Op(f) => {
            if setups.is_empty() || tools.is_empty() {
                status_line(ui, "Add a tool (CAM Tool) and a setup (CAM Setup) first", StatusTone::Warning);
            }
            choice_row(ui, "Setup", "cam_op_setup", &mut f.setup, &setups);
            let tool_labels: Vec<&str> = tools.iter().map(String::as_str).collect();
            choice_row(ui, "Tool", "cam_op_tool", &mut f.tool, &tool_labels);
            enter |= text_row(ui, "Name", &mut f.name, "").0;
            if !f.pocket {
                let sides: Vec<&str> = SIDES.iter().map(|k| k.1).collect();
                choice_row(ui, "Side", "cam_op_side", &mut f.side, &sides);
            }
            choice_row(ui, "Rotate", "cam_op_turn", &mut f.turn, &TURNS);
            enter |= hint_row(ui, "Bottom", &mut f.bottom, "Z, mm");
            enter |= hint_row(ui, "Top", &mut f.top, tr("stock top"));
            enter |= hint_row(ui, "Stepdown", &mut f.stepdown, tr("tool's"));
            if f.pocket {
                enter |= hint_row(ui, "Stepover", &mut f.stepover, tr("tool's"));
            }
            enter |= hint_row(ui, "Stock to Leave", &mut f.leave, "0 mm");
            enter |= hint_row(ui, "Ramp Angle", &mut f.ramp, "0 deg");
            form_row(ui, "Climb", |ui| check(ui, &mut f.climb));
            if !f.pocket {
                form_row(ui, "Tabs", |ui| check(ui, &mut f.tabs));
                if f.tabs {
                    enter |= hint_row(ui, "Tab Width", &mut f.tab_width, "8 mm");
                    enter |= hint_row(ui, "Tab Height", &mut f.tab_height, "3 mm");
                    enter |= hint_row(ui, "Tab Spacing", &mut f.tab_spacing, "250 mm");
                }
            }
            form_row(ui, "Dogbones", |ui| check(ui, &mut f.dogbones));
            enter |= hint_row(ui, "Feed", &mut f.feed, tr("tool's"));
            enter |= hint_row(ui, "Plunge Feed", &mut f.plunge_feed, tr("tool's"));
            enter |= hint_row(ui, "Spindle Speed", &mut f.spindle_rpm, tr("tool's"));
        }
        Mach::DeleteOp { setup, op } => {
            choice_row(ui, "Setup", "cam_del_setup", setup, &setups);
            let ops: Vec<&str> = cam.setups.get(*setup).map(|s| s.operations.iter().map(CamOperation::name).collect()).unwrap_or_default();
            choice_row(ui, "Operation", "cam_del_op", op, &ops);
            if ops.is_empty() {
                status_line(ui, "This setup has no operations", StatusTone::Dim);
            }
        }
        Mach::Post(f) => {
            choice_row(ui, "Setup", "cam_post_setup", &mut f.setup, &setups);
            let mut posts = vec!["RS-274"];
            posts.extend(cam.posts.iter().map(|p| p.name.as_str()));
            choice_row(ui, "Post", "cam_post_post", &mut f.post, &posts);
            enter |= text_row(ui, "Program", &mut f.program, setups.get(f.setup).copied().unwrap_or_default()).0;
            let default_file = default_file(app, f);
            enter |= text_row(ui, "File", &mut f.file, &default_file).0;
        }
    }
    enter
}

fn tool_label(number: u32, diameter: f64, description: &str) -> String {
    let d = shown(diameter);
    if description.is_empty() { format!("T{number}: {d} mm") } else { format!("T{number}: {d} mm {description}") }
}

/// The fields of the tool picked in Tool, or a new tool's.
fn load_tool(app: &SolveApp, f: &mut ToolForm) {
    f.loaded = f.existing;
    let Some(t) = f.existing.checked_sub(1).and_then(|i| app.session.doc.cam.tools.get(i)) else {
        let next = (1u32..).find(|n| app.session.doc.cam.tool(*n).is_none()).unwrap_or(1);
        *f = ToolForm { number: next.to_string(), flutes: "2".into(), ..ToolForm::default() };
        return;
    };
    let kind = serde_json::to_value(t.kind).ok();
    f.number = t.number.to_string();
    f.kind = TOOL_KINDS.iter().position(|k| kind.as_ref().and_then(Value::as_str) == Some(k.0)).unwrap_or(0);
    f.diameter = shown(t.diameter);
    f.flute_length = shown(t.flute_length);
    f.flutes = t.flutes.to_string();
    f.spindle_rpm = shown_opt(t.spindle_rpm);
    f.feed = shown_opt(t.feed);
    f.plunge_feed = shown_opt(t.plunge_feed);
    f.stepdown = shown_opt(t.stepdown);
    f.stepover = shown_opt(t.stepover);
    f.tip_angle = shown_opt(t.tip_angle);
    f.taper_angle = shown_opt(t.taper_angle);
    f.tip_diameter = if t.tip_diameter > 0.0 { shown(t.tip_diameter) } else { String::new() };
    f.description = t.description.clone();
}

/// The fields of the setup picked in Setup, or a new setup's.
fn load_setup(app: &SolveApp, f: &mut SetupForm) {
    f.loaded = f.existing;
    let cam = &app.session.doc.cam;
    let Some(s) = f.existing.checked_sub(1).and_then(|i| cam.setups.get(i)) else {
        let taken: Vec<&str> = cam.setups.iter().map(|s| s.name.as_str()).collect();
        *f = SetupForm { name: free_name(tr("Setup"), &taken), ..SetupForm::default() };
        return;
    };
    f.name = s.name.clone();
    f.offset = usize::from(s.work_offset.clamp(1, 6) - 1);
    f.stock = s.stock.is_some();
    if let Some(b) = &s.stock {
        f.min = b.min.map(shown);
        f.max = b.max.map(shown);
    }
}

/// Where Post Process writes when File is left empty: beside the design, named after the
/// program, with the post's extension.
fn default_file(app: &SolveApp, f: &PostForm) -> String {
    let cam = &app.session.doc.cam;
    let Some(setup) = cam.setups.get(f.setup) else { return String::new() };
    let ext = f.post.checked_sub(1).and_then(|i| cam.posts.get(i)).map_or("nc", |p| p.extension.as_str());
    let program = if f.program.trim().is_empty() { setup.name.as_str() } else { f.program.trim() };
    let name = format!("{}.{ext}", program.replace(['/', '\\'], "_"));
    match app.session.path.as_deref().map(std::path::Path::new).and_then(std::path::Path::parent) {
        Some(dir) if !dir.as_os_str().is_empty() => dir.join(name).to_string_lossy().into_owned(),
        _ => name,
    }
}

/// A length typed in the design's units (parameters allowed), in millimetres; `None` when empty.
fn length(app: &SolveApp, text: &str, what: &str) -> Result<Option<f64>, String> {
    value(app, text, what, expr::Kind::Length)
}

fn value(app: &SolveApp, text: &str, what: &str, kind: expr::Kind) -> Result<Option<f64>, String> {
    let text = text.trim();
    if text.is_empty() {
        return Ok(None);
    }
    let doc = &app.session.doc;
    let (values, _) = doc.param_values();
    let lookup = |name: &str| values.get(name).copied().ok_or_else(|| solidcraft_engine::doc::DocError::Expr(format!("unknown parameter `{name}`")));
    let scale = solidcraft_engine::doc::display_units::unit_scale(&doc.units).0;
    let defaults = expr::Defaults { len: 1.0 / scale, ..expr::Defaults::default() };
    let v = expr::eval_with_defaults(&text.replace(',', "."), &lookup, defaults)
        .and_then(|v| v.to_kind_in(kind, defaults))
        .map_err(|_| format!("{}: not a value ({text})", tr(what)))?;
    if v.is_finite() { Ok(Some(v)) } else { Err(format!("{}: not a value ({text})", tr(what))) }
}

/// A plain number: a speed (rev/min), a feed (mm/min) or a count.
fn number(app: &SolveApp, text: &str, what: &str) -> Result<Option<f64>, String> {
    value(app, text, what, expr::Kind::Unitless)
}

fn required(v: Option<f64>, what: &str) -> Result<f64, String> {
    v.ok_or_else(|| format!("{} is required", tr(what)))
}

/// Puts `v` under `key` when there is one.
fn put_num(p: &mut Value, key: &str, v: Option<f64>) {
    if let Some(v) = v {
        p[key] = json!(v);
    }
}

/// The command the dialog's OK runs.
pub fn commands(app: &SolveApp, m: &Mach, inputs: &[SelInput]) -> Result<Vec<(String, Value)>, String> {
    let cam = &app.session.doc.cam;
    let setup_name = |i: usize| cam.setups.get(i).map(|s| s.name.clone()).ok_or_else(|| tr("Add a setup first (CAM Setup)").to_string());
    let p = match m {
        Mach::Tool(f) => {
            let n = number(app, &f.number, "Number")?.filter(|n| n.fract() == 0.0 && (1.0..=9999.0).contains(n));
            let n = n.ok_or_else(|| format!("{} must be a whole number 1…9999", tr("Number")))?;
            let mut p = json!({
                "number": n as u32,
                "kind": TOOL_KINDS.get(f.kind).map_or("flat_endmill", |k| k.0),
                "diameter": required(length(app, &f.diameter, "Diameter")?, "Diameter")?,
                "flute_length": required(length(app, &f.flute_length, "Flute Length")?, "Flute Length")?,
            });
            put_num(&mut p, "flutes", number_or(app, &f.flutes, "Flutes")?);
            put_num(&mut p, "spindle_rpm", number(app, &f.spindle_rpm, "Spindle Speed")?);
            put_num(&mut p, "feed", number(app, &f.feed, "Feed")?);
            put_num(&mut p, "plunge_feed", number(app, &f.plunge_feed, "Plunge Feed")?);
            put_num(&mut p, "stepdown", length(app, &f.stepdown, "Stepdown")?);
            put_num(&mut p, "stepover", length(app, &f.stepover, "Stepover")?);
            match TOOL_KINDS.get(f.kind).map(|k| k.0) {
                Some("vbit") => {
                    put_num(&mut p, "tip_angle", value(app, &f.tip_angle, "Tip Angle", expr::Kind::Angle)?.map(f64::to_degrees));
                    put_num(&mut p, "tip_diameter", length(app, &f.tip_diameter, "Tip Diameter")?);
                }
                Some("tapered_ball") => {
                    put_num(&mut p, "taper_angle", value(app, &f.taper_angle, "Taper Angle", expr::Kind::Angle)?.map(f64::to_degrees));
                    put_num(&mut p, "tip_diameter", length(app, &f.tip_diameter, "Tip Diameter")?);
                }
                _ => {}
            }
            if !f.description.trim().is_empty() {
                p["description"] = json!(f.description.trim());
            }
            p
        }
        Mach::Setup(f) => {
            let name = if f.existing == 0 { f.name.trim().to_string() } else { setup_name(f.existing - 1)? };
            if name.is_empty() {
                return Err(format!("{} is required", tr("Name")));
            }
            let mut p = json!({"name": name, "work_offset": f.offset + 1});
            if f.stock {
                let corner = |c: &[String; 3], what: &str| -> Result<[f64; 3], String> {
                    let mut out = [0.0; 3];
                    for ((o, text), axis) in out.iter_mut().zip(c).zip(["X", "Y", "Z"]) {
                        let label = format!("{} {axis}", tr(what));
                        *o = required(length(app, text, &label)?, &label)?;
                    }
                    Ok(out)
                };
                p["stock"] = json!({"min": corner(&f.min, "Stock Min")?, "max": corner(&f.max, "Stock Max")?});
            } else if f.existing != 0 {
                p["stock"] = Value::Null;
            }
            p
        }
        Mach::Op(f) => {
            let setup = setup_name(f.setup)?;
            let tool = cam.tools.get(f.tool).map(|t| t.number).ok_or_else(|| tr("Add a tool first (CAM Tool)").to_string())?;
            let profiles = crate::selection::items(inputs, 0).profiles();
            let Some(sketch) = profiles.first().map(|x| x.0) else { return Err(tr("Pick the profiles to cut").to_string()) };
            if profiles.iter().any(|x| x.0 != sketch) {
                return Err(tr("The profiles must be in one sketch").to_string());
            }
            let name = f.name.trim();
            if name.is_empty() {
                return Err(format!("{} is required", tr("Name")));
            }
            let mut p = json!({
                "setup": setup,
                "name": name,
                "tool": tool,
                "sketch": sketch,
                "profiles": profiles.iter().map(|x| x.1).collect::<Vec<_>>(),
                "bottom": required(length(app, &f.bottom, "Bottom")?, "Bottom")?,
                "climb": f.climb,
            });
            if !f.pocket {
                p["side"] = json!(SIDES.get(f.side).map_or("outside", |k| k.0));
            }
            if f.turn != 0 {
                p["rotate"] = json!(f.turn * 90);
            }
            put_num(&mut p, "top", length(app, &f.top, "Top")?);
            put_num(&mut p, "stepdown", length(app, &f.stepdown, "Stepdown")?);
            if f.pocket {
                put_num(&mut p, "stepover", length(app, &f.stepover, "Stepover")?);
            }
            put_num(&mut p, "stock_to_leave", length(app, &f.leave, "Stock to Leave")?);
            put_num(&mut p, "ramp_angle", value(app, &f.ramp, "Ramp Angle", expr::Kind::Angle)?.map(f64::to_degrees));
            if !f.pocket && f.tabs {
                let mut tabs = Map::new();
                for (key, text, what) in
                    [("width", &f.tab_width, "Tab Width"), ("height", &f.tab_height, "Tab Height"), ("spacing", &f.tab_spacing, "Tab Spacing")]
                {
                    if let Some(v) = length(app, text, what)? {
                        tabs.insert(key.into(), json!(v));
                    }
                }
                p["tabs"] = if tabs.is_empty() { json!(true) } else { Value::Object(tabs) };
            }
            if f.dogbones {
                p["dogbones"] = json!(true);
            }
            put_num(&mut p, "feed", number(app, &f.feed, "Feed")?);
            put_num(&mut p, "plunge_feed", number(app, &f.plunge_feed, "Plunge Feed")?);
            put_num(&mut p, "spindle_rpm", number(app, &f.spindle_rpm, "Spindle Speed")?);
            p
        }
        Mach::DeleteOp { setup, op } => {
            let name = setup_name(*setup)?;
            let op = cam.setups.get(*setup).and_then(|s| s.operations.get(*op)).ok_or_else(|| tr("This setup has no operations").to_string())?;
            json!({"setup": name, "name": op.name()})
        }
        Mach::Post(f) => {
            let mut p = json!({"setup": setup_name(f.setup)?});
            if let Some(post) = f.post.checked_sub(1).and_then(|i| cam.posts.get(i)) {
                p["post"] = json!(post.name);
            }
            if !f.program.trim().is_empty() {
                p["program"] = json!(f.program.trim());
            }
            let file = if f.file.trim().is_empty() { default_file(app, f) } else { f.file.trim().to_string() };
            if !file.is_empty() {
                p["path"] = json!(file);
            }
            p
        }
    };
    Ok(vec![(m.command().to_string(), p)])
}

/// A count where empty means the command's default.
fn number_or(app: &SolveApp, text: &str, what: &str) -> Result<Option<f64>, String> {
    let v = number(app, text, what)?;
    match v {
        Some(n) if n.fract() != 0.0 || n < 1.0 => Err(format!("{} must be a whole number", tr(what))),
        other => Ok(other),
    }
}

#[cfg(test)]
#[path = "dialogs_machining_tests.rs"]
mod tests;
