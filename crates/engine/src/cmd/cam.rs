//! CAM commands: tools, setups, 2D contour/pocket/engraving/v-carving/drilling operations, and G-code posting through
//! `solidcraft-cam`. Toolpaths are computed at post time and never stored on the design.

use serde_json::{Value, json};
use solidcraft_cam::{
    Cut, DrillParams, Feeds, Heightmap, Heights, Operation, Post, Program, Rest, Side, Tabs, Tool, ToolKind, contour, drill, emit, engrave,
    join_chains, pocket, readback, vcarve,
};
use solidcraft_doc::ProfileSel;
use solidcraft_doc::cam::{
    CamOperation, CamSettings, CamSetup, HolePoint, MAX_CHAINS, MAX_DRILL_HOLES, MAX_LOOP_POINTS, MAX_OPS, MAX_POSTS, MAX_SETUPS, MAX_TOOLS,
    PolygonRegion, PolylineChain, StockBox, default_contour_params, default_cut, default_engrave_params, default_pocket_params,
    default_vcarve_params,
};
use solidcraft_geom::{Region2, Vec2};
use solidcraft_sketch::CurveKind;

use super::CommandSpec;
use crate::params::{bad, bool_, num, str_, string_list, vec2, vec2_list};
use crate::{EngineError, Result, Session};

/// Chord error of sketch profiles turned into an operation's polygons (mm).
const PROFILE_TOLERANCE: f64 = 0.005;

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.tool", "CAM Tool", tool_cmd).at("MANUFACTURE", "SETUP").icon("tool").params(
        "number (1…9999); kind?: flat_endmill|ball_endmill|drill|vbit|tapered_ball (default flat_endmill); diameter (mm, the widest \
         the cutting edges get); flute_length (mm); tip_angle (vbit: included tip angle, degrees); taper_angle (tapered_ball: per side, \
         degrees); tip_diameter? (vbit: the flat at the tip, default 0 for a sharp point; tapered_ball: its ball, required); description?; \
         flutes?; spindle_rpm?; feed?; plunge_feed?; stepdown?; stepover?; delete?: bool. A tool that narrows to its tip cuts a contour \
         or pocket wall with its top edge on the profile; a pocket floor needs a tip flat",
    ),
    CommandSpec::new("cam.setup", "CAM Setup", setup_cmd).at("MANUFACTURE", "SETUP").icon("setup").params(
        "name; work_offset?: 1…6 (G54…G59; a new setup takes 1); stock?: {min:[x,y,z], max:[x,y,z]} (work coordinates; \
             the operations' clearance and retract heights must be above its top) | null (none); an update changes only what \
             it is given; delete?: bool",
    ),
    CommandSpec::new("cam.contour", "2D Contour", contour_cmd).at("MANUFACTURE", "MILLING").icon("contour").params(
        "setup; name; tool (number); outer: [[x,y],…] (or regions: [{outer, holes?}], or sketch: id|name with profiles?: \
         all|[index]|[{point:[x,y]}] (default: every profile that is not the inside of another's hole; copied as polygons \
         within 0.005 mm), rotate?: 0|90|180|270 (deg, about the sketch origin), at?: [x,y] (where the lower left corner \
         of the profiles' bounds goes; default where they are)); holes?: [[[x,y],…]]; side?: outside|inside|on (default \
         outside: a part with holes is cut free, every hole before the outline round it); climb?: bool (default true); \
         stock_to_leave?; tolerance?; ramp_angle?: deg (default 0); tabs?: true (8 wide, 3 high, about every 250 mm, \
         holes too) | {width?, height?, spacing?, inner?: bool (a part's holes get tabs too, default true)} (mm; bridges \
         left on the last passes so a part cut through stays in the stock; none on a loop shorter than four of them); \
         dogbones?: bool (notch every inside corner the tool leaves round, so a square part fits; default false); \
         top? (default the setup's stock top, else 0); retract? (default top + 2); clearance? (default top + 10); bottom \
         (required unless cut given); stepdown?; feed?; plunge_feed?; spindle_rpm? (stepdown, feeds and speed default to the \
         tool's; refused when neither has them); coolant?; rest_from?: name of an earlier contour of this setup on the same side, \
         cut with a larger tool: this one then cuts only the corners and openings that tool could not reach (no tabs, dogbones or ramps); \
         cut?: {heights, feeds, stepdown} → {setup, operation, bounds, too_small?: openings the tool does not fit, left uncut, \
         rest?: {from, cut_length} | {from, nothing_left: true} (then nothing is saved)}",
    ),
    CommandSpec::new("cam.pocket", "2D Pocket", pocket_cmd).at("MANUFACTURE", "MILLING").icon("pocket").params(
        "setup; name; tool (number); outer: [[x,y],…] (or regions, or sketch with profiles?, rotate?, at? as cam.contour \
         takes them); holes?; stepover? (default the tool's, else half its diameter); climb?; stock_to_leave?; tolerance?; \
         ramp_angle?: deg; dogbones?: bool (notch the pocket's inside corners, so a square part fits); rest_from?: name of an \
         earlier pocket of this setup cleared with a larger tool: this one then cuts only its wall ring's corners and what it did not \
         fit into (no dogbones or ramps); clearance?; retract?; top?; bottom; stepdown?; feed?; plunge_feed?; spindle_rpm?; coolant?; \
         cut? → {setup, operation, bounds, too_small?, rest?: {from, cut_length} | {from, nothing_left: true} (then nothing is saved)}",
    ),
    CommandSpec::new("cam.engrave", "Engrave", engrave_cmd).icon("contour").params(
        "setup; name; tool (number); sketch: id|name with curves?: all | [ids] (default all drawn curves: lines, arcs, circles, splines \
         and text outlines; not construction, centreline or projected geometry), rotate?, at? (as cam.contour takes them) | paths: \
         [[[x,y],…],…] (a path whose last point is its first is closed); the tool centre follows each curve, open or closed, at every \
         depth pass (an open one back along itself), nothing is offset; top?; retract?; clearance?; bottom; stepdown?; feed?; \
         plunge_feed?; spindle_rpm?; coolant?; tolerance? (ends closer than this are one curve, default 0.01) → {setup, operation, \
         bounds, chains}",
    ),
    CommandSpec::new("cam.vcarve", "V-Carve", vcarve_cmd).icon("contour").params(
        "setup; name; tool (a vbit); outer | regions | sketch with profiles?, rotate?, at? (as cam.contour takes them); the V-bit runs \
         the inward offsets of the regions, each as deep as its cone touches the wall there: sharp edges at the top, a ridge where the \
         offsets meet; max_depth? (mm below the top; inner passes stay there and the tip clears a flat floor; required when the carving \
         would go deeper than the tool's flutes); stepover? (mm between passes, default 0.1); tolerance?; top?; retract?; clearance?; \
         feed?; plunge_feed?; spindle_rpm?; coolant? → {setup, operation, bounds, depth, rings, flat_clearing?: {wall_offset, bottom} (the \
         floor a flat end mill could clear instead: cam.pocket with stock_to_leave = wall_offset − its radius, bottom)}",
    ),
    CommandSpec::new("cam.drill", "Drill", drill_cmd).icon("hole").params(
        "setup; name; tool (number; any kind, a V-bit with a depth per point makes halftone dots); points: [[x,y],…] | [[x,y,z],…] \
         (z: that hole's bottom, work Z) | sketch: id|name (its lone points and circle centres, with rotate?, at? as cam.contour takes \
         them); bottom | depth (mm below the top: the holes' bottom unless a point has its own z); top?; retract?; clearance?; peck? \
         (mm per peck, back up to the retract height between pecks); dwell? (seconds at the bottom); plunge_feed? (the feed down; \
         `feed` and `stepdown` are not needed); spindle_rpm?; coolant? → {setup, operation, bounds, holes}",
    ),
    CommandSpec::new("cam.delete_op", "Delete CAM Operation", delete_op).at("MANUFACTURE", "MILLING").icon("delete").params("setup; name"),
    CommandSpec::new("cam.post", "Post Process", post_cmd).at("MANUFACTURE", "POST").icon("post").noundo().params(
        "setup; post?: rs274 | name of a saved post (default rs274); program?: name (default the setup's); \
         verify?: bool (default true: read the G-code back and check it against the toolpaths); path?: file to write the \
         program to (then the result has `lines` instead of `gcode`) → {gcode | path, lines, extension, post, operations, \
         readback?}",
    ),
    CommandSpec::new("cam.simulate", "Simulate Stock", simulate_cmd).icon("post").noundo().params(
        "setup; operations?: [names] (default all, in order); resolution?: mm per pixel (default 0.2); stock?: {min:[x,y,z], \
         max:[x,y,z]} (default the setup's; required when it has none); path: the PNG to write (16-bit grey, the stock top white, the \
         deepest cut black, +Y up) → {path, size: [w, h], resolution, stock, deepest, removed_mm3, operations: [{name, removed_mm3, \
         standing_in_regions_percent? (a pocket or v-carve: how much of its regions still stands above its bottom, what the tool \
         could not reach)}]}",
    ),
    CommandSpec::new("cam.post.define", "Define Post", post_define_cmd).icon("post").params(
        "post: {name, extension, units, decimals, feed_decimals, comments, header, tool_change, tool_change_first, spindle_on, \
         spindle_off, coolant_on, coolant_off, footer, arcs, arc_tolerance, …} (a post as data, saved with the design and \
         replacing one of the same name) | name + delete: true",
    ),
    CommandSpec::new("cam.list", "List CAM", list_cmd).icon("list").noundo().params("→ tools, setups (with operations) and saved posts"),
];

fn tool_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.tool";
    let number = num(p, "number")
        .filter(|n| *n >= 1.0 && *n <= 9999.0 && n.fract() == 0.0)
        .map(|n| n as u32)
        .ok_or_else(|| bad(cmd, "`number` must be an integer 1…9999"))?;
    if bool_(p, "delete").unwrap_or(false) {
        let users: Vec<String> = s
            .doc
            .cam
            .setups
            .iter()
            .flat_map(|x| x.operations.iter().filter(|o| o.tool() == number).map(move |o| format!("`{}`/`{}`", x.name, o.name())))
            .collect();
        if !users.is_empty() {
            return Err(bad(cmd, format!("tool {number} is used by {}: delete those operations or give them another tool first", users.join(", "))));
        }
        let doc = s.doc_mut();
        let before = doc.cam.tools.len();
        doc.cam.tools.retain(|t| t.number != number);
        if doc.cam.tools.len() == before {
            return Err(bad(cmd, format!("no tool {number}")));
        }
        return Ok(json!({"deleted": number}));
    }
    let kind = match str_(p, "kind").unwrap_or("flat_endmill") {
        "flat_endmill" => ToolKind::FlatEndmill,
        "ball_endmill" => ToolKind::BallEndmill,
        "drill" => ToolKind::Drill,
        "vbit" | "v_bit" => ToolKind::VBit,
        "tapered_ball" => ToolKind::TaperedBall,
        other => return Err(bad(cmd, format!("unknown tool kind `{other}` (flat_endmill, ball_endmill, drill, vbit, tapered_ball)"))),
    };
    let diameter = num(p, "diameter").ok_or_else(|| bad(cmd, "`diameter` is required (mm)"))?;
    let flute_length = num(p, "flute_length").ok_or_else(|| bad(cmd, "`flute_length` is required (mm)"))?;
    let mut tool = Tool::new(number, kind, diameter, flute_length);
    if let Some(d) = str_(p, "description") {
        tool.description = d.chars().take(256).collect();
    }
    if let Some(f) = num(p, "flutes").filter(|n| *n >= 1.0 && *n <= 100.0 && n.fract() == 0.0) {
        tool.flutes = f as u32;
    }
    tool.spindle_rpm = num(p, "spindle_rpm").or(tool.spindle_rpm);
    tool.feed = num(p, "feed").or(tool.feed);
    tool.plunge_feed = num(p, "plunge_feed").or(tool.plunge_feed);
    tool.stepdown = num(p, "stepdown").or(tool.stepdown);
    tool.stepover = num(p, "stepover").or(tool.stepover);
    tool.tip_angle = num(p, "tip_angle");
    tool.taper_angle = num(p, "taper_angle");
    tool.tip_diameter = num(p, "tip_diameter").unwrap_or(0.0);
    tool.validate().map_err(|e| bad(cmd, e.to_string()))?;
    let doc = s.doc_mut();
    if doc.cam.tools.len() >= MAX_TOOLS && !doc.cam.tools.iter().any(|t| t.number == number) {
        return Err(bad(cmd, format!("a design has at most {MAX_TOOLS} tools")));
    }
    match doc.cam.tools.iter_mut().find(|t| t.number == number) {
        Some(slot) => *slot = tool.clone(),
        None => doc.cam.tools.push(tool.clone()),
    }
    doc.cam.tools.sort_by_key(|t| t.number);
    Ok(json!({"tool": tool}))
}

fn setup_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.setup";
    let name = str_(p, "name").map(str::trim).filter(|n| !n.is_empty() && n.len() <= 128).ok_or_else(|| bad(cmd, "`name` is required"))?.to_string();
    if bool_(p, "delete").unwrap_or(false) {
        let doc = s.doc_mut();
        let before = doc.cam.setups.len();
        doc.cam.setups.retain(|x| x.name != name);
        if doc.cam.setups.len() == before {
            return Err(bad(cmd, format!("no setup `{name}`")));
        }
        return Ok(json!({"deleted": name}));
    }
    let work_offset = match p.get("work_offset") {
        None => None,
        Some(_) => Some(
            num(p, "work_offset")
                .filter(|n| n.fract() == 0.0 && (1.0..=6.0).contains(n))
                .map(|n| n as u8)
                .ok_or_else(|| bad(cmd, "`work_offset` must be an integer 1…6 (G54…G59)"))?,
        ),
    };
    let stock = match p.get("stock") {
        None => None,
        Some(Value::Null) => Some(None),
        Some(v) => Some(Some(stock_box(cmd, v)?)),
    };
    let mut setup = s.doc.cam.setup(&name).cloned().unwrap_or_else(|| CamSetup::new(&name));
    if let Some(w) = work_offset {
        setup.work_offset = w;
    }
    if let Some(stock) = stock {
        setup.stock = stock;
    }
    setup.validate().map_err(|e| bad(cmd, e.to_string()))?;
    let doc = s.doc_mut();
    if doc.cam.setups.len() >= MAX_SETUPS && !doc.cam.setups.iter().any(|x| x.name == name) {
        return Err(bad(cmd, format!("a design has at most {MAX_SETUPS} setups")));
    }
    match doc.cam.setups.iter_mut().find(|x| x.name == name) {
        Some(slot) => {
            slot.work_offset = setup.work_offset;
            slot.stock = setup.stock;
        }
        None => doc.cam.setups.push(setup.clone()),
    }
    Ok(json!({"setup": setup_json(s.doc.cam.setup(&name).unwrap_or(&setup))}))
}

fn stock_box(cmd: &str, v: &Value) -> Result<StockBox> {
    let min = v.get("min").and_then(Value::as_array).filter(|a| a.len() == 3);
    let max = v.get("max").and_then(Value::as_array).filter(|a| a.len() == 3);
    let (Some(min), Some(max)) = (min, max) else {
        return Err(bad(cmd, "`stock` must be {min:[x,y,z], max:[x,y,z]}"));
    };
    let parse = |a: &[Value]| -> Result<[f64; 3]> {
        Ok([
            a[0].as_f64().filter(|x| x.is_finite()).ok_or_else(|| bad(cmd, "stock coordinates must be finite"))?,
            a[1].as_f64().filter(|x| x.is_finite()).ok_or_else(|| bad(cmd, "stock coordinates must be finite"))?,
            a[2].as_f64().filter(|x| x.is_finite()).ok_or_else(|| bad(cmd, "stock coordinates must be finite"))?,
        ])
    };
    let box_ = StockBox { min: parse(min)?, max: parse(max)? };
    box_.validate().map_err(|e| bad(cmd, e.to_string()))?;
    Ok(box_)
}

fn contour_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    op_cmd(s, p, "cam.contour", true)
}

fn pocket_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    op_cmd(s, p, "cam.pocket", false)
}

/// What every operation command starts with: its setup, its name and its tool.
struct OpHead {
    setup: String,
    name: String,
    tool_n: u32,
    tool: Tool,
    /// The top of the setup's stock, where the cut's top defaults to.
    stock_top: Option<f64>,
}

fn op_head(s: &Session, cmd: &str, p: &Value) -> Result<OpHead> {
    let setup = str_(p, "setup").map(str::trim).filter(|n| !n.is_empty()).ok_or_else(|| bad(cmd, "`setup` is required"))?.to_string();
    let name = str_(p, "name").map(str::trim).filter(|n| !n.is_empty() && n.len() <= 128).ok_or_else(|| bad(cmd, "`name` is required"))?.to_string();
    let tool_n =
        num(p, "tool").filter(|n| *n >= 1.0 && n.fract() == 0.0).map(|n| n as u32).ok_or_else(|| bad(cmd, "`tool` must be a tool number"))?;
    let tool = s.doc.cam.tool(tool_n).cloned().ok_or_else(|| bad(cmd, format!("no tool {tool_n} (cam.tool first)")))?;
    let stock_top = s.doc.cam.setup(&setup).ok_or_else(|| bad(cmd, format!("no setup `{setup}` (cam.setup first)")))?.stock.map(|b| b.max[2]);
    Ok(OpHead { setup, name, tool_n, tool, stock_top })
}

/// A machine runs what the program says: no speed made up for a tool that has none.
fn spindle_param(cmd: &str, p: &Value, tool: &Tool) -> Result<Option<f64>> {
    num(p, "spindle_rpm")
        .or(tool.spindle_rpm)
        .map(Some)
        .ok_or_else(|| bad(cmd, format!("no spindle speed: give `spindle_rpm` or set tool {}'s (cam.tool spindle_rpm)", tool.number)))
}

/// Checks `op`, proves its toolpath builds, and stores it in its setup (replacing one of the
/// same name). The reply names the setup and the operation, gives the bounds of what it cuts and
/// the openings its tool does not fit, and carries `extra`'s fields.
fn save_op(s: &mut Session, cmd: &str, head: &OpHead, op: CamOperation, extra: Value) -> Result<Value> {
    op.validate().map_err(|e| bad(cmd, e.to_string()))?;
    if let Some(setup) = s.doc.cam.setup(&head.setup) {
        setup.check_clear_of_stock(&op).map_err(|e| bad(cmd, e.to_string()))?;
    }
    // Prove the toolpath builds before saving the operation.
    build_path(&head.tool, &op).map_err(|e| bad(cmd, e))?;
    let (bounds, too_small) = reach_report(&head.tool, &op);
    let doc = s.doc_mut();
    let setup = doc.cam.setup_mut(&head.setup).ok_or_else(|| bad(cmd, format!("no setup `{}` (cam.setup first)", head.setup)))?;
    if let Some(slot) = setup.operations.iter_mut().find(|o| o.name() == head.name) {
        *slot = op;
    } else {
        if setup.operations.len() >= MAX_OPS {
            return Err(bad(cmd, format!("a setup has at most {MAX_OPS} operations")));
        }
        setup.operations.push(op);
    }
    let mut out = json!({"setup": head.setup, "operation": head.name, "bounds": bounds});
    if !too_small.is_empty() {
        out["too_small"] = json!(too_small);
    }
    if let (Some(out), Some(extra)) = (out.as_object_mut(), extra.as_object()) {
        for (k, v) in extra {
            out.insert(k.clone(), v.clone());
        }
    }
    Ok(out)
}

fn op_cmd(s: &mut Session, p: &Value, cmd: &str, is_contour: bool) -> Result<Value> {
    let head = op_head(s, cmd, p)?;
    let (tool, tool_n, name) = (&head.tool, head.tool_n, head.name.clone());
    let regions = regions_param(s, cmd, p)?;
    let cut = cut_param(cmd, p, tool, head.stock_top, MILLING)?;
    let spindle_rpm = spindle_param(cmd, p, tool)?;
    let coolant = bool_(p, "coolant").unwrap_or(false);
    let depth = cut.heights.top - cut.heights.bottom;
    let rest_from = str_(p, "rest_from").map(str::trim).filter(|n| !n.is_empty()).map(str::to_string);
    let rest = match &rest_from {
        None => None,
        Some(from) => Some(rest_after(s, cmd, &head.setup, from, tool, depth, is_contour, str_(p, "side"))?),
    };
    let op = if is_contour {
        let mut params = default_contour_params();
        if let Some(side) = str_(p, "side") {
            params.side = parse_side(cmd, side)?;
        }
        if let Some(c) = bool_(p, "climb") {
            params.climb = c;
        }
        if let Some(v) = num(p, "stock_to_leave") {
            params.stock_to_leave = v;
        }
        if let Some(v) = num(p, "tolerance") {
            params.tolerance = v;
        }
        if let Some(deg) = num(p, "ramp_angle") {
            params.ramp_angle = deg.to_radians();
        }
        params.tabs = tabs_param(cmd, p)?;
        params.dogbones = bool_(p, "dogbones").unwrap_or(false);
        params.rest = rest;
        CamOperation::Contour { name, tool: tool_n, regions, params, cut, spindle_rpm, coolant, rest_from: rest_from.clone() }
    } else {
        if !(tool.floor_diameter() > 0.0) {
            return Err(bad(
                cmd,
                format!(
                    "tool {tool_n} narrows to a point and cannot clear a pocket floor: give it a `tip_diameter`, use a flat end mill, or cam.engrave the outline"
                ),
            ));
        }
        let stepover = num(p, "stepover").or(tool.stepover).unwrap_or(tool.floor_diameter() * 0.5);
        let mut params = default_pocket_params(stepover);
        if let Some(c) = bool_(p, "climb") {
            params.climb = c;
        }
        if let Some(v) = num(p, "stock_to_leave") {
            params.stock_to_leave = v;
        }
        if let Some(v) = num(p, "tolerance") {
            params.tolerance = v;
        }
        if let Some(deg) = num(p, "ramp_angle") {
            params.ramp_angle = deg.to_radians();
        }
        params.dogbones = bool_(p, "dogbones").unwrap_or(false);
        params.rest = rest;
        CamOperation::Pocket { name, tool: tool_n, regions, params, cut, spindle_rpm, coolant, rest_from: rest_from.clone() }
    };
    let mut extra = json!({});
    if let Some(from) = &rest_from {
        // What is left for this tool: nothing is an answer, not an operation.
        match build_path(tool, &op) {
            Err(e) if e.starts_with("nothing to cut") => {
                return Ok(json!({"setup": head.setup, "operation": head.name, "rest": {"from": from, "nothing_left": true}}));
            }
            Err(e) => return Err(bad(cmd, e)),
            Ok(path) => extra["rest"] = json!({"from": from, "cut_length": path.stats().cut_length}),
        }
    }
    save_op(s, cmd, &head, op, extra)
}

/// How the earlier operation `from` of `setup` cut, for a rest cut after it with `tool`: a
/// contour after a contour on the same side, a pocket after a pocket, with a larger tool.
#[allow(clippy::too_many_arguments)]
fn rest_after(s: &Session, cmd: &str, setup: &str, from: &str, tool: &Tool, depth: f64, is_contour: bool, side: Option<&str>) -> Result<Rest> {
    let earlier = s
        .doc
        .cam
        .setup(setup)
        .and_then(|x| x.operations.iter().find(|o| o.name() == from))
        .ok_or_else(|| bad(cmd, format!("no operation `{from}` in setup `{setup}` to rest after")))?;
    let (leave, earlier_cut) = match (earlier, is_contour) {
        (CamOperation::Contour { params, cut, .. }, true) => {
            let same_side = match side.unwrap_or("outside") {
                "outside" => params.side == Side::Outside,
                "inside" => params.side == Side::Inside,
                _ => false,
            };
            if !same_side {
                return Err(bad(cmd, format!("`{from}` runs on another side of the profile; rest machining follows a contour on the same side")));
            }
            (params.stock_to_leave, cut)
        }
        (CamOperation::Pocket { params, cut, .. }, false) => (params.stock_to_leave, cut),
        (CamOperation::Contour { .. }, false) => return Err(bad(cmd, format!("`{from}` is a contour; a pocket rests after a pocket"))),
        (CamOperation::Pocket { .. }, true) => return Err(bad(cmd, format!("`{from}` is a pocket; a contour rests after a contour"))),
        _ => return Err(bad(cmd, format!("`{from}` is no contour or pocket to rest after"))),
    };
    let earlier_tool =
        s.doc.cam.tool(earlier.tool()).ok_or_else(|| bad(cmd, format!("`{from}` uses tool {}, which the design no longer has", earlier.tool())))?;
    let radius = earlier_tool.radius_at(earlier_cut.heights.top - earlier_cut.heights.bottom);
    if !(radius > tool.radius_at(depth) + 1e-9) {
        return Err(bad(cmd, format!("tool {} is not smaller than tool {} of `{from}`: nothing is left for it", tool.number, earlier_tool.number)));
    }
    Ok(Rest { radius, wall: radius + leave })
}

fn engrave_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.engrave";
    let head = op_head(s, cmd, p)?;
    let mut params = default_engrave_params();
    if let Some(v) = num(p, "tolerance").filter(|t| *t > 0.0 && t.is_finite()) {
        params.tolerance = v;
    }
    // Pieces whose ends meet are one curve: saved joined, so the design shows what is engraved.
    let chains: Vec<PolylineChain> = join_chains(chains_param(s, cmd, p)?.iter().map(PolylineChain::to_chain).collect(), params.tolerance)
        .into_iter()
        .map(|c| PolylineChain { points: c.points.iter().map(|q| [q.x, q.y]).collect(), closed: c.closed })
        .collect();
    let cut = cut_param(cmd, p, &head.tool, head.stock_top, MILLING)?;
    let spindle_rpm = spindle_param(cmd, p, &head.tool)?;
    let coolant = bool_(p, "coolant").unwrap_or(false);
    let n = chains.len();
    let op = CamOperation::Engrave { name: head.name.clone(), tool: head.tool_n, chains, params, cut, spindle_rpm, coolant };
    save_op(s, cmd, &head, op, json!({"chains": n}))
}

fn vcarve_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.vcarve";
    let head = op_head(s, cmd, p)?;
    let tool = &head.tool;
    if tool.kind != ToolKind::VBit {
        return Err(bad(cmd, format!("tool {} is no V-bit: v-carving needs a tool whose depth sets its width (cam.tool kind vbit)", head.tool_n)));
    }
    let regions = regions_param(s, cmd, p)?;
    let max_depth = match p.get("max_depth") {
        None => None,
        Some(v) => Some(v.as_f64().filter(|d| d.is_finite() && *d > 0.0).ok_or_else(|| bad(cmd, "`max_depth` must be a positive number (mm)"))?),
    };
    // The cut goes as deep as the carving needs, but no deeper than asked or than the flutes.
    let top = num(p, "top").or(head.stock_top).unwrap_or(0.0);
    let limit = max_depth.unwrap_or(tool.flute_length).min(tool.flute_length);
    let mut with_bottom = p.clone();
    with_bottom["bottom"] = json!(top - limit);
    let mut cut = cut_param(cmd, &with_bottom, tool, head.stock_top, DRILLING)?;
    let spindle_rpm = spindle_param(cmd, p, tool)?;
    let coolant = bool_(p, "coolant").unwrap_or(false);
    let mut params = default_vcarve_params();
    if let Some(v) = num(p, "stepover") {
        params.stepover = v;
    }
    if let Some(v) = num(p, "tolerance") {
        params.tolerance = v;
    }
    let geometry: Vec<Region2> = regions.iter().map(|r| r.to_region().map_err(|e| bad(cmd, e.to_string()))).collect::<Result<_>>()?;
    let carved = vcarve(&geometry, tool, &params, &cut).map_err(|e| bad(cmd, e.to_string()))?;
    if carved.flat_floor && max_depth.is_none() {
        return Err(bad(
            cmd,
            format!(
                "the carving would go {:.2} mm deep, more than tool {}'s {:.2} mm of flutes: give `max_depth` (the tip then clears a flat floor there) or cut less at a time",
                carved.natural_depth, head.tool_n, tool.flute_length
            ),
        ));
    }
    // The operation keeps the depth it reached, so posting it builds the same passes.
    cut.heights.bottom = carved.deepest;
    let depth = top - carved.deepest;
    let mut extra = json!({"depth": depth, "rings": carved.rings});
    if carved.flat_floor {
        let tan_half = (tool.tip_angle.unwrap_or(0.0).to_radians() * 0.5).tan();
        extra["flat_clearing"] = json!({"wall_offset": tool.tip_diameter * 0.5 + depth * tan_half, "bottom": carved.deepest});
    }
    let op = CamOperation::VCarve { name: head.name.clone(), tool: head.tool_n, regions, params, cut, spindle_rpm, coolant };
    save_op(s, cmd, &head, op, extra)
}

fn drill_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.drill";
    let head = op_head(s, cmd, p)?;
    let top = num(p, "top").or(head.stock_top).unwrap_or(0.0);
    let common = match (num(p, "bottom"), num(p, "depth")) {
        (Some(_), Some(_)) => return Err(bad(cmd, "give `bottom` or `depth`, not both")),
        (Some(b), None) => Some(b),
        (None, Some(d)) => Some(top - d),
        (None, None) => None,
    };
    let holes = holes_param(s, cmd, p, common)?;
    let deepest = holes.iter().map(|h| h.bottom).fold(f64::INFINITY, f64::min);
    // The cut's bottom is the deepest hole's; the feeds and the depth per pass are the plunge.
    let mut with_bottom = p.clone();
    with_bottom["bottom"] = json!(deepest);
    let cut = cut_param(cmd, &with_bottom, &head.tool, head.stock_top, DRILLING)?;
    let spindle_rpm = spindle_param(cmd, p, &head.tool)?;
    let coolant = bool_(p, "coolant").unwrap_or(false);
    let params = DrillParams { peck: num(p, "peck"), dwell: num(p, "dwell").unwrap_or(0.0) };
    let n = holes.len();
    let op = CamOperation::Drill { name: head.name.clone(), tool: head.tool_n, holes, params, cut, spindle_rpm, coolant };
    save_op(s, cmd, &head, op, json!({"holes": n}))
}

/// The holes to drill: `points` with a bottom each (their own z, else `common`), or the sketch's
/// lone points and circle centres.
fn holes_param(s: &Session, cmd: &str, p: &Value, common: Option<f64>) -> Result<Vec<HolePoint>> {
    let at: Vec<(Vec2, Option<f64>)> = if p.get("sketch").is_some() {
        sketch_points(s, cmd, p)?.into_iter().map(|q| (q, None)).collect()
    } else {
        let points = p.get("points").and_then(Value::as_array).ok_or_else(|| bad(cmd, "give `points`: [[x,y] | [x,y,z],…] or `sketch`"))?;
        if points.len() > MAX_DRILL_HOLES {
            return Err(bad(cmd, format!("a drilling has at most {MAX_DRILL_HOLES} holes")));
        }
        points
            .iter()
            .map(|q| {
                let a = q
                    .as_array()
                    .filter(|a| a.len() == 2 || a.len() == 3)
                    .ok_or_else(|| bad(cmd, "`points` must be a list of [x, y] or [x, y, z]"))?;
                let n = |i: usize| {
                    a.get(i).and_then(Value::as_f64).filter(|v| v.is_finite()).ok_or_else(|| bad(cmd, "point coordinates must be finite numbers"))
                };
                let z = if a.len() == 3 { Some(n(2)?) } else { None };
                Ok((Vec2::new(n(0)?, n(1)?), z))
            })
            .collect::<Result<_>>()?
    };
    at.into_iter()
        .map(|(q, z)| {
            let bottom = z.or(common).ok_or_else(|| bad(cmd, "give `bottom` or `depth`, or a z for every point"))?;
            let hole = HolePoint { at: [q.x, q.y], bottom };
            hole.validate().map_err(|e| bad(cmd, e.to_string()))?;
            Ok(hole)
        })
        .collect()
}

/// Where a sketch marks holes: its points that belong to no curve (not the origin, not projected
/// ones) and the centres of its circles, construction circles included.
fn sketch_points(s: &Session, cmd: &str, p: &Value) -> Result<Vec<Vec2>> {
    let id = super::features::feature_sketch(s, p, cmd)?;
    let st = s.model.state();
    let ss = st.sketch(id).ok_or_else(|| EngineError::Other("that sketch is not evaluated".into()))?;
    let sk = &ss.sketch;
    let mut used = vec![false; sk.points.len()];
    for c in &sk.curves {
        for i in c.kind.point_ids() {
            if let Some(u) = used.get_mut(i) {
                *u = true;
            }
        }
    }
    let mut pts: Vec<Vec2> = Vec::new();
    for (i, pt) in sk.points.iter().enumerate().skip(1) {
        if !used[i] && pt.link.is_none() {
            pts.extend(sk.point(i));
        }
    }
    for (ci, c) in sk.curves.iter().enumerate() {
        if let CurveKind::Circle { c: centre, .. } = c.kind
            && !sk.is_text_curve(ci)
        {
            pts.extend(sk.point(centre));
        }
    }
    let mut unique: Vec<Vec2> = Vec::with_capacity(pts.len());
    for q in pts {
        if !unique.iter().any(|u| u.dist(q) <= 1e-6) {
            unique.push(q);
        }
    }
    if unique.is_empty() {
        return Err(bad(cmd, "the sketch has no points or circle centres to drill"));
    }
    if unique.len() > MAX_DRILL_HOLES {
        return Err(bad(cmd, format!("a drilling has at most {MAX_DRILL_HOLES} holes")));
    }
    let mut loops = vec![unique];
    placed(cmd, p, &mut loops)?;
    Ok(loops.pop().unwrap_or_default())
}

/// The curves to engrave: the sketch's (or the named ones), or `paths` given as point lists.
fn chains_param(s: &Session, cmd: &str, p: &Value) -> Result<Vec<PolylineChain>> {
    if p.get("sketch").is_some() {
        return sketch_chains(s, cmd, p);
    }
    let paths = p.get("paths").and_then(Value::as_array).ok_or_else(|| bad(cmd, "give `sketch` (with `curves`) or `paths`: [[[x,y],…],…]"))?;
    if paths.len() > MAX_CHAINS {
        return Err(bad(cmd, format!("an engraving has at most {MAX_CHAINS} curves")));
    }
    paths
        .iter()
        .map(|path| {
            let mut pts = path
                .as_array()
                .ok_or_else(|| bad(cmd, "`paths` must be a list of point lists"))?
                .iter()
                .map(|q| vec2(q).map(|q| [q.x, q.y]).ok_or_else(|| bad(cmd, "path points must be [x, y]")))
                .collect::<Result<Vec<_>>>()?;
            let closed = pts.len() >= 4 && pts.first().zip(pts.last()).is_some_and(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]) <= 1e-9);
            if closed {
                pts.pop();
            }
            let chain = PolylineChain { points: pts, closed };
            chain.validate().map_err(|e| bad(cmd, e.to_string()))?;
            Ok(chain)
        })
        .collect()
}

/// The sketch's drawn curves as polylines within `PROFILE_TOLERANCE`: all of them, or the ones
/// `curves` names. Construction, centreline and infinite lines are not drawn; projected geometry
/// is not either, but text outlines are.
fn sketch_chains(s: &Session, cmd: &str, p: &Value) -> Result<Vec<PolylineChain>> {
    let id = super::features::feature_sketch(s, p, cmd)?;
    let st = s.model.state();
    let ss = st.sketch(id).ok_or_else(|| EngineError::Other("that sketch is not evaluated".into()))?;
    let sk = &ss.sketch;
    let drawn = |i: usize| {
        let c = &sk.curves[i];
        !c.construction && !c.centerline && !c.infinite && (!sk.is_linked_curve(i) || sk.is_text_curve(i))
    };
    let picked: Vec<usize> = match p.get("curves") {
        None | Some(Value::Null) => (0..sk.curves.len()).filter(|&i| drawn(i)).collect(),
        Some(Value::String(all)) if all == "all" => (0..sk.curves.len()).filter(|&i| drawn(i)).collect(),
        Some(Value::Array(_)) => string_list(cmd, p, "curves")?
            .iter()
            .map(|name| {
                let i = sk.curve_index(name).ok_or_else(|| bad(cmd, format!("unknown sketch curve `{name}`")))?;
                if sk.curves[i].infinite {
                    return Err(bad(cmd, format!("`{name}` is an infinite line, which cannot be engraved")));
                }
                Ok(i)
            })
            .collect::<Result<_>>()?,
        Some(_) => return Err(bad(cmd, "`curves` must be \"all\" or a list of sketch curve ids")),
    };
    let mut loops: Vec<Vec<Vec2>> = Vec::new();
    let mut closed_flags = Vec::new();
    for ci in picked {
        let kind_closed =
            matches!(sk.curves[ci].kind, CurveKind::Circle { .. } | CurveKind::Ellipse { .. } | CurveKind::Spline { periodic: true, .. });
        let mut pts: Vec<Vec2> = Vec::new();
        for seg in sk.exact_pieces(ci) {
            for q in seg.polyline(PROFILE_TOLERANCE) {
                if !pts.last().is_some_and(|l| l.dist(q) <= 1e-9) {
                    pts.push(q);
                }
            }
        }
        if pts.len() > MAX_LOOP_POINTS {
            return Err(bad(cmd, format!("a curve needs more than {MAX_LOOP_POINTS} points within {PROFILE_TOLERANCE} mm")));
        }
        let closed = kind_closed || (pts.len() >= 4 && pts.first().zip(pts.last()).is_some_and(|(a, b)| a.dist(*b) <= 1e-9));
        if closed && pts.len() > 1 && pts.first().zip(pts.last()).is_some_and(|(a, b)| a.dist(*b) <= 1e-9) {
            pts.pop();
        }
        if pts.len() < 2 || (closed && pts.len() < 3) {
            continue;
        }
        loops.push(pts);
        closed_flags.push(closed);
    }
    if loops.is_empty() {
        return Err(bad(cmd, "the sketch has no curve to engrave"));
    }
    if loops.len() > MAX_CHAINS {
        return Err(bad(cmd, format!("an engraving has at most {MAX_CHAINS} curves")));
    }
    placed(cmd, p, &mut loops)?;
    Ok(loops
        .into_iter()
        .zip(closed_flags)
        .map(|(pts, closed)| PolylineChain { points: pts.into_iter().map(|q| [q.x, q.y]).collect(), closed })
        .collect())
}

/// Turns `loops` by the quarter turns `rotate` asks for (about the sketch origin) and moves
/// them so that the lower left corner of their bounds is at `at`, when given.
fn placed(cmd: &str, p: &Value, loops: &mut [Vec<Vec2>]) -> Result<()> {
    // Quarter turns only: the part keeps its edges along the machine's axes.
    let deg = num(p, "rotate").unwrap_or(0.0);
    let turn = [0.0, 90.0, 180.0, 270.0].iter().position(|&d| d == deg).ok_or_else(|| bad(cmd, "`rotate` must be 0, 90, 180 or 270"))?;
    for q in loops.iter_mut().flatten() {
        *q = (0..turn).fold(*q, |q, _| Vec2::new(-q.y, q.x));
    }
    if let Some(at) = p.get("at") {
        let at = vec2(at).ok_or_else(|| bad(cmd, "`at` must be [x, y]"))?;
        let lo = loops.iter().flatten().fold(Vec2::new(f64::INFINITY, f64::INFINITY), |m, q| Vec2::new(m.x.min(q.x), m.y.min(q.y)));
        let shift = at - lo;
        for q in loops.iter_mut().flatten() {
            *q += shift;
        }
    }
    Ok(())
}

/// The bounds of an operation's regions, and the openings its tool cannot enter: holes of a part
/// cut round the outside, or regions cut or cleared from the inside, narrower than the tool. The
/// offset drops them, so they would be left uncut without a word.
fn reach_report(tool: &Tool, op: &CamOperation) -> (Value, Vec<Value>) {
    /// Which loops of a region the tool has to enter.
    enum Openings {
        /// A part cut round the outside: its holes.
        Holes,
        /// An opening cut or cleared from the inside: its outline.
        Outer,
        /// The tool centre runs on the line: nothing is offset, nothing dropped.
        None,
    }
    let (regions, openings, leave): (&[PolygonRegion], Openings, f64) = match op {
        CamOperation::Contour { regions, params, .. } => {
            let o = match params.side {
                Side::Outside => Openings::Holes,
                Side::Inside => Openings::Outer,
                Side::On => Openings::None,
            };
            (regions, o, params.stock_to_leave)
        }
        CamOperation::Pocket { regions, params, .. } => (regions, Openings::Outer, params.stock_to_leave),
        CamOperation::VCarve { regions, .. } => (regions, Openings::None, 0.0),
        CamOperation::Engrave { .. } | CamOperation::Drill { .. } => (&[], Openings::None, 0.0),
    };
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    let mut take = |p: &[f64; 2]| (lo[0], lo[1], hi[0], hi[1]) = (lo[0].min(p[0]), lo[1].min(p[1]), hi[0].max(p[0]), hi[1].max(p[1]));
    match op {
        CamOperation::Engrave { chains, .. } => chains.iter().flat_map(|c| &c.points).for_each(&mut take),
        CamOperation::Drill { holes, .. } => holes.iter().map(|h| &h.at).for_each(&mut take),
        _ => regions.iter().flat_map(|r| &r.outer).for_each(&mut take),
    }
    let depth = op.cut().heights.top - op.cut().heights.bottom;
    let reach = tool.radius_at(depth) + leave;
    let mut small = Vec::new();
    for region in regions.iter().filter_map(|r| r.to_region().ok()) {
        let loops: Vec<_> = match openings {
            Openings::Holes => region.holes,
            Openings::Outer => vec![region.outer],
            Openings::None => Vec::new(),
        };
        for lp in loops {
            // Inscribed radius within a hundredth of a millimetre is enough to tell.
            let r = lp.inradius(0.01);
            if r < reach - 1e-6 {
                let pts = lp.polyline(0.05).into_vec();
                let (mut a, mut b) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
                for q in &pts {
                    (a[0], a[1], b[0], b[1]) = (a[0].min(q.x), a[1].min(q.y), b[0].max(q.x), b[1].max(q.y));
                }
                small.push(json!({"min": a, "max": b, "widest_circle": 2.0 * r}));
            }
        }
    }
    (json!({"min": lo, "max": hi}), small)
}

/// `tabs`: `true` for the usual ones, or some of them changed; `false` or left out: none.
fn tabs_param(cmd: &str, p: &Value) -> Result<Option<Tabs>> {
    let usual = Tabs { width: 8.0, height: 3.0, spacing: 250.0, inner: true };
    match p.get("tabs") {
        None | Some(Value::Null) | Some(Value::Bool(false)) => Ok(None),
        Some(Value::Bool(true)) => Ok(Some(usual)),
        Some(t @ Value::Object(_)) => Ok(Some(Tabs {
            width: num(t, "width").unwrap_or(usual.width),
            height: num(t, "height").unwrap_or(usual.height),
            spacing: num(t, "spacing").unwrap_or(usual.spacing),
            inner: bool_(t, "inner").unwrap_or(usual.inner),
        })),
        Some(_) => Err(bad(cmd, "`tabs` must be true, false or {width, height, spacing, inner}")),
    }
}

fn parse_side(cmd: &str, side: &str) -> Result<Side> {
    match side {
        "outside" => Ok(Side::Outside),
        "inside" => Ok(Side::Inside),
        "on" => Ok(Side::On),
        other => Err(bad(cmd, format!("`side` must be outside, inside or on (not `{other}`)"))),
    }
}

fn regions_param(s: &Session, cmd: &str, p: &Value) -> Result<Vec<PolygonRegion>> {
    if p.get("sketch").is_some() {
        return sketch_regions(s, cmd, p);
    }
    if let Some(arr) = p.get("regions").and_then(Value::as_array) {
        return arr.iter().map(|v| polygon_region(cmd, v)).collect();
    }
    let outer = vec2_list(cmd, p, "outer", solidcraft_doc::cam::MAX_LOOP_POINTS)?;
    if outer.len() < 3 {
        return Err(bad(cmd, "`outer` needs at least 3 points (or pass `regions`)"));
    }
    let holes = match p.get("holes") {
        None => Vec::new(),
        Some(Value::Array(hs)) => hs
            .iter()
            .map(|h| {
                let pts = h.as_array().ok_or_else(|| bad(cmd, "`holes` must be a list of point lists"))?;
                if pts.len() < 3 || pts.len() > solidcraft_doc::cam::MAX_LOOP_POINTS {
                    return Err(bad(cmd, "each hole needs 3 to 10000 points"));
                }
                pts.iter()
                    .map(|v| crate::params::vec2(v).map(|p| [p.x, p.y]).ok_or_else(|| bad(cmd, "hole points must be [x, y]")))
                    .collect::<Result<Vec<_>>>()
            })
            .collect::<Result<Vec<_>>>()?,
        _ => return Err(bad(cmd, "`holes` must be a list of point lists")),
    };
    let region = PolygonRegion { outer: outer.into_iter().map(|p| [p.x, p.y]).collect(), holes };
    region.validate().map_err(|e| bad(cmd, e.to_string()))?;
    Ok(vec![region])
}

/// An operation's regions from a sketch's profiles: by default every profile that is not the
/// inside of another one's hole (so the part, not its openings), turned by `rotate` and moved so
/// the lower left corner of their bounds lies at `at`.
fn sketch_regions(s: &Session, cmd: &str, p: &Value) -> Result<Vec<PolygonRegion>> {
    let id = super::features::feature_sketch(s, p, cmd)?;
    let st = s.model.state();
    let ss = st.sketch(id).ok_or_else(|| EngineError::Other("that sketch is not evaluated".into()))?;
    let all = &ss.profiles;
    let in_a_hole = |r: &Region2| {
        let q = r.interior_point();
        all.iter().any(|o| o.region.holes.iter().any(|h| h.contains(q)))
    };
    let at_point = |pt: Vec2| {
        all.iter()
            .filter(|o| o.region.outer.contains(pt) && !o.region.holes.iter().any(|h| h.contains(pt)))
            .min_by(|a, b| a.area.total_cmp(&b.area))
            .map(|o| o.region.clone())
            .ok_or_else(|| bad(cmd, format!("no profile at [{}, {}]", pt.x, pt.y)))
    };
    let picked: Vec<Region2> = match super::features::profiles(p, cmd)? {
        ProfileSel::All => all.iter().map(|o| o.region.clone()).filter(|r| !in_a_hole(r)).collect(),
        ProfileSel::Indices { indices } => indices
            .iter()
            .map(|&i| all.get(i).map(|o| o.region.clone()).ok_or_else(|| bad(cmd, format!("the sketch has no profile {i}"))))
            .collect::<Result<_>>()?,
        ProfileSel::Points { points } => points.into_iter().map(at_point).collect::<Result<_>>()?,
        selection => solidcraft_doc::profile_selection_indices(ss, &selection)
            .map_err(|e| bad(cmd, e.to_string()))?
            .into_iter()
            .filter_map(|i| all.get(i).map(|o| o.region.clone()))
            .collect(),
    };
    if picked.is_empty() {
        return Err(bad(cmd, "the sketch has no closed profile to cut"));
    }
    let ring = |lp: &solidcraft_geom::Loop2| -> Result<Vec<Vec2>> {
        let pts: Vec<Vec2> = lp.polyline(PROFILE_TOLERANCE).into_vec();
        if pts.len() > MAX_LOOP_POINTS {
            return Err(bad(cmd, format!("a profile loop needs more than {MAX_LOOP_POINTS} points within {PROFILE_TOLERANCE} mm")));
        }
        Ok(pts)
    };
    // Every loop in one list (each outer followed by its holes), so they are placed together.
    let mut loops: Vec<Vec<Vec2>> = Vec::new();
    let mut hole_counts = Vec::with_capacity(picked.len());
    for r in &picked {
        loops.push(ring(&r.outer)?);
        hole_counts.push(r.holes.len());
        for h in &r.holes {
            loops.push(ring(h)?);
        }
    }
    placed(cmd, p, &mut loops)?;
    let xy = |pts: Vec<Vec2>| pts.into_iter().map(|q| [q.x, q.y]).collect::<Vec<_>>();
    let mut rest = loops.into_iter();
    hole_counts
        .into_iter()
        .map(|n| {
            let outer = rest.next().map(xy).unwrap_or_default();
            let holes = (0..n).filter_map(|_| rest.next().map(xy)).collect();
            let region = PolygonRegion { outer, holes };
            region.validate().map_err(|e| bad(cmd, e.to_string()))?;
            Ok(region)
        })
        .collect()
}

fn polygon_region(cmd: &str, v: &Value) -> Result<PolygonRegion> {
    let outer = v
        .get("outer")
        .and_then(Value::as_array)
        .ok_or_else(|| bad(cmd, "each region needs `outer`: [[x,y],…]"))?
        .iter()
        .map(|p| crate::params::vec2(p).map(|p| [p.x, p.y]).ok_or_else(|| bad(cmd, "region points must be [x, y]")))
        .collect::<Result<Vec<_>>>()?;
    let holes = match v.get("holes") {
        None => Vec::new(),
        Some(Value::Array(hs)) => hs
            .iter()
            .map(|h| {
                h.as_array()
                    .ok_or_else(|| bad(cmd, "holes must be point lists"))?
                    .iter()
                    .map(|p| crate::params::vec2(p).map(|p| [p.x, p.y]).ok_or_else(|| bad(cmd, "hole points must be [x, y]")))
                    .collect::<Result<Vec<_>>>()
            })
            .collect::<Result<Vec<_>>>()?,
        _ => return Err(bad(cmd, "`holes` must be a list of point lists")),
    };
    let region = PolygonRegion { outer, holes };
    region.validate().map_err(|e| bad(cmd, e.to_string()))?;
    Ok(region)
}

/// What an operation needs of the cutting data: milling needs a feed along the cut and a depth
/// per pass; drilling only plunges.
#[derive(Clone, Copy)]
struct Needs {
    feed: bool,
    stepdown: bool,
}

const MILLING: Needs = Needs { feed: true, stepdown: true };
const DRILLING: Needs = Needs { feed: false, stepdown: false };

/// The heights, feeds and depth per pass of an operation. Its top is where the stock's is when the
/// setup has one; the retract and clearance heights sit 2 and 10 mm above the top unless given.
fn cut_param(cmd: &str, p: &Value, tool: &Tool, stock_top: Option<f64>, needs: Needs) -> Result<Cut> {
    if let Some(raw) = p.get("cut") {
        return serde_json::from_value(raw.clone()).map_err(|e| bad(cmd, format!("`cut` is invalid: {e}")));
    }
    let bottom = num(p, "bottom").ok_or_else(|| bad(cmd, "`bottom` is required (final Z, mm)"))?;
    let top = num(p, "top").or(stock_top).unwrap_or(0.0);
    let retract = num(p, "retract").unwrap_or(top + 2.0);
    let clearance = num(p, "clearance").unwrap_or(top + 10.0);
    let n = tool.number;
    // As with the feeds: no depth per pass made up (the whole depth at once could break the tool).
    let stepdown = match num(p, "stepdown").or(tool.stepdown) {
        Some(v) => Some(v),
        None if needs.stepdown => return Err(bad(cmd, format!("no stepdown: give `stepdown` or set tool {n}'s (cam.tool stepdown)"))),
        None => None,
    };
    let feed = match num(p, "feed").or(tool.feed) {
        Some(v) => Some(v),
        None if needs.feed => return Err(bad(cmd, format!("no feed: give `feed` or set tool {n}'s (cam.tool feed)"))),
        None => None,
    };
    let plunge = num(p, "plunge_feed")
        .or(tool.plunge_feed)
        .ok_or_else(|| bad(cmd, format!("no plunge feed: give `plunge_feed` or set tool {n}'s (cam.tool plunge_feed)")))?;
    // Drilling only plunges: its depth per pass is the whole depth, its feed the plunge feed.
    let stepdown = stepdown.unwrap_or((top - bottom).max(1e-6));
    let feed = feed.unwrap_or(plunge);
    let mut cut = default_cut(bottom, stepdown);
    cut.heights = Heights { clearance, retract, top, bottom };
    cut.feeds = Feeds { cut: feed, plunge };
    cut.heights.validate().map_err(|e| bad(cmd, e.to_string()))?;
    cut.feeds.validate().map_err(|e| bad(cmd, e.to_string()))?;
    Ok(cut)
}

fn delete_op(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.delete_op";
    let setup_name = str_(p, "setup").map(str::trim).filter(|n| !n.is_empty()).ok_or_else(|| bad(cmd, "`setup` is required"))?.to_string();
    let name = str_(p, "name").map(str::trim).filter(|n| !n.is_empty()).ok_or_else(|| bad(cmd, "`name` is required"))?.to_string();
    let doc = s.doc_mut();
    let setup = doc.cam.setup_mut(&setup_name).ok_or_else(|| bad(cmd, format!("no setup `{setup_name}`")))?;
    let before = setup.operations.len();
    setup.operations.retain(|o| o.name() != name);
    if setup.operations.len() == before {
        return Err(bad(cmd, format!("no operation `{name}` in setup `{setup_name}`")));
    }
    Ok(json!({"deleted": name, "setup": setup_name}))
}

fn post_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.post";
    let setup_name = str_(p, "setup").map(str::trim).filter(|n| !n.is_empty()).ok_or_else(|| bad(cmd, "`setup` is required"))?.to_string();
    s.doc.cam.validate().map_err(|e| bad(cmd, e.to_string()))?;
    let setup = s.doc.cam.setup(&setup_name).cloned().ok_or_else(|| bad(cmd, format!("no setup `{setup_name}`")))?;
    if setup.operations.is_empty() {
        return Err(bad(cmd, format!("setup `{setup_name}` has no operations")));
    }
    let post_name = str_(p, "post").unwrap_or("rs274");
    let post = resolve_post(&s.doc.cam, post_name).map_err(|e| bad(cmd, e))?;
    post.validate().map_err(|e| bad(cmd, e.to_string()))?;
    let program_name = str_(p, "program").unwrap_or(setup.name.as_str()).to_string();
    let mut program = Program { name: program_name, operations: Vec::new(), work_offset: Some(setup.work_offset) };
    for op in &setup.operations {
        let tool = s.doc.cam.tool(op.tool()).cloned().ok_or_else(|| bad(cmd, format!("operation `{}` needs tool {}", op.name(), op.tool())))?;
        let path = build_path(&tool, op).map_err(|e| bad(cmd, e))?;
        // A machine runs what the program says: no speed made up here either.
        let spindle_rpm = op.spindle_rpm().or(tool.spindle_rpm).ok_or_else(|| {
            bad(cmd, format!("operation `{}` has no spindle speed: give it `spindle_rpm` or set tool {}'s", op.name(), tool.number))
        })?;
        program.operations.push(Operation { name: op.name().into(), tool, spindle_rpm, coolant: op.coolant(), path });
    }
    let gcode = emit(&program, &post).map_err(|e| bad(cmd, e.to_string()))?;
    let mut out = json!({
        "lines": gcode.lines().count(),
        "extension": post.extension,
        "post": post.name,
        "operations": program.operations.len(),
        "work_offset": setup.work_offset,
    });
    if bool_(p, "verify").unwrap_or(true) {
        let rb = readback(&gcode, &program, &post).map_err(|e| bad(cmd, e.to_string()))?;
        if !rb.within() {
            return Err(bad(cmd, format!("G-code read-back deviation {:.4} mm exceeds allowed {:.4} mm", rb.max_deviation, rb.allowed)));
        }
        out["readback"] = json!({
            "max_deviation": rb.max_deviation,
            "allowed": rb.allowed,
            "cut_length": rb.cut_length,
            "cut_minutes": rb.cut_minutes,
            "within": rb.within(),
        });
    }
    // Written only once it has read back as the toolpaths.
    if let Some(path) = str_(p, "path").map(str::trim).filter(|x| !x.is_empty()) {
        solidcraft_io::vfs::write(path, gcode.as_bytes()).map_err(|e| EngineError::Other(format!("{path}: {e}")))?;
        out["path"] = json!(path);
    } else {
        out["gcode"] = json!(gcode);
    }
    Ok(out)
}

fn simulate_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.simulate";
    let setup_name = str_(p, "setup").map(str::trim).filter(|n| !n.is_empty()).ok_or_else(|| bad(cmd, "`setup` is required"))?.to_string();
    let setup = s.doc.cam.setup(&setup_name).cloned().ok_or_else(|| bad(cmd, format!("no setup `{setup_name}`")))?;
    let stock = match p.get("stock") {
        Some(v) => stock_box(cmd, v)?,
        None => setup.stock.ok_or_else(|| bad(cmd, format!("setup `{setup_name}` has no stock: give `stock` {{min, max}} or cam.setup stock")))?,
    };
    let step = num(p, "resolution").unwrap_or(0.2);
    if !(step > 0.0) {
        return Err(bad(cmd, "`resolution` must be above 0 (mm per pixel)"));
    }
    let out_path =
        str_(p, "path").map(str::trim).filter(|x| !x.is_empty()).ok_or_else(|| bad(cmd, "`path` is required: the PNG to write"))?.to_string();
    let ops: Vec<&CamOperation> = match p.get("operations") {
        None | Some(Value::Null) => setup.operations.iter().collect(),
        Some(_) => string_list(cmd, p, "operations")?
            .iter()
            .map(|n| setup.operations.iter().find(|o| o.name() == n).ok_or_else(|| bad(cmd, format!("no operation `{n}` in setup `{setup_name}`"))))
            .collect::<Result<_>>()?,
    };
    if ops.is_empty() {
        return Err(bad(cmd, format!("setup `{setup_name}` has no operations to simulate")));
    }
    let mut map = Heightmap::new(Vec2::new(stock.min[0], stock.min[1]), Vec2::new(stock.max[0], stock.max[1]), stock.max[2], step)
        .map_err(|e| bad(cmd, e.to_string()))?;
    let mut per_op = Vec::new();
    for op in ops {
        let tool = s.doc.cam.tool(op.tool()).cloned().ok_or_else(|| bad(cmd, format!("operation `{}` needs tool {}", op.name(), op.tool())))?;
        let path = build_path(&tool, op).map_err(|e| bad(cmd, e))?;
        let removed = map.cut(&tool, &path).map_err(|e| bad(cmd, e.to_string()))?;
        let mut entry = json!({"name": op.name(), "removed_mm3": removed});
        if let CamOperation::Pocket { regions, cut, .. } | CamOperation::VCarve { regions, cut, .. } = op {
            let (mut standing, mut inside) = (0usize, 0usize);
            for region in regions.iter().filter_map(|r| r.to_region().ok()) {
                let (a, b) = map.standing_in(&region, cut.heights.bottom);
                standing += a;
                inside += b;
            }
            if inside > 0 {
                entry["standing_in_regions_percent"] = json!(100.0 * standing as f64 / inside as f64);
            }
        }
        per_op.push(entry);
    }
    // The picture: the stock top white, the deepest cut black, rows from +Y down as a picture is.
    let deepest = map.deepest();
    let span = (map.top - deepest).max(1e-9);
    let mut grey: Vec<u16> = Vec::with_capacity(map.width * map.height);
    for y in (0..map.height).rev() {
        for x in 0..map.width {
            let v = (f64::from(map.z[y * map.width + x]) - deepest) / span;
            grey.push((v * 65535.0).round().clamp(0.0, 65535.0) as u16);
        }
    }
    let (w, h) = (u32::try_from(map.width), u32::try_from(map.height));
    let img = match (w, h) {
        (Ok(w), Ok(h)) => image::ImageBuffer::<image::Luma<u16>, Vec<u16>>::from_raw(w, h, grey),
        _ => None,
    };
    let img = img.ok_or_else(|| EngineError::Other("the height map does not fit a picture".into()))?;
    let mut bytes = std::io::Cursor::new(Vec::new());
    img.write_to(&mut bytes, image::ImageFormat::Png).map_err(|e| EngineError::Other(format!("PNG: {e}")))?;
    solidcraft_io::vfs::write(&out_path, &bytes.into_inner()).map_err(|e| EngineError::Other(format!("{out_path}: {e}")))?;
    Ok(json!({
        "path": out_path,
        "size": [map.width, map.height],
        "resolution": step,
        "stock": {"min": stock.min, "max": stock.max},
        "deepest": deepest,
        "removed_mm3": map.removed(),
        "operations": per_op,
    }))
}

fn post_define_cmd(s: &mut Session, p: &Value) -> Result<Value> {
    let cmd = "cam.post.define";
    if bool_(p, "delete").unwrap_or(false) {
        let name = str_(p, "name").map(str::trim).filter(|n| !n.is_empty()).ok_or_else(|| bad(cmd, "`name` is required"))?.to_string();
        let doc = s.doc_mut();
        let before = doc.cam.posts.len();
        doc.cam.posts.retain(|q| q.name != name);
        if doc.cam.posts.len() == before {
            return Err(bad(cmd, format!("no saved post `{name}`")));
        }
        return Ok(json!({"deleted": name}));
    }
    let value = p.get("post").filter(|v| v.is_object()).ok_or_else(|| bad(cmd, "`post` is required: the post as data"))?;
    let mut post: Post = serde_json::from_value(value.clone()).map_err(|e| bad(cmd, format!("`post`: {e}")))?;
    post.name = post.name.trim().to_string();
    if post.name.is_empty() || post.name.len() > 128 {
        return Err(bad(cmd, "a post name must be 1 to 128 characters"));
    }
    // A saved post is looked up before the built-in ones and would hide one of the same name.
    if Post::builtin(&post.name).is_some() {
        return Err(bad(cmd, format!("`{}` is a built-in post: give yours another name", post.name)));
    }
    post.validate().map_err(|e| bad(cmd, e.to_string()))?;
    let doc = s.doc_mut();
    let posts = &mut doc.cam.posts;
    if posts.len() >= MAX_POSTS && !posts.iter().any(|q| q.name == post.name) {
        return Err(bad(cmd, format!("a design saves at most {MAX_POSTS} posts")));
    }
    match posts.iter_mut().find(|q| q.name == post.name) {
        Some(slot) => *slot = post.clone(),
        None => posts.push(post.clone()),
    }
    Ok(json!({"post": post.name}))
}

fn resolve_post(cam: &CamSettings, name: &str) -> std::result::Result<Post, String> {
    if let Some(p) = cam.post(name) {
        return Ok(p.clone());
    }
    Post::builtin(name).ok_or_else(|| format!("unknown post `{name}` (built-in: {}; or save one on the design)", Post::BUILTIN.join(", ")))
}

fn build_path(tool: &Tool, op: &CamOperation) -> std::result::Result<solidcraft_cam::Toolpath, String> {
    let regions = |rs: &[PolygonRegion]| rs.iter().map(|r| r.to_region().map_err(|e| e.to_string())).collect::<std::result::Result<Vec<_>, _>>();
    match op {
        CamOperation::Contour { regions: rs, params, cut, .. } => contour(&regions(rs)?, tool, params, cut).map_err(|e| e.to_string()),
        CamOperation::Pocket { regions: rs, params, cut, .. } => pocket(&regions(rs)?, tool, params, cut).map_err(|e| e.to_string()),
        CamOperation::Engrave { chains, params, cut, .. } => {
            engrave(&chains.iter().map(PolylineChain::to_chain).collect::<Vec<_>>(), tool, params, cut).map_err(|e| e.to_string())
        }
        CamOperation::Drill { holes, params, cut, .. } => {
            drill(&holes.iter().map(HolePoint::to_hole).collect::<Vec<_>>(), tool, params, cut).map_err(|e| e.to_string())
        }
        CamOperation::VCarve { regions: rs, params, cut, .. } => vcarve(&regions(rs)?, tool, params, cut).map(|v| v.path).map_err(|e| e.to_string()),
    }
}

fn list_cmd(s: &mut Session, _p: &Value) -> Result<Value> {
    Ok(json!({
        "tools": s.doc.cam.tools,
        "setups": s.doc.cam.setups.iter().map(setup_json).collect::<Vec<_>>(),
        "posts": s.doc.cam.posts.iter().map(|p| p.name.clone()).collect::<Vec<_>>(),
        "builtin_posts": Post::BUILTIN,
    }))
}

fn setup_json(setup: &CamSetup) -> Value {
    json!({
        "name": setup.name,
        "work_offset": setup.work_offset,
        "stock": setup.stock,
        "operations": setup.operations.iter().map(|op| match op {
            CamOperation::Contour { name, tool, regions, params, cut, spindle_rpm, coolant, rest_from } => json!({
                "type": "contour", "name": name, "tool": tool, "regions": regions.len(),
                "params": params, "cut": cut, "spindle_rpm": spindle_rpm, "coolant": coolant, "rest_from": rest_from,
            }),
            CamOperation::Pocket { name, tool, regions, params, cut, spindle_rpm, coolant, rest_from } => json!({
                "type": "pocket", "name": name, "tool": tool, "regions": regions.len(),
                "params": params, "cut": cut, "spindle_rpm": spindle_rpm, "coolant": coolant, "rest_from": rest_from,
            }),
            CamOperation::Engrave { name, tool, chains, params, cut, spindle_rpm, coolant } => json!({
                "type": "engrave", "name": name, "tool": tool, "chains": chains.len(),
                "params": params, "cut": cut, "spindle_rpm": spindle_rpm, "coolant": coolant,
            }),
            CamOperation::Drill { name, tool, holes, params, cut, spindle_rpm, coolant } => json!({
                "type": "drill", "name": name, "tool": tool, "holes": holes.len(),
                "params": params, "cut": cut, "spindle_rpm": spindle_rpm, "coolant": coolant,
            }),
            CamOperation::VCarve { name, tool, regions, params, cut, spindle_rpm, coolant } => json!({
                "type": "v_carve", "name": name, "tool": tool, "regions": regions.len(),
                "params": params, "cut": cut, "spindle_rpm": spindle_rpm, "coolant": coolant,
            }),
        }).collect::<Vec<_>>(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::json;

    fn run(s: &mut Session, id: &str, p: Value) -> Value {
        match s.execute(id, &p) {
            Ok(v) => v,
            Err(e) => panic!("{id} {p}: {e}"),
        }
    }

    #[test]
    fn contour_and_pocket_post_to_verified_gcode() {
        let mut s = Session::default();
        run(
            &mut s,
            "cam.tool",
            json!({"number": 1, "diameter": 6, "flute_length": 20, "spindle_rpm": 8000, "feed": 600, "plunge_feed": 200, "stepdown": 3}),
        );
        run(&mut s, "cam.setup", json!({"name": "Top", "work_offset": 1, "stock": {"min": [0, 0, -10], "max": [40, 30, 0]}}));
        run(
            &mut s,
            "cam.contour",
            json!({
                "setup": "Top", "name": "Outside", "tool": 1,
                "outer": [[0,0],[40,0],[40,30],[0,30]],
                "side": "outside", "bottom": -6, "stepdown": 3,
            }),
        );
        run(
            &mut s,
            "cam.pocket",
            json!({
                "setup": "Top", "name": "Pocket", "tool": 1,
                "outer": [[5,5],[35,5],[35,25],[5,25]],
                "stepover": 2.5, "bottom": -3, "stepdown": 3,
            }),
        );
        let posted = run(&mut s, "cam.post", json!({"setup": "Top"}));
        let gcode = posted["gcode"].as_str().unwrap();
        assert!(gcode.contains("G21"));
        assert!(gcode.contains("G54"));
        assert!(gcode.contains("M30"));
        assert!(posted["readback"]["within"].as_bool().unwrap());
        assert_eq!(posted["operations"], 2);

        // Round-trip through the design file.
        let cold = Session::new(solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap());
        assert_eq!(cold.doc.cam.tools.len(), 1);
        assert_eq!(cold.doc.cam.setups[0].operations.len(), 2);
        assert!(cold.doc.format_pending.iter().any(|n| n == "cam-milling"));
        let again = run(&mut Session::new(cold.doc.as_ref().clone()), "cam.post", json!({"setup": "Top"}));
        assert!(again["readback"]["within"].as_bool().unwrap());
    }

    #[test]
    fn refuses_unknown_tool_and_empty_post() {
        let mut s = Session::default();
        assert!(s.execute("cam.contour", &json!({"setup": "A", "name": "C", "tool": 1, "outer": [[0,0],[10,0],[10,10]], "bottom": -1})).is_err());
        run(&mut s, "cam.tool", json!({"number": 1, "diameter": 6, "flute_length": 20}));
        run(&mut s, "cam.setup", json!({"name": "A"}));
        assert!(s.execute("cam.post", &json!({"setup": "A"})).is_err());
        assert!(s.execute("cam.delete_op", &json!({"setup": "A", "name": "missing"})).is_err());
    }

    #[test]
    fn engraving_tools_are_made_and_a_cut_on_the_line_reports_nothing_too_small() {
        let mut s = Session::default();
        let t = run(
            &mut s,
            "cam.tool",
            json!({"number": 14, "kind": "vbit", "diameter": 5, "flute_length": 4.5, "tip_angle": 80, "spindle_rpm": 20000, "feed": 800, "plunge_feed": 300, "stepdown": 1}),
        );
        assert_eq!((t["tool"]["kind"].as_str(), t["tool"]["tip_angle"].as_f64()), (Some("vbit"), Some(80.0)));
        let no_ball = json!({"number": 10, "kind": "tapered_ball", "diameter": 3.175, "flute_length": 15, "taper_angle": 5.05});
        assert!(s.execute("cam.tool", &no_ball).unwrap_err().to_string().contains("tip_diameter"));
        run(
            &mut s,
            "cam.tool",
            json!({"number": 10, "kind": "tapered_ball", "diameter": 3.175, "flute_length": 15, "taper_angle": 5.05, "tip_diameter": 0.5}),
        );
        assert!(
            s.execute("cam.tool", &json!({"number": 3, "kind": "v_bit", "diameter": 6, "flute_length": 5}))
                .unwrap_err()
                .to_string()
                .contains("tip_angle")
        );
        assert!(
            s.execute("cam.tool", &json!({"number": 3, "kind": "cone", "diameter": 6, "flute_length": 5}))
                .unwrap_err()
                .to_string()
                .contains("tapered_ball")
        );
        run(&mut s, "cam.setup", json!({"name": "S"}));
        // A 4 mm hole in a plate is too small for a 6 mm cutter going inside it; on the line
        // nothing is offset, so nothing is too small.
        run(
            &mut s,
            "cam.tool",
            json!({"number": 1, "diameter": 6, "flute_length": 20, "spindle_rpm": 12000, "feed": 1000, "plunge_feed": 300, "stepdown": 2}),
        );
        let plate = |side: &str, name: &str| json!({"setup": "S", "name": name, "tool": 1, "outer": [[0,0],[40,0],[40,30],[0,30]], "holes": [[[18,13],[22,13],[22,17],[18,17]]], "side": side, "bottom": -2});
        let outside = run(&mut s, "cam.contour", plate("outside", "Out"));
        assert_eq!(outside["too_small"].as_array().map(Vec::len), Some(1), "{outside}");
        let on = run(&mut s, "cam.contour", plate("on", "On"));
        assert!(on.get("too_small").is_none(), "{on}");
        // A sharp V-bit cannot clear a pocket floor; its contour chamfers the edge, and the program says which bit.
        let pocket = json!({"setup": "S", "name": "P", "tool": 14, "outer": [[0,0],[40,0],[40,30],[0,30]], "bottom": -1});
        assert!(s.execute("cam.pocket", &pocket).unwrap_err().to_string().contains("tip_diameter"));
        run(&mut s, "cam.contour", json!({"setup": "S", "name": "Chamfer", "tool": 14, "outer": [[0,0],[40,0],[40,30],[0,30]], "bottom": -1}));
        let posted = run(&mut s, "cam.post", json!({"setup": "S"}));
        assert!(posted["gcode"].as_str().unwrap().contains("T14 D5 V-bit 80deg"), "{}", posted["gcode"]);
    }

    /// A face drawn in strokes: a lip line, an eye, a brow, and a construction guide.
    fn strokes() -> Session {
        let mut s = Session::default();
        run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Strokes"}));
        run(&mut s, "sketch.line", json!({"points": [[0, 0], [30, 0]], "ids": ["lip"]}));
        run(&mut s, "sketch.circle.center", json!({"center": [50, 20], "radius": 5, "id": "eye"}));
        run(&mut s, "sketch.spline.fit_point", json!({"points": [[0, 10], [10, 14], [20, 10]], "connect": false}));
        run(&mut s, "sketch.line", json!({"points": [[0, 30], [30, 30]], "construction": true, "ids": ["guide"]}));
        run(&mut s, "sketch.finish", json!({}));
        run(
            &mut s,
            "cam.tool",
            json!({"number": 14, "kind": "vbit", "diameter": 5, "flute_length": 4.5, "tip_angle": 80, "spindle_rpm": 20000, "feed": 800, "plunge_feed": 300, "stepdown": 0.5}),
        );
        run(&mut s, "cam.setup", json!({"name": "S"}));
        s
    }

    #[test]
    fn an_engraving_follows_the_sketch_curves_and_skips_construction_and_projections() {
        let mut s = strokes();
        let r = run(&mut s, "cam.engrave", json!({"setup": "S", "name": "Face", "tool": 14, "sketch": "Strokes", "bottom": -1}));
        assert_eq!(r["chains"], json!(3), "the line, the circle and the spline, not the guide: {r}");
        assert!(r.get("too_small").is_none(), "{r}");
        let list = run(&mut s, "cam.list", json!({}));
        let op = &list["setups"][0]["operations"][0];
        assert_eq!((op["type"].as_str(), op["chains"].as_u64()), (Some("engrave"), Some(3)), "{op}");
        let CamOperation::Engrave { chains, .. } = &s.doc.cam.setups[0].operations[0] else { panic!("{:?}", s.doc.cam.setups[0].operations) };
        assert_eq!(chains.iter().filter(|c| c.closed).count(), 1, "the circle: {chains:?}");
        let posted = run(&mut s, "cam.post", json!({"setup": "S"}));
        assert_eq!(posted["readback"]["within"], json!(true), "{posted}");
        assert!(posted["gcode"].as_str().unwrap().contains("V-bit 80deg"));
        // Named curves only; a name that is no curve is refused.
        let r = run(&mut s, "cam.engrave", json!({"setup": "S", "name": "Eye", "tool": 14, "sketch": "Strokes", "curves": ["eye"], "bottom": -0.5}));
        assert_eq!(r["chains"], json!(1));
        assert_eq!(r["bounds"], json!({"min": [45.0, 15.0], "max": [55.0, 25.0]}), "{r}");
        let bogus = json!({"setup": "S", "name": "X", "tool": 14, "sketch": "Strokes", "curves": ["bogus"], "bottom": -0.5});
        assert!(s.execute("cam.engrave", &bogus).unwrap_err().to_string().contains("bogus"));
        // Placed like a contour: turned a quarter and moved to `at`.
        let r = run(
            &mut s,
            "cam.engrave",
            json!({"setup": "S", "name": "Moved", "tool": 14, "sketch": "Strokes", "curves": ["lip"], "rotate": 90, "at": [100, 100], "bottom": -0.5}),
        );
        assert_eq!(r["bounds"], json!({"min": [100.0, 100.0], "max": [100.0, 130.0]}), "{r}");
        // The open line that cam.contour refuses is what cam.engrave is for.
        let open = json!({"setup": "S", "name": "L", "tool": 14, "outer": [[0,0],[30,0],[15,0]], "bottom": -1});
        assert!(s.execute("cam.contour", &open).unwrap_err().to_string().contains("area"));
    }

    #[test]
    fn raw_paths_engrave_open_chains_back_along_themselves() {
        let mut s = Session::default();
        run(&mut s, "cam.tool", json!({"number": 1, "diameter": 2, "flute_length": 10, "spindle_rpm": 15000, "feed": 600, "plunge_feed": 200}));
        run(&mut s, "cam.setup", json!({"name": "S"}));
        let paths = json!([[[0, 0], [10, 0], [10, 5]], [[20, 0], [30, 0], [30, 10], [20, 10], [20, 0]]]);
        let r = run(&mut s, "cam.engrave", json!({"setup": "S", "name": "Zig", "tool": 1, "paths": paths, "bottom": -1, "stepdown": 0.5}));
        assert_eq!(r["chains"], json!(2), "{r}");
        assert_eq!(r["bounds"], json!({"min": [0.0, 0.0], "max": [30.0, 10.0]}), "{r}");
        let CamOperation::Engrave { chains, .. } = &s.doc.cam.setups[0].operations[0] else { panic!() };
        assert_eq!(chains.iter().map(|c| (c.points.len(), c.closed)).collect::<Vec<_>>(), vec![(3, false), (4, true)]);
        // Two passes: the open path is cut out and back, so it is plunged at both ends, once each.
        let tool = s.doc.cam.tool(1).cloned().unwrap();
        let path = build_path(&tool, &s.doc.cam.setups[0].operations[0]).unwrap();
        let feeds: Vec<_> = path
            .moves
            .iter()
            .filter_map(|m| match m {
                solidcraft_cam::Move::Feed { to, .. } => Some(*to),
                _ => None,
            })
            .collect();
        let at_end = feeds.iter().position(|q| q.x == 10.0 && q.y == 5.0 && q.z == -0.5).unwrap();
        assert_eq!((feeds[at_end + 1].z, feeds[at_end + 3].x, feeds[at_end + 3].y), (-1.0, 0.0, 0.0), "{feeds:?}");
        let posted = run(&mut s, "cam.post", json!({"setup": "S"}));
        assert_eq!(posted["readback"]["within"], json!(true), "{posted}");
        // One point is no curve; neither a sketch nor paths is nothing to engrave.
        let bad = json!({"setup": "S", "name": "Bad", "tool": 1, "paths": [[[0, 0]]], "bottom": -1, "stepdown": 0.5});
        assert!(s.execute("cam.engrave", &bad).is_err());
        let none = json!({"setup": "S", "name": "Bad", "tool": 1, "bottom": -1, "stepdown": 0.5});
        assert!(s.execute("cam.engrave", &none).unwrap_err().to_string().contains("paths"));
    }

    #[test]
    fn a_vbit_engraves_text_outlines() {
        let mut s = strokes();
        run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Label"}));
        run(&mut s, "sketch.text", json!({"text": "AB", "at": [0, 0], "height": 10}));
        run(&mut s, "sketch.finish", json!({}));
        let r = run(&mut s, "cam.engrave", json!({"setup": "S", "name": "Label", "tool": 14, "sketch": "Label", "bottom": -0.6}));
        let CamOperation::Engrave { chains, .. } = &s.doc.cam.setups[0].operations[0] else { panic!() };
        assert!(chains.len() >= 2 && chains.iter().all(|c| c.closed), "the letters' outlines, all closed: {r}");
    }

    #[test]
    fn holes_are_drilled_from_points_and_a_sketchs_circle_centres() {
        let mut s = Session::default();
        run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Holes"}));
        for x in [10, 30, 50] {
            run(&mut s, "sketch.circle.center", json!({"center": [x, 10], "radius": 2.5}));
        }
        run(&mut s, "sketch.point", json!({"point": [70, 10]}));
        run(&mut s, "sketch.finish", json!({}));
        // A drill with only a plunge feed and a speed: no cut feed, no stepdown needed.
        run(&mut s, "cam.tool", json!({"number": 3, "kind": "drill", "diameter": 5, "flute_length": 40, "spindle_rpm": 3000, "plunge_feed": 150}));
        run(&mut s, "cam.setup", json!({"name": "S"}));
        let r = run(&mut s, "cam.drill", json!({"setup": "S", "name": "Holes", "tool": 3, "sketch": "Holes", "depth": 10, "peck": 4}));
        assert_eq!(r["holes"], json!(4), "{r}");
        assert_eq!(r["bounds"], json!({"min": [10.0, 10.0], "max": [70.0, 10.0]}), "{r}");
        let list = run(&mut s, "cam.list", json!({}));
        assert_eq!(list["setups"][0]["operations"][0]["type"], json!("drill"));
        let posted = run(&mut s, "cam.post", json!({"setup": "S"}));
        let gcode = posted["gcode"].as_str().unwrap();
        for z in ["Z-4", "Z-8", "Z-10", "Z-3.9"] {
            assert!(gcode.contains(z), "{z} missing:\n{gcode}");
        }
        assert_eq!(posted["readback"]["within"], json!(true), "{posted}");
        // The same holes as points, placed by `at`; `bottom` and `depth` together are refused.
        let r = run(&mut s, "cam.drill", json!({"setup": "S", "name": "Moved", "tool": 3, "sketch": "Holes", "at": [100, 0], "bottom": -5}));
        assert_eq!(r["bounds"], json!({"min": [100.0, 0.0], "max": [160.0, 0.0]}), "{r}");
        let both = json!({"setup": "S", "name": "B", "tool": 3, "points": [[0, 0]], "bottom": -5, "depth": 5});
        assert!(s.execute("cam.drill", &both).unwrap_err().to_string().contains("not both"));
    }

    #[test]
    fn a_vbit_drills_halftone_dots_with_per_point_depth() {
        let mut s = strokes();
        let dots = json!({"setup": "S", "name": "Dots", "tool": 14, "points": [[0, 0, -0.3], [5, 0, -0.8], [10, 0]], "depth": 0.5, "dwell": 0.1});
        let r = run(&mut s, "cam.drill", dots);
        assert_eq!(r["holes"], json!(3), "{r}");
        let CamOperation::Drill { holes, cut, .. } = &s.doc.cam.setups[0].operations[0] else { panic!() };
        assert_eq!(holes.iter().map(|h| h.bottom).collect::<Vec<_>>(), vec![-0.3, -0.8, -0.5]);
        assert_eq!(cut.heights.bottom, -0.8);
        let posted = run(&mut s, "cam.post", json!({"setup": "S"}));
        assert_eq!(posted["gcode"].as_str().unwrap().matches("G4 P0.1").count(), 3, "{}", posted["gcode"]);
        // A hole above the top, a peck of 0, and no depth at all are refused.
        for bad in [
            json!({"setup": "S", "name": "B", "tool": 14, "points": [[0, 0, 0.5]], "depth": 1}),
            json!({"setup": "S", "name": "B", "tool": 14, "points": [[0, 0]], "depth": 1, "peck": 0}),
            json!({"setup": "S", "name": "B", "tool": 14, "points": [[0, 0]]}),
            json!({"setup": "S", "name": "B", "tool": 14, "points": [[0]], "depth": 1}),
        ] {
            assert!(s.execute("cam.drill", &bad).is_err(), "{bad}");
        }
    }

    #[test]
    fn a_vcarve_goes_as_deep_as_the_cone_reaches_and_stops_at_max_depth() {
        let mut s = strokes();
        // A 10 × 10 square with the 80° bit (4.5 mm of flutes): the middle is 5 in, 5 / tan 40° = 5.96 deep.
        let square = json!({"setup": "S", "name": "Carve", "tool": 14, "outer": [[0,0],[10,0],[10,10],[0,10]], "stepover": 0.5});
        let err = s.execute("cam.vcarve", &square).unwrap_err().to_string();
        assert!(err.contains("max_depth") && err.contains("4.50"), "{err}");
        let mut held = square.clone();
        held["max_depth"] = json!(2);
        let r = run(&mut s, "cam.vcarve", held);
        assert_eq!(r["depth"], json!(2.0), "{r}");
        assert!(r["rings"].as_u64().unwrap() >= 9, "{r}");
        // The flat floor: a flat end mill's wall would sit 2 · tan 40° from the outline.
        let wall = r["flat_clearing"]["wall_offset"].as_f64().unwrap();
        assert!((wall - 2.0 * 40f64.to_radians().tan()).abs() < 1e-9, "{r}");
        assert_eq!(r["flat_clearing"]["bottom"], json!(-2.0));
        let CamOperation::VCarve { cut, .. } = &s.doc.cam.setups[0].operations[0] else { panic!() };
        assert_eq!(cut.heights.bottom, -2.0);
        // A shallow stroke needs no limit: 1 mm wide, so 0.4 in at the last pass, 0.4 / tan 40° deep.
        let r =
            run(&mut s, "cam.vcarve", json!({"setup": "S", "name": "Stroke", "tool": 14, "outer": [[0,20],[30,20],[30,21],[0,21]], "stepover": 0.1}));
        assert!(r.get("flat_clearing").is_none(), "{r}");
        assert!((r["depth"].as_f64().unwrap() - 0.4 / 40f64.to_radians().tan()).abs() < 1e-9, "{r}");
        assert_eq!(r["bounds"], json!({"min": [0.0, 20.0], "max": [30.0, 21.0]}));
        let posted = run(&mut s, "cam.post", json!({"setup": "S"}));
        assert_eq!(posted["readback"]["within"], json!(true), "{posted}");
        assert_eq!(run(&mut s, "cam.list", json!({}))["setups"][0]["operations"][1]["type"], json!("v_carve"));
        // Not with a flat end mill.
        run(
            &mut s,
            "cam.tool",
            json!({"number": 1, "diameter": 6, "flute_length": 20, "spindle_rpm": 12000, "feed": 1000, "plunge_feed": 300, "stepdown": 2}),
        );
        let flat = json!({"setup": "S", "name": "Flat", "tool": 1, "outer": [[0,0],[10,0],[10,10],[0,10]]});
        assert!(s.execute("cam.vcarve", &flat).unwrap_err().to_string().contains("V-bit"));
    }

    #[test]
    fn rest_machining_follows_a_larger_tool_into_the_corners() {
        let mut s = Session::default();
        run(
            &mut s,
            "cam.tool",
            json!({"number": 1, "diameter": 6, "flute_length": 20, "spindle_rpm": 12000, "feed": 1000, "plunge_feed": 300, "stepdown": 2}),
        );
        run(
            &mut s,
            "cam.tool",
            json!({"number": 2, "diameter": 2, "flute_length": 8, "spindle_rpm": 18000, "feed": 500, "plunge_feed": 150, "stepdown": 1}),
        );
        run(&mut s, "cam.setup", json!({"name": "S"}));
        let square = json!([[0, 0], [20, 0], [20, 20], [0, 20]]);
        let first = run(&mut s, "cam.pocket", json!({"setup": "S", "name": "Rough", "tool": 1, "outer": square, "bottom": -2}));
        assert!(first.get("rest").is_none());
        let rest =
            run(&mut s, "cam.pocket", json!({"setup": "S", "name": "Corners", "tool": 2, "outer": square, "bottom": -2, "rest_from": "Rough"}));
        assert_eq!(rest["rest"]["from"], json!("Rough"), "{rest}");
        let length = rest["rest"]["cut_length"].as_f64().unwrap();
        // Four corners on two passes, linked by feeds across the cleared floor: far less than
        // the 4 × 72 mm of rings a full pocket with the 2 mm cutter would run.
        assert!(length > 0.0 && length < 200.0, "{rest}");
        let list = run(&mut s, "cam.list", json!({}));
        assert_eq!(list["setups"][0]["operations"][1]["rest_from"], json!("Rough"), "{list}");
        let CamOperation::Pocket { params, .. } = &s.doc.cam.setups[0].operations[1] else { panic!() };
        assert_eq!(params.rest.map(|r| (r.radius, r.wall)), Some((3.0, 3.0)));
        let posted = run(&mut s, "cam.post", json!({"setup": "S"}));
        assert_eq!(posted["readback"]["within"], json!(true), "{posted}");
        // A round pocket leaves nothing: the answer says so and saves no operation.
        let disc: Vec<[f64; 2]> = (0..72)
            .map(|i| {
                let a = (i as f64) * std::f64::consts::TAU / 72.0;
                [50.0 + 10.0 * a.cos(), 10.0 + 10.0 * a.sin()]
            })
            .collect();
        run(&mut s, "cam.pocket", json!({"setup": "S", "name": "Disc", "tool": 1, "outer": disc, "bottom": -2}));
        let none = run(&mut s, "cam.pocket", json!({"setup": "S", "name": "Disc rest", "tool": 2, "outer": disc, "bottom": -2, "rest_from": "Disc"}));
        assert_eq!(none["rest"]["nothing_left"], json!(true), "{none}");
        assert!(s.doc.cam.setups[0].operations.iter().all(|o| o.name() != "Disc rest"));
        // Not after an unknown operation, not with the same tool, not a pocket after a contour.
        run(&mut s, "cam.contour", json!({"setup": "S", "name": "Edge", "tool": 1, "outer": square, "bottom": -2}));
        for (bad, says) in [
            (json!({"setup": "S", "name": "X", "tool": 2, "outer": square, "bottom": -2, "rest_from": "Nope"}), "Nope"),
            (json!({"setup": "S", "name": "X", "tool": 1, "outer": square, "bottom": -2, "rest_from": "Rough"}), "not smaller"),
            (json!({"setup": "S", "name": "X", "tool": 2, "outer": square, "bottom": -2, "rest_from": "Edge"}), "contour"),
        ] {
            let err = s.execute("cam.pocket", &bad).unwrap_err().to_string();
            assert!(err.contains(says), "{err}");
        }
        // A contour inside after a contour inside: the corners again.
        run(&mut s, "cam.contour", json!({"setup": "S", "name": "Inside", "tool": 1, "outer": square, "side": "inside", "bottom": -2}));
        let r = run(
            &mut s,
            "cam.contour",
            json!({"setup": "S", "name": "Inside rest", "tool": 2, "outer": square, "side": "inside", "bottom": -2, "rest_from": "Inside"}),
        );
        assert!(r["rest"]["cut_length"].as_f64().unwrap() > 0.0, "{r}");
        let other_side = json!({"setup": "S", "name": "X", "tool": 2, "outer": square, "side": "outside", "bottom": -2, "rest_from": "Inside"});
        assert!(s.execute("cam.contour", &other_side).unwrap_err().to_string().contains("side"));
    }

    #[test]
    fn the_simulation_writes_a_height_map_and_tells_what_stands() {
        let mut s = Session::default();
        run(
            &mut s,
            "cam.tool",
            json!({"number": 1, "diameter": 6, "flute_length": 20, "spindle_rpm": 8000, "feed": 600, "plunge_feed": 200, "stepdown": 3}),
        );
        run(&mut s, "cam.setup", json!({"name": "Top", "stock": {"min": [0, 0, -10], "max": [40, 40, 0]}}));
        run(
            &mut s,
            "cam.pocket",
            json!({"setup": "Top", "name": "Pocket", "tool": 1, "outer": [[5,5],[35,5],[35,20],[5,20]], "stepover": 2.5, "bottom": -3}),
        );
        // A bar with a 2 mm stem hanging from it: the 6 mm cutter clears the bar, never the stem.
        let bar = json!([[5, 26], [19, 26], [19, 21], [21, 21], [21, 26], [35, 26], [35, 36], [5, 36]]);
        run(&mut s, "cam.pocket", json!({"setup": "Top", "name": "Bar", "tool": 1, "outer": bar, "bottom": -1}));
        let dir = std::env::temp_dir().join(format!("solidcraft-cam-sim-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("stock.png");
        let r = run(&mut s, "cam.simulate", json!({"setup": "Top", "path": file.to_string_lossy(), "resolution": 0.5}));
        assert_eq!(r["size"], json!([80, 80]), "{r}");
        assert_eq!(r["deepest"], json!(-3.0), "{r}");
        // The pocket takes 30 × 15 × 3 less the four corners the 6 mm cutter leaves round.
        let removed = r["operations"][0]["removed_mm3"].as_f64().unwrap();
        let expect = (450.0 - 4.0 * (9.0 - 9.0 * std::f64::consts::PI / 4.0)) * 3.0;
        assert!((removed - expect).abs() < expect * 0.03, "{removed} vs {expect}");
        assert!(r["operations"][0]["standing_in_regions_percent"].as_f64().unwrap() < 2.0, "{r}");
        // The stem is 10 of the bar's 310 mm², standing; the corners of the bar add a little.
        let standing = r["operations"][1]["standing_in_regions_percent"].as_f64().unwrap();
        assert!((2.5..8.0).contains(&standing), "{r}");
        assert!(r["removed_mm3"].as_f64().unwrap() > removed + 250.0, "the bar too: {r}");
        let bytes = std::fs::read(&file).unwrap();
        assert_eq!(&bytes[..8], b"\x89PNG\r\n\x1a\n");
        // One operation only; an unknown one, no stock and no path are refused.
        let one = run(&mut s, "cam.simulate", json!({"setup": "Top", "operations": ["Bar"], "path": file.to_string_lossy()}));
        assert_eq!(one["operations"].as_array().map(Vec::len), Some(1));
        assert!(s.execute("cam.simulate", &json!({"setup": "Top", "operations": ["Nope"], "path": file.to_string_lossy()})).is_err());
        assert!(s.execute("cam.simulate", &json!({"setup": "Top"})).unwrap_err().to_string().contains("path"));
        run(&mut s, "cam.setup", json!({"name": "Bare"}));
        run(&mut s, "cam.contour", json!({"setup": "Bare", "name": "C", "tool": 1, "outer": [[0,0],[40,0],[40,30],[0,30]], "bottom": -2}));
        assert!(s.execute("cam.simulate", &json!({"setup": "Bare", "path": file.to_string_lossy()})).unwrap_err().to_string().contains("stock"));
        let r = run(
            &mut s,
            "cam.simulate",
            json!({"setup": "Bare", "path": file.to_string_lossy(), "stock": {"min": [-5, -5, -10], "max": [45, 35, 0]}, "resolution": 1}),
        );
        assert_eq!(r["size"], json!([50, 40]), "{r}");
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn list_and_delete() {
        let mut s = Session::default();
        run(
            &mut s,
            "cam.tool",
            json!({"number": 2, "diameter": 3, "flute_length": 12, "kind": "ball_endmill", "spindle_rpm": 18000, "feed": 900, "plunge_feed": 300}),
        );
        run(&mut s, "cam.setup", json!({"name": "S"}));
        run(
            &mut s,
            "cam.contour",
            json!({"setup": "S", "name": "C", "tool": 2, "outer": [[0,0],[20,0],[20,10],[0,10]], "bottom": -2, "stepdown": 1}),
        );
        let list = run(&mut s, "cam.list", json!({}));
        assert_eq!(list["tools"].as_array().unwrap().len(), 1);
        assert_eq!(list["setups"][0]["operations"].as_array().unwrap().len(), 1);
        run(&mut s, "cam.delete_op", json!({"setup": "S", "name": "C"}));
        run(&mut s, "cam.setup", json!({"name": "S", "delete": true}));
        run(&mut s, "cam.tool", json!({"number": 2, "delete": true}));
        assert!(s.doc.cam.is_empty());
    }

    #[test]
    fn a_post_saved_with_the_design_writes_its_programs() {
        let mut s = Session::default();
        let ring = |rpm: u32| json!({"setup": "Top", "name": "Ring", "tool": 1, "outer": [[0,0],[40,0],[40,30],[0,30]], "bottom": -2, "stepdown": 2, "spindle_rpm": rpm});
        run(&mut s, "cam.tool", json!({"number": 1, "diameter": 6, "flute_length": 20, "feed": 1200, "plunge_feed": 400}));
        run(&mut s, "cam.setup", json!({"name": "Top"}));
        run(&mut s, "cam.contour", ring(2000));
        let mut post = serde_json::to_value(Post::rs274()).unwrap();
        post["name"] = json!("Half turns");
        post["header"] = json!(["%", "G17 {units} G90 {work_offset}"]);
        post["max_arc_sweep"] = json!(180);
        post["spindle_range"] = json!({"min": 3000, "max": 24000});
        post["one_tool"] = json!(true);
        run(&mut s, "cam.post.define", json!({"post": post}));
        // 2000 rev/min is below what the machine runs.
        let refused = s.execute("cam.post", &json!({"setup": "Top", "post": "Half turns"})).unwrap_err().to_string();
        assert!(refused.contains("outside the spindle range"), "{refused}");
        run(&mut s, "cam.delete_op", json!({"setup": "Top", "name": "Ring"}));
        run(&mut s, "cam.contour", ring(12000));
        let posted = run(&mut s, "cam.post", json!({"setup": "Top", "post": "Half turns"}));
        assert!(posted["gcode"].as_str().unwrap().lines().nth(1) == Some("%"), "{}", posted["gcode"]);
        assert!(posted["readback"]["within"].as_bool().unwrap());

        // Saved with the design, replaced by name, and removed again.
        let cold = Session::new(solidcraft_doc::Document::from_json(&s.doc.to_json()).unwrap());
        assert_eq!(cold.doc.cam.posts.len(), 1);
        post["decimals"] = json!(3);
        run(&mut s, "cam.post.define", json!({"post": post}));
        assert_eq!(s.doc.cam.posts.len(), 1);
        assert_eq!(s.doc.cam.posts[0].decimals, 3);
        run(&mut s, "cam.post.define", json!({"name": "Half turns", "delete": true}));
        assert!(s.doc.cam.posts.is_empty());

        // Bad posts and names are refused.
        let mut builtin = post.clone();
        builtin["name"] = json!("LinuxCNC");
        assert!(s.execute("cam.post.define", &json!({"post": builtin})).is_err());
        let mut broken = post.clone();
        broken["max_arc_sweep"] = json!(720);
        assert!(s.execute("cam.post.define", &json!({"post": broken})).is_err());
        assert!(s.execute("cam.post.define", &json!({"post": "rs274"})).is_err());
        assert!(s.execute("cam.post.define", &json!({"name": "missing", "delete": true})).is_err());
    }

    #[test]
    fn a_part_drawn_in_a_sketch_is_cut_free_and_written_to_a_file() {
        let mut s = Session::default();
        // A 100 × 50 plate with a round hole, a square one and a 4 mm hole a 6 mm tool can't enter.
        run(&mut s, "sketch.create", json!({"plane": "XY", "name": "Plate"}));
        run(&mut s, "sketch.rectangle.two_point", json!({"p0": [0, 0], "p1": [100, 50]}));
        run(&mut s, "sketch.circle.center", json!({"center": [30, 25], "radius": 10}));
        run(&mut s, "sketch.rectangle.two_point", json!({"p0": [60, 15], "p1": [80, 35]}));
        run(&mut s, "sketch.circle.center", json!({"center": [90, 40], "radius": 2}));
        run(&mut s, "sketch.finish", json!({}));
        run(&mut s, "cam.tool", json!({"number": 24, "diameter": 6, "flute_length": 25}));
        run(&mut s, "cam.setup", json!({"name": "Sheet", "stock": {"min": [0, 0, -18], "max": [70, 120, 0]}}));

        // Cutting data are the tool's or the operation's, never made up.
        let op =
            json!({"setup": "Sheet", "name": "Part", "tool": 24, "sketch": "Plate", "rotate": 90, "at": [10, 10], "bottom": -18.3, "stepdown": 6.1});
        let refused = s.execute("cam.contour", &op).unwrap_err().to_string();
        assert!(refused.contains("no feed"), "{refused}");
        run(&mut s, "cam.tool", json!({"number": 24, "diameter": 6, "flute_length": 25, "spindle_rpm": 16700, "feed": 4010, "plunge_feed": 1203}));

        // The plate, not the insides of its holes; turned upright and moved to (10, 10).
        let made = run(&mut s, "cam.contour", op);
        assert_eq!(made["bounds"], json!({"min": [10.0, 10.0], "max": [60.0, 110.0]}));
        let Some(CamOperation::Contour { regions, .. }) = s.doc.cam.setups[0].operations.first() else { panic!("{:?}", s.doc.cam) };
        assert_eq!((regions.len(), regions[0].holes.len()), (1, 3));
        let small = made["too_small"].as_array().unwrap();
        assert_eq!(small.len(), 1, "{made}");
        assert!((small[0]["widest_circle"].as_f64().unwrap() - 4.0).abs() < 0.05, "{made}");
        assert!(
            s.execute("cam.contour", &json!({"setup": "Sheet", "name": "X", "tool": 24, "sketch": "Plate", "rotate": 45, "bottom": -1})).is_err()
        );

        // One hole picked by a point inside it, cut from the inside.
        let hole = run(
            &mut s,
            "cam.contour",
            json!({"setup": "Sheet", "name": "Hole", "tool": 24, "sketch": "Plate", "profiles": [{"point": [30, 25]}], "side": "inside", "bottom": -1, "stepdown": 1}),
        );
        assert_eq!(hole["bounds"], json!({"min": [20.0, 15.0], "max": [40.0, 35.0]}));
        run(&mut s, "cam.delete_op", json!({"setup": "Sheet", "name": "Hole"}));

        // Tabs hold the plate (and its holes' waste), dogbones notch the square hole's corners.
        let held = json!({"setup": "Sheet", "name": "Part", "tool": 24, "sketch": "Plate", "rotate": 90, "at": [10, 10], "bottom": -18.3,
            "stepdown": 6.1, "tabs": true, "dogbones": true});
        run(&mut s, "cam.contour", held.clone());
        let Some(CamOperation::Contour { params, .. }) = s.doc.cam.setups[0].operations.first() else { panic!("{:?}", s.doc.cam) };
        assert_eq!((params.tabs.map(|t| (t.width, t.height, t.inner)), params.dogbones), (Some((8.0, 3.0, true)), true));
        for bad_tabs in [json!({"height": 18.3}), json!("yes"), json!({"width": -1})] {
            let mut op = held.clone();
            op["tabs"] = bad_tabs.clone();
            assert!(s.execute("cam.contour", &op).is_err(), "{bad_tabs}");
        }

        let dir = std::env::temp_dir().join(format!("solidcraft-cam-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let file = dir.join("Plate.ngc");
        let posted = run(&mut s, "cam.post", json!({"setup": "Sheet", "path": file.to_string_lossy()}));
        let text = std::fs::read_to_string(&file).unwrap();
        assert!(posted.get("gcode").is_none() && posted["readback"]["within"] == json!(true), "{posted}");
        assert_eq!(posted["lines"].as_u64(), Some(text.lines().count() as u64));
        // Cut through, but the last pass rises over the tabs, 3 mm above the bottom.
        assert!(text.contains("Z-18.3") && text.contains("Z-15.3") && text.contains("S16700") && text.contains("F4010"));
        std::fs::remove_dir_all(&dir).ok();
    }

    /// The tool and setup the other tests share: a 6 mm cutter with all its cutting data.
    fn tooled() -> Session {
        let mut s = Session::default();
        run(
            &mut s,
            "cam.tool",
            json!({"number": 1, "diameter": 6, "flute_length": 22, "spindle_rpm": 16000, "feed": 1500, "plunge_feed": 500, "stepdown": 3}),
        );
        s
    }

    /// An update of a setup changes what it is given and keeps the rest: a setup on G55 stays on
    /// G55 when its stock is set, and keeps its stock when its work offset changes.
    #[test]
    fn a_setup_update_keeps_what_it_is_not_given() {
        let mut s = tooled();
        run(&mut s, "cam.setup", json!({"name": "Top", "work_offset": 2}));
        run(&mut s, "cam.setup", json!({"name": "Top", "stock": {"min": [0, 0, -18], "max": [100, 50, 0]}}));
        assert_eq!((s.doc.cam.setups[0].work_offset, s.doc.cam.setups[0].stock.is_some()), (2, true));
        run(&mut s, "cam.contour", json!({"setup": "Top", "name": "C", "tool": 1, "outer": [[0,0],[40,0],[40,30],[0,30]], "bottom": -3}));
        run(&mut s, "cam.setup", json!({"name": "Top", "work_offset": 3}));
        let top = &s.doc.cam.setups[0];
        assert_eq!((top.work_offset, top.stock.is_some(), top.operations.len()), (3, true, 1));
        assert!(run(&mut s, "cam.post", json!({"setup": "Top"}))["gcode"].as_str().unwrap().contains("G56"));
        run(&mut s, "cam.setup", json!({"name": "Top", "stock": null}));
        assert_eq!((s.doc.cam.setups[0].work_offset, s.doc.cam.setups[0].stock), (3, None));
        for bad in [json!(1.5), json!(0), json!(7), json!("2")] {
            assert!(s.execute("cam.setup", &json!({"name": "Top", "work_offset": bad})).is_err(), "{bad}");
        }
        assert_eq!(s.doc.cam.setups[0].work_offset, 3);
    }

    /// No cutting data made up: no depth per pass when neither the operation nor its tool has one,
    /// and no spindle speed when posting an operation saved without one.
    #[test]
    fn no_stepdown_or_spindle_speed_is_made_up() {
        let mut s = Session::default();
        run(&mut s, "cam.tool", json!({"number": 1, "diameter": 6, "flute_length": 22, "spindle_rpm": 16000, "feed": 1500, "plunge_feed": 500}));
        run(&mut s, "cam.setup", json!({"name": "Top"}));
        let op = json!({"setup": "Top", "name": "C", "tool": 1, "outer": [[0,0],[40,0],[40,30],[0,30]], "bottom": -18});
        let refused = s.execute("cam.contour", &op).unwrap_err().to_string();
        assert!(refused.contains("no stepdown"), "{refused}");
        let mut given = op.clone();
        given["stepdown"] = json!(6);
        run(&mut s, "cam.contour", given);
        // As a design file without the field (an older one, or edited by hand) holds it.
        if let Some(CamOperation::Contour { spindle_rpm, .. }) = s.doc_mut().cam.setups[0].operations.first_mut() {
            *spindle_rpm = None;
        }
        s.doc_mut().cam.tools[0].spindle_rpm = None;
        let refused = s.execute("cam.post", &json!({"setup": "Top"})).unwrap_err().to_string();
        assert!(refused.contains("no spindle speed") && !refused.contains("6000"), "{refused}");
    }

    /// With a stock on the setup, an operation starts at its top and moves rapidly above it; one
    /// whose rapid moves would run inside it is refused, also when the stock grows later.
    #[test]
    fn rapid_moves_stay_above_the_stock() {
        let mut s = tooled();
        run(&mut s, "cam.setup", json!({"name": "Sheet", "stock": {"min": [0, 0, 0], "max": [600, 400, 18]}}));
        let op = json!({"setup": "Sheet", "name": "C", "tool": 1, "outer": [[0,0],[40,0],[40,30],[0,30]], "bottom": -0.3, "stepdown": 6.1});
        let mut low = op.clone();
        low["top"] = json!(0);
        low["retract"] = json!(2);
        let refused = s.execute("cam.contour", &low).unwrap_err().to_string();
        assert!(refused.contains("inside the stock") && refused.contains("Z18"), "{refused}");
        run(&mut s, "cam.contour", op);
        let Some(CamOperation::Contour { cut, .. }) = s.doc.cam.setups[0].operations.first() else { panic!("{:?}", s.doc.cam) };
        assert_eq!((cut.heights.top, cut.heights.retract, cut.heights.clearance), (18.0, 20.0, 28.0));
        let thicker = s.execute("cam.setup", &json!({"name": "Sheet", "stock": {"min": [0, 0, 0], "max": [600, 400, 25]}})).unwrap_err();
        assert!(thicker.to_string().contains("inside the stock"), "{thicker}");
        assert_eq!(s.doc.cam.setups[0].stock.map(|b| b.max[2]), Some(18.0));
    }

    /// A tool an operation uses is not deleted under it, and two tools with one number (as a file
    /// edited by hand can hold) are refused before anything is posted.
    #[test]
    fn tools_in_use_stay_and_duplicates_are_refused() {
        let mut s = tooled();
        run(&mut s, "cam.setup", json!({"name": "Top"}));
        run(&mut s, "cam.pocket", json!({"setup": "Top", "name": "P", "tool": 1, "outer": [[0,0],[40,0],[40,30],[0,30]], "bottom": -3}));
        let refused = s.execute("cam.tool", &json!({"number": 1, "delete": true})).unwrap_err().to_string();
        assert!(refused.contains("`Top`/`P`"), "{refused}");
        assert_eq!(s.doc.cam.tools.len(), 1);
        let twin = s.doc.cam.tools[0].clone();
        s.doc_mut().cam.tools.push(twin);
        let refused = s.execute("cam.post", &json!({"setup": "Top"})).unwrap_err().to_string();
        assert!(refused.contains("two CAM tools"), "{refused}");
    }
}
