//! CAM commands: tools, setups, 2D contour/pocket operations, and G-code posting through
//! `solidcraft-cam`. Toolpaths are computed at post time and never stored on the design.

use serde_json::{Value, json};
use solidcraft_cam::{Cut, Feeds, Heights, Operation, Post, Program, Side, Tabs, Tool, ToolKind, contour, emit, pocket, readback};
use solidcraft_doc::ProfileSel;
use solidcraft_doc::cam::{
    CamOperation, CamSettings, CamSetup, MAX_LOOP_POINTS, MAX_OPS, MAX_POSTS, MAX_SETUPS, MAX_TOOLS, PolygonRegion, StockBox, default_contour_params,
    default_cut, default_pocket_params,
};
use solidcraft_geom::{Region2, Vec2};

use super::CommandSpec;
use crate::params::{bad, bool_, num, str_, vec2, vec2_list};
use crate::{EngineError, Result, Session};

/// Chord error of sketch profiles turned into an operation's polygons (mm).
const PROFILE_TOLERANCE: f64 = 0.005;

pub static COMMANDS: &[CommandSpec] = &[
    CommandSpec::new("cam.tool", "CAM Tool", tool_cmd).at("MANUFACTURE", "SETUP").icon("tool").params(
        "number (1…9999); kind?: flat_endmill|ball_endmill|drill (default flat_endmill); diameter (mm); flute_length (mm); \
         description?; flutes?; spindle_rpm?; feed?; plunge_feed?; stepdown?; stepover?; delete?: bool",
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
         tool's; refused when neither has them); coolant?; cut?: {heights, feeds, \
         stepdown} → {setup, operation, bounds, too_small?: openings the tool does not fit, left uncut}",
    ),
    CommandSpec::new("cam.pocket", "2D Pocket", pocket_cmd).at("MANUFACTURE", "MILLING").icon("pocket").params(
        "setup; name; tool (number); outer: [[x,y],…] (or regions, or sketch with profiles?, rotate?, at? as cam.contour \
         takes them); holes?; stepover? (default the tool's, else half its diameter); climb?; stock_to_leave?; tolerance?; \
         ramp_angle?: deg; dogbones?: bool (notch the pocket's inside corners, so a square part fits); clearance?; \
         retract?; top?; bottom; stepdown?; feed?; plunge_feed?; spindle_rpm?; coolant?; cut? → {setup, operation, \
         bounds, too_small?}",
    ),
    CommandSpec::new("cam.delete_op", "Delete CAM Operation", delete_op).at("MANUFACTURE", "MILLING").icon("delete").params("setup; name"),
    CommandSpec::new("cam.post", "Post Process", post_cmd).at("MANUFACTURE", "POST").icon("post").noundo().params(
        "setup; post?: rs274 | name of a saved post (default rs274); program?: name (default the setup's); \
         verify?: bool (default true: read the G-code back and check it against the toolpaths); path?: file to write the \
         program to (then the result has `lines` instead of `gcode`) → {gcode | path, lines, extension, post, operations, \
         readback?}",
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
        other => return Err(bad(cmd, format!("unknown tool kind `{other}` (flat_endmill, ball_endmill, drill)"))),
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

fn op_cmd(s: &mut Session, p: &Value, cmd: &str, is_contour: bool) -> Result<Value> {
    let setup_name = str_(p, "setup").map(str::trim).filter(|n| !n.is_empty()).ok_or_else(|| bad(cmd, "`setup` is required"))?.to_string();
    let name = str_(p, "name").map(str::trim).filter(|n| !n.is_empty() && n.len() <= 128).ok_or_else(|| bad(cmd, "`name` is required"))?.to_string();
    let tool_n =
        num(p, "tool").filter(|n| *n >= 1.0 && n.fract() == 0.0).map(|n| n as u32).ok_or_else(|| bad(cmd, "`tool` must be a tool number"))?;
    let tool = s.doc.cam.tool(tool_n).cloned().ok_or_else(|| bad(cmd, format!("no tool {tool_n} (cam.tool first)")))?;
    let stock_top =
        s.doc.cam.setup(&setup_name).ok_or_else(|| bad(cmd, format!("no setup `{setup_name}` (cam.setup first)")))?.stock.map(|b| b.max[2]);
    let regions = regions_param(s, cmd, p)?;
    let cut = cut_param(cmd, p, &tool, stock_top)?;
    // A machine runs what the program says: no speed made up for a tool that has none.
    let spindle_rpm = num(p, "spindle_rpm")
        .or(tool.spindle_rpm)
        .ok_or_else(|| bad(cmd, format!("no spindle speed: give `spindle_rpm` or set tool {tool_n}'s (cam.tool spindle_rpm)")))?;
    let spindle_rpm = Some(spindle_rpm);
    let coolant = bool_(p, "coolant").unwrap_or(false);
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
        CamOperation::Contour { name: name.clone(), tool: tool_n, regions, params, cut, spindle_rpm, coolant }
    } else {
        let stepover = num(p, "stepover").or(tool.stepover).unwrap_or(tool.diameter * 0.5);
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
        CamOperation::Pocket { name: name.clone(), tool: tool_n, regions, params, cut, spindle_rpm, coolant }
    };
    op.validate().map_err(|e| bad(cmd, e.to_string()))?;
    if let Some(setup) = s.doc.cam.setup(&setup_name) {
        setup.check_clear_of_stock(&op).map_err(|e| bad(cmd, e.to_string()))?;
    }
    // Prove the toolpath builds before saving the operation.
    build_path(&tool, &op).map_err(|e| bad(cmd, e))?;
    let (bounds, too_small) = reach_report(&tool, &op);
    let doc = s.doc_mut();
    let setup = doc.cam.setup_mut(&setup_name).ok_or_else(|| bad(cmd, format!("no setup `{setup_name}` (cam.setup first)")))?;
    if let Some(slot) = setup.operations.iter_mut().find(|o| o.name() == name) {
        *slot = op;
    } else {
        if setup.operations.len() >= MAX_OPS {
            return Err(bad(cmd, format!("a setup has at most {MAX_OPS} operations")));
        }
        setup.operations.push(op);
    }
    let mut out = json!({"setup": setup_name, "operation": name, "bounds": bounds});
    if !too_small.is_empty() {
        out["too_small"] = json!(too_small);
    }
    Ok(out)
}

/// The bounds of an operation's regions, and the openings its tool cannot enter: holes of a part
/// cut round the outside, or regions cut or cleared from the inside, narrower than the tool. The
/// offset drops them, so they would be left uncut without a word.
fn reach_report(tool: &Tool, op: &CamOperation) -> (Value, Vec<Value>) {
    let (regions, inside, leave) = match op {
        CamOperation::Contour { regions, params, .. } => (regions, params.side != Side::Outside, params.stock_to_leave),
        CamOperation::Pocket { regions, params, .. } => (regions, true, params.stock_to_leave),
    };
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for p in regions.iter().flat_map(|r| &r.outer) {
        (lo[0], lo[1], hi[0], hi[1]) = (lo[0].min(p[0]), lo[1].min(p[1]), hi[0].max(p[0]), hi[1].max(p[1]));
    }
    let reach = tool.radius() + leave;
    let mut small = Vec::new();
    for region in regions.iter().filter_map(|r| r.to_region().ok()) {
        let openings: Vec<_> = if inside { vec![region.outer] } else { region.holes };
        for lp in openings {
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
    // Quarter turns only: the part keeps its edges along the machine's axes.
    let deg = num(p, "rotate").unwrap_or(0.0);
    let turn = [0.0, 90.0, 180.0, 270.0].iter().position(|&d| d == deg).ok_or_else(|| bad(cmd, "`rotate` must be 0, 90, 180 or 270"))?;
    let rotate = |q: Vec2| (0..turn).fold(q, |q, _| Vec2::new(-q.y, q.x));
    let ring = |lp: &solidcraft_geom::Loop2| -> Result<Vec<Vec2>> {
        let pts: Vec<Vec2> = lp.polyline(PROFILE_TOLERANCE).into_vec().into_iter().map(rotate).collect();
        if pts.len() > MAX_LOOP_POINTS {
            return Err(bad(cmd, format!("a profile loop needs more than {MAX_LOOP_POINTS} points within {PROFILE_TOLERANCE} mm")));
        }
        Ok(pts)
    };
    let mut loops: Vec<(Vec<Vec2>, Vec<Vec<Vec2>>)> =
        picked.iter().map(|r| Ok((ring(&r.outer)?, r.holes.iter().map(ring).collect::<Result<Vec<_>>>()?))).collect::<Result<_>>()?;
    if let Some(at) = p.get("at") {
        let at = vec2(at).ok_or_else(|| bad(cmd, "`at` must be [x, y]"))?;
        let lo = loops.iter().flat_map(|(o, _)| o).fold(Vec2::new(f64::INFINITY, f64::INFINITY), |m, q| Vec2::new(m.x.min(q.x), m.y.min(q.y)));
        let shift = at - lo;
        for (outer, holes) in &mut loops {
            for q in outer.iter_mut().chain(holes.iter_mut().flatten()) {
                *q += shift;
            }
        }
    }
    loops
        .into_iter()
        .map(|(outer, holes)| {
            let xy = |pts: Vec<Vec2>| pts.into_iter().map(|q| [q.x, q.y]).collect::<Vec<_>>();
            let region = PolygonRegion { outer: xy(outer), holes: holes.into_iter().map(xy).collect() };
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

/// The heights, feeds and depth per pass of an operation. Its top is where the stock's is when the
/// setup has one; the retract and clearance heights sit 2 and 10 mm above the top unless given.
fn cut_param(cmd: &str, p: &Value, tool: &Tool, stock_top: Option<f64>) -> Result<Cut> {
    if let Some(raw) = p.get("cut") {
        return serde_json::from_value(raw.clone()).map_err(|e| bad(cmd, format!("`cut` is invalid: {e}")));
    }
    let bottom = num(p, "bottom").ok_or_else(|| bad(cmd, "`bottom` is required (final Z, mm)"))?;
    let top = num(p, "top").or(stock_top).unwrap_or(0.0);
    let retract = num(p, "retract").unwrap_or(top + 2.0);
    let clearance = num(p, "clearance").unwrap_or(top + 10.0);
    let n = tool.number;
    // As with the feeds: no depth per pass made up (the whole depth at once could break the tool).
    let stepdown = num(p, "stepdown")
        .or(tool.stepdown)
        .ok_or_else(|| bad(cmd, format!("no stepdown: give `stepdown` or set tool {n}'s (cam.tool stepdown)")))?;
    let feed = num(p, "feed").or(tool.feed).ok_or_else(|| bad(cmd, format!("no feed: give `feed` or set tool {n}'s (cam.tool feed)")))?;
    let plunge = num(p, "plunge_feed")
        .or(tool.plunge_feed)
        .ok_or_else(|| bad(cmd, format!("no plunge feed: give `plunge_feed` or set tool {n}'s (cam.tool plunge_feed)")))?;
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
        let (spindle_rpm, coolant) = match op {
            CamOperation::Contour { spindle_rpm, coolant, .. } | CamOperation::Pocket { spindle_rpm, coolant, .. } => {
                // A machine runs what the program says: no speed made up here either.
                let rpm = spindle_rpm.or(tool.spindle_rpm).ok_or_else(|| {
                    bad(cmd, format!("operation `{}` has no spindle speed: give it `spindle_rpm` or set tool {}'s", op.name(), tool.number))
                })?;
                (rpm, *coolant)
            }
        };
        program.operations.push(Operation { name: op.name().into(), tool, spindle_rpm, coolant, path });
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
    let regions: Vec<_> = match op {
        CamOperation::Contour { regions, .. } | CamOperation::Pocket { regions, .. } => {
            regions.iter().map(|r| r.to_region().map_err(|e| e.to_string())).collect::<std::result::Result<Vec<_>, _>>()?
        }
    };
    match op {
        CamOperation::Contour { params, cut, .. } => contour(&regions, tool, params, cut).map_err(|e| e.to_string()),
        CamOperation::Pocket { params, cut, .. } => pocket(&regions, tool, params, cut).map_err(|e| e.to_string()),
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
            CamOperation::Contour { name, tool, regions, params, cut, spindle_rpm, coolant } => json!({
                "type": "contour", "name": name, "tool": tool, "regions": regions.len(),
                "params": params, "cut": cut, "spindle_rpm": spindle_rpm, "coolant": coolant,
            }),
            CamOperation::Pocket { name, tool, regions, params, cut, spindle_rpm, coolant } => json!({
                "type": "pocket", "name": name, "tool": tool, "regions": regions.len(),
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
