//! Contracts and result types for the manufacturing commands in `cmd/cam.rs`: tools, setups,
//! the 2D milling, engraving, v-carving and drilling operations, posting and the stock
//! simulation.
//!
//! The handlers read their parameters from the raw JSON. The keys whose shape a handler checks
//! itself (a region, a stock box, a side, a tool kind and so on) are read here by the handler's
//! own reader, so they fail with its message; plain numbers, flags and names are plain types,
//! which a handler reads more leniently (a value of the wrong type as if the key were absent).
//! Lengths are numbers in millimetres, feeds in mm/min, angles in degrees: none is an expression.

use std::borrow::Cow;

use schemars::{JsonSchema, Schema, SchemaGenerator, json_schema};
use serde::de::Error as _;
use serde::{Deserialize, Deserializer, Serialize};
use serde_json::{Value, json};
use solidcraft_cam::{Cut, Post};

use super::check::{keyed, whole};
use super::types::{IdOrName, Point2, ProfilesParam, StringList, present};
use crate::{EngineError, cmd, params};

/// A handler's refusal as the message alone, marked complete: the handler names the key itself.
fn handlers(e: EngineError) -> String {
    match e {
        EngineError::BadParams { msg, .. } => whole(msg),
        other => whole(other),
    }
}

/// A value kept as it came and checked by `$check`, whose error is the deserializer's message
/// (a handler's own wording through [`handlers`], or one that starts with "must").
macro_rules! checked_value {
    ($(#[$doc:meta])* $name:ident, $schema:expr, $check:expr) => {
        $(#[$doc])*
        #[derive(Clone, Debug, PartialEq)]
        pub struct $name(pub Value);

        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D: Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
                let v = Value::deserialize(d)?;
                let check: fn(&Value) -> Result<(), String> = $check;
                check(&v).map_err(D::Error::custom)?;
                Ok($name(v))
            }
        }

        impl JsonSchema for $name {
            fn schema_name() -> Cow<'static, str> {
                stringify!($name).into()
            }
            fn json_schema(_: &mut SchemaGenerator) -> Schema {
                $schema
            }
        }
    };
}

/// A point `[x, y]` or `{x, y}` in mm, as `params::vec2` reads it.
fn point() -> Value {
    let c = json!({"type": "number", "exclusiveMinimum": -1e9, "exclusiveMaximum": 1e9});
    json!({"anyOf": [
        {"type": "array", "items": c, "minItems": 2, "maxItems": 2},
        {"type": "object", "properties": {"x": c, "y": c}, "required": ["x", "y"]}
    ]})
}

/// A corner `[x, y, z]` of a box, in mm.
fn corner() -> Value {
    json!({"type": "array", "items": {"type": "number"}, "minItems": 3, "maxItems": 3})
}

checked_value!(
    /// `number`: a tool's number, a whole number from 1 to 9999.
    ToolNumber,
    json_schema!({"description": "A tool number: a whole number from 1 to 9999.", "type": "number", "minimum": 1, "maximum": 9999}),
    |v| match v.as_f64() {
        Some(n) if (1.0..=9999.0).contains(&n) && n.fract() == 0.0 => Ok(()),
        _ => Err(whole("`number` must be an integer 1…9999")),
    }
);

checked_value!(
    /// `tool`: the number of the tool an operation cuts with, a whole number from 1.
    OperationTool,
    json_schema!({"description": "The number of a tool of the design (cam.tool): a whole number from 1.", "type": "number", "minimum": 1}),
    |v| match v.as_f64().and_then(params::finite) {
        Some(n) if n >= 1.0 && n.fract() == 0.0 => Ok(()),
        _ => Err(whole("`tool` must be a tool number")),
    }
);

checked_value!(
    /// `kind`: a tool kind by name, as `cam::tool_kind` reads it.
    ToolKindParam,
    json_schema!({
        "description": "The tool's shape. `v_bit` is another spelling of `vbit`.",
        "type": "string",
        "enum": ["flat_endmill", "ball_endmill", "drill", "vbit", "v_bit", "tapered_ball"]
    }),
    |v| match v {
        Value::String(kind) => cmd::cam_tool_kind("", kind).map(|_| ()).map_err(handlers),
        _ => Err("must be a tool kind: flat_endmill, ball_endmill, drill, vbit or tapered_ball".into()),
    }
);

checked_value!(
    /// `work_offset`: 1 to 6, for G54 to G59.
    WorkOffset,
    json_schema!({"description": "The work offset: 1 to 6, for G54 to G59.", "type": "integer", "minimum": 1, "maximum": 6}),
    |v| match v.as_f64().and_then(params::finite) {
        Some(n) if n.fract() == 0.0 && (1.0..=6.0).contains(&n) => Ok(()),
        _ => Err(whole("`work_offset` must be an integer 1…6 (G54…G59)")),
    }
);

checked_value!(
    /// `stock`: a box `{min: [x, y, z], max: [x, y, z]}` in work coordinates, as
    /// `cam::stock_box` reads and checks it.
    StockParam,
    json_schema!({
        "description": "The stock: a box in work coordinates (mm), `min` below `max` on every axis.",
        "type": "object",
        "properties": {"min": corner(), "max": corner()},
        "required": ["min", "max"]
    }),
    |v| cmd::cam_stock_box("", v).map(|_| ()).map_err(handlers)
);

checked_value!(
    /// `side`: where a contour runs, as `cam::parse_side` reads it.
    SideParam,
    json_schema!({"description": "Where the tool runs: outside the profile, inside it, or with its centre on it.", "type": "string", "enum": ["outside", "inside", "on"]}),
    |v| match v {
        Value::String(side) => cmd::cam_side("", side).map(|_| ()).map_err(handlers),
        _ => Err("must be outside, inside or on".into()),
    }
);

checked_value!(
    /// `tabs`: `true` for the usual tabs, `false` for none, or the usual ones with some of
    /// their sizes changed, as `cam::tabs_param` reads it.
    TabsParam,
    json_schema!({
        "description": "Bridges left on the last passes so a part cut through stays in the stock: true (8 wide, 3 high, about every 250 mm, holes too), false (none), or the sizes to change. A loop shorter than four tabs gets none.",
        "anyOf": [
            {"type": "boolean"},
            {
                "type": "object",
                "properties": {
                    "width": {"description": "mm along the cut. Default: 8.", "type": "number"},
                    "height": {"description": "mm above the bottom. Default: 3.", "type": "number"},
                    "spacing": {"description": "mm between tabs, about. Default: 250.", "type": "number"},
                    "inner": {"description": "Whether a part's holes get tabs too. Default: true.", "type": "boolean"}
                }
            }
        ]
    }),
    |v| cmd::cam_tabs_param("", &json!({"tabs": v})).map(|_| ()).map_err(handlers)
);

checked_value!(
    /// `outer`: the outline of one region, 3 to 10000 points, as `params::vec2_list` reads it.
    OuterParam,
    json_schema!({
        "description": "The outline of one region: its corners in work coordinates (mm), in order. For several regions give `regions`.",
        "type": "array",
        "items": point(),
        "minItems": 3,
        "maxItems": solidcraft_doc::cam::MAX_LOOP_POINTS
    }),
    |v| params::vec2_list("", &json!({"outer": v}), "outer", solidcraft_doc::cam::MAX_LOOP_POINTS).map(|_| ()).map_err(handlers)
);

checked_value!(
    /// `holes`: the openings inside `outer`, as `cam::outline_holes` reads them.
    HolesParam,
    json_schema!({
        "description": "The openings inside `outer`: a list of outlines, each 3 to 10000 points.",
        "type": "array",
        "items": {"type": "array", "items": point(), "minItems": 3, "maxItems": solidcraft_doc::cam::MAX_LOOP_POINTS}
    }),
    |v| cmd::cam_outline_holes("", v).map(|_| ()).map_err(handlers)
);

checked_value!(
    /// One of `regions`: `{outer, holes?}`, as `cam::polygon_region` reads and checks it.
    RegionParam,
    json_schema!({
        "description": "A region: its outline and the openings inside it, as corner points in work coordinates (mm).",
        "type": "object",
        "properties": {
            "outer": {"type": "array", "items": point(), "minItems": 3},
            "holes": {"type": "array", "items": {"type": "array", "items": point(), "minItems": 3}}
        },
        "required": ["outer"]
    }),
    |v| cmd::cam_polygon_region("", v).map(|_| ()).map_err(handlers)
);

checked_value!(
    /// `rotate`: a quarter turn of the sketch's geometry about its origin.
    RotateParam,
    json_schema!({"description": "Degrees the sketch's geometry is turned about the sketch origin, counter-clockwise.", "type": "number", "enum": [0, 90, 180, 270]}),
    |v| match v.as_f64() {
        Some(deg) if [0.0, 90.0, 180.0, 270.0].contains(&deg) => Ok(()),
        _ => Err("must be 0, 90, 180 or 270".into()),
    }
);

checked_value!(
    /// `curves`: `"all"`, or the ids of the sketch curves to engrave (`params::string_list`).
    CurvesParam,
    json_schema!({
        "description": "The sketch curves to engrave: \"all\", or a list of curve ids. Default: all drawn curves (lines, arcs, circles, splines and text outlines; not construction, centreline or projected geometry).",
        "anyOf": [{"type": "string", "enum": ["all"]}, {"type": "array", "items": {"type": "string"}}]
    }),
    |v| match v {
        Value::String(all) if all == "all" => Ok(()),
        Value::Array(_) => params::string_list("", &json!({"curves": v}), "curves").map(|_| ()).map_err(handlers),
        _ => Err(whole("`curves` must be \"all\" or a list of sketch curve ids")),
    }
);

checked_value!(
    /// One of `paths`: a curve as its points, as `cam::path_chain` reads and checks it.
    PathParam,
    json_schema!({
        "description": "A curve as its points in work coordinates (mm); one whose last point is its first is closed.",
        "type": "array",
        "items": point(),
        "minItems": 2
    }),
    |v| cmd::cam_path_chain("", v).map(|_| ()).map_err(handlers)
);

checked_value!(
    /// One of a drilling's `points`: `[x, y]`, or `[x, y, z]` with the hole's own bottom, as
    /// `cam::drill_point` reads it.
    DrillPointParam,
    json_schema!({"description": "A hole: [x, y] in work coordinates (mm), or [x, y, z] with that hole's bottom (work Z).", "type": "array", "items": {"type": "number"}, "minItems": 2, "maxItems": 3}),
    |v| cmd::cam_drill_point("", v).map(|_| ()).map_err(handlers)
);

checked_value!(
    /// `max_depth`: how deep a carving may go, above 0.
    MaxDepth,
    json_schema!({"description": "mm below the top, above 0.", "type": "number", "exclusiveMinimum": 0}),
    |v| match v.as_f64() {
        Some(depth) if depth.is_finite() && depth > 0.0 => Ok(()),
        _ => Err("must be a positive number (mm)".into()),
    }
);

checked_value!(
    /// `cut`: an operation's heights, feeds and depth per pass as saved, read as the handler
    /// reads it (`solidcraft_cam::Cut`).
    CutParam,
    json_schema!({
        "description": "The heights, feeds and depth per pass as one saved value, as cam.list shows an operation's `cut`. Given, it replaces `top`, `bottom`, `retract`, `clearance`, `stepdown`, `feed` and `plunge_feed`.",
        "type": "object",
        "properties": {
            "heights": {
                "type": "object",
                "properties": {"clearance": {"type": "number"}, "retract": {"type": "number"}, "top": {"type": "number"}, "bottom": {"type": "number"}},
                "required": ["clearance", "retract", "top", "bottom"]
            },
            "feeds": {"type": "object", "properties": {"cut": {"type": "number"}, "plunge": {"type": "number"}}, "required": ["cut", "plunge"]},
            "stepdown": {"type": "number"}
        },
        "required": ["heights", "feeds", "stepdown"]
    }),
    |v| serde_json::from_value::<Cut>(v.clone()).map(|_| ()).map_err(|e| whole(format!("`cut` is invalid: {e}")))
);

checked_value!(
    /// `post`: a post processor as data, read as the handler reads it (`solidcraft_cam::Post`).
    PostData,
    json_schema!({
        "description": "A post processor as data: `name`, `extension` (without the dot), `units` (mm or inch), `decimals`, `feed_decimals`, `comments` (parentheses, semicolon or none), the line lists `header`, `tool_change`, `spindle_on`, `spindle_off`, `coolant_on`, `coolant_off` and `footer`, `tool_change_first`, `arcs` and `arc_tolerance`; optionally `line_numbers` {start, step}, `helical_arcs`, `trailing_decimal`, `two_digit_codes`, `dwell` and `max_arc_sweep`. A built-in post's name is refused.",
        "type": "object"
    }),
    |v| match v {
        Value::Object(_) => serde_json::from_value::<Post>(v.clone()).map(|_| ()).map_err(keyed),
        _ => Err(whole("`post` is required: the post as data")),
    }
);

/// `stock` of a setup: a box, or `null` for none; the handler tells the two apart, so `null`
/// isn't read as left out.
fn stock_or_none<'de, D: Deserializer<'de>>(d: D) -> Result<Option<Option<StockParam>>, D::Error> {
    Option::<StockParam>::deserialize(d).map(Some)
}

fn stock_or_none_schema(generator: &mut SchemaGenerator) -> Schema {
    let stock = generator.subschema_for::<StockParam>();
    json_schema!({
        "description": "The stock: a box in work coordinates (mm), whose top the operations' clearance and retract heights must be above. `null` removes it. Left out, the setup keeps what it has.",
        "anyOf": [stock, {"type": "null"}]
    })
}

/// Add a cutting tool to the design, replace the one of the same number, or delete it.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamToolParams {
    /// The tool's number, which operations and the program call it by.
    pub number: ToolNumber,
    /// The tool's shape. Default: flat_endmill.
    #[serde(default)]
    pub kind: Option<ToolKindParam>,
    /// mm, the widest the cutting edges get. Required unless `delete` is true.
    #[serde(default)]
    pub diameter: Option<f64>,
    /// mm of cutting edge along the tool. Required unless `delete` is true.
    #[serde(default)]
    pub flute_length: Option<f64>,
    /// A V-bit's included tip angle, in degrees. Required for a vbit.
    #[serde(default)]
    pub tip_angle: Option<f64>,
    /// A tapered ball end mill's taper per side, in degrees. Required for a tapered_ball.
    #[serde(default)]
    pub taper_angle: Option<f64>,
    /// mm: a V-bit's flat at the tip (default 0, a sharp point), or a tapered ball end mill's
    /// ball, which it needs.
    #[serde(default)]
    pub tip_diameter: Option<f64>,
    /// What the tool is, for the program's comments; at most 256 characters are kept.
    #[serde(default)]
    pub description: Option<String>,
    /// How many flutes: a whole number from 1 to 100. Default: 2.
    #[serde(default)]
    pub flutes: Option<f64>,
    /// The spindle speed operations use unless they give their own, in rpm.
    #[serde(default)]
    pub spindle_rpm: Option<f64>,
    /// The feed along the cut operations use unless they give their own, in mm/min.
    #[serde(default)]
    pub feed: Option<f64>,
    /// The feed down into the material operations use unless they give their own, in mm/min.
    #[serde(default)]
    pub plunge_feed: Option<f64>,
    /// The depth per pass operations use unless they give their own, in mm.
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// The distance between passes a pocket uses unless it gives its own, in mm.
    #[serde(default)]
    pub stepover: Option<f64>,
    /// true deletes the tool `number`; refused while an operation uses it. Default: false.
    #[serde(default)]
    pub delete: Option<bool>,
}

/// Add a setup (the part in its stock, with a work offset), change one, or delete it with its
/// operations. An update changes only what it is given.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamSetupParams {
    /// The setup's name, at most 128 characters.
    pub name: String,
    /// The work offset. Default for a new setup: 1 (G54).
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "WorkOffset")]
    pub work_offset: Option<WorkOffset>,
    #[serde(default, deserialize_with = "stock_or_none")]
    #[schemars(schema_with = "stock_or_none_schema")]
    pub stock: Option<Option<StockParam>>,
    /// true deletes the setup `name` and its operations. Default: false.
    #[serde(default)]
    pub delete: Option<bool>,
}

/// Cut along the outline of regions: a part cut free from the outside, an opening cut from the
/// inside, or the tool centre on the line. The regions come from `outer` (with `holes`), from
/// `regions`, or from a sketch's profiles. An operation of the same name in the setup is replaced.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamContourParams {
    /// The setup the operation belongs to (cam.setup).
    pub setup: String,
    /// The operation's name, at most 128 characters.
    pub name: String,
    /// The tool to cut with.
    pub tool: OperationTool,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "OuterParam")]
    pub outer: Option<OuterParam>,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "HolesParam")]
    pub holes: Option<HolesParam>,
    /// Several regions, instead of `outer` and `holes`.
    #[serde(default)]
    pub regions: Option<Vec<RegionParam>>,
    /// A sketch whose profiles are the regions, copied as polygons within 0.005 mm; instead of
    /// `outer` and `regions`.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "IdOrName")]
    pub sketch: Option<IdOrName>,
    /// Which profiles of `sketch`. Default: every profile that is not the inside of another
    /// one's hole.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "ProfilesParam")]
    pub profiles: Option<ProfilesParam>,
    /// With `sketch`. Default: 0.
    #[serde(default)]
    pub rotate: Option<RotateParam>,
    /// With `sketch`: where the lower left corner of the profiles' bounds goes. Default: where
    /// they are.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "Point2")]
    pub at: Option<Point2>,
    /// Default: outside, where a part with holes is cut free, every hole before the outline
    /// round it.
    #[serde(default)]
    pub side: Option<SideParam>,
    /// Climb milling (true) or conventional. Default: true.
    #[serde(default)]
    pub climb: Option<bool>,
    /// mm left on the wall for a later pass. Default: 0.
    #[serde(default)]
    pub stock_to_leave: Option<f64>,
    /// mm of chord error allowed. Default: 0.01.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Degrees the tool ramps down along the cut at; 0 plunges. Default: 0.
    #[serde(default)]
    pub ramp_angle: Option<f64>,
    #[serde(default)]
    pub tabs: Option<TabsParam>,
    /// Notch inside corners, so a square part fits. Default: false.
    #[serde(default)]
    pub dogbones: Option<bool>,
    /// An earlier contour of this setup on the same side, cut with a larger tool: this one
    /// then cuts only what that tool left.
    #[serde(default)]
    pub rest_from: Option<String>,
    /// Work Z the tool moves between cuts at. Default: 10 above `top`.
    #[serde(default)]
    pub clearance: Option<f64>,
    /// Work Z the tool lifts to within the operation. Default: 2 above `top`.
    #[serde(default)]
    pub retract: Option<f64>,
    /// Work Z where the material starts. Default: the top of the setup's stock, else 0.
    #[serde(default)]
    pub top: Option<f64>,
    /// Work Z the cut ends at. Required unless `cut` is given.
    #[serde(default)]
    pub bottom: Option<f64>,
    /// mm per depth pass. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// mm/min along the cut. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub feed: Option<f64>,
    /// mm/min down into the material. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub plunge_feed: Option<f64>,
    /// The spindle speed in rpm. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub spindle_rpm: Option<f64>,
    /// Coolant on during the operation. Default: false.
    #[serde(default)]
    pub coolant: Option<bool>,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "CutParam")]
    pub cut: Option<CutParam>,
}

/// Clear the inside of regions down to a flat floor. The regions come from `outer` (with
/// `holes`), from `regions`, or from a sketch's profiles. The tool needs a flat at its tip. An
/// operation of the same name in the setup is replaced.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamPocketParams {
    /// The setup the operation belongs to (cam.setup).
    pub setup: String,
    /// The operation's name, at most 128 characters.
    pub name: String,
    /// The tool to cut with.
    pub tool: OperationTool,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "OuterParam")]
    pub outer: Option<OuterParam>,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "HolesParam")]
    pub holes: Option<HolesParam>,
    /// Several regions, instead of `outer` and `holes`.
    #[serde(default)]
    pub regions: Option<Vec<RegionParam>>,
    /// A sketch whose profiles are the regions, copied as polygons within 0.005 mm; instead of
    /// `outer` and `regions`.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "IdOrName")]
    pub sketch: Option<IdOrName>,
    /// Which profiles of `sketch`. Default: every profile that is not the inside of another
    /// one's hole.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "ProfilesParam")]
    pub profiles: Option<ProfilesParam>,
    /// With `sketch`. Default: 0.
    #[serde(default)]
    pub rotate: Option<RotateParam>,
    /// With `sketch`: where the lower left corner of the profiles' bounds goes. Default: where
    /// they are.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "Point2")]
    pub at: Option<Point2>,
    /// mm between passes. Default: the tool's, else half the diameter of its tip flat.
    #[serde(default)]
    pub stepover: Option<f64>,
    /// Climb milling (true) or conventional. Default: true.
    #[serde(default)]
    pub climb: Option<bool>,
    /// mm left on the wall for a later pass. Default: 0.
    #[serde(default)]
    pub stock_to_leave: Option<f64>,
    /// mm of chord error allowed. Default: 0.01.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Degrees the tool ramps down at; 0 plunges. Default: 0.
    #[serde(default)]
    pub ramp_angle: Option<f64>,
    /// Notch the pocket's inside corners, so a square part fits. Default: false.
    #[serde(default)]
    pub dogbones: Option<bool>,
    /// An earlier pocket of this setup cleared with a larger tool: this one then cuts only its
    /// wall ring's corners and what that tool did not fit into (no dogbones or ramps).
    #[serde(default)]
    pub rest_from: Option<String>,
    /// Work Z the tool moves between cuts at. Default: 10 above `top`.
    #[serde(default)]
    pub clearance: Option<f64>,
    /// Work Z the tool lifts to within the operation. Default: 2 above `top`.
    #[serde(default)]
    pub retract: Option<f64>,
    /// Work Z where the material starts. Default: the top of the setup's stock, else 0.
    #[serde(default)]
    pub top: Option<f64>,
    /// Work Z of the pocket's floor. Required unless `cut` is given.
    #[serde(default)]
    pub bottom: Option<f64>,
    /// mm per depth pass. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// mm/min along the cut. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub feed: Option<f64>,
    /// mm/min down into the material. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub plunge_feed: Option<f64>,
    /// The spindle speed in rpm. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub spindle_rpm: Option<f64>,
    /// Coolant on during the operation. Default: false.
    #[serde(default)]
    pub coolant: Option<bool>,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "CutParam")]
    pub cut: Option<CutParam>,
}

/// Engrave curves: the tool centre follows each curve, open or closed, at every depth pass (an
/// open one back along itself); nothing is offset. The curves are a sketch's, or `paths`. An
/// operation of the same name in the setup is replaced.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamEngraveParams {
    /// The setup the operation belongs to (cam.setup).
    pub setup: String,
    /// The operation's name, at most 128 characters.
    pub name: String,
    /// The tool to cut with.
    pub tool: OperationTool,
    /// A sketch whose curves are engraved, copied as polylines within 0.005 mm; instead of `paths`.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "IdOrName")]
    pub sketch: Option<IdOrName>,
    #[serde(default)]
    pub curves: Option<CurvesParam>,
    /// With `sketch`. Default: 0.
    #[serde(default)]
    pub rotate: Option<RotateParam>,
    /// With `sketch`: where the lower left corner of the curves' bounds goes. Default: where
    /// they are.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "Point2")]
    pub at: Option<Point2>,
    /// The curves as point lists, instead of `sketch`.
    #[serde(default)]
    pub paths: Option<Vec<PathParam>>,
    /// mm: ends closer than this are one curve. Default: 0.01.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Work Z the tool moves between cuts at. Default: 10 above `top`.
    #[serde(default)]
    pub clearance: Option<f64>,
    /// Work Z the tool lifts to within the operation. Default: 2 above `top`.
    #[serde(default)]
    pub retract: Option<f64>,
    /// Work Z where the material starts. Default: the top of the setup's stock, else 0.
    #[serde(default)]
    pub top: Option<f64>,
    /// Work Z the engraving ends at. Required unless `cut` is given.
    #[serde(default)]
    pub bottom: Option<f64>,
    /// mm per depth pass. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// mm/min along the cut. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub feed: Option<f64>,
    /// mm/min down into the material. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub plunge_feed: Option<f64>,
    /// The spindle speed in rpm. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub spindle_rpm: Option<f64>,
    /// Coolant on during the operation. Default: false.
    #[serde(default)]
    pub coolant: Option<bool>,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "CutParam")]
    pub cut: Option<CutParam>,
}

/// Carve regions with a V-bit: it runs the inward offsets of the regions, each as deep as its
/// cone touches the wall there, so edges are sharp at the top and a ridge stands where the
/// offsets meet. The bottom follows from the carving. An operation of the same name in the setup
/// is replaced.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamVCarveParams {
    /// The setup the operation belongs to (cam.setup).
    pub setup: String,
    /// The operation's name, at most 128 characters.
    pub name: String,
    /// The tool to cut with: a vbit.
    pub tool: OperationTool,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "OuterParam")]
    pub outer: Option<OuterParam>,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "HolesParam")]
    pub holes: Option<HolesParam>,
    /// Several regions, instead of `outer` and `holes`.
    #[serde(default)]
    pub regions: Option<Vec<RegionParam>>,
    /// A sketch whose profiles are the regions, copied as polygons within 0.005 mm; instead of
    /// `outer` and `regions`.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "IdOrName")]
    pub sketch: Option<IdOrName>,
    /// Which profiles of `sketch`. Default: every profile that is not the inside of another
    /// one's hole.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "ProfilesParam")]
    pub profiles: Option<ProfilesParam>,
    /// With `sketch`. Default: 0.
    #[serde(default)]
    pub rotate: Option<RotateParam>,
    /// With `sketch`: where the lower left corner of the profiles' bounds goes. Default: where
    /// they are.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "Point2")]
    pub at: Option<Point2>,
    /// The deepest the carving goes: inner passes stay there and the tip clears a flat floor.
    /// Required when the carving would go deeper than the tool's flutes.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "MaxDepth")]
    pub max_depth: Option<MaxDepth>,
    /// mm between passes. Default: 0.1.
    #[serde(default)]
    pub stepover: Option<f64>,
    /// mm of chord error allowed. Default: 0.01.
    #[serde(default)]
    pub tolerance: Option<f64>,
    /// Work Z the tool moves between cuts at. Default: 10 above `top`.
    #[serde(default)]
    pub clearance: Option<f64>,
    /// Work Z the tool lifts to within the operation. Default: 2 above `top`.
    #[serde(default)]
    pub retract: Option<f64>,
    /// Work Z where the material starts. Default: the top of the setup's stock, else 0.
    #[serde(default)]
    pub top: Option<f64>,
    /// mm per depth pass, saved with the operation; the carving itself goes to each ring's
    /// depth at once. Default: the tool's, else the whole depth.
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// mm/min along the cut. Default: the tool's, else the plunge feed.
    #[serde(default)]
    pub feed: Option<f64>,
    /// mm/min down into the material. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub plunge_feed: Option<f64>,
    /// The spindle speed in rpm. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub spindle_rpm: Option<f64>,
    /// Coolant on during the operation. Default: false.
    #[serde(default)]
    pub coolant: Option<bool>,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "CutParam")]
    pub cut: Option<CutParam>,
}

/// Drill holes at points, or at a sketch's lone points and circle centres. Any kind of tool
/// drills; a V-bit with a depth per point makes halftone dots. An operation of the same name in
/// the setup is replaced.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamDrillParams {
    /// The setup the operation belongs to (cam.setup).
    pub setup: String,
    /// The operation's name, at most 128 characters.
    pub name: String,
    /// The tool to drill with.
    pub tool: OperationTool,
    /// Where to drill, at most 100000 holes; instead of `sketch`.
    #[serde(default)]
    pub points: Option<Vec<DrillPointParam>>,
    /// A sketch whose lone points and circle centres are the holes; instead of `points`.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "IdOrName")]
    pub sketch: Option<IdOrName>,
    /// With `sketch`. Default: 0.
    #[serde(default)]
    pub rotate: Option<RotateParam>,
    /// With `sketch`: where the lower left corner of the holes' bounds goes. Default: where
    /// they are.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "Point2")]
    pub at: Option<Point2>,
    /// Work Z of the holes' bottom, unless a point has its own z. Not with `depth`.
    #[serde(default)]
    pub bottom: Option<f64>,
    /// mm below `top` of the holes' bottom, unless a point has its own z. Not with `bottom`.
    #[serde(default)]
    pub depth: Option<f64>,
    /// Work Z where the material starts. Default: the top of the setup's stock, else 0.
    #[serde(default)]
    pub top: Option<f64>,
    /// Work Z the tool lifts to between pecks and holes. Default: 2 above `top`.
    #[serde(default)]
    pub retract: Option<f64>,
    /// Work Z the tool moves between holes at. Default: 10 above `top`.
    #[serde(default)]
    pub clearance: Option<f64>,
    /// mm per peck, with a lift to the retract height between pecks. Default: one plunge.
    #[serde(default)]
    pub peck: Option<f64>,
    /// Seconds the tool stays at the bottom. Default: 0.
    #[serde(default)]
    pub dwell: Option<f64>,
    /// mm/min down into the material. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub plunge_feed: Option<f64>,
    /// Saved with the operation; drilling only plunges. Default: the tool's, else the plunge feed.
    #[serde(default)]
    pub feed: Option<f64>,
    /// Saved with the operation; drilling pecks by `peck`. Default: the tool's, else the whole depth.
    #[serde(default)]
    pub stepdown: Option<f64>,
    /// The spindle speed in rpm. Default: the tool's; one of the two is needed.
    #[serde(default)]
    pub spindle_rpm: Option<f64>,
    /// Coolant on during the operation. Default: false.
    #[serde(default)]
    pub coolant: Option<bool>,
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "CutParam")]
    pub cut: Option<CutParam>,
}

/// Delete an operation from a setup.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamDeleteOpParams {
    /// The setup the operation belongs to.
    pub setup: String,
    /// The operation to delete.
    pub name: String,
}

/// Write a setup's operations as a G-code program, in order, and read it back against the
/// toolpaths.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamPostParams {
    /// The setup to post.
    pub setup: String,
    /// The post processor: a built-in one or one saved on the design (cam.post.define).
    /// Default: rs274.
    #[serde(default)]
    pub post: Option<String>,
    /// The program's name. Default: the setup's.
    #[serde(default)]
    pub program: Option<String>,
    /// Read the G-code back and refuse it when it is further from the toolpaths than the post's
    /// rounding allows. Default: true.
    #[serde(default)]
    pub verify: Option<bool>,
    /// The file to write the program to; the reply then has `path` instead of `gcode`.
    #[serde(default)]
    pub path: Option<String>,
}

/// Cut a setup's operations out of its stock as a height map and write it as a picture: a
/// 16-bit grey PNG, the stock top white, the deepest cut black, +Y up.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamSimulateParams {
    /// The setup to simulate.
    pub setup: String,
    /// The operations to cut, in this order. Default: all of the setup's, in order.
    #[serde(default)]
    pub operations: Option<StringList>,
    /// mm per pixel, above 0. Default: 0.2.
    #[serde(default)]
    pub resolution: Option<f64>,
    /// The stock to cut. Default: the setup's; required when it has none.
    #[serde(default, deserialize_with = "present", skip_serializing_if = "Option::is_none")]
    #[schemars(with = "StockParam")]
    pub stock: Option<StockParam>,
    /// The PNG to write.
    pub path: String,
}

/// Save a post processor on the design, replacing one of the same name, or delete a saved one.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamPostDefineParams {
    /// The post to save. Required unless `delete` is true.
    #[serde(default)]
    pub post: Option<PostData>,
    /// With `delete`: the saved post to delete.
    #[serde(default)]
    pub name: Option<String>,
    /// true deletes the saved post `name`. Default: false.
    #[serde(default)]
    pub delete: Option<bool>,
}

/// List the design's tools, its setups with their operations, and the posts.
#[derive(Debug, Deserialize, JsonSchema)]
#[schemars(deny_unknown_fields)]
pub struct CamListParams {}

/// A tool as the design saves it.
#[derive(Debug, Serialize, JsonSchema)]
pub struct ToolData {
    /// The tool's number.
    pub number: u32,
    /// flat_endmill, ball_endmill, drill, vbit or tapered_ball.
    pub kind: String,
    /// mm, the widest the cutting edges get.
    pub diameter: f64,
    /// mm of cutting edge along the tool.
    pub flute_length: f64,
    /// What the tool is.
    pub description: String,
    /// How many flutes.
    pub flutes: u32,
    /// The tool's spindle speed, in rpm, when it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<f64>,
    /// The tool's feed along the cut, in mm/min, when it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed: Option<f64>,
    /// The tool's feed down, in mm/min, when it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plunge_feed: Option<f64>,
    /// The tool's depth per pass, in mm, when it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepdown: Option<f64>,
    /// The tool's distance between passes, in mm, when it has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepover: Option<f64>,
    /// A V-bit's included tip angle, in degrees.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_angle: Option<f64>,
    /// A tapered ball end mill's taper per side, in degrees.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub taper_angle: Option<f64>,
    /// mm: the flat or the ball at the tip, when the tool has one.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_diameter: Option<f64>,
}

/// What `cam.tool` returns: the tool as saved, or the number of the one deleted.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamToolReply {
    /// The tool as saved.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tool: Option<ToolData>,
    /// The number of the tool that was deleted.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deleted: Option<u32>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// A box in work coordinates, in mm.
#[derive(Debug, Serialize, JsonSchema)]
pub struct StockData {
    /// The corner with the lowest coordinates.
    pub min: [f64; 3],
    /// The corner with the highest coordinates.
    pub max: [f64; 3],
}

/// An operation's heights, in work Z.
#[derive(Debug, Serialize, JsonSchema)]
pub struct HeightsData {
    /// Where the tool moves between cuts.
    pub clearance: f64,
    /// Where the tool lifts to within the operation.
    pub retract: f64,
    /// Where the material starts.
    pub top: f64,
    /// Where the cut ends.
    pub bottom: f64,
}

/// An operation's feeds, in mm/min.
#[derive(Debug, Serialize, JsonSchema)]
pub struct FeedsData {
    /// Along the cut.
    pub cut: f64,
    /// Down into the material.
    pub plunge: f64,
}

/// An operation's heights, feeds and depth per pass.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CutData {
    pub heights: HeightsData,
    pub feeds: FeedsData,
    /// mm per depth pass.
    pub stepdown: f64,
}

/// An operation of a setup, as `cam.list` and `cam.setup` show it.
#[derive(Debug, Serialize, JsonSchema)]
pub struct OperationData {
    /// contour, pocket, engrave, drill or v_carve.
    #[serde(rename = "type")]
    pub kind: String,
    /// The operation's name.
    pub name: String,
    /// The number of its tool.
    pub tool: u32,
    /// A contour's, pocket's or v-carving's number of regions.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub regions: Option<usize>,
    /// An engraving's number of curves.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub chains: Option<usize>,
    /// A drilling's number of holes.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub holes: Option<usize>,
    /// The operation's own parameters as saved, by its type.
    pub params: Value,
    pub cut: CutData,
    /// Its spindle speed in rpm.
    pub spindle_rpm: Option<f64>,
    /// Whether the coolant is on.
    pub coolant: bool,
    /// A contour's or pocket's earlier operation it rests after, or `null`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest_from: Option<String>,
}

/// A setup with its operations.
#[derive(Debug, Serialize, JsonSchema)]
pub struct SetupData {
    /// The setup's name.
    pub name: String,
    /// 1 to 6, for G54 to G59.
    pub work_offset: u8,
    /// Its stock, or `null`.
    pub stock: Option<StockData>,
    /// Its operations, in the order they are posted.
    pub operations: Vec<OperationData>,
}

/// What `cam.setup` returns: the setup as saved, or the name of the one deleted.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamSetupReply {
    /// The setup as saved.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub setup: Option<SetupData>,
    /// The name of the setup that was deleted.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deleted: Option<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// The bounds of what an operation cuts, in work X and Y (mm).
#[derive(Debug, Serialize, JsonSchema)]
pub struct BoundsData {
    pub min: [f64; 2],
    pub max: [f64; 2],
}

/// An opening the tool does not fit into, which is left uncut.
#[derive(Debug, Serialize, JsonSchema)]
pub struct TooSmall {
    pub min: [f64; 2],
    pub max: [f64; 2],
    /// The diameter of the widest circle that fits into it, in mm.
    pub widest_circle: f64,
}

/// What a rest cut found left by the earlier operation.
#[derive(Debug, Serialize, JsonSchema)]
pub struct RestData {
    /// The earlier operation.
    pub from: String,
    /// mm this operation cuts.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub cut_length: Option<f64>,
    /// true when the earlier tool left nothing: no operation is saved then.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub nothing_left: Option<bool>,
}

/// What `cam.contour` and `cam.pocket` return.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamMillingReply {
    /// The setup.
    pub setup: String,
    /// The operation's name.
    pub operation: String,
    /// What it cuts; left out when a rest cut found nothing left.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub bounds: Option<BoundsData>,
    /// The openings the tool does not fit into.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub too_small: Vec<TooSmall>,
    /// With `rest_from`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rest: Option<RestData>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// What `cam.engrave` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamEngraveReply {
    /// The setup.
    pub setup: String,
    /// The operation's name.
    pub operation: String,
    pub bounds: BoundsData,
    /// How many curves are engraved, pieces whose ends meet counted as one.
    pub chains: usize,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// The floor a flat end mill could clear instead of the V-bit's tip.
#[derive(Debug, Serialize, JsonSchema)]
pub struct FlatClearing {
    /// mm from the regions' walls to where the floor starts: a pocket with `stock_to_leave`
    /// of this less the end mill's radius clears it.
    pub wall_offset: f64,
    /// Work Z of the floor.
    pub bottom: f64,
}

/// What `cam.vcarve` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamVCarveReply {
    /// The setup.
    pub setup: String,
    /// The operation's name.
    pub operation: String,
    pub bounds: BoundsData,
    /// mm below the top the carving goes.
    pub depth: f64,
    /// How many passes inwards.
    pub rings: usize,
    /// With `max_depth`, when the carving stops at a flat floor.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub flat_clearing: Option<FlatClearing>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// What `cam.drill` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamDrillReply {
    /// The setup.
    pub setup: String,
    /// The operation's name.
    pub operation: String,
    pub bounds: BoundsData,
    /// How many holes.
    pub holes: usize,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// What `cam.delete_op` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamDeleteOpReply {
    /// The operation that was deleted.
    pub deleted: String,
    /// Its setup.
    pub setup: String,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// The program read back against the toolpaths it was written from.
#[derive(Debug, Serialize, JsonSchema)]
pub struct ReadbackData {
    /// mm the program's cutting moves are from the toolpaths at most.
    pub max_deviation: f64,
    /// mm the post's rounding and arcs allow.
    pub allowed: f64,
    /// mm of cutting moves in the program.
    pub cut_length: f64,
    /// Minutes the cutting moves take at their feeds.
    pub cut_minutes: f64,
    /// Whether the deviation is within what is allowed.
    pub within: bool,
}

/// What `cam.post` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamPostReply {
    /// How many lines the program has.
    pub lines: usize,
    /// The post's file extension, without the dot.
    pub extension: String,
    /// The post's name.
    pub post: String,
    /// How many operations were written.
    pub operations: usize,
    /// The setup's work offset: 1 to 6, for G54 to G59.
    pub work_offset: u8,
    /// Unless `verify` is false.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub readback: Option<ReadbackData>,
    /// The file the program was written to, with `path`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub path: Option<String>,
    /// The program, without `path`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gcode: Option<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// What one operation removed in the simulation.
#[derive(Debug, Serialize, JsonSchema)]
pub struct SimulatedOperation {
    /// The operation.
    pub name: String,
    /// mm³ it removed.
    pub removed_mm3: f64,
    /// A pocket or v-carving: how much of its regions still stands above its bottom, what the
    /// tool could not reach.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub standing_in_regions_percent: Option<f64>,
}

/// What `cam.simulate` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamSimulateReply {
    /// The PNG that was written.
    pub path: String,
    /// The picture's width and height in pixels.
    pub size: [usize; 2],
    /// mm per pixel.
    pub resolution: f64,
    /// The stock that was cut.
    pub stock: StockData,
    /// Work Z of the deepest cut.
    pub deepest: f64,
    /// mm³ removed in all.
    pub removed_mm3: f64,
    /// The operations, in the order they cut.
    pub operations: Vec<SimulatedOperation>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// What `cam.post.define` returns: the name of the post saved, or of the one deleted.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamPostDefineReply {
    /// The post that was saved.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub post: Option<String>,
    /// The saved post that was deleted.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub deleted: Option<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

/// What `cam.list` returns.
#[derive(Debug, Serialize, JsonSchema)]
pub struct CamListReply {
    /// The design's tools, by number.
    pub tools: Vec<ToolData>,
    /// The design's setups.
    pub setups: Vec<SetupData>,
    /// The names of the posts saved on the design.
    pub posts: Vec<String>,
    /// The names of the built-in posts.
    pub builtin_posts: Vec<String>,
    /// In the warn parameter-check mode: what the call's parameters got wrong.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub param_warnings: Vec<String>,
}

#[cfg(test)]
#[path = "cam_tests.rs"]
mod tests;
