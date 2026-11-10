//! Post processors: how a program is written for one kind of controller. A post is data (it
//! can be read from JSON), not code: line templates around the motion blocks, and a few
//! switches for how the motion itself is written.
//!
//! Templates may use `{program}` (program name), `{units}` (`G21` or `G20`), `{work_offset}`
//! (the setup's work offset, `G54` to `G59`), `{tool}` (tool number), `{diameter}` (tool
//! diameter in output units) and `{rpm}` (spindle speed); the dwell template uses `{seconds}` or
//! `{milliseconds}`.

use serde::{Deserialize, Serialize};

use crate::{CamError, Result, invalid};

/// Most lines in one template block.
const MAX_TEMPLATE_LINES: usize = 64;
/// Longest template line.
const MAX_TEMPLATE_LEN: usize = 200;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Units {
    Mm,
    Inch,
}

impl Units {
    /// Millimetres per output unit.
    pub fn mm(self) -> f64 {
        match self {
            Units::Mm => 1.0,
            Units::Inch => 25.4,
        }
    }
    pub fn code(self) -> &'static str {
        match self {
            Units::Mm => "G21",
            Units::Inch => "G20",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CommentStyle {
    /// `(comment)`
    Parentheses,
    /// `; comment`
    Semicolon,
    /// No comments are written.
    None,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LineNumbers {
    pub start: u32,
    pub step: u32,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Post {
    pub name: String,
    /// File extension without the dot.
    pub extension: String,
    pub units: Units,
    /// Decimals of coordinates (at most 6).
    pub decimals: u8,
    /// Decimals of feed rates (at most 4).
    pub feed_decimals: u8,
    #[serde(default)]
    pub line_numbers: Option<LineNumbers>,
    pub comments: CommentStyle,
    /// Written once at the start.
    pub header: Vec<String>,
    /// Written before the first operation of each tool.
    pub tool_change: Vec<String>,
    /// Whether `tool_change` is also written for the program's first tool.
    pub tool_change_first: bool,
    pub spindle_on: Vec<String>,
    pub spindle_off: Vec<String>,
    pub coolant_on: Vec<String>,
    pub coolant_off: Vec<String>,
    /// Written once at the end.
    pub footer: Vec<String>,
    /// Whether arcs are written as `G2`/`G3` with `I` `J`; otherwise as short lines.
    pub arcs: bool,
    /// Chord error of arcs written as lines (mm).
    pub arc_tolerance: f64,
    /// Whether an arc that also moves in Z (a helix, as ramps make) is written as an arc; otherwise as short lines.
    #[serde(default = "yes")]
    pub helical_arcs: bool,
    /// Whether whole numbers are written with a decimal point (`X10.`), for controllers that
    /// read `X10` as ten of their smallest unit.
    #[serde(default)]
    pub trailing_decimal: bool,
    /// Whether motion codes are written with two digits (`G01` rather than `G1`).
    #[serde(default)]
    pub two_digit_codes: bool,
    /// How a dwell is written: `{seconds}` or `{milliseconds}` (a whole number) stands for its length.
    #[serde(default = "default_dwell")]
    pub dwell: String,
    /// Largest sweep of one arc block (degrees), for controllers that limit it (to a half turn
    /// or a quadrant); a longer arc is written as equal parts. `None`: up to a full turn per block.
    #[serde(default)]
    pub max_arc_sweep: Option<f64>,
    /// The spindle speeds the machine runs; a program asking for another is refused.
    #[serde(default)]
    pub spindle_range: Option<SpindleRange>,
    /// Whether a program may use one tool only, for machines that cannot change tools within a
    /// program: one with more tools is refused rather than written.
    #[serde(default)]
    pub one_tool: bool,
    /// How far the machine travels on each axis (mm): a program that spans more on any axis is
    /// refused, since no work zero brings all of it within reach.
    #[serde(default)]
    pub travel: Option<Travel>,
}

/// The travel of a machine's axes (mm).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Travel {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// The spindle speeds a machine runs (rev/min).
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpindleRange {
    pub min: f64,
    pub max: f64,
}

impl SpindleRange {
    pub fn contains(&self, rpm: f64) -> bool {
        rpm >= self.min && rpm <= self.max
    }
}

fn yes() -> bool {
    true
}

fn default_dwell() -> String {
    "G4 P{seconds}".into()
}

fn lines(l: &[&str]) -> Vec<String> {
    l.iter().map(|s| s.to_string()).collect()
}

impl Post {
    /// Controllers that follow the NIST RS-274/NGC language closely (LinuxCNC and relatives):
    /// automatic tool change with tool length offsets.
    pub fn rs274() -> Post {
        Post {
            name: "rs274".into(),
            extension: "ngc".into(),
            units: Units::Mm,
            decimals: 4,
            feed_decimals: 1,
            line_numbers: None,
            comments: CommentStyle::Parentheses,
            header: lines(&["G17 {units} G40 G49 {work_offset} G80 G90 G94"]),
            tool_change: lines(&["M5", "T{tool} M6", "G43 H{tool}"]),
            tool_change_first: true,
            spindle_on: lines(&["S{rpm} M3"]),
            spindle_off: lines(&["M5"]),
            coolant_on: lines(&["M8"]),
            coolant_off: lines(&["M9"]),
            footer: lines(&["M30"]),
            arcs: true,
            arc_tolerance: 0.005,
            helical_arcs: true,
            trailing_decimal: false,
            two_digit_codes: false,
            dwell: default_dwell(),
            max_arc_sweep: None,
            spindle_range: None,
            one_tool: false,
            travel: None,
        }
    }

    /// The built-in post called `name`.
    pub fn builtin(name: &str) -> Option<Post> {
        match name.trim().to_ascii_lowercase().as_str() {
            "rs274" | "linuxcnc" => Some(Post::rs274()),
            _ => None,
        }
    }

    pub const BUILTIN: &'static [&'static str] = &["rs274"];

    /// Whether the post writes the work offset (a `{work_offset}` in its header or tool change).
    pub fn writes_work_offset(&self) -> bool {
        self.header.iter().chain(&self.tool_change).any(|l| l.contains("{work_offset}"))
    }

    pub fn validate(&self) -> Result<()> {
        if self.decimals > 6 || self.feed_decimals > 4 {
            return invalid("a post writes at most 6 decimals for coordinates and 4 for feeds");
        }
        if !(self.arc_tolerance > 0.0) || !self.arc_tolerance.is_finite() {
            return invalid("the post's arc tolerance must be above 0");
        }
        if self.line_numbers.is_some_and(|n| n.step == 0) {
            return invalid("the post's line number step must be above 0");
        }
        if self.max_arc_sweep.is_some_and(|deg| !(1.0..360.0).contains(&deg)) {
            return invalid("the post's largest arc sweep must be at least 1° and below 360°");
        }
        if self.spindle_range.is_some_and(|r| !(r.min >= 0.0 && r.min <= r.max && r.max <= 1.0e6)) {
            return invalid("the post's spindle range must run from its lowest to its highest speed, 0 to 1000000 rev/min");
        }
        if self.travel.is_some_and(|t| [t.x, t.y, t.z].iter().any(|v| !(*v > 0.0 && *v <= 1.0e5))) {
            return invalid("the post's travel must be above 0 and at most 100 m on every axis");
        }
        if self.extension.is_empty() || self.extension.len() > 8 || !self.extension.chars().all(|c| c.is_ascii_alphanumeric()) {
            return invalid("the post's file extension must be 1 to 8 letters or digits");
        }
        if self.dwell.len() > MAX_TEMPLATE_LEN || self.dwell.chars().any(char::is_control) {
            return invalid("the post's dwell template is too long or has control characters");
        }
        fill(&self.dwell, &Values::default())?;
        for block in [&self.header, &self.tool_change, &self.spindle_on, &self.spindle_off, &self.coolant_on, &self.coolant_off, &self.footer] {
            if block.len() > MAX_TEMPLATE_LINES {
                return Err(CamError::TooLarge(format!("a post template has more than {MAX_TEMPLATE_LINES} lines")));
            }
            for line in block {
                if line.len() > MAX_TEMPLATE_LEN || line.chars().any(char::is_control) {
                    return invalid("a post template line is too long or has control characters");
                }
                // Catches unknown placeholders before any program is written.
                fill(line, &Values::default())?;
            }
        }
        Ok(())
    }
}

/// What the placeholders stand for.
#[derive(Clone, Debug, Default)]
pub(crate) struct Values {
    pub program: String,
    pub units: String,
    pub work_offset: String,
    pub tool: String,
    pub diameter: String,
    pub rpm: String,
    pub seconds: String,
    pub milliseconds: String,
}

/// `line` with its placeholders replaced.
pub(crate) fn fill(line: &str, v: &Values) -> Result<String> {
    let mut out = String::with_capacity(line.len());
    let mut rest = line;
    while let Some(open) = rest.find('{') {
        let (before, after) = rest.split_at(open);
        out.push_str(before);
        let Some(close) = after.find('}') else { return invalid(format!("post template `{line}`: `{{` without `}}`")) };
        let (name, tail) = after.split_at(close);
        out.push_str(match name.strip_prefix('{').unwrap_or(name) {
            "program" => &v.program,
            "units" => &v.units,
            "work_offset" => &v.work_offset,
            "tool" => &v.tool,
            "diameter" => &v.diameter,
            "rpm" => &v.rpm,
            "seconds" => &v.seconds,
            "milliseconds" => &v.milliseconds,
            other => return invalid(format!("post template `{line}`: unknown placeholder `{{{other}}}`")),
        });
        rest = tail.strip_prefix('}').unwrap_or(tail);
    }
    out.push_str(rest);
    Ok(out)
}
