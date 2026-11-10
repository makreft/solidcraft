//! Cutting tools.

use serde::{Deserialize, Serialize};

use crate::{Result, invalid};

/// Largest tool number a post processor is asked to write.
pub const MAX_TOOL_NUMBER: u32 = 9999;
/// Largest tool diameter (mm).
pub const MAX_TOOL_DIAMETER: f64 = 1000.0;

#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ToolKind {
    FlatEndmill,
    BallEndmill,
    Drill,
    /// An engraving cutter: a cone with an included tip angle, perhaps a small flat at the tip.
    #[serde(rename = "vbit", alias = "v_bit")]
    VBit,
    /// A ball end on a tapered shank: the ball at the tip, then a cone widening up to the shank.
    TaperedBall,
}

impl ToolKind {
    pub fn label(self) -> &'static str {
        match self {
            ToolKind::FlatEndmill => "flat end mill",
            ToolKind::BallEndmill => "ball end mill",
            ToolKind::Drill => "drill",
            ToolKind::VBit => "V-bit",
            ToolKind::TaperedBall => "tapered ball end mill",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Tool {
    /// The pocket the machine holds it in (`T` word).
    pub number: u32,
    pub kind: ToolKind,
    /// Cutting diameter (mm): the widest the cutting edges get.
    pub diameter: f64,
    /// Length of the cutting edges (mm).
    pub flute_length: f64,
    #[serde(default)]
    pub description: String,
    /// Number of cutting edges.
    #[serde(default = "two")]
    pub flutes: u32,
    /// The cutting data operations start from: spindle speed (rev/min) and feeds (mm/min).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spindle_rpm: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub feed: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub plunge_feed: Option<f64>,
    /// The depth per pass and the distance between passes operations start from (mm).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepdown: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub stepover: Option<f64>,
    /// The included angle of a V-bit's tip (degrees).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub tip_angle: Option<f64>,
    /// A tapered ball end mill's taper, per side (degrees).
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub taper_angle: Option<f64>,
    /// The flat at a V-bit's tip, or a tapered ball end mill's ball (mm); 0 for a sharp point.
    #[serde(default, skip_serializing_if = "is_zero")]
    pub tip_diameter: f64,
}

fn two() -> u32 {
    2
}

fn is_zero(v: &f64) -> bool {
    *v == 0.0
}

/// How wide a ball of radius `r` is `d` below its tip.
fn ball_width(r: f64, d: f64) -> f64 {
    if d < r { 2.0 * (r * r - (r - d) * (r - d)).max(0.0).sqrt() } else { 2.0 * r }
}

impl Tool {
    pub fn new(number: u32, kind: ToolKind, diameter: f64, flute_length: f64) -> Tool {
        Tool {
            number,
            kind,
            diameter,
            flute_length,
            description: String::new(),
            flutes: 2,
            spindle_rpm: None,
            feed: None,
            plunge_feed: None,
            stepdown: None,
            stepover: None,
            tip_angle: None,
            taper_angle: None,
            tip_diameter: 0.0,
        }
    }
    pub fn radius(&self) -> f64 {
        self.diameter * 0.5
    }

    /// How wide the tool cuts `depth` below its tip (mm): the full diameter for a cylinder,
    /// less for a ball below its equator, for a V-bit and for a tapered ball end mill, whose
    /// cutting edges narrow towards the tip.
    pub fn width_at_depth(&self, depth: f64) -> f64 {
        let d = if depth.is_finite() { depth.max(0.0) } else { 0.0 };
        let width = match self.kind {
            ToolKind::FlatEndmill | ToolKind::Drill => self.diameter,
            ToolKind::BallEndmill => ball_width(self.radius(), d),
            ToolKind::VBit => self.tip_diameter + 2.0 * d * (self.tip_angle.unwrap_or(0.0).to_radians() * 0.5).tan(),
            ToolKind::TaperedBall => {
                let r = self.tip_diameter * 0.5;
                let cone = self.tip_diameter + 2.0 * (d - r).max(0.0) * self.taper_angle.unwrap_or(0.0).to_radians().tan();
                ball_width(r, d).max(cone)
            }
        };
        width.min(self.diameter)
    }

    /// Half the tool's width at the top of a cut `depth` deep: the offset that puts the top edge of
    /// the wall on the profile. A cylinder's radius; less for a tool that narrows to its tip.
    pub fn radius_at(&self, depth: f64) -> f64 {
        self.width_at_depth(depth) * 0.5
    }

    /// How wide the tool clears a floor (mm): its diameter, or the flat (or ball) at the tip of a
    /// tool that narrows towards it; 0 for a sharp point.
    pub fn floor_diameter(&self) -> f64 {
        match self.kind {
            ToolKind::FlatEndmill | ToolKind::BallEndmill | ToolKind::Drill => self.diameter,
            ToolKind::VBit | ToolKind::TaperedBall => self.tip_diameter,
        }
    }

    /// The tool for a program's comments: its kind, and the tip that shapes its cut.
    pub fn describe(&self) -> String {
        match self.kind {
            ToolKind::VBit => {
                let angle = self.tip_angle.unwrap_or(0.0);
                if self.tip_diameter > 0.0 { format!("V-bit {angle}deg tip {}", self.tip_diameter) } else { format!("V-bit {angle}deg") }
            }
            ToolKind::TaperedBall => format!("tapered ball R{} {}deg", self.tip_diameter * 0.5, self.taper_angle.unwrap_or(0.0)),
            kind => kind.label().to_string(),
        }
    }

    pub fn validate(&self) -> Result<()> {
        if self.number > MAX_TOOL_NUMBER {
            return invalid(format!("tool number {} is above {MAX_TOOL_NUMBER}", self.number));
        }
        if !(self.diameter > 0.0) || !(self.diameter <= MAX_TOOL_DIAMETER) {
            return invalid("the tool diameter must be above 0 and at most 1000 mm");
        }
        if !(self.flute_length > 0.0) || !self.flute_length.is_finite() {
            return invalid("the tool's flute length must be above 0");
        }
        if !(1..=100).contains(&self.flutes) {
            return invalid("a tool has 1 to 100 flutes");
        }
        if [self.spindle_rpm, self.feed, self.plunge_feed, self.stepdown, self.stepover].iter().flatten().any(|v| !(*v > 0.0) || !(*v <= 1.0e6)) {
            return invalid("a tool's spindle speed, feeds, stepdown and stepover must be above 0");
        }
        if !self.tip_diameter.is_finite() || self.tip_diameter < 0.0 {
            return invalid("the tip diameter must be 0 or more");
        }
        match self.kind {
            ToolKind::VBit => {
                if !self.tip_angle.is_some_and(|a| a > 0.0 && a < 180.0) {
                    return invalid("a V-bit needs `tip_angle`: its included tip angle, above 0 and below 180 degrees");
                }
                if !(self.tip_diameter < self.diameter) {
                    return invalid("the tip diameter must be 0 or more and below the tool diameter");
                }
            }
            ToolKind::TaperedBall => {
                if !self.taper_angle.is_some_and(|a| a > 0.0 && a < 90.0) || !(self.tip_diameter > 0.0 && self.tip_diameter <= self.diameter) {
                    return invalid(
                        "a tapered ball end mill needs `taper_angle` (its half-angle per side, above 0 and below 90 degrees) and `tip_diameter` (its ball, above 0 and at most the tool diameter)",
                    );
                }
            }
            ToolKind::FlatEndmill | ToolKind::BallEndmill | ToolKind::Drill => {
                if self.tip_angle.is_some() || self.taper_angle.is_some() || self.tip_diameter != 0.0 {
                    return invalid("only a V-bit has a tip angle, only a tapered ball end mill a taper angle and a tip diameter");
                }
            }
        }
        Ok(())
    }
}
