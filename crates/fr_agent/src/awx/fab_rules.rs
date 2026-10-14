//! AWX/JLC floor ladder and via-in-pad clamp from the fixed caad45cd source (MIT).
use super::escape_moves::{rounded, Pad, Point};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum FabTier {
    Standard,
    Advanced,
    Auto,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EscalationPolicy {
    Off,
    Board,
    Fab,
}
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct FabFloor {
    pub clearance: f64,
    pub track_width: f64,
    pub via_diameter: f64,
    pub via_drill: f64,
    pub hole_to_hole: f64,
    pub pad_hole_to_hole: f64,
    pub annular: f64,
    pub board_edge: f64,
}
impl FabFloor {
    fn set(&mut self, key: &str, value: f64) {
        match key {
            "clearance" => self.clearance = value,
            "track_width" => self.track_width = value,
            "via_diameter" => self.via_diameter = value,
            "via_drill" => self.via_drill = value,
            "hole_to_hole" => self.hole_to_hole = value,
            "pad_hole_to_hole" => self.pad_hole_to_hole = value,
            "annular" => self.annular = value,
            "board_edge" => self.board_edge = value,
            _ => {}
        }
    }
    fn get(&self, key: &str) -> Option<f64> {
        match key {
            "clearance" => Some(self.clearance),
            "track_width" => Some(self.track_width),
            "via_diameter" => Some(self.via_diameter),
            "via_drill" => Some(self.via_drill),
            "hole_to_hole" => Some(self.hole_to_hole),
            "pad_hole_to_hole" => Some(self.pad_hole_to_hole),
            "annular" => Some(self.annular),
            "board_edge" => Some(self.board_edge),
            _ => None,
        }
    }
}
fn floors(layers: usize, advanced: bool) -> FabFloor {
    let multi = layers > 2;
    FabFloor {
        clearance: if advanced {
            if multi {
                0.09
            } else {
                0.10
            }
        } else if multi {
            0.10
        } else {
            0.127
        },
        track_width: if advanced {
            if multi {
                0.0762
            } else {
                0.10
            }
        } else if multi {
            0.0889
        } else {
            0.127
        },
        via_diameter: if advanced { 0.25 } else { 0.45 },
        via_drill: if advanced { 0.15 } else { 0.20 },
        hole_to_hole: 0.20,
        pad_hole_to_hole: 0.45,
        annular: if advanced {
            if multi {
                0.15
            } else {
                0.18
            }
        } else if multi {
            0.20
        } else {
            0.25
        },
        board_edge: 0.20,
    }
}
fn apply_floors(rungs: Vec<FabFloor>, minimums: &BTreeMap<String, f64>) -> Vec<FabFloor> {
    let mut out = Vec::new();
    for mut rung in rungs {
        for (key, &v) in minimums {
            if rung.get(key).is_some_and(|old| old < v) {
                rung.set(key, v);
            }
        }
        if out.last() != Some(&rung) {
            out.push(rung);
        }
    }
    out
}
pub fn fab_floor_ladder(
    layers: usize,
    tier: FabTier,
    policy: EscalationPolicy,
    overrides: &BTreeMap<String, f64>,
    board_floors: &BTreeMap<String, f64>,
) -> Vec<FabFloor> {
    let mut rungs = if !overrides.is_empty() {
        let mut floor = floors(layers, tier == FabTier::Advanced);
        for (key, &v) in overrides {
            floor.set(key, v);
        }
        vec![floor]
    } else {
        match tier {
            FabTier::Standard => vec![floors(layers, false)],
            FabTier::Advanced => vec![floors(layers, true)],
            FabTier::Auto => {
                let standard = floors(layers, false);
                let advanced = floors(layers, true);
                let mut rungs = vec![standard.clone()];
                if layers > 2 && advanced.via_diameter < 0.30 && 0.30 < standard.via_diameter {
                    let mut fine = standard;
                    fine.via_diameter = 0.30;
                    fine.via_drill = 0.15;
                    fine.annular = rounded((0.30 - 0.15) / 2.0, 4);
                    rungs.push(fine);
                }
                rungs.push(advanced);
                rungs
            }
        }
    };
    if policy == EscalationPolicy::Board {
        rungs = apply_floors(rungs, board_floors);
    }
    rungs
}
pub fn escalation_rungs(
    layers: usize,
    tier: FabTier,
    policy: EscalationPolicy,
    overrides: &BTreeMap<String, f64>,
    board_floors: &BTreeMap<String, f64>,
    extra_floors: &BTreeMap<String, f64>,
) -> Vec<FabFloor> {
    if policy == EscalationPolicy::Off {
        return vec![];
    }
    apply_floors(
        fab_floor_ladder(layers, tier, policy, overrides, board_floors),
        extra_floors,
    )
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ViaClamp {
    pub diameter: f64,
    pub drill: f64,
    pub status: String,
    pub rung: usize,
}
pub fn clamp_via_to_pad(
    diameter: f64,
    drill: f64,
    pad: &Pad,
    floors: &[FabFloor],
    location: Option<Point>,
) -> ViaClamp {
    let reach = if let Some(q) = location {
        (pad.size[0] - 2.0 * (q[0] - pad.point[0]).abs())
            .min(pad.size[1] - 2.0 * (q[1] - pad.point[1]).abs())
    } else {
        pad.size[0].min(pad.size[1])
    };
    if diameter <= reach + 1e-9 {
        return ViaClamp {
            diameter,
            drill,
            status: "fits".into(),
            rung: 0,
        };
    }
    if floors.is_empty() {
        return ViaClamp {
            diameter,
            drill,
            status: "floor".into(),
            rung: 0,
        };
    }
    for (rung, floor) in floors.iter().enumerate() {
        if reach < floor.via_diameter - 1e-9 {
            continue;
        }
        let annular = (floor.via_diameter - floor.via_drill) / 2.0;
        let size = rounded(reach, 4);
        let drill = drill.min(size - 2.0 * annular).max(floor.via_drill);
        return ViaClamp {
            diameter: size,
            drill: rounded(drill, 4),
            status: "clamped".into(),
            rung,
        };
    }
    let rung = floors.len() - 1;
    ViaClamp {
        diameter: floors[rung].via_diameter,
        drill: floors[rung].via_drill,
        status: "floor".into(),
        rung,
    }
}
