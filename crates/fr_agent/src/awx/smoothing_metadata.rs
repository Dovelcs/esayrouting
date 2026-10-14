//! Explicit pad metadata required to reproduce the source smoother's dense-field guard.
use crate::awx::{
    board_snapshot::BoardSnapshot, smoothing_chains::DensePadField, whole_context::RoutingRules,
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// One native pad's exact source-parser metadata. The map key is the native pad
/// reference used by `BoardSnapshot`; centers always come from the live board.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SmoothingPadMetadata {
    /// Original source `pad_type` string (for example `smd`, `thru_hole`, or
    /// `np_thru_hole`). This is required; native DSN drill geometry is not a
    /// reliable substitute for pad type.
    pub pad_type: String,
    /// Exact source `size_x`/`size_y` dimensions in mm, in board axes after the
    /// source parser's pad-frame resolution; these are not a rotated native bbox.
    pub size_mm: [f64; 2],
    /// Exact source drill diameter in mm; specify 0 when the source has no drill.
    pub drill_mm: f64,
    /// Exact source pad layer list. Use `layers_known = false` and an empty
    /// list only when the source parser had no layers attribute; this is distinct
    /// from a known empty list.
    pub layers_known: bool,
    pub layers: Vec<String>,
}

/// Explicit metadata keyed by native pad reference. When smoothing is enabled,
/// every pad in the captured board must have one entry and no entries may point
/// outside that board.
#[derive(Clone, Debug, Default, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SmoothingMetadata {
    pub pads: BTreeMap<String, SmoothingPadMetadata>,
}

#[derive(Clone, Copy)]
struct PadAtBoard {
    center: [f64; 2],
    size: [f64; 2],
    drill: f64,
    is_npth: bool,
    layers_known: bool,
    has_copper_layer: bool,
}

impl SmoothingMetadata {
    pub fn validate(&self) -> Result<(), String> {
        for (native_ref, pad) in &self.pads {
            if native_ref.trim().is_empty()
                || pad.pad_type.trim().is_empty()
                || pad.pad_type.trim() != pad.pad_type
            {
                return Err(
                    "smoothing metadata requires non-empty pad references and pad_type".into(),
                );
            }
            if pad.size_mm.iter().any(|v| !v.is_finite() || *v <= 0.0) {
                return Err(format!(
                    "smoothing pad {native_ref} size_mm must be finite and positive"
                ));
            }
            if !pad.drill_mm.is_finite() || pad.drill_mm < 0.0 {
                return Err(format!(
                    "smoothing pad {native_ref} drill_mm must be finite and nonnegative"
                ));
            }
            if !pad.layers_known && !pad.layers.is_empty() {
                return Err(format!(
                    "smoothing pad {native_ref} has layer names although layers_known is false"
                ));
            }
            let mut seen = BTreeSet::new();
            if pad
                .layers
                .iter()
                .any(|l| l.trim().is_empty() || !seen.insert(l))
            {
                return Err(format!(
                    "smoothing pad {native_ref} has an empty or duplicate layer"
                ));
            }
        }
        Ok(())
    }

    /// Produces the source's dense pin-field regions from explicit pad type,
    /// size, drill, and layer metadata plus real native pad centers. The source
    /// uses the global route config's track width and clearance, not per-chain
    /// widths or clearance classes.
    pub fn dense_fields(
        &self,
        snapshot: &BoardSnapshot,
        rules: &RoutingRules,
    ) -> Result<Vec<DensePadField>, String> {
        self.validate()?;
        if !rules.track_width.is_finite()
            || rules.track_width <= 0.0
            || !rules.clearance.is_finite()
            || rules.clearance < 0.0
        {
            return Err("smoothing dense-field rules require a positive track width and nonnegative clearance".into());
        }
        let actual: BTreeSet<_> = snapshot.pads.iter().map(|p| p.key.as_str()).collect();
        if actual.len() != snapshot.pads.len() {
            return Err("board snapshot contains duplicate native pad references".into());
        }
        let declared: BTreeSet<_> = self.pads.keys().map(String::as_str).collect();
        if actual != declared {
            let missing: Vec<_> = actual.difference(&declared).copied().collect();
            let extra: Vec<_> = declared.difference(&actual).copied().collect();
            return Err(format!(
                "smoothing metadata pad coverage mismatch; missing={missing:?}, unknown={extra:?}"
            ));
        }

        let mut groups = BTreeMap::<String, Vec<PadAtBoard>>::new();
        for pad in &snapshot.pads {
            let metadata = &self.pads[&pad.key];
            if pad.point.iter().any(|v| !v.is_finite()) {
                return Err(format!(
                    "smoothing pad {} has a non-finite native board center",
                    pad.key
                ));
            }
            let is_npth = metadata.pad_type == "np_thru_hole";
            groups
                .entry(pad.component_ref.clone())
                .or_default()
                .push(PadAtBoard {
                    center: pad.point,
                    size: metadata.size_mm,
                    drill: metadata.drill_mm,
                    is_npth,
                    layers_known: metadata.layers_known,
                    has_copper_layer: metadata.layers.iter().any(|name| name.ends_with(".Cu")),
                });
        }

        let mut dense = Vec::new();
        for (component_ref, pads) in groups {
            // pad_field_is_corridor counts every non-NPTH source pad, including
            // aperture-only records, when applying the 16-pad and median-size rules.
            let corridor_pads: Vec<_> = pads.iter().filter(|p| !p.is_npth).collect();
            if corridor_pads.len() < 16 {
                continue;
            }
            let mut sizes: Vec<_> = corridor_pads
                .iter()
                .map(|p| p.size[0].min(p.size[1]))
                .collect();
            sizes.sort_by(f64::total_cmp);
            let median_pad_size_mm = sizes[sizes.len() / 2]; // source uses the upper middle item

            // detect_bga_pitch uses non-aperture pads, which differs from the
            // broader set used for the corridor's pad-count/size calculation.
            let pitch_pads: Vec<_> = pads.iter().filter(|p| !is_aperture_only(p)).collect();
            let pitch_mm = source_pitch(&pitch_pads);
            let channel = rules.track_width + 2.0 * rules.clearance;
            if channel <= 0.0 || pitch_mm - median_pad_size_mm >= 0.6 * channel {
                continue;
            }
            // get_footprint_bounds uses every pad's true center and unrotated
            // source size fields, including NPTH and aperture-only records.
            let mut bounds = [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ];
            for pad in &pads {
                bounds[0] = bounds[0].min(pad.center[0] - pad.size[0] / 2.0);
                bounds[1] = bounds[1].min(pad.center[1] - pad.size[1] / 2.0);
                bounds[2] = bounds[2].max(pad.center[0] + pad.size[0] / 2.0);
                bounds[3] = bounds[3].max(pad.center[1] + pad.size[1] / 2.0);
            }
            dense.push(DensePadField {
                component_ref,
                bounds_mm: bounds,
                pitch_mm,
                median_pad_size_mm,
            });
        }
        Ok(dense)
    }
}

fn is_aperture_only(pad: &PadAtBoard) -> bool {
    !pad.is_npth && pad.drill <= 0.0 && pad.layers_known && !pad.has_copper_layer
}

fn source_pitch(pads: &[&PadAtBoard]) -> f64 {
    if pads.len() < 2 {
        return 1.0;
    }
    let mut axis_pitches = Vec::new();
    for axis in 0..2 {
        let mut positions: Vec<_> = pads.iter().map(|p| p.center[axis]).collect();
        positions.sort_by(f64::total_cmp);
        positions.dedup_by(|a, b| *a == *b);
        let mut gaps: Vec<_> = positions
            .windows(2)
            .map(|w| w[1] - w[0])
            .filter(|gap| *gap >= 0.01)
            .collect();
        if !gaps.is_empty() {
            gaps.sort_by(f64::total_cmp);
            let mid = gaps.len() / 2;
            axis_pitches.push(if gaps.len() % 2 == 1 {
                gaps[mid]
            } else {
                (gaps[mid - 1] + gaps[mid]) / 2.0
            });
        }
    }
    axis_pitches.into_iter().reduce(f64::min).unwrap_or(1.0)
}
