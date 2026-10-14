//! Structured aggregate for one AWX execution's per-bus reports.
use super::integration::AwxBusReport;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AwxRunStats {
    pub bus_count: usize,
    pub group_count: usize,
    pub complete_groups: usize,
    pub partial_groups: usize,
    pub refused_groups: usize,
    pub requested_nets: usize,
    pub routed_nets: usize,
    pub refused_nets: usize,
    /// Outer joint-fanout feedback rounds, summed over reported buses.
    pub fanout_rounds: usize,
    /// Sum of `WholeLoopOutcome.rounds`; None means caller did not provide all outcomes.
    pub whole_rounds: Option<usize>,
    /// Number of endpoint-to-via runs actually relayered; None means caller did not provide reports.
    pub relay_count: Option<usize>,
    /// Sum of measured whole-route invocation durations, in microseconds.
    pub whole_route_elapsed_us: Option<u64>,
    /// Sum of native relayer and snapshot-capture durations, in microseconds.
    pub relayer_elapsed_us: Option<u64>,
    pub passive_movements: usize,
    pub passive_held: usize,
    pub via_nudges: usize,
    pub drc_new_violations: usize,
    pub drc_native_violations: usize,
    pub vias: usize,
    pub copper_mm: f64,
}

/// Event totals not carried by AwxBusReport. Each Some value is an observed aggregate,
/// including Some(0) when the stage ran and recorded no corresponding event.
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct AwxRuntimeEvents {
    pub whole_rounds: Option<usize>,
    pub relay_count: Option<usize>,
    pub whole_route_elapsed_us: Option<u64>,
    pub relayer_elapsed_us: Option<u64>,
}

impl AwxRunStats {
    pub fn from_buses(buses: &[AwxBusReport], events: Option<&AwxRuntimeEvents>) -> Self {
        let mut out = Self {
            bus_count: buses.len(),
            whole_rounds: events.and_then(|e| e.whole_rounds),
            relay_count: events.and_then(|e| e.relay_count),
            whole_route_elapsed_us: events.and_then(|e| e.whole_route_elapsed_us),
            relayer_elapsed_us: events.and_then(|e| e.relayer_elapsed_us),
            ..Self::default()
        };
        let mut requested = BTreeSet::new();
        let mut routed = BTreeSet::new();
        let mut refused = BTreeSet::new();
        for bus in buses {
            let bus_requested: BTreeSet<_> = bus.requested.iter().cloned().collect();
            let bus_routed: BTreeSet<_> = bus.routed.iter().cloned().collect();
            requested.extend(bus_requested.iter().cloned());
            routed.extend(bus_routed.iter().cloned());
            refused.extend(bus.refused.keys().cloned());
            out.group_count = out.group_count.saturating_add(bus.groups.len());
            out.fanout_rounds = out.fanout_rounds.saturating_add(bus.rounds);
            out.drc_new_violations = out
                .drc_new_violations
                .saturating_add(bus.grade.new_violations);
            out.drc_native_violations = out
                .drc_native_violations
                .saturating_add(bus.grade.native_violations);
            out.vias = out.vias.saturating_add(bus.grade.vias);
            out.copper_mm += bus.grade.copper_mm;
            if let Some(placement) = &bus.placement {
                out.passive_movements = out
                    .passive_movements
                    .saturating_add(usize::from(placement.moved.is_some()));
                out.passive_held = out.passive_held.saturating_add(placement.held.len());
            }
            out.via_nudges = out.via_nudges.saturating_add(bus.via_nudged.len());

            let all_routed = !bus_requested.is_empty() && bus_requested.is_subset(&bus_routed);
            let clean = bus.grade.open.is_empty() && bus.grade.new_violations == 0;
            let (complete, partial, refused_groups) = if all_routed && clean {
                (bus.groups.len(), 0, 0)
            } else if !bus_routed.is_empty() {
                (0, bus.groups.len(), 0)
            } else {
                (0, 0, bus.groups.len())
            };
            out.complete_groups = out.complete_groups.saturating_add(complete);
            out.partial_groups = out.partial_groups.saturating_add(partial);
            out.refused_groups = out.refused_groups.saturating_add(refused_groups);
        }
        out.requested_nets = requested.len();
        out.routed_nets = routed.len();
        out.refused_nets = refused.len();
        out
    }
}
