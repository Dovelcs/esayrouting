//! Port of awx/relayer.py from the pinned routing-tools source.
//!
//! Reassigns the existing endpoint-to-via stub copper to the endpoint layer selected by
//! a whole-route plan. Geometry is unchanged; this transforms a freshly captured context
//! so subsequent solving, auditing, and realization see the copper on its actual layer.
use super::board_snapshot::{BoardSnapshot, LogicalComponent};
use super::{
    corridor::distance,
    whole_context::{CopperVia, WholeContext},
    whole_frame::FlowFrame,
    whole_types::WholePlan,
};
use crate::board_adapter::BoardAdapter;
use std::collections::{BTreeMap, BTreeSet};

const END_TOL_MM: f64 = 1e-3;

#[derive(Clone, Debug, Default)]
pub struct RelayReport {
    pub moved: Vec<String>,
    pub left: Vec<String>,
    pub dropped_vias: usize,
    pub dropped_via_sites: Vec<(i32, [f64; 2])>,
    /// Count of endpoint-to-via copper runs actually reassigned to another layer.
    pub relayed_ends: usize,
    /// Wall-clock duration of native layer changes, via removal, and snapshot capture.
    pub elapsed_us: Option<u64>,
}

/// Apply the layer changes to a native board and return a fresh snapshot for the next
/// `context_builder::build` call. The supplied context must have been built from this exact
/// board revision. Mutations happen on a clone and replace `board` only after copper edits and
/// snapshot capture both succeed.
pub fn relay_native_board(
    board: &mut BoardAdapter,
    context: &WholeContext,
    plan: &WholePlan,
    flow: FlowFrame,
    physical_layers: &[String],
    components: &[LogicalComponent],
    via_drills: &BTreeMap<String, f64>,
) -> Result<(BoardSnapshot, RelayReport), String> {
    let started = std::time::Instant::now();
    let (relayed, report) = relay_context(context, plan)?;
    if context.base_segments.len() != relayed.base_segments.len() {
        return Err("AWX relayer: transformed context changed segment inventory".into());
    }
    let mut native_moves = Vec::new();
    for (old, new) in context.base_segments.iter().zip(&relayed.base_segments) {
        if old.net_id != new.net_id || old.a != new.a || old.b != new.b {
            return Err("AWX relayer: context transformation changed copper geometry".into());
        }
        if old.layer == new.layer {
            continue;
        }
        let from_name = flow.to_board_layer(&old.layer, physical_layers)?;
        let to_name = flow.to_board_layer(&new.layer, physical_layers)?;
        let from = board.board.layer_structure.get_no(&from_name);
        let to = board.board.layer_structure.get_no(&to_name);
        if from < 0 || to < 0 {
            return Err(format!(
                "AWX relayer: layer is absent from native board: {} -> {}",
                from_name, to_name
            ));
        }
        native_moves.push((
            old.net_id,
            board.point(old.a)?,
            board.point(old.b)?,
            from,
            to,
        ));
    }
    let mut candidate = board.clone();
    candidate.board.relayer_trace_segments(&native_moves)?;
    let native_vias: Vec<_> = report
        .dropped_via_sites
        .iter()
        .map(|(net, point)| Ok((*net, candidate.point(*point)?)))
        .collect::<Result<_, String>>()?;
    candidate.board.remove_relayer_vias(&native_vias)?;
    let snapshot = BoardSnapshot::capture(&candidate, components, via_drills)?;
    let mut report = report;
    report.elapsed_us = Some(started.elapsed().as_micros().min(u64::MAX as u128) as u64);
    *board = candidate;
    Ok((snapshot, report))
}

fn near(a: [f64; 2], b: [f64; 2]) -> bool {
    distance(a, b) < END_TOL_MM
}
fn via_covers(v: &CopperVia, p: [f64; 2]) -> bool {
    distance(v.point, p) < v.diameter / 2.0
}

/// Move selected via-end runs according to the plan. `context` must be rebuilt from the
/// board copper that the next solve will actually use (the fanout board for a first solve,
/// or its original fanout base for a re-solve). Pair legs move together or remain unchanged.
pub fn relay_context(
    context: &WholeContext,
    plan: &WholePlan,
) -> Result<(WholeContext, RelayReport), String> {
    let mut out = context.clone();
    let mut report = RelayReport::default();
    let mut lanes: Vec<_> = plan.layers.iter().collect();
    lanes.sort_by(|a, b| a.0.cmp(b.0));
    let mut requests: Vec<(String, String, usize, [f64; 2], String)> = Vec::new();
    for (lane, selected) in lanes {
        if selected.is_empty() {
            continue;
        }
        let members: Vec<String> = context
            .pairs
            .get(lane)
            .map(|p| p.to_vec())
            .unwrap_or_else(|| vec![lane.clone()]);
        for (end, target) in [
            (0usize, selected.first().unwrap()),
            (1usize, selected.last().unwrap()),
        ] {
            for net in &members {
                let Some(ep) = context.ends.get(net) else {
                    continue;
                };
                let old = if end == 0 {
                    context.tooth_layer.get(net)
                } else {
                    context.dest_layer.get(net)
                };
                let Some(old) = old else {
                    continue;
                };
                if old != target {
                    let point = if end == 0 { ep.tooth } else { ep.berth };
                    requests.push((lane.clone(), net.clone(), end, point, target.clone()));
                }
            }
        }
    }
    let mut assignments: BTreeMap<usize, String> = BTreeMap::new();
    let mut affected_vias: BTreeMap<usize, String> = BTreeMap::new();
    let mut failed_groups = BTreeSet::new();
    let mut resolved = Vec::new();
    for req in &requests {
        let (_, net, end, point, target) = req;
        let nid = *context
            .byname
            .get(net)
            .ok_or_else(|| format!("AWX relayer: unknown net {net}"))?;
        let old_layer = if *end == 0 {
            &context.tooth_layer[net]
        } else {
            &context.dest_layer[net]
        };
        let run = find_run(context, nid, *point, old_layer);
        match run {
            Some((segments, via)) => resolved.push((req.clone(), Some((segments, via)))),
            None => {
                let on_via = context
                    .base_vias
                    .iter()
                    .position(|v| v.net_id == nid && via_covers(v, *point));
                resolved.push((req.clone(), on_via.map(|i| (Vec::new(), i))));
                if on_via.is_none() {
                    failed_groups.insert((req.0.clone(), req.2));
                }
            }
        }
        let _ = target;
    }
    for ((lane, net, end, _point, target), got) in resolved {
        if failed_groups.contains(&(lane.clone(), end)) {
            report.left.push(net);
            continue;
        }
        let Some((run, via)) = got else {
            report.left.push(net);
            continue;
        };
        for ix in run {
            assignments.insert(ix, target.clone());
        }
        affected_vias.insert(via, target.clone());
        report.relayed_ends = report.relayed_ends.saturating_add(1);
        if end == 0 {
            out.tooth_layer.insert(net.clone(), target);
        } else {
            out.dest_layer.insert(net.clone(), target);
        }
        report.moved.push(net);
    }
    for (&ix, layer) in &assignments {
        out.base_segments[ix].layer = layer.clone();
    }

    // Drop a via only when the moved run and every remaining copper connection/pad leave it
    // on exactly one copper layer, matching relayer.py's connectivity check.
    let mut drop = BTreeSet::new();
    for (&vi, target) in &affected_vias {
        let via = &out.base_vias[vi];
        let mut layers = BTreeSet::from([target.clone()]);
        for (ix, seg) in out.base_segments.iter().enumerate() {
            if assignments.contains_key(&ix) || seg.net_id != via.net_id {
                continue;
            }
            if via_covers(via, seg.a) || via_covers(via, seg.b) {
                layers.insert(seg.layer.clone());
            }
        }
        for pad in &out.pads {
            if pad.net_id != via.net_id {
                continue;
            }
            if (via.point[0] - pad.point[0]).abs() <= pad.size[0] / 2.0
                && (via.point[1] - pad.point[1]).abs() <= pad.size[1] / 2.0
            {
                for layer in &pad.layers {
                    if layer.ends_with(".Cu") {
                        layers.insert(layer.clone());
                    }
                }
            }
        }
        if layers.len() == 1 {
            drop.insert(vi);
            report.dropped_via_sites.push((via.net_id, via.point));
        }
    }
    report.dropped_vias = drop.len();
    report.dropped_via_sites.sort_by(|a, b| {
        a.0.cmp(&b.0)
            .then_with(|| a.1[0].total_cmp(&b.1[0]))
            .then_with(|| a.1[1].total_cmp(&b.1[1]))
    });
    out.base_vias = out
        .base_vias
        .into_iter()
        .enumerate()
        .filter_map(|(i, v)| (!drop.contains(&i)).then_some(v))
        .collect();
    report.moved.sort();
    report.moved.dedup();
    report.left.sort();
    report.left.dedup();
    Ok((out, report))
}

fn find_run(
    ctx: &WholeContext,
    net_id: i32,
    end: [f64; 2],
    layer: &str,
) -> Option<(Vec<usize>, usize)> {
    let segments: Vec<usize> = ctx
        .base_segments
        .iter()
        .enumerate()
        .filter_map(|(i, s)| (s.net_id == net_id && s.layer == layer).then_some(i))
        .collect();
    let mut current = end;
    let mut used = BTreeSet::new();
    let mut run = Vec::new();
    for _ in 0..=segments.len() {
        if let Some(v) = ctx
            .base_vias
            .iter()
            .position(|v| v.net_id == net_id && via_covers(v, current))
        {
            return (!run.is_empty()).then_some((run, v));
        }
        let next: Vec<_> = segments
            .iter()
            .copied()
            .filter(|i| !used.contains(i))
            .filter(|i| {
                let s = &ctx.base_segments[*i];
                near(s.a, current) || near(s.b, current)
            })
            .collect();
        if next.len() != 1 {
            return None;
        }
        let i = next[0];
        used.insert(i);
        run.push(i);
        let s = &ctx.base_segments[i];
        current = if near(s.a, current) { s.b } else { s.a };
    }
    None
}
