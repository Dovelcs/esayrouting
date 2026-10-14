//! Conservative R-tree broad phase; native engine remains the exact narrow phase.
//! R-tree spatial indexing: Guttman, SIGMOD 1984. rstar implements the mature index.
use crate::{
    board_adapter::BoardAdapter, execution::reservation_conflict, planner::PlanCandidate,
    spec::RouteSpec,
};
use rstar::{RTree, RTreeObject, AABB};
use std::{
    collections::{BTreeMap, BTreeSet},
    time::Instant,
};
#[derive(Clone)]
struct Envelope {
    candidate: usize,
    bounds: AABB<[f64; 2]>,
}
impl RTreeObject for Envelope {
    type Envelope = AABB<[f64; 2]>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}
fn envelope(candidate: usize, min: [f64; 2], max: [f64; 2], margin: f64) -> Envelope {
    Envelope {
        candidate,
        bounds: AABB::from_corners(
            [min[0] - margin, min[1] - margin],
            [max[0] + margin, max[1] + margin],
        ),
    }
}
fn nearby_pairs(
    board: &BoardAdapter,
    candidates: &[PlanCandidate],
    deadline: Instant,
) -> Result<BTreeSet<(usize, usize)>, String> {
    let mut by_layer: BTreeMap<String, Vec<Envelope>> = BTreeMap::new();
    // Round-to-grid can move each endpoint; halfwidth and clearance round upward.
    let rounding = 4.0 / board.units_per_mm;
    for (index, c) in candidates.iter().enumerate() {
        if Instant::now() >= deadline {
            return Err("budget exhausted in conflict broad phase".into());
        }
        for leg in &c.legs {
            for s in leg.points.windows(2) {
                by_layer
                    .entry(leg.layer.clone())
                    .or_default()
                    .push(envelope(
                        index,
                        [s[0][0].min(s[1][0]), s[0][1].min(s[1][1])],
                        [s[0][0].max(s[1][0]), s[0][1].max(s[1][1])],
                        leg.width_mm / 2.0
                            + leg.clearance_mm.max(
                                board.board.rules.clearance_matrix.max_value_on_layer(
                                    board.board.layer_structure.get_no(&leg.layer),
                                ) as f64
                                    / board.units_per_mm,
                            )
                            + rounding,
                    ));
            }
        }
        for r in &c.reservations {
            by_layer
                .entry(r.layer.clone())
                .or_default()
                .push(envelope(index, r.min, r.max, rounding));
        }
        for via in &c.vias {
            // Through-via copper may obstruct every stack layer, regardless of route endpoints.
            let clear = spec_clearance(c);
            for layer in &board.board.layer_structure.layers {
                by_layer
                    .entry(layer.name.clone())
                    .or_default()
                    .push(envelope(
                        index,
                        via.point,
                        via.point,
                        via.diameter_mm / 2.0 + clear + rounding,
                    ));
            }
        }
    }
    let mut pairs = BTreeSet::new();
    for objects in by_layer.into_values() {
        let tree = RTree::bulk_load(objects.clone());
        for a in &objects {
            if Instant::now() >= deadline {
                return Err("budget exhausted in conflict broad phase".into());
            }
            for b in tree.locate_in_envelope_intersecting(&a.bounds) {
                if a.candidate >= b.candidate
                    || candidates[a.candidate].selection_set_id
                        == candidates[b.candidate].selection_set_id
                {
                    continue;
                }
                pairs.insert((a.candidate, b.candidate));
            }
        }
    }
    // Native via clearances can be padstack/rule dependent. Until a certified
    // envelope is available, do not spatially prune any pair involving a via.
    for (i, a) in candidates.iter().enumerate() {
        if Instant::now() >= deadline {
            return Err("budget exhausted in via broad phase".into());
        }
        for (j, b) in candidates.iter().enumerate().skip(i + 1) {
            if Instant::now() >= deadline {
                return Err("budget exhausted in via broad phase".into());
            }
            if (!a.vias.is_empty() || !b.vias.is_empty())
                && a.selection_set_id != b.selection_set_id
            {
                pairs.insert((i, j));
            }
        }
    }
    Ok(pairs)
}
fn spec_clearance(c: &PlanCandidate) -> f64 {
    c.legs.iter().map(|l| l.clearance_mm).fold(0.0, f64::max)
}
pub fn build_conflicts(
    board: &BoardAdapter,
    spec: &RouteSpec,
    candidates: &[PlanCandidate],
    deadline: Instant,
) -> Result<Vec<(String, String)>, String> {
    let pairs: Vec<_> = nearby_pairs(board, candidates, deadline)?
        .into_iter()
        .collect();
    if Instant::now() >= deadline {
        return Err("budget exhausted in conflict narrow phase".into());
    }
    if pairs.is_empty() {
        return Ok(Vec::new());
    }
    let workers = spec.execution.threads.min(pairs.len()).max(1);
    let chunk_size = pairs.len().div_ceil(workers);
    let chunks: Result<Vec<Vec<(String, String)>>, String> = std::thread::scope(|scope| {
        let mut handles = Vec::new();
        for chunk in pairs.chunks(chunk_size) {
            // Each worker owns its native board snapshot; no shared board or unsafe Sync.
            let worker_board = board.clone();
            handles.push(scope.spawn(move || {
                let mut conflicts = Vec::new();
                for &(i, j) in chunk {
                    if Instant::now() >= deadline {
                        return Err("budget exhausted in conflict narrow phase".into());
                    }
                    let a = &candidates[i];
                    let b = &candidates[j];
                    if reservation_conflict(a, b) || {
                        let mut trial = worker_board.clone();
                        trial
                            .insert_candidate(a, spec)
                            .and_then(|_| trial.insert_candidate(b, spec))
                            .is_err()
                    } {
                        conflicts.push((a.id.clone(), b.id.clone()));
                    }
                }
                if Instant::now() >= deadline {
                    return Err("budget exhausted in conflict narrow phase".into());
                }
                Ok(conflicts)
            }));
        }
        handles
            .into_iter()
            .map(|handle| {
                handle
                    .join()
                    .map_err(|_| "conflict worker panicked".to_owned())?
            })
            .collect()
    });
    let mut conflicts: Vec<_> = chunks?.into_iter().flatten().collect();
    conflicts.sort();
    if Instant::now() >= deadline {
        return Err("budget exhausted in conflict narrow phase".into());
    }
    Ok(conflicts)
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::planner::{ReservedRegion, RouteLeg};
    use std::time::Duration;
    fn candidate(id: usize) -> PlanCandidate {
        PlanCandidate {
            id: id.to_string(),
            selection_set_id: id.to_string(),
            group_id: "g".into(),
            cost: 0,
            legs: vec![],
            vias: vec![],
            reservations: vec![],
            requires_any: vec![],
        }
    }
    #[test]
    fn broadphase_covers_exact_reservation_conflicts() {
        let board = BoardAdapter::load(include_bytes!("../tests/fixtures/two_chip.dsn")).unwrap();
        let mut candidates = Vec::new();
        for i in 0..24 {
            let mut c = candidate(i);
            let x = (i % 6) as f64 * 0.35;
            let y = (i / 6) as f64 * 0.2;
            c.reservations.push(ReservedRegion {
                owner_net: format!("n{i}"),
                layer: "Top".into(),
                min: [x, y],
                max: [x + 0.25, y + 0.25],
            });
            if i % 2 == 0 {
                c.legs.push(RouteLeg {
                    net: format!("n{i}"),
                    layer: "Top".into(),
                    points: vec![[x, y], [x + 0.4, y + 0.3]],
                    width_mm: 0.15,
                    clearance_mm: 0.1,
                });
            }
            candidates.push(c);
        }
        let pairs =
            nearby_pairs(&board, &candidates, Instant::now() + Duration::from_secs(2)).unwrap();
        for i in 0..candidates.len() {
            for j in i + 1..candidates.len() {
                if reservation_conflict(&candidates[i], &candidates[j]) {
                    assert!(pairs.contains(&(i, j)), "missing exact conflict {i},{j}");
                }
            }
        }
    }
    #[test]
    fn expired_broadphase_fails_without_partial_conflicts() {
        let board = BoardAdapter::load(include_bytes!("../tests/fixtures/two_chip.dsn")).unwrap();
        assert!(nearby_pairs(&board, &[candidate(0)], Instant::now()).is_err());
    }
    #[test]
    fn owned_native_workers_have_identical_exact_conflicts() {
        fn assert_send<T: Send>() {}
        assert_send::<BoardAdapter>();
        let board = BoardAdapter::load(include_bytes!("../tests/fixtures/two_chip.dsn")).unwrap();
        let mut spec = crate::spec::RouteSpec {
            version: 1,
            route_remaining: false,
            protected_nets: vec![],
            allow_partial_output: false,
            input: "board.dsn".into(),
            output: "board.ses".into(),
            nets: vec![],
            endpoint_groups: vec![],
            channels: vec![],
            groups: vec![],
            execution: crate::spec::Execution::default(),
            advanced: None,
            awx: None,
        };
        let mut candidates = Vec::new();
        for i in 0..24 {
            let mut c = candidate(i);
            c.reservations.push(ReservedRegion {
                owner_net: format!("n{i}"),
                layer: "Top".into(),
                min: [i as f64 * 0.1, 0.0],
                max: [i as f64 * 0.1 + 0.3, 0.3],
            });
            candidates.push(c);
        }
        spec.execution.threads = 1;
        let serial = build_conflicts(
            &board,
            &spec,
            &candidates,
            Instant::now() + Duration::from_secs(2),
        )
        .unwrap();
        spec.execution.threads = 8;
        let parallel = build_conflicts(
            &board,
            &spec,
            &candidates,
            Instant::now() + Duration::from_secs(2),
        )
        .unwrap();
        assert!(!serial.is_empty());
        assert_eq!(serial, parallel);
    }
    #[test]
    fn zero_pairs_return_without_spawning_zero_chunks() {
        let board = BoardAdapter::load(include_bytes!("../tests/fixtures/two_chip.dsn")).unwrap();
        let spec = crate::spec::RouteSpec {
            version: 1,
            route_remaining: false,
            protected_nets: vec![],
            allow_partial_output: false,
            input: "board.dsn".into(),
            output: "board.ses".into(),
            nets: vec![],
            endpoint_groups: vec![],
            channels: vec![],
            groups: vec![],
            execution: crate::spec::Execution::default(),
            advanced: None,
            awx: None,
        };
        assert!(
            build_conflicts(&board, &spec, &[], Instant::now() + Duration::from_secs(1))
                .unwrap()
                .is_empty()
        );
        let mut a = candidate(1);
        let mut b = candidate(2);
        a.reservations.push(ReservedRegion {
            owner_net: "a".into(),
            layer: "Top".into(),
            min: [0.0, 0.0],
            max: [1.0, 1.0],
        });
        b.reservations.push(ReservedRegion {
            owner_net: "b".into(),
            layer: "Top".into(),
            min: [10.0, 0.0],
            max: [11.0, 1.0],
        });
        assert!(build_conflicts(
            &board,
            &spec,
            &[a, b],
            Instant::now() + Duration::from_secs(1)
        )
        .unwrap()
        .is_empty());
    }
}
