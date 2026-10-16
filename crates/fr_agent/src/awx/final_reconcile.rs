//! Final-reconcile policy helpers, kept independent from the route driver.
use crate::board_adapter::BoardAdapter;
use fr_engine::{board::ItemKind, drc::NetIncompletes, ids::NetNo};
use serde::Serialize;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Debug, Serialize)]
pub(crate) struct QualityScore {
    pub broken_nets: usize,
    pub incomplete_connections: u32,
    pub disconnected_pins: usize,
}

pub(crate) fn quality_score(board: &BoardAdapter) -> QualityScore {
    let mut broken_nets = 0;
    let mut disconnected_pins = 0;
    for net in board.board.rules.nets.iter() {
        let total = board.board.get_pins().into_iter()
            .filter(|&key| board.board.item(key).contains_net(net.net_number)).count();
        if total < 2 { continue; }
        let largest = board.board.get_connected_sets(net.net_number).iter()
            .map(|set| set.iter().filter(|&key| board.board.item(key).is_pin()).count())
            .max().unwrap_or(0);
        if largest < total {
            broken_nets += 1;
            disconnected_pins += total - largest;
        }
    }
    QualityScore {
        broken_nets,
        incomplete_connections: fr_engine::pipeline::stats::incomplete_count(&board.board, None).max(0) as u32,
        disconnected_pins,
    }
}

pub(crate) fn open_nets(board: &BoardAdapter, protected: &BTreeSet<String>) -> Vec<NetNo> {
    board.board.rules.nets.iter().filter_map(|net| {
        if protected.contains(&net.name) { return None; }
        let items = board.board.get_connectable_items(net.net_number);
        (NetIncompletes::new(&board.board, net.net_number, &items).count() > 0)
            .then_some(net.net_number)
    }).collect()
}

pub(crate) fn grant_authority(
    board: &BoardAdapter,
    hints: &BTreeMap<NetNo, Vec<NetNo>>,
    scope: &BTreeSet<NetNo>,
    protected: &BTreeSet<String>,
    authorized: &mut BTreeSet<NetNo>,
) -> Vec<NetNo> {
    let mut added = Vec::new();
    for target in scope {
        let Some(owners) = hints.get(target) else { continue; };
        for &owner in owners {
            if authorized.len() >= 12 { return added; }
            if authorized.contains(&owner) { continue; }
            let Some(net) = board.board.rules.nets.get(owner) else { continue; };
            if protected.contains(&net.name) || net.contains_plane() { continue; }
            if board.board.get_items().into_iter().any(|key| {
                let item = board.board.item(key);
                item.contains_net(owner) && matches!(&item.kind, ItemKind::ConductionArea(_))
            }) { continue; }
            let copper: Vec<_> = board.board.get_traces().into_iter().chain(board.board.get_vias())
                .filter(|&key| board.board.item(key).contains_net(owner)).collect();
            if copper.is_empty() || copper.iter().all(|&key| board.is_source_protected_copper(key)) {
                continue;
            }
            authorized.insert(owner);
            added.push(owner);
        }
    }
    added
}

pub(crate) fn assert_lap_candidate(
    before: &BoardAdapter,
    after: &BoardAdapter,
    source: &BoardAdapter,
    protected: &BTreeSet<String>,
    mutable_nets: &BTreeSet<NetNo>,
) -> Result<(), String> {
    let mut validation = before.clone();
    let mut guard = protected.clone();
    for net in before.board.rules.nets.iter() {
        if !mutable_nets.contains(&net.net_number) { guard.insert(net.name.clone()); }
    }
    let mutable_names: BTreeSet<_> = mutable_nets.iter()
        .filter_map(|&net| before.board.rules.nets.get(net).map(|entry| entry.name.clone()))
        .collect();
    super::native_execution::strip_nets(&mut validation, &mutable_names)?;
    after.assert_session_invariants(&validation, &guard, false)?;
    if !after.new_violations_since(before).is_empty() {
        return Err("final-reconcile lap introduced exact clearance violations".into());
    }
    let old = quality_score(before);
    let new = quality_score(after);
    if new.broken_nets > old.broken_nets
        || new.incomplete_connections > old.incomplete_connections
        || new.disconnected_pins > old.disconnected_pins
    {
        return Err(format!("final-reconcile quality regressed: before={old:?} after={new:?}"));
    }
    // Strict source provenance is checked independently of the lap guard.
    after.assert_invariants(source, protected, true)?;
    Ok(())
}

pub(crate) struct PriorityRestoreCandidate {
    pub board: BoardAdapter,
    pub victim: NetNo,
    pub owners: Vec<NetNo>,
    pub verdict: &'static str,
}

pub(crate) fn prepare_priority_restore(
    current: &BoardAdapter,
    source: &BoardAdapter,
    protected: &BTreeSet<String>,
    victim: NetNo,
    saved: &BoardAdapter,
    routed: &BTreeSet<NetNo>,
    restore_protected: &BTreeSet<NetNo>,
) -> Result<Option<PriorityRestoreCandidate>, String> {
    let name = |board: &BoardAdapter, n: NetNo| board.board.rules.nets.get(n).map(|net| net.name.clone());
    let Some(victim_name) = name(current, victim) else { return Ok(None); };
    let vi = NetIncompletes::new(&current.board, victim, &current.board.get_connectable_items(victim));
    if vi.count() == 0 { return Ok(None); }
    // Require actual saved routing copper for this victim; an empty payload is not a restore.
    if !saved.board.get_traces().into_iter().chain(saved.board.get_vias())
        .any(|key| saved.board.item(key).contains_net(victim)) { return Ok(None); }

    let victim_set = BTreeSet::from([victim_name.clone()]);
    let mut restored = current.clone();
    super::native_execution::restore_nets(saved, &mut restored, &victim_set)?;
    let report = restored.new_clearance_violations_since(current);
    let mut owner_set = BTreeSet::new();
    let item_keys: BTreeMap<_, _> = restored.board.get_items().into_iter()
        .map(|key| (restored.board.item(key).id().0, key)).collect();
    for row in report.records {
        let mut pair_items = Vec::with_capacity(2);
        for id in [row.record.first_id, row.record.second_id] {
            let Some(&key) = item_keys.get(&id) else { return Ok(None); };
            let item = restored.board.item(key);
            if !item.is_trace() && !item.is_via() { return Ok(None); }
            if item.net_numbers().len() != 1 { return Ok(None); }
            pair_items.push(item);
        }
        if !pair_items.iter().any(|item| item.contains_net(victim)) {
            return Ok(None);
        }
        for item in pair_items {
            let owner = item.net_numbers()[0];
            if owner != victim { owner_set.insert(owner); }
        }
    }
    let owners: Vec<_> = owner_set.into_iter().collect();
    let forbidden: BTreeSet<_> = restore_protected.iter().copied().chain([victim]).collect();
    if owners.is_empty() || owners.len() > 3 || owners.iter().any(|owner| {
        *owner == victim || forbidden.contains(owner) || !routed.contains(owner)
            || name(current, *owner).map_or(true, |owner_name| protected.contains(&owner_name))
            || !current.board.get_traces().into_iter().chain(current.board.get_vias()).any(|key| {
                current.board.item(key).contains_net(*owner)
                    && !current.is_source_protected_copper(key)
            })
    }) { return Ok(None); }

    let owner_names: BTreeSet<_> = owners.iter().filter_map(|&owner| name(current, owner)).collect();
    let mut candidate = current.clone();
    super::native_execution::strip_nets(&mut candidate, &owner_names)?;
    super::native_execution::restore_nets(source, &mut candidate, &owner_names)?;
    super::native_execution::restore_nets(saved, &mut candidate, &victim_set)?;
    if candidate.assert_invariants(source, protected, true).is_err() {
        return Ok(None);
    }
    let open = NetIncompletes::new(&candidate.board, victim, &candidate.board.get_connectable_items(victim)).count();
    Ok(Some(PriorityRestoreCandidate {
        board: candidate,
        victim,
        owners,
        verdict: if open == 0 { "full" } else { "full_open" },
    }))
}

#[cfg(test)]
#[path = "final_reconcile_tests.rs"]
mod tests;
