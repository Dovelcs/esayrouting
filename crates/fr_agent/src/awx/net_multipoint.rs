//! Original multipoint PLAN primitives for the native board front.
//!
//! Source counterparts (`work/kicad_routing_tools/py_router/`):
//! * `connectivity.get_multipoint_net_pads` (=1610) / `_get_multipoint_net_pads_unordered`
//!   (=1637) — terminal-row assembly: bare 3+ pads on their first active routing layer,
//!   otherwise one representative per existing-copper island (via-preferred, then a free
//!   end, then a pad on the island, then a segment start) plus every pad with no copper.
//!   The row list is then sorted by `(orig_x, orig_y, layer_index)`.
//! * `connectivity.get_terminal_component_info` (=588) — component labelling of terminals
//!   by the authoritative copper connectivity, with canonical component ids relabelled by
//!   the smallest terminal index the component owns.
//! * `connectivity.compute_component_mst_edges` (=736) — O(C^2) Prim over components, each
//!   component link realized by the minimum-Manhattan terminal pair (strict `<` relaxation,
//!   sorted component ids, tree started from the first component).
//! * `single_ended_routing.route_multipoint_main` (=3965) — the main-edge selection entry:
//!   edges sorted longest-first with a stable pad-index tie-break; with no bus corridor and
//!   no dense-fanout metadata the `_select_multipoint_main_edge` overrides are inert, so the
//!   longest-first order is the result. The `<= 8` attempt ladder and the phase-3 tap loop
//!   (=4710) are driven by a separate owner.
//!
//! This integration has no attraction/corridor geometry, no typed BGA/fanout metadata and
//! no TYPED body/ring/fab-drill records, so those source branches are not reproduced here
//! (nothing is invented to stand in for them):
//! * bus-corridor span re-realization / promotion and `KICAD_MULTIPOINT_DENSE_FIRST`
//!   (`_select_multipoint_main_edge` overrides 1 and 2) — require data this API does not
//!   carry; the longest-first default is used.
//! * multi-board-outline per-outline MST split (`#479`) — this API is single-outline.
//! * phase-3-specific machinery (island-wide launch cell maps, pour-launch ladders, via-in-pad
//!   rescue, stub layer switching, length-matching) — separate owner.
//!
//! Unlike the source (which rebuilds a Python segment-endpoint graph), existing-copper islands
//! and terminal/copper membership use the engine's authoritative `get_connected_sets` — the
//! same overlap-aware Pin/Via/copper connectivity the checker grades with. Two terminals that
//! the engine already joins (for example two same-net pads whose copper overlaps, or a pad
//! already landed on by a trace) therefore share one component and are never re-routed
//! between each other.

use crate::board_adapter::BoardAdapter;
use fr_engine::board::ItemKey;
use fr_engine::ids::{ItemId, LayerNo, NetNo};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

/// Source `kicad_parser.POSITION_DECIMALS`.
const POSITION_DECIMALS: f64 = 1000.0;
/// Source `connectivity.COINCIDENCE_TOL` (mm).
const COINCIDENCE_TOL: f64 = 0.02;
/// Source `connectivity.find_stub_free_ends` pad-attach tolerance (mm).
const PAD_ATTACH_TOL: f64 = 0.05;

type P2 = [f64; 2];

/// One terminal of a multipoint net, matching the source `pad_info` row fields that the
/// downstream phases consume.
#[derive(Clone, Debug)]
pub(crate) struct MultipointTerminal {
    /// The native board item this terminal represents: the pad pin, the preferred via, or the
    /// trace carrying a representative free end / segment start.
    pub item: ItemKey,
    /// Original (unquantized) position in millimetres — the source `info[3]`/`info[4]`.
    pub point_mm: [f64; 2],
    /// The active routing layer name this terminal launches on — the source `layer_names[idx]`.
    pub layer: String,
    /// Canonical component id (see module docs), not the arbitrary native group id.
    pub component: usize,
}

/// A multipoint routing plan: the ordered terminal rows and the component MST edges.
#[derive(Clone, Debug)]
pub(crate) struct MultipointPlan {
    pub terminals: Vec<MultipointTerminal>,
    /// `(term_a, term_b, distance_mm)` sorted by `(-distance, term_a, term_b)`; `term_a` is on
    /// the already-spanned side of the tree when the edge was added. `edges[0]` is the main-edge
    /// candidate (longest-first default).
    pub edges: Vec<(usize, usize, f64)>,
}

/// One source `Segment` (a single trace window), matching `pcb_data.segments` granularity.
#[derive(Clone, Copy)]
struct SegRef {
    key: ItemKey,
    a: P2,
    b: P2,
    layer: LayerNo,
}

struct PadRow {
    key: ItemKey,
    point: P2,
    layer: String,
}

struct Island {
    vias: Vec<ItemKey>,
    segs: Vec<SegRef>,
}

/// Plan the multipoint topology of `net`: terminal rows plus the component MST edges.
///
/// Returns `Ok(None)` only when the net is not a multipoint net at all — the exact source
/// eligibility: fewer than three terminals. A net with three or more terminals that the
/// authoritative copper already joins into a single component still returns
/// `Ok(Some(plan))` with an EMPTY edge list (the source's `already_connected` case, which
/// the caller treats as nothing left to route), never `Ok(None)` and never a plain
/// two-terminal fallback.
pub(crate) fn plan_native_multipoint(
    board: &BoardAdapter,
    net: NetNo,
    active_layers: &[(i32, String)],
) -> Result<Option<MultipointPlan>, String> {
    validate(board, net, active_layers)?;
    let terminals = build_terminals(board, net, active_layers)?;
    if terminals.len() < 3 {
        return Ok(None);
    }
    let mut terminals = terminals;
    // A single authoritative component is `already_connected`: keep the plan (with its
    // component-labelled terminals) and let the empty edge list signal nothing to route.
    let components = canonical_components(board, net, &terminals)?;
    for (terminal, &component) in terminals.iter_mut().zip(components.iter()) {
        terminal.component = component;
    }

    let positions: Vec<P2> = terminals.iter().map(|t| t.point_mm).collect();
    let mut edges = component_mst_edges(&positions, &components);
    edges.sort_by(|x, y| {
        (-x.2)
            .partial_cmp(&-y.2)
            .unwrap_or(Ordering::Equal)
            .then_with(|| x.0.cmp(&y.0))
            .then_with(|| x.1.cmp(&y.1))
    });
    Ok(Some(MultipointPlan { terminals, edges }))
}

/// Re-derive canonical component ids for `terminals` from the board's CURRENT authoritative
/// same-net copper connectivity.
///
/// Source phase 3 calls `get_terminal_component_info` again after phase 1 has committed its
/// main copper, because the union-find id space is rewritten by that commit and the stale
/// phase-1 labels can collide with a fresh island. This helper is the reusable fresh-label
/// step for that owner: it returns one canonical id per terminal, in the same
/// "smallest owning terminal index" order [`plan_native_multipoint`] uses.
pub(crate) fn canonical_components(
    board: &BoardAdapter,
    net: NetNo,
    terminals: &[MultipointTerminal],
) -> Result<Vec<usize>, String> {
    if !board.units_per_mm.is_finite() || board.units_per_mm <= 0.0 {
        return Err("invalid BoardAdapter unit scale for multipoint components".into());
    }
    if net <= 0 {
        return Err("multipoint components require a positive net".into());
    }

    let n = terminals.len();
    let sets = board.board.get_connected_sets(net);
    let mut set_of_id: BTreeMap<ItemId, usize> = BTreeMap::new();
    for (si, set) in sets.iter().enumerate() {
        for id in set.ids() {
            set_of_id.insert(id, si);
        }
    }

    // Union-find over terminals: every terminal whose item belongs to the same authoritative
    // connected set is one component.
    let mut parent: Vec<usize> = (0..n).collect();
    fn find(parent: &mut [usize], mut a: usize) -> usize {
        while parent[a] != a {
            let p = parent[a];
            let gp = parent[p];
            parent[a] = gp;
            a = gp;
        }
        a
    }
    let mut by_set: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for (i, terminal) in terminals.iter().enumerate() {
        if let Some(&si) = set_of_id.get(&board.board.item(terminal.item).id()) {
            by_set.entry(si).or_default().push(i);
        }
        // A terminal with no authoritative set membership (removed item, non-connectable
        // item) keeps its own singleton component — the source's per-terminal unique label.
    }
    for members in by_set.values() {
        for &member in &members[1..] {
            let (ra, rb) = (find(&mut parent, members[0]), find(&mut parent, member));
            if ra != rb {
                parent[ra] = rb;
            }
        }
    }

    let raw: Vec<usize> = (0..n).map(|i| find(&mut parent, i)).collect();
    Ok(canonicalize(&raw, n))
}

// ---------------------------------------------------------------------------------------------
// Terminal assembly (source get_multipoint_net_pads)

fn build_terminals(
    board: &BoardAdapter,
    net: NetNo,
    active_layers: &[(i32, String)],
) -> Result<Vec<MultipointTerminal>, String> {
    let scale = board.units_per_mm;
    let pads = collect_pads(board, net, active_layers)?;
    let segs = collect_segments(board, net, scale);

    let mut rows: Vec<(ItemKey, P2, String)> = Vec::new();

    if segs.is_empty() {
        // Source Case 1: no copper, 3+ pads — one terminal per physical pad on its first
        // active routing layer.
        if pads.len() < 3 {
            return Ok(Vec::new());
        }
        for pad in &pads {
            rows.push((pad.key, pad.point, pad.layer.clone()));
        }
    } else {
        // Source Cases 2+3: one representative per existing-copper island, then every pad
        // with no copper.
        let sets = board.board.get_connected_sets(net);
        let mut set_of_id: BTreeMap<ItemId, usize> = BTreeMap::new();
        for (si, set) in sets.iter().enumerate() {
            for id in set.ids() {
                set_of_id.insert(id, si);
            }
        }

        let mut islands: Vec<(usize, Island)> = Vec::new();
        for (si, set) in sets.iter().enumerate() {
            let mut vias = Vec::new();
            let mut island_segs = Vec::new();
            for key in set.iter() {
                let item = board.board.item(key);
                if item.is_via() {
                    vias.push(key);
                } else if item.is_trace() {
                    let layer = item.trace().layer();
                    let corners = item.trace().polyline().corner_approx_arr();
                    for w in corners.windows(2) {
                        island_segs.push(SegRef {
                            key,
                            a: [w[0].x / scale, w[0].y / scale],
                            b: [w[1].x / scale, w[1].y / scale],
                            layer,
                        });
                    }
                }
            }
            // Only segment-bearing existing copper is a source group: a lone via (copper
            // with no trace on the island) never generates a representative.
            if !island_segs.is_empty() {
                islands.push((si, Island { vias, segs: island_segs }));
            }
        }
        let island_ids: BTreeSet<usize> = islands.iter().map(|(si, _)| *si).collect();

        let pad_positions: Vec<P2> = pads.iter().map(|p| p.point).collect();
        for (si, island) in &islands {
            if let Some(row) = island_terminal(
                board,
                *si,
                island,
                &pads,
                &pad_positions,
                &set_of_id,
                active_layers,
            ) {
                rows.push(row);
            }
        }

        for pad in &pads {
            let attached = set_of_id
                .get(&board.board.item(pad.key).id())
                .is_some_and(|si| island_ids.contains(si));
            if !attached {
                rows.push((pad.key, pad.point, pad.layer.clone()));
            }
        }
    }

    // Source stable sort by (orig_x, orig_y, layer_index).
    rows.sort_by(|x, y| {
        x.1[0]
            .partial_cmp(&y.1[0])
            .unwrap_or(Ordering::Equal)
            .then_with(|| x.1[1].partial_cmp(&y.1[1]).unwrap_or(Ordering::Equal))
            .then_with(|| layer_rank(active_layers, &x.2).cmp(&layer_rank(active_layers, &y.2)))
    });

    Ok(rows
        .into_iter()
        .map(|(item, point_mm, layer)| MultipointTerminal {
            item,
            point_mm,
            layer,
            component: 0,
        })
        .collect())
}

/// One representative for an existing-copper island: via-preferred, then a free end, then a pad
/// on the island, then a segment start — each required to land on an active routing layer.
fn island_terminal(
    board: &BoardAdapter,
    island_set: usize,
    island: &Island,
    pads: &[PadRow],
    pad_positions: &[P2],
    set_of_id: &BTreeMap<ItemId, usize>,
    active_layers: &[(i32, String)],
) -> Option<(ItemKey, P2, String)> {
    let scale = board.units_per_mm;

    // (1) Prefer a via of the island (position-sorted for GUI/CLI order independence).
    let mut vias: Vec<(P2, ItemKey, LayerNo)> = island
        .vias
        .iter()
        .map(|&key| {
            let center = board.board.item(key).center(&board.board).to_float();
            let first = board.board.item(key).first_layer(&board.board);
            ([center.x / scale, center.y / scale], key, first)
        })
        .collect();
    vias.sort_by(|x, y| {
        x.0[0]
            .partial_cmp(&y.0[0])
            .unwrap_or(Ordering::Equal)
            .then_with(|| x.0[1].partial_cmp(&y.0[1]).unwrap_or(Ordering::Equal))
    });
    if let Some((point, key, via_first_layer)) = vias.first().copied() {
        // Source `_vlayer`: the layer of an island segment whose endpoint touches the via,
        // else the island's first segment layer (a lone via falls back to its own first layer).
        let layer = island
            .segs
            .iter()
            .find(|seg| {
                (seg.a[0] - point[0]).abs() < PAD_ATTACH_TOL
                    && (seg.a[1] - point[1]).abs() < PAD_ATTACH_TOL
                    || (seg.b[0] - point[0]).abs() < PAD_ATTACH_TOL
                        && (seg.b[1] - point[1]).abs() < PAD_ATTACH_TOL
            })
            .map(|seg| seg.layer)
            .or_else(|| island.segs.first().map(|seg| seg.layer))
            .unwrap_or(via_first_layer);
        if let Some(name) = layer_name(active_layers, layer) {
            return Some((key, point, name));
        }
    }

    // (2) A free end of the island's copper.
    let free = find_stub_free_ends(&island.segs, pad_positions);
    if let Some(&(x, y, layer, key)) = free.first() {
        if let Some(name) = layer_name(active_layers, layer) {
            return Some((key, [x, y], name));
        }
    }

    // (3) A pad sitting on the island (the source's `pads_on_group[gi][0]`).
    for pad in pads {
        if set_of_id.get(&board.board.item(pad.key).id()) == Some(&island_set) {
            return Some((pad.key, pad.point, pad.layer.clone()));
        }
    }

    // (4) Any segment endpoint of the island.
    if let Some(seg) = island.segs.first() {
        if let Some(name) = layer_name(active_layers, seg.layer) {
            return Some((seg.key, seg.a, name));
        }
    }
    None
}

// ---------------------------------------------------------------------------------------------
// Component MST (source compute_component_mst_edges)

fn component_mst_edges(positions: &[P2], components: &[usize]) -> Vec<(usize, usize, f64)> {
    let n = positions.len();
    let mut comp_terminals: BTreeMap<usize, Vec<usize>> = BTreeMap::new();
    for i in 0..n {
        comp_terminals
            .entry(components.get(i).copied().unwrap_or(i))
            .or_default()
            .push(i);
    }
    let comp_ids: Vec<usize> = comp_terminals.keys().copied().collect();
    if comp_ids.len() <= 1 {
        return Vec::new();
    }

    let mut best: BTreeMap<usize, (f64, usize, usize)> = comp_ids
        .iter()
        .map(|&c| (c, (f64::INFINITY, usize::MAX, usize::MAX)))
        .collect();

    let start = comp_ids[0];
    best.remove(&start);
    relax(start, &comp_terminals, positions, &mut best);

    let mut edges = Vec::with_capacity(comp_ids.len() - 1);
    while !best.is_empty() {
        // `min(best, key=(dist, comp_id))`: strict `<` keeps the first (smallest id) on ties.
        let mut next_comp = usize::MAX;
        let mut next_key = (f64::INFINITY, usize::MAX);
        for (&c, &(d, _, _)) in best.iter() {
            if d < next_key.0 || (d == next_key.0 && c < next_key.1) {
                next_key = (d, c);
                next_comp = c;
            }
        }
        let (d, a, b) = best.remove(&next_comp).expect("selected component present");
        edges.push((a, b, d));
        relax(next_comp, &comp_terminals, positions, &mut best);
    }
    edges
}

fn relax(
    from_comp: usize,
    comp_terminals: &BTreeMap<usize, Vec<usize>>,
    positions: &[P2],
    best: &mut BTreeMap<usize, (f64, usize, usize)>,
) {
    for &a in &comp_terminals[&from_comp] {
        let (ax, ay) = (positions[a][0], positions[a][1]);
        let keys: Vec<usize> = best.keys().copied().collect();
        for c in keys {
            let mut entry = best[&c];
            for &b in &comp_terminals[&c] {
                let d = (ax - positions[b][0]).abs() + (ay - positions[b][1]).abs();
                if d < entry.0 {
                    entry = (d, a, b);
                }
            }
            best.insert(c, entry);
        }
    }
}

fn canonicalize(raw: &[usize], n: usize) -> Vec<usize> {
    let mut first_terminal: BTreeMap<usize, usize> = BTreeMap::new();
    for (i, &root) in raw.iter().enumerate().take(n) {
        first_terminal.entry(root).or_insert(i);
    }
    let mut roots: Vec<usize> = first_terminal.keys().copied().collect();
    roots.sort_by_key(|root| first_terminal[root]);
    let remap: BTreeMap<usize, usize> = roots
        .iter()
        .enumerate()
        .map(|(canonical, &root)| (root, canonical))
        .collect();
    raw.iter().map(|root| remap[root]).collect()
}

// ---------------------------------------------------------------------------------------------
// Free ends (source connectivity.find_stub_free_ends)

fn find_stub_free_ends(group: &[SegRef], pad_positions: &[P2]) -> Vec<(f64, f64, LayerNo, ItemKey)> {
    if group.is_empty() {
        return Vec::new();
    }
    let mut points: Vec<(f64, f64, LayerNo)> = Vec::with_capacity(group.len() * 2);
    for seg in group {
        points.push((seg.a[0], seg.a[1], seg.layer));
        points.push((seg.b[0], seg.b[1], seg.layer));
    }

    // Layer-aware coincidence clustering (`cluster_coincident_points`).
    let mut parent: Vec<usize> = (0..points.len()).collect();
    fn find(parent: &mut [usize], mut a: usize) -> usize {
        while parent[a] != a {
            let p = parent[a];
            let gp = parent[p];
            parent[a] = gp;
            a = gp;
        }
        a
    }
    for i in 0..points.len() {
        for j in (i + 1)..points.len() {
            let (xa, ya, la) = points[i];
            let (xb, yb, lb) = points[j];
            if la == lb && (xa - xb).abs() < COINCIDENCE_TOL && (ya - yb).abs() < COINCIDENCE_TOL {
                let (ra, rb) = (find(&mut parent, i), find(&mut parent, j));
                if ra != rb {
                    parent[ra] = rb;
                }
            }
        }
    }

    let mut members: Vec<Vec<usize>> = Vec::new();
    let mut index: BTreeMap<usize, usize> = BTreeMap::new();
    for pi in 0..points.len() {
        let root = find(&mut parent, pi);
        match index.get(&root) {
            Some(&k) => members[k].push(pi),
            None => {
                index.insert(root, members.len());
                members.push(vec![pi]);
            }
        }
    }

    let near_pad = |x: f64, y: f64| {
        pad_positions
            .iter()
            .any(|p| (x - p[0]).abs() < PAD_ATTACH_TOL && (y - p[1]).abs() < PAD_ATTACH_TOL)
    };

    let mut free_ends: Vec<(f64, f64, LayerNo, ItemKey)> = Vec::new();
    for pis in &members {
        let mut seg_hits: BTreeMap<usize, i32> = BTreeMap::new();
        for &pi in pis {
            *seg_hits.entry(pi / 2).or_insert(0) += 1;
        }
        let open_segs: Vec<usize> = seg_hits
            .iter()
            .filter(|(_, &count)| count == 1)
            .map(|(&si, _)| si)
            .collect();

        if open_segs.len() == 1 {
            let si = open_segs[0];
            let seg = group[si];
            let in_cluster = *pis.iter().find(|&&pi| pi / 2 == si).expect("open segment hit");
            let far = if in_cluster % 2 == 0 { seg.b } else { seg.a };
            // Deterministic max((dist^2, x, y, layer)).
            let mut best = points[pis[0]];
            let mut best_key = (
                (best.0 - far[0]).powi(2) + (best.1 - far[1]).powi(2),
                best.0,
                best.1,
                best.2,
            );
            for &pi in &pis[1..] {
                let p = points[pi];
                let key = (
                    (p.0 - far[0]).powi(2) + (p.1 - far[1]).powi(2),
                    p.0,
                    p.1,
                    p.2,
                );
                if key > best_key {
                    best_key = key;
                    best = p;
                }
            }
            if !near_pad(best.0, best.1) {
                free_ends.push((py_round3(best.0), py_round3(best.1), best.2, seg.key));
            }
        } else if open_segs.is_empty() && members.len() == 1 {
            let mut seen: BTreeSet<(i64, i64, LayerNo)> = BTreeSet::new();
            for &pi in pis {
                let (x, y, layer) = points[pi];
                let key = (
                    round_half_even(x * POSITION_DECIMALS) as i64,
                    round_half_even(y * POSITION_DECIMALS) as i64,
                    layer,
                );
                if seen.insert(key) && !near_pad(x, y) {
                    free_ends.push((py_round3(x), py_round3(y), layer, group[pi / 2].key));
                }
            }
        }
    }

    free_ends.sort_by(|a, b| {
        a.0.partial_cmp(&b.0)
            .unwrap_or(Ordering::Equal)
            .then_with(|| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
            .then_with(|| a.2.cmp(&b.2))
    });
    free_ends
}

// ---------------------------------------------------------------------------------------------
// Copper extraction

fn collect_pads(
    board: &BoardAdapter,
    net: NetNo,
    active_layers: &[(i32, String)],
) -> Result<Vec<PadRow>, String> {
    let mut rows = Vec::new();
    for pad in board.pads() {
        let Some(key) = pad.key else { continue };
        let item = board.board.item(key);
        if !item.is_pin() || !item.contains_net(net) {
            continue;
        }
        // Unnumbered pads (paste/thermal artifacts) are never routing targets (source #94).
        if pad.pad.trim().is_empty() {
            continue;
        }
        // Source `drop_off_board_pads`: a pad whose center is off the actual board
        // outline is not a terminal. This uses the loader-retained native outline
        // rings, not the engine's coarse `is_on_board` flag.
        if !pad_on_board(board, pad.point) {
            continue;
        }
        rows.push(PadRow {
            key,
            point: pad.point,
            layer: pad_active_layer(&pad.layers, active_layers),
        });
    }
    Ok(rows)
}

/// Source `drop_off_board_pads`: whether a pad center (mm) lies on the board.
///
/// Uses the actual native outline rings retained by the loader (board units) — no
/// fabricated bounding box: when no outline was loaded every pad is kept. A centre
/// inside a native cutout (a real hole through the board) counts as off-board.
fn pad_on_board(board: &BoardAdapter, point_mm: P2) -> bool {
    if board.board_outline_polygons.is_empty() {
        return true;
    }
    let scale = board.units_per_mm;
    let point = [point_mm[0] * scale, point_mm[1] * scale];
    board
        .board_outline_polygons
        .iter()
        .any(|ring| point_inside_ring(point, ring))
        && !board
            .board_cutouts
            .iter()
            .any(|ring| point_inside_ring(point, ring))
}

/// Even-odd ray cast, matching `board_adapter`/`native_outline` outline containment.
fn point_inside_ring(point: [f64; 2], ring: &[[i32; 2]]) -> bool {
    if ring.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = ring.len() - 1;
    for i in 0..ring.len() {
        let (xi, yi) = (ring[i][0] as f64, ring[i][1] as f64);
        let (xj, yj) = (ring[j][0] as f64, ring[j][1] as f64);
        if (yi > point[1]) != (yj > point[1])
            && point[0] < (xj - xi) * (point[1] - yi) / (yj - yi) + xi
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// The pad's first layer that is an active routing layer, else the first active layer
/// (source `expand_pad_layers` / `layer_map` with its `config.layers[0]` fallback).
fn pad_active_layer(layers: &[String], active_layers: &[(i32, String)]) -> String {
    for name in layers {
        if active_layers.iter().any(|(_, candidate)| candidate == name) {
            return name.clone();
        }
    }
    active_layers
        .first()
        .map(|(_, name)| name.clone())
        .unwrap_or_default()
}

fn collect_segments(board: &BoardAdapter, net: NetNo, scale: f64) -> Vec<SegRef> {
    let mut segs = Vec::new();
    for key in board.board.get_traces() {
        let item = board.board.item(key);
        if !item.contains_net(net) {
            continue;
        }
        let layer = item.trace().layer();
        let corners = item.trace().polyline().corner_approx_arr();
        for w in corners.windows(2) {
            segs.push(SegRef {
                key,
                a: [w[0].x / scale, w[0].y / scale],
                b: [w[1].x / scale, w[1].y / scale],
                layer,
            });
        }
    }
    segs
}

// ---------------------------------------------------------------------------------------------
// Scalar / layer helpers

fn layer_name(active_layers: &[(i32, String)], native: LayerNo) -> Option<String> {
    active_layers
        .iter()
        .find(|(no, _)| *no == native)
        .map(|(_, name)| name.clone())
}

fn layer_rank(active_layers: &[(i32, String)], name: &str) -> usize {
    active_layers
        .iter()
        .position(|(_, candidate)| candidate == name)
        .unwrap_or(usize::MAX)
}

fn round_half_even(x: f64) -> f64 {
    let f = x.floor();
    let diff = x - f;
    if diff < 0.5 {
        f
    } else if diff > 0.5 {
        f + 1.0
    } else if (f as i64) % 2 == 0 {
        f
    } else {
        f + 1.0
    }
}

/// Python `round(x, 3)`.
fn py_round3(v: f64) -> f64 {
    round_half_even(v * POSITION_DECIMALS) / POSITION_DECIMALS
}

fn validate(
    board: &BoardAdapter,
    net: NetNo,
    active_layers: &[(i32, String)],
) -> Result<(), String> {
    if !board.units_per_mm.is_finite() || board.units_per_mm <= 0.0 {
        return Err("invalid BoardAdapter unit scale for multipoint plan".into());
    }
    if net <= 0 {
        return Err("multipoint plan requires a positive net".into());
    }
    if active_layers.is_empty() {
        return Err("multipoint plan requires at least one active routing layer".into());
    }
    Ok(())
}
