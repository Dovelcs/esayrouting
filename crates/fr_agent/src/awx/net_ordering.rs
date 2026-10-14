//! Original MPS net ordering, ported verbatim from the Python source.
//!
//! Source counterparts (`work/kicad_routing_tools/py_router/`):
//! * `net_queries.compute_mps_net_ordering` (=2268) — the driver, unit order and
//!   net order returned to the endpoint front.
//! * `net_queries._build_mps_unit_mappings` (=1997) — with `diff_pairs=None`
//!   every net is its own unit (this fixed API carries no pair metadata).
//! * `net_queries._compute_mps_unit_endpoints` (=2043) — per-unit endpoints.
//! * `net_queries._compute_mps_center` (=2079) — mean of all endpoints.
//! * `net_queries._compute_mps_unit_layers` (=2098) — stub-group / pad layer sets.
//! * `net_queries._compute_mps_unit_distances` (=2159) — endpoint distance (the
//!   BGA branch needs `bga_exclusion_zones`, which this API does not carry, so the
//!   source `else` branch is taken, exactly as source does with no zones).
//! * `net_queries._greedy_order_mps_units` (=2203) — round-based greedy pick.
//! * `net_queries.compute_routing_aware_distance` (=1803) / `get_unit_routing_info`
//!   (=1871) / `find_containing_or_nearest_bga_zone` (=1760) — BGA distance branch,
//!   unreachable without BGA metadata; not reproduced (see below).
//! * `connectivity.get_net_routing_endpoints` (=1988) — net endpoints.
//! * `connectivity.get_net_mst_segments` (=1943) / `compute_mst_segments` (=1933) /
//!   `compute_mst_edges` (=1867) — Prim MST over stub free ends / pad positions.
//! * `connectivity.find_connected_groups` (=471) — layer-aware endpoint grouping
//!   with via bridging (`cluster_coincident_points` =418).
//! * `connectivity.find_stub_free_ends` (=798) — free ends of one stub group.
//! * `connectivity.segments_intersect` (=1848) — crossing test.
//! * `routing_defaults.DEFAULT_ORDERING_STRATEGY = "mps"` — the root default this
//!   entry point implements.
//!
//! Unsupported source branches (reported, never approximated):
//! * Chip/BGA boundary ordering. The source `use_boundary_ordering=True` default
//!   builds `chip_boundary.build_chip_list` (=27) from the footprint/package
//!   partition and only records boundary info when an endpoint pair lies on two
//!   *different* chips (`src_chip != tgt_chip`). This integration's board is a
//!   single flattened component, so every endpoint is on the same chip and the
//!   source records no boundary info either — the source then falls through to
//!   the segment-intersection / angular methods implemented here, so the port is
//!   exact for that board. A multi-chip board cannot be expressed through
//!   `BoardAdapter` (no footprint partition): the branch is disabled and, if it
//!   were ever requested, the code returns an explicit error instead of inventing
//!   a substitute heuristic.
//! * Diff-pair units. `_build_mps_unit_mappings` groups P/N nets when `diff_pairs`
//!   is supplied; this API passes none, so the source's `diff_pairs=None` branch
//!   (one net per unit) is the one implemented.
//!
//! All geometry is reproduced in millimetres (native board units / `units_per_mm`)
//! because every source tolerance (`COINCIDENCE_TOL = 0.02`, `POSITION_DECIMALS =
//! 3`, pad attach `0.05`) is in mm. Layer identity uses native physical layer
//! indices, not name-suffix string parsing.

use crate::awx::numeric;
use crate::board_adapter::BoardAdapter;
use fr_engine::ids::{LayerNo, NetNo};
use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet};

/// Source `routing_utils.POSITION_DECIMALS` / `kicad_parser.POSITION_DECIMALS`.
const POSITION_DECIMALS: f64 = 1000.0; // 10^3
/// Source `connectivity.COINCIDENCE_TOL`.
const COINCIDENCE_TOL: f64 = 0.02;
/// Source `connectivity.find_stub_free_ends` pad-attach tolerance.
const PAD_ATTACH_TOL: f64 = 0.05;

/// `compute_mps_net_ordering(reverse_rounds=False)`.
const REVERSE_ROUNDS: bool = false;
/// `compute_mps_net_ordering(crossing_layer_check=True)`.
const CROSSING_LAYER_CHECK: bool = true;

/// `compute_mps_net_ordering(use_boundary_ordering=True)` is the source default,
/// but its chip-boundary crossing requires a footprint/BGA partition this API
/// does not expose. Disabled (see module docs); if enabled we must error.
const USE_BOUNDARY_ORDERING: bool = false;

type P2 = [f64; 2];

#[derive(Clone, Copy)]
struct Seg {
    a: P2,
    b: P2,
    layer: LayerNo,
}

struct PadCopper {
    pos: P2,
    layers: Vec<LayerNo>,
}

#[derive(Default)]
struct NetCopper {
    segs: Vec<Seg>,
    vias: Vec<P2>,
    pads: Vec<PadCopper>,
}

// ---------------------------------------------------------------------------------------------
// Python-compatible scalar helpers

/// Python `round(x, 3)` (round-half-to-even on the scaled value).
fn py_round3(v: f64) -> f64 {
    round_half_even(v * POSITION_DECIMALS) / POSITION_DECIMALS
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

/// Exact rounded position key (source `pos_key`) as integers for set membership.
fn key3(p: P2) -> (i64, i64) {
    (
        round_half_even(p[0] * POSITION_DECIMALS) as i64,
        round_half_even(p[1] * POSITION_DECIMALS) as i64,
    )
}

/// Source `math.hypot` for segment length / group rank.
fn hypot_len(a: P2, b: P2) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

/// Source `math.sqrt(dx*dx + dy*dy)` (MST + unit distances).
fn sqrt_dist(a: P2, b: P2) -> f64 {
    let dx = a[0] - b[0];
    let dy = a[1] - b[1];
    (dx * dx + dy * dy).sqrt()
}

fn cmp_f64(a: f64, b: f64) -> Ordering {
    a.partial_cmp(&b).unwrap_or(Ordering::Equal)
}

fn cmp_p2(a: P2, b: P2) -> Ordering {
    cmp_f64(a[0], b[0]).then_with(|| cmp_f64(a[1], b[1]))
}

fn cmp_anchor(a: &(P2, P2, LayerNo), b: &(P2, P2, LayerNo)) -> Ordering {
    cmp_p2(a.0, b.0)
        .then_with(|| cmp_p2(a.1, b.1))
        .then_with(|| a.2.cmp(&b.2))
}

fn centroid(points: &[P2]) -> Result<P2, String> {
    let n = points.len() as f64;
    Ok([
        numeric::fsum(points.iter().map(|p| p[0]))? / n,
        numeric::fsum(points.iter().map(|p| p[1]))? / n,
    ])
}

// ---------------------------------------------------------------------------------------------
// Native copper extraction

/// Gather the net's own copper in the ported shape: split traces into single
/// segments (source `pcb_data.segments`), vias, and native pin centers with their
/// real copper layers. No drill, package or footprint metadata is consulted.
fn gather_copper(board: &BoardAdapter, target: &BTreeSet<NetNo>) -> BTreeMap<NetNo, NetCopper> {
    let b = &board.board;
    let scale = board.units_per_mm;
    let mut map: BTreeMap<NetNo, NetCopper> = BTreeMap::new();

    for key in b.get_traces() {
        let item = b.item(key);
        let layer = item.trace().layer();
        let corners = item.trace().polyline().corner_approx_arr();
        if corners.len() < 2 {
            continue;
        }
        for w in corners.windows(2) {
            let seg = Seg {
                a: [w[0].x / scale, w[0].y / scale],
                b: [w[1].x / scale, w[1].y / scale],
                layer,
            };
            for &n in item.net_numbers() {
                if target.contains(&n) {
                    map.entry(n).or_default().segs.push(seg);
                }
            }
        }
    }

    for key in b.get_vias() {
        let item = b.item(key);
        let p = item.center(b).to_float();
        let pos = [p.x / scale, p.y / scale];
        for &n in item.net_numbers() {
            if target.contains(&n) {
                map.entry(n).or_default().vias.push(pos);
            }
        }
    }

    // Native pad centers with their real copper (signal) layers. `board.pads()`
    // resolves each native pin's center and its signal-layer span; no drill,
    // plating or package geometry is consulted.
    for pad in board.pads() {
        let Some(key) = pad.key else { continue };
        let item = b.item(key);
        let Some(&n) = item.net_numbers().first() else {
            continue;
        };
        if !target.contains(&n) {
            continue;
        }
        let layers: Vec<LayerNo> = pad
            .layers
            .iter()
            .map(|name| b.layer_structure.get_no(name))
            .filter(|&idx| idx >= 0)
            .collect();
        map.entry(n).or_default().pads.push(PadCopper {
            pos: pad.point,
            layers,
        });
    }

    map
}

// ---------------------------------------------------------------------------------------------
// connectivity.find_connected_groups (layer-aware, via bridged)

/// Union-find grouping of segments: an endpoint pair joins two segments when it
/// coincides within `COINCIDENCE_TOL` on the same layer, or on different layers
/// with a via at the (rounded) join. Returns groups in segment insertion order.
fn find_connected_groups(segs: &[Seg], vias: &[P2]) -> Vec<Vec<Seg>> {
    let n = segs.len();
    if n == 0 {
        return Vec::new();
    }
    let via_positions: BTreeSet<(i64, i64)> = vias.iter().map(|&v| key3(v)).collect();

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
    let union = |parent: &mut Vec<usize>, a: usize, b: usize| {
        let (ra, rb) = (find(parent, a), find(parent, b));
        if ra != rb {
            parent[ra] = rb;
        }
    };

    for i in 0..n {
        for j in (i + 1)..n {
            let mut joined = false;
            for ea in [segs[i].a, segs[i].b] {
                for eb in [segs[j].a, segs[j].b] {
                    if (ea[0] - eb[0]).abs() < COINCIDENCE_TOL
                        && (ea[1] - eb[1]).abs() < COINCIDENCE_TOL
                    {
                        if segs[i].layer == segs[j].layer || via_positions.contains(&key3(ea)) {
                            joined = true;
                            break;
                        }
                    }
                }
                if joined {
                    break;
                }
            }
            if joined {
                union(&mut parent, i, j);
            }
        }
    }

    let mut index: BTreeMap<usize, usize> = BTreeMap::new();
    let mut groups: Vec<Vec<Seg>> = Vec::new();
    for i in 0..n {
        let r = find(&mut parent, i);
        match index.get(&r) {
            Some(&idx) => groups[idx].push(segs[i]),
            None => {
                index.insert(r, groups.len());
                groups.push(vec![segs[i]]);
            }
        }
    }
    groups
}

// ---------------------------------------------------------------------------------------------
// connectivity.find_stub_free_ends

/// Free ends of one stub group: endpoint clusters whose degree is one, minus any
/// endpoint sitting on a pad center. Returns `(x, y, layer)` sorted ascending;
/// coordinates rounded to `POSITION_DECIMALS`.
fn find_stub_free_ends(group: &[Seg], pad_positions: &[P2]) -> Vec<(f64, f64, LayerNo)> {
    if group.is_empty() {
        return Vec::new();
    }
    let mut points: Vec<(f64, f64, LayerNo)> = Vec::with_capacity(group.len() * 2);
    for s in group {
        points.push((s.a[0], s.a[1], s.layer));
        points.push((s.b[0], s.b[1], s.layer));
    }

    // Cluster coincidence, layer-aware (source `cluster_coincident_points`).
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
            if la == lb
                && (xa - xb).abs() < COINCIDENCE_TOL
                && (ya - yb).abs() < COINCIDENCE_TOL
            {
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
        let r = find(&mut parent, pi);
        match index.get(&r) {
            Some(&idx) => members[idx].push(pi),
            None => {
                index.insert(r, members.len());
                members.push(vec![pi]);
            }
        }
    }

    let near_pad = |x: f64, y: f64| {
        pad_positions
            .iter()
            .any(|p| (x - p[0]).abs() < PAD_ATTACH_TOL && (y - p[1]).abs() < PAD_ATTACH_TOL)
    };

    let mut free_ends: Vec<(f64, f64, LayerNo)> = Vec::new();
    for pis in &members {
        let mut seg_hits: BTreeMap<usize, i32> = BTreeMap::new();
        for &pi in pis {
            *seg_hits.entry(pi / 2).or_insert(0) += 1;
        }
        let open_segs: Vec<usize> = seg_hits
            .iter()
            .filter(|(_, &c)| c == 1)
            .map(|(&si, _)| si)
            .collect();

        if open_segs.len() == 1 {
            let si = open_segs[0];
            let seg = group[si];
            let in_cluster = *pis.iter().find(|&&pi| pi / 2 == si).unwrap();
            let far = if in_cluster % 2 == 0 { seg.b } else { seg.a };
            // max((x-far)^2 + (y-far)^2, x, y, layer): deterministic geometric tie-break.
            let mut best = points[pis[0]];
            let mut best_key = ((best.0 - far[0]).powi(2) + (best.1 - far[1]).powi(2), best.0, best.1, best.2);
            for &pi in &pis[1..] {
                let p = points[pi];
                let k = ((p.0 - far[0]).powi(2) + (p.1 - far[1]).powi(2), p.0, p.1, p.2);
                if k > best_key {
                    best_key = k;
                    best = p;
                }
            }
            if !near_pad(best.0, best.1) {
                free_ends.push((py_round3(best.0), py_round3(best.1), best.2));
            }
        } else if open_segs.is_empty() && members.len() == 1 {
            let mut seen = BTreeSet::new();
            for &pi in pis {
                let (x, y, layer) = points[pi];
                let key = (
                    round_half_even(x * POSITION_DECIMALS) as i64,
                    round_half_even(y * POSITION_DECIMALS) as i64,
                    layer,
                );
                if seen.insert(key) && !near_pad(x, y) {
                    free_ends.push((py_round3(x), py_round3(y), layer));
                }
            }
        }
    }

    free_ends.sort_by(|a, b| {
        cmp_f64(a.0, b.0)
            .then_with(|| cmp_f64(a.1, b.1))
            .then_with(|| a.2.cmp(&b.2))
    });
    free_ends
}

// ---------------------------------------------------------------------------------------------
// connectivity.compute_mst_segments / get_net_mst_segments

/// Prim's algorithm, exactly as `compute_mst_edges(use_manhattan=False)`: node 0
/// seeds the tree, ties pick the first index, distances are Euclidean.
fn compute_mst_edges(points: &[P2]) -> Vec<(usize, usize, f64)> {
    let n = points.len();
    if n < 2 {
        return Vec::new();
    }
    if n == 2 {
        return vec![(0, 1, sqrt_dist(points[0], points[1]))];
    }
    let mut in_tree = vec![false; n];
    let mut nearest = vec![0usize; n];
    let mut min_dist = (0..n).map(|j| sqrt_dist(points[0], points[j])).collect::<Vec<_>>();
    in_tree[0] = true;
    min_dist[0] = f64::INFINITY;
    let mut edges = Vec::with_capacity(n - 1);

    for _ in 0..n - 1 {
        let mut best_j: Option<usize> = None;
        let mut best_dist = f64::INFINITY;
        for j in 0..n {
            if in_tree[j] {
                continue;
            }
            if best_j.is_none() || min_dist[j] < best_dist {
                best_j = Some(j);
                best_dist = min_dist[j];
            }
        }
        let Some(best_j) = best_j else { break };
        if !best_dist.is_finite() {
            break;
        }
        in_tree[best_j] = true;
        edges.push((nearest[best_j], best_j, best_dist));
        for j in 0..n {
            if !in_tree[j] {
                let d = sqrt_dist(points[best_j], points[j]);
                if d < min_dist[j] {
                    min_dist[j] = d;
                    nearest[j] = best_j;
                }
            }
        }
    }
    edges
}

fn compute_mst_segments(points: &[P2]) -> Vec<(P2, P2)> {
    compute_mst_edges(points)
        .into_iter()
        .map(|(a, b, _)| (points[a], points[b]))
        .collect()
}

/// `connectivity.get_net_mst_segments`.
fn net_mst_segments(copper: &NetCopper) -> Result<Vec<(P2, P2)>, String> {
    let pad_positions: Vec<P2> = copper.pads.iter().map(|p| p.pos).collect();
    if !copper.segs.is_empty() {
        let groups = find_connected_groups(&copper.segs, &copper.vias);
        if groups.len() >= 2 {
            let mut points: Vec<P2> = Vec::new();
            for group in &groups {
                let free = find_stub_free_ends(group, &pad_positions);
                if !free.is_empty() {
                    points.push([free[0].0, free[0].1]);
                } else {
                    let mut pts: Vec<P2> = Vec::with_capacity(group.len() * 2);
                    for s in group {
                        pts.push(s.a);
                        pts.push(s.b);
                    }
                    points.push(centroid(&pts)?);
                }
            }
            return Ok(compute_mst_segments(&points));
        }
    }
    if copper.pads.len() >= 2 {
        let points: Vec<P2> = copper.pads.iter().map(|p| p.pos).collect();
        return Ok(compute_mst_segments(&points));
    }
    Ok(Vec::new())
}

// ---------------------------------------------------------------------------------------------
// connectivity.get_net_routing_endpoints

/// `connectivity.get_net_routing_endpoints`: up to two representative points.
fn net_routing_endpoints(copper: &NetCopper) -> Result<Vec<P2>, String> {
    let pad_positions: Vec<P2> = copper.pads.iter().map(|p| p.pos).collect();

    // Case 1: two or more stub groups -> rank by copper, use their centroids.
    if copper.segs.len() >= 2 {
        let groups = find_connected_groups(&copper.segs, &copper.vias);
        if groups.len() >= 2 {
            let order = ranked_order(&groups)?;
            let mut centroids: Vec<P2> = Vec::new();
            for &gi in &order {
                let group = &groups[gi];
                let mut pts: Vec<P2> = Vec::with_capacity(group.len() * 2);
                for s in group {
                    pts.push(s.a);
                    pts.push(s.b);
                }
                if !pts.is_empty() {
                    centroids.push(centroid(&pts)?);
                }
            }
            return Ok(centroids.into_iter().take(2).collect());
        }
    }

    // Case 2: one stub group + pads -> stub centroid and unconnected-pad centroid.
    if !copper.segs.is_empty() && !copper.pads.is_empty() {
        let groups = find_connected_groups(&copper.segs, &copper.vias);
        if groups.len() == 1 {
            let group = &groups[0];
            let mut pts: Vec<P2> = Vec::with_capacity(group.len() * 2);
            for s in group {
                pts.push(s.a);
                pts.push(s.b);
            }
            // `seg_points` uses rounded positions, then a 0.05 pad-attach test.
            let seg_points: Vec<(f64, f64)> = pts
                .iter()
                .map(|p| (py_round3(p[0]), py_round3(p[1])))
                .collect();
            let unconnected: Vec<P2> = copper
                .pads
                .iter()
                .filter(|pad| {
                    let px = py_round3(pad.pos[0]);
                    let py = py_round3(pad.pos[1]);
                    !seg_points
                        .iter()
                        .any(|sp| (px - sp.0).abs() < PAD_ATTACH_TOL && (py - sp.1).abs() < PAD_ATTACH_TOL)
                })
                .map(|pad| pad.pos)
                .collect();
            if !unconnected.is_empty() {
                let stub = centroid(&pts)?;
                let pad = centroid(&unconnected)?;
                return Ok(vec![stub, pad]);
            }
        }
    }

    // Case 3: no stubs, just pads -> first pad and centroid of the rest.
    if copper.segs.is_empty() && copper.pads.len() >= 2 {
        let first = copper.pads[0].pos;
        let other: Vec<P2> = copper.pads[1..].iter().map(|p| p.pos).collect();
        return Ok(vec![first, centroid(&other)?]);
    }

    Ok(Vec::new())
}

// ---------------------------------------------------------------------------------------------
// _compute_mps_unit_layers

/// `net_queries._compute_mps_unit_layers` for a single net: layer sets of the two
/// main stub groups, falling back to the pads' native signal layers.
fn unit_layer_sets(copper: &NetCopper) -> Result<(BTreeSet<LayerNo>, BTreeSet<LayerNo>), String> {
    let mut src: BTreeSet<LayerNo> = BTreeSet::new();
    let mut tgt: BTreeSet<LayerNo> = BTreeSet::new();

    if !copper.segs.is_empty() {
        let groups = find_connected_groups(&copper.segs, &copper.vias);
        if groups.len() >= 2 {
            let order = ranked_order(&groups)?;
            for s in &groups[order[0]] {
                src.insert(s.layer);
            }
            for s in &groups[order[1]] {
                tgt.insert(s.layer);
            }
        }
    }

    if src.is_empty() && tgt.is_empty() {
        for pad in &copper.pads {
            for &layer in &pad.layers {
                src.insert(layer);
                tgt.insert(layer);
            }
        }
    }

    Ok((src, tgt))
}

// ---------------------------------------------------------------------------------------------
// Group ranking (source `_grp_rank` / `_sg_rank`)

fn group_rank(group: &[Seg]) -> Result<(usize, f64, (P2, P2, LayerNo)), String> {
    let total = numeric::fsum(group.iter().map(|s| hypot_len(s.a, s.b)))?;
    let mut anchor: Option<(P2, P2, LayerNo)> = None;
    for s in group {
        let (mn, mx) = if cmp_p2(s.a, s.b) == Ordering::Greater {
            (s.b, s.a)
        } else {
            (s.a, s.b)
        };
        let cand = (mn, mx, s.layer);
        anchor = match anchor {
            None => Some(cand),
            Some(cur) => {
                if cmp_anchor(&cand, &cur) == Ordering::Less {
                    Some(cand)
                } else {
                    Some(cur)
                }
            }
        };
    }
    let anchor = anchor.ok_or("empty segment group in MPS group ranking")?;
    Ok((group.len(), total, anchor))
}

/// `(-len, -total, anchor)` ascending, matching Python's `sorted(key=_grp_rank)`.
fn cmp_rank(
    a: &(usize, f64, (P2, P2, LayerNo)),
    b: &(usize, f64, (P2, P2, LayerNo)),
) -> Ordering {
    (-(a.0 as i64))
        .cmp(&-(b.0 as i64))
        .then_with(|| cmp_f64(-a.1, -b.1))
        .then_with(|| cmp_anchor(&a.2, &b.2))
}

/// Group indices ordered by `_grp_rank`, computing each rank once (Python's
/// `sorted(key=...)`), so a summation overflow propagates instead of panicking.
fn ranked_order(groups: &[Vec<Seg>]) -> Result<Vec<usize>, String> {
    let mut keyed: Vec<(usize, (usize, f64, (P2, P2, LayerNo)))> =
        Vec::with_capacity(groups.len());
    for (index, group) in groups.iter().enumerate() {
        keyed.push((index, group_rank(group)?));
    }
    keyed.sort_by(|a, b| cmp_rank(&a.1, &b.1));
    Ok(keyed.into_iter().map(|(index, _)| index).collect())
}

// ---------------------------------------------------------------------------------------------
// connectivity.segments_intersect

fn segments_intersect(a1: P2, a2: P2, b1: P2, b2: P2) -> bool {
    let ccw = |a: P2, b: P2, c: P2| -> bool {
        (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
    };
    const EPS: f64 = 0.001;
    for p1 in [a1, a2] {
        for p2 in [b1, b2] {
            if (p1[0] - p2[0]).abs() < EPS && (p1[1] - p2[1]).abs() < EPS {
                return false;
            }
        }
    }
    (ccw(a1, b1, b2) != ccw(a2, b1, b2)) && (ccw(a1, a2, b1) != ccw(a1, a2, b2))
}

// ---------------------------------------------------------------------------------------------
// _greedy_order_mps_units

fn greedy_order(
    unit_list: &[NetNo],
    conflicts: &BTreeMap<NetNo, BTreeSet<NetNo>>,
    distances: &BTreeMap<NetNo, f64>,
) -> Vec<NetNo> {
    let dist_of = |uid: NetNo| *distances.get(&uid).unwrap_or(&0.0);

    let mut all_rounds: Vec<(Vec<NetNo>, i32)> = Vec::new();
    let mut remaining: BTreeSet<NetNo> = unit_list.iter().copied().collect();
    let mut round_num = 0;

    while !remaining.is_empty() {
        round_num += 1;
        let mut round_winners: Vec<NetNo> = Vec::new();
        let mut round_losers: BTreeSet<NetNo> = BTreeSet::new();
        let mut round_remaining: BTreeSet<NetNo> = remaining.clone();

        while !round_remaining.is_empty() {
            // min by (active conflicts, distance, uid).
            let mut best: Option<NetNo> = None;
            let mut best_key: Option<(usize, f64, NetNo)> = None;
            for &uid in &round_remaining {
                let active = conflicts
                    .get(&uid)
                    .map(|s| s.intersection(&round_remaining).count())
                    .unwrap_or(0);
                let key = (active, dist_of(uid), uid);
                let better = match best_key {
                    None => true,
                    Some(k) => {
                        key.0 < k.0
                            || (key.0 == k.0
                                && (cmp_f64(key.1, k.1) == Ordering::Less
                                    || (cmp_f64(key.1, k.1) == Ordering::Equal && key.2 < k.2)))
                    }
                };
                if better {
                    best_key = Some(key);
                    best = Some(uid);
                }
            }
            let Some(best) = best else { break };
            round_winners.push(best);
            round_remaining.remove(&best);

            let losers: Vec<NetNo> = conflicts
                .get(&best)
                .map(|s| s.intersection(&round_remaining).copied().collect())
                .unwrap_or_default();
            for loser in losers {
                round_losers.insert(loser);
                round_remaining.remove(&loser);
            }
        }

        all_rounds.push((round_winners, round_num));
        remaining = round_losers;
    }

    if REVERSE_ROUNDS {
        all_rounds.reverse();
    }

    let mut ordered: Vec<NetNo> = Vec::new();
    for (mut winners, _rn) in all_rounds {
        winners.sort_by(|a, b| {
            cmp_f64(dist_of(*a), dist_of(*b)).then_with(|| a.cmp(b))
        });
        ordered.extend(winners);
    }
    ordered
}

// ---------------------------------------------------------------------------------------------
// compute_mps_net_ordering (driver)

/// Original MPS net ordering for the endpoint front.
///
/// Mirrors `net_queries.compute_mps_net_ordering` with the source defaults:
/// `diff_pairs=None` (one unit per net), `center=None`,
/// `use_boundary_ordering=True` (see module docs — chip metadata is absent so the
/// no-chip method is taken), `bga_exclusion_zones=None`, `reverse_rounds=False`,
/// `crossing_layer_check=True`, `use_segment_intersection=None` (auto).
pub(crate) fn order_native_nets_mps(
    board: &BoardAdapter,
    nets: &[(NetNo, String)],
) -> Result<Vec<(NetNo, String)>, String> {
    if nets.is_empty() {
        return Ok(Vec::new());
    }

    // Names keyed by net id (input order; first occurrence wins).
    let mut names: BTreeMap<NetNo, String> = BTreeMap::new();
    for (no, name) in nets {
        names.entry(*no).or_insert_with(|| name.clone());
    }

    // Step 1: units = unique input nets in order (`_build_mps_unit_mappings`).
    let mut unit_ids: Vec<NetNo> = Vec::new();
    let mut seen_units: BTreeSet<NetNo> = BTreeSet::new();
    for (no, _) in nets {
        if seen_units.insert(*no) {
            unit_ids.push(*no);
        }
    }

    let target: BTreeSet<NetNo> = unit_ids.iter().copied().collect();
    let copper = gather_copper(board, &target);

    let empty = NetCopper::default();
    let copper_of = |n: NetNo| copper.get(&n).unwrap_or(&empty);

    // Step 2: per-unit endpoints.
    let mut unit_endpoints: BTreeMap<NetNo, Vec<P2>> = BTreeMap::new();
    let mut endpoint_order: Vec<NetNo> = Vec::new();
    for &uid in &unit_ids {
        let endpoints = net_routing_endpoints(copper_of(uid))?;
        if endpoints.len() >= 2 {
            unit_endpoints.insert(uid, endpoints.into_iter().take(2).collect());
            endpoint_order.push(uid);
        }
    }
    if unit_endpoints.is_empty() {
        return Ok(nets.to_vec());
    }

    // Step 3: center = mean of all endpoints.
    let mut all_points: Vec<P2> = Vec::new();
    for eps in unit_endpoints.values() {
        all_points.extend_from_slice(eps);
    }
    let center = centroid(&all_points)?;

    // Step 4: MST segments per unit (crossing method).
    let mut unit_mst: BTreeMap<NetNo, Vec<(P2, P2)>> = BTreeMap::new();
    for &uid in &unit_ids {
        let segs = net_mst_segments(copper_of(uid))?;
        if !segs.is_empty() {
            unit_mst.insert(uid, segs);
        }
    }

    if USE_BOUNDARY_ORDERING {
        return Err(
            "Unsupported: MPS chip/BGA boundary ordering requires native footprint/BGA \
             partition metadata not exposed by BoardAdapter"
                .into(),
        );
    }

    // Auto-detect (`use_segment_intersection=None`): with no BGA zones this is
    // `len(unit_mst_segments) > 0`.
    let use_segment_intersection = !unit_mst.is_empty();

    // Angles: fallback method (source computes these whenever boundary info is
    // absent, which is always here).
    let angle_from_center = |p: P2| -> f64 {
        let mut ang = (p[1] - center[1]).atan2(p[0] - center[0]);
        if ang < 0.0 {
            ang += 2.0 * std::f64::consts::PI;
        }
        ang
    };
    let mut unit_angles: BTreeMap<NetNo, (f64, f64)> = BTreeMap::new();
    for (&uid, eps) in &unit_endpoints {
        let mut a1 = angle_from_center(eps[0]);
        let mut a2 = angle_from_center(eps[1]);
        if a1 > a2 {
            std::mem::swap(&mut a1, &mut a2);
        }
        unit_angles.insert(uid, (a1, a2));
    }

    // Step 5: layer sets.
    let mut unit_layers: BTreeMap<NetNo, (BTreeSet<LayerNo>, BTreeSet<LayerNo>)> = BTreeMap::new();
    for &uid in &unit_ids {
        unit_layers.insert(uid, unit_layer_sets(copper_of(uid))?);
    }

    // Step 6: conflicts. Both source graphs are built; `geometric` (all crossings)
    // is only surfaced by `return_extended_info`, which this API does not request,
    // while `conflicts` (layer-filtered) is the graph the greedy order uses.
    let unit_list: Vec<NetNo> = endpoint_order.clone();
    let mut geometric: BTreeMap<NetNo, BTreeSet<NetNo>> =
        unit_list.iter().map(|&u| (u, BTreeSet::new())).collect();
    let mut conflicts: BTreeMap<NetNo, BTreeSet<NetNo>> =
        unit_list.iter().map(|&u| (u, BTreeSet::new())).collect();

    let units_cross = |a: NetNo, b: NetNo| -> bool {
        if use_segment_intersection {
            let empty_segs: Vec<(P2, P2)> = Vec::new();
            let segs_a = unit_mst.get(&a).unwrap_or(&empty_segs);
            let segs_b = unit_mst.get(&b).unwrap_or(&empty_segs);
            for sa in segs_a {
                for sb in segs_b {
                    if segments_intersect(sa.0, sa.1, sb.0, sb.1) {
                        return true;
                    }
                }
            }
            false
        } else if let (Some(&(a1, a2)), Some(&(b1, b2))) = (unit_angles.get(&a), unit_angles.get(&b))
        {
            (a1 < b1 && b1 < a2 && a2 < b2) || (b1 < a1 && a1 < b2 && b2 < a2)
        } else {
            false
        }
    };
    let units_share_layer = |a: NetNo, b: NetNo| -> bool {
        let empty_layers = (BTreeSet::<LayerNo>::new(), BTreeSet::<LayerNo>::new());
        let (a_src, a_tgt) = unit_layers.get(&a).unwrap_or(&empty_layers);
        let (b_src, b_tgt) = unit_layers.get(&b).unwrap_or(&empty_layers);
        let a_all: BTreeSet<LayerNo> = a_src.union(a_tgt).copied().collect();
        let b_all: BTreeSet<LayerNo> = b_src.union(b_tgt).copied().collect();
        if a_all.is_empty() || b_all.is_empty() {
            return true;
        }
        a_all.intersection(&b_all).next().is_some()
    };

    for i in 0..unit_list.len() {
        for j in (i + 1)..unit_list.len() {
            let a = unit_list[i];
            let b = unit_list[j];
            if units_cross(a, b) {
                geometric.get_mut(&a).unwrap().insert(b);
                geometric.get_mut(&b).unwrap().insert(a);
                // `crossing_layer_check` gates the layer-filtered graph.
                let layer_ok = if CROSSING_LAYER_CHECK {
                    units_share_layer(a, b)
                } else {
                    true
                };
                if layer_ok {
                    conflicts.get_mut(&a).unwrap().insert(b);
                    conflicts.get_mut(&b).unwrap().insert(a);
                }
            }
        }
    }

    // Step 7: distances (no BGA zones -> endpoint distance).
    let mut distances: BTreeMap<NetNo, f64> = BTreeMap::new();
    for &uid in &unit_list {
        if let Some(eps) = unit_endpoints.get(&uid) {
            distances.insert(uid, sqrt_dist(eps[1], eps[0]));
        } else {
            distances.insert(uid, f64::INFINITY);
        }
    }

    // Step 8: greedy ordering.
    let ordered_units = greedy_order(&unit_list, &conflicts, &distances);

    // Expand units back to nets, then append nets without endpoints in input order.
    let mut ordered: Vec<NetNo> = Vec::new();
    let mut ordered_set: BTreeSet<NetNo> = BTreeSet::new();
    for uid in ordered_units {
        ordered.push(uid);
        ordered_set.insert(uid);
    }
    for (no, _) in nets {
        if !ordered_set.contains(no) {
            ordered.push(*no);
        }
    }

    let result: Vec<(NetNo, String)> = ordered
        .into_iter()
        .map(|no| {
            let name = names
                .get(&no)
                .cloned()
                .or_else(|| board.board.rules.nets.get(no).map(|n| n.name.clone()))
                .unwrap_or_default();
            (no, name)
        })
        .collect();
    Ok(result)
}
