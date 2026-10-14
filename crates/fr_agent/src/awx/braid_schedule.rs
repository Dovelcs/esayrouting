//! Typed port of AWX `schedule.py` at caad45cd252f9a50f1fb4b3b15461bdf1b9457a1.
//! Geometry supplies launch/target order; this module assigns page layers and
//! exposes the schedule relations used by braid plan-only and routing.
use super::{
    cp_model::{BoolVar, CpSolverStatus, Model, SatParameters},
    cp_sat_bridge,
};
use std::collections::{BTreeMap, BTreeSet};

const PAGES_SCALE: f64 = 1_000_000.0;
const SWIM_TIE: f64 = 0.01;

#[derive(Clone, Debug)]
pub struct ScheduleInput {
    pub launch: Vec<String>,
    pub target: Vec<String>,
    pub tooth_layer: BTreeMap<String, String>,
    pub dest_layer: BTreeMap<String, String>,
    /// Corresponds to BRAID_EXACT_PAGES. When requested, use CP-SAT; if no
    /// assignment is returned, preserve schedule.py's greedy fallback.
    pub exact_pages: bool,
    /// `prices.PAGES` (default 2.0), used by the exact assignment objective.
    pub swimmer_page_price: f64,
}

#[derive(Clone, Debug)]
pub struct BraidSchedule {
    pub launch: Vec<String>,
    pub target: Vec<String>,
    pub trank: BTreeMap<String, usize>,
    pub lidx: BTreeMap<String, usize>,
    pub tooth_layer: BTreeMap<String, String>,
    pub page: BTreeMap<String, Option<String>>,
    pub b_page: Vec<String>,
    pub swimmers: Vec<String>,
    pub divers: BTreeSet<String>,
    pub birth_b: BTreeSet<String>,
    /// Swimmers first, followed by B-page divers, each in serial-pass order.
    pub priority: Vec<String>,
    /// Serial pass result. It is equal to `target` for a valid schedule.
    pub serial_order: Vec<String>,
    /// True only if the exact CP-SAT path supplied the assignment.
    pub exact_pages_used: bool,
}

impl BraidSchedule {
    /// Source Schedule.inverted: a pair crosses iff the two orders disagree.
    pub fn inverted(&self, a: &str, b: &str) -> Result<bool, String> {
        let ia = *self
            .lidx
            .get(a)
            .ok_or_else(|| format!("unknown scheduled lane {a}"))?;
        let ib = *self
            .lidx
            .get(b)
            .ok_or_else(|| format!("unknown scheduled lane {b}"))?;
        let ta = *self
            .trank
            .get(a)
            .ok_or_else(|| format!("unknown target lane {a}"))?;
        let tb = *self
            .trank
            .get(b)
            .ok_or_else(|| format!("unknown target lane {b}"))?;
        Ok((ia < ib) != (ta < tb))
    }

    pub fn page_of(&self, net: &str) -> Option<&str> {
        self.page.get(net).and_then(Option::as_deref)
    }
}

pub fn build_schedule(input: ScheduleInput) -> Result<BraidSchedule, String> {
    let mut counts = BTreeMap::<&str, i64>::new();
    for net in &input.launch {
        *counts.entry(net).or_default() += 1;
    }
    for net in &input.target {
        *counts.entry(net).or_default() -= 1;
    }
    if input.launch.len() != input.target.len() || counts.values().any(|&n| n != 0) {
        return Err("schedule launch and target must contain the same lanes".into());
    }
    if !input.swimmer_page_price.is_finite() || input.swimmer_page_price < 0.0 {
        return Err("schedule swimmer page price must be finite and nonnegative".into());
    }
    let n = input.launch.len();
    let lidx: BTreeMap<_, _> = input
        .launch
        .iter()
        .enumerate()
        .map(|(i, s)| (s.clone(), i))
        .collect();
    let trank: BTreeMap<_, _> = input
        .target
        .iter()
        .enumerate()
        .map(|(i, s)| (s.clone(), i))
        .collect();
    if lidx.len() != n || trank.len() != n {
        return Err("schedule launch and target lanes must be unique".into());
    }
    let tooth_layer: BTreeMap<_, _> = input
        .launch
        .iter()
        .map(|net| {
            (
                net.clone(),
                input
                    .tooth_layer
                    .get(net)
                    .cloned()
                    .unwrap_or_else(|| "F.Cu".into()),
            )
        })
        .collect();
    let dest_layer: BTreeMap<_, _> = input
        .launch
        .iter()
        .map(|net| {
            (
                net.clone(),
                input
                    .dest_layer
                    .get(net)
                    .cloned()
                    .unwrap_or_else(|| "F.Cu".into()),
            )
        })
        .collect();

    let exact = if input.exact_pages {
        exact_pages(
            &input.launch,
            &input.target,
            &trank,
            &tooth_layer,
            &dest_layer,
            input.swimmer_page_price,
        )?
    } else {
        None
    };
    let exact_pages_used = exact.is_some();
    let mut page =
        exact.unwrap_or_else(|| greedy_pages(&input.launch, &trank, &tooth_layer, &dest_layer));

    // Rebuild page entries for every lane; the greedy/exact routines keep
    // source order stable and page=None represents a swimmer.
    for net in &input.launch {
        page.entry(net.clone()).or_insert(None);
    }
    let b_page: Vec<_> = input
        .launch
        .iter()
        .filter(|n| page.get(*n).and_then(Option::as_deref) == Some("B.Cu"))
        .cloned()
        .collect();
    let swimmers: Vec<_> = input
        .launch
        .iter()
        .filter(|n| page.get(*n).is_some_and(Option::is_none))
        .cloned()
        .collect();
    let divers: BTreeSet<_> = input
        .launch
        .iter()
        .filter(|n| page.get(*n).and_then(Option::as_deref) != Some("F.Cu"))
        .cloned()
        .collect();
    let birth_b: BTreeSet<_> = divers
        .iter()
        .filter(|n| tooth_layer.get(*n).is_some_and(|l| l == "B.Cu"))
        .cloned()
        .collect();

    let mut ups: Vec<_> = divers
        .iter()
        .filter(|n| trank[*n] < lidx[*n])
        .cloned()
        .collect();
    ups.sort_by_key(|n| trank[n]);
    let mut downs: Vec<_> = divers
        .iter()
        .filter(|n| trank[*n] >= lidx[*n])
        .cloned()
        .collect();
    downs.sort_by_key(|n| std::cmp::Reverse(trank[n]));
    let mut priority: Vec<_> = ups
        .iter()
        .chain(&downs)
        .filter(|n| page.get(*n).is_some_and(Option::is_none))
        .cloned()
        .collect();
    priority.extend(
        ups.iter()
            .chain(&downs)
            .filter(|n| page.get(*n).and_then(Option::as_deref) == Some("B.Cu"))
            .cloned(),
    );

    let mut serial_order = input.launch.clone();
    let mut placed: BTreeSet<_> = input
        .launch
        .iter()
        .filter(|n| !divers.contains(*n))
        .cloned()
        .collect();
    for net in &priority {
        let index = serial_order
            .iter()
            .position(|n| n == net)
            .ok_or("schedule diver missing from serial order")?;
        let mut rest = serial_order.clone();
        rest.remove(index);
        let mut want = 0;
        for (j, other) in rest.iter().enumerate() {
            if placed.contains(other) && trank[other] < trank[net] {
                want = j + 1;
            }
        }
        rest.insert(want, net.clone());
        serial_order = rest;
        placed.insert(net.clone());
    }
    if serial_order != input.target {
        return Err(format!(
            "schedule serial pass does not reach target order: {serial_order:?} != {:?}",
            input.target
        ));
    }
    Ok(BraidSchedule {
        launch: input.launch,
        target: input.target,
        trank,
        lidx,
        tooth_layer,
        page,
        b_page,
        swimmers,
        divers,
        birth_b,
        priority,
        serial_order,
        exact_pages_used,
    })
}

fn greedy_pages(
    launch: &[String],
    trank: &BTreeMap<String, usize>,
    tooth_layer: &BTreeMap<String, String>,
    dest_layer: &BTreeMap<String, String>,
) -> BTreeMap<String, Option<String>> {
    let ranks: Vec<_> = launch.iter().map(|n| trank[n]).collect();
    let mut page: BTreeMap<String, Option<String>> =
        launch.iter().map(|n| (n.clone(), None)).collect();
    for layer in ["F.Cu", "B.Cu"] {
        let born: Vec<_> = launch
            .iter()
            .enumerate()
            .filter_map(|(i, n)| (tooth_layer[n] == layer).then_some(i))
            .collect();
        if born.is_empty() {
            continue;
        }
        let subranks: Vec<_> = born.iter().map(|&i| ranks[i]).collect();
        let weights: Vec<_> = born
            .iter()
            .map(|&i| {
                let n = &launch[i];
                bool_cost(tooth_layer[n] == layer) + bool_cost(dest_layer[n] == layer) - 1.0
            })
            .collect();
        for i in lis_keep_weighted(&subranks, &weights) {
            page.insert(launch[born[i]].clone(), Some(layer.to_string()));
        }
    }
    let mut rest: Vec<_> = launch
        .iter()
        .filter(|n| page.get(*n).is_some_and(Option::is_none))
        .cloned()
        .collect();
    let inv_count = |n: &str| -> usize {
        let ni = launch.iter().position(|x| x == n).unwrap();
        launch
            .iter()
            .filter(|other| *other != n)
            .filter(|other| {
                let oi = launch.iter().position(|x| x == *other).unwrap();
                (ni < oi) != (trank[n] < trank[other.as_str()])
            })
            .count()
    };
    // Stable sort preserves launch order for equal inversion counts.
    rest.sort_by_key(|n| std::cmp::Reverse(inv_count(n)));
    for n in rest {
        let own = tooth_layer.get(&n).map(String::as_str).unwrap_or("F.Cu");
        let other = if own == "F.Cu" { "B.Cu" } else { "F.Cu" };
        for layer in [own, other] {
            let conflict = launch
                .iter()
                .filter(|o| page.get(*o).and_then(Option::as_deref) == Some(layer))
                .any(|o| inverted(n.as_str(), o, launch, trank));
            if !conflict {
                page.insert(n.clone(), Some(layer.to_string()));
                break;
            }
        }
    }
    page
}

fn exact_pages(
    launch: &[String],
    target: &[String],
    trank: &BTreeMap<String, usize>,
    tooth_layer: &BTreeMap<String, String>,
    dest_layer: &BTreeMap<String, String>,
    swimmer_price: f64,
) -> Result<Option<BTreeMap<String, Option<String>>>, String> {
    if launch.is_empty() {
        return Ok(Some(BTreeMap::new()));
    }
    let n = launch.len();
    let inv: Vec<_> = (0..n)
        .flat_map(|i| {
            ((i + 1)..n)
                .filter_map(move |j| (trank[&launch[j]] < trank[&launch[i]]).then_some((i, j)))
        })
        .collect();
    let mut model = Model::new();
    let x: Vec<[BoolVar; 2]> = launch
        .iter()
        .map(|net| {
            [
                model.bool(&format!("page_f_{net}")),
                model.bool(&format!("page_b_{net}")),
            ]
        })
        .collect();
    for vars in &x {
        model.le(
            &[(vars[0].as_int_var(), 1), (vars[1].as_int_var(), 1)],
            1,
            &[],
        )?;
    }
    for (i, j) in inv {
        for p in 0..2 {
            model.le(
                &[(x[i][p].as_int_var(), 1), (x[j][p].as_int_var(), 1)],
                1,
                &[],
            )?;
        }
    }
    let scale = 1_000_000.0;
    let mut objective = Vec::with_capacity(2 * n);
    for (i, net) in launch.iter().enumerate() {
        for (p, layer) in ["F.Cu", "B.Cu"].iter().enumerate() {
            let on = bool_cost(tooth_layer[net] == *layer) + bool_cost(dest_layer[net] == *layer);
            let coefficient = ((-(swimmer_price + SWIM_TIE) + 2.0 - on) * scale).round();
            if !coefficient.is_finite() || coefficient.abs() > i64::MAX as f64 {
                return Err("exact schedule page objective coefficient overflow".into());
            }
            objective.push((x[i][p].as_int_var(), coefficient as i64));
        }
    }
    model.minimize(&objective);
    let params = SatParameters {
        num_workers: Some(1),
        ..Default::default()
    };
    let response = cp_sat_bridge::solve(&model.inner, &params, None)?;
    if !matches!(
        response.status(),
        CpSolverStatus::Feasible | CpSolverStatus::Optimal
    ) {
        return Ok(None);
    }
    let mut result = BTreeMap::new();
    for (i, net) in launch.iter().enumerate() {
        let mut selected = None;
        for (p, layer) in ["F.Cu", "B.Cu"].iter().enumerate() {
            let value = response
                .solution
                .get(x[i][p].index() as usize)
                .copied()
                .ok_or_else(|| format!("exact schedule response lacks lane {net}"))?;
            if value > 0 {
                selected = Some((*layer).to_string());
                break;
            }
        }
        result.insert(net.clone(), selected);
    }
    // Keep target in this seam so malformed duplicate/order inputs cannot be
    // accidentally accepted if called independently of build_schedule.
    if target.len() != launch.len() {
        return Err("exact schedule target length mismatch".into());
    }
    Ok(Some(result))
}

fn lis_keep_weighted(ranks: &[usize], weights: &[f64]) -> Vec<usize> {
    let n = ranks.len();
    if n == 0 {
        return vec![];
    }
    let mut best: Vec<_> = weights.iter().map(|w| (1usize, 1.0 + 0.5 * *w)).collect();
    let mut prev = vec![None; n];
    for i in 0..n {
        for j in 0..i {
            if ranks[j] < ranks[i] {
                let cand = (best[j].0 + 1, best[j].1 + 1.0 + 0.5 * weights[i]);
                if cand.0 > best[i].0 || (cand.0 == best[i].0 && cand.1 > best[i].1) {
                    best[i] = cand;
                    prev[i] = Some(j);
                }
            }
        }
    }
    let mut at = (0..n)
        .max_by(|&a, &b| {
            best[a]
                .0
                .cmp(&best[b].0)
                .then_with(|| best[a].1.total_cmp(&best[b].1))
                .then_with(|| b.cmp(&a))
        })
        .unwrap();
    let mut out = Vec::new();
    loop {
        out.push(at);
        match prev[at] {
            Some(p) => at = p,
            None => break,
        }
    }
    out.reverse();
    out
}

fn inverted(a: &str, b: &str, launch: &[String], trank: &BTreeMap<String, usize>) -> bool {
    let ia = launch.iter().position(|n| n == a).unwrap();
    let ib = launch.iter().position(|n| n == b).unwrap();
    (ia < ib) != (trank[a] < trank[b])
}

fn bool_cost(value: bool) -> f64 {
    if value {
        1.0
    } else {
        0.0
    }
}
