//! Complete joint_escape core model at AWX caad45cd (MIT): menus, served tiers and exact run colouring.
use super::{
    conflict_groups::{self as cg, GroupKey, Member, Tag},
    cp_model::{BoolVar, CpSolverResponse, CpSolverStatus, IntVar, Model, SatParameters},
    escape_moves::{self as em, Direction, Grid, Leg, Move, MoveKind, Pad, Point, RUN},
    joint_geometry::{
        self as geo, Drop, JointGeometry, JointSizes, NetLeg, Piece, PieceVia, Pocket, Strap,
        ZoneRegion,
    },
};
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
use std::time::Instant;
pub const W_BUS: i64 = 1_000_000;
pub const W_OTHER: i64 = 10_000;
pub const W_DROP: i64 = 10_000;
pub const C_VIA: i64 = 300;
pub const C_MM: i64 = 20;
pub const C_DEV_MM: i64 = 1000;
pub const C_DEV_KIND: i64 = 3000;
pub const C_VIP: i64 = 300;
pub const C_FINE_VIA: i64 = 300;
pub const C_PLANE_LAYER: i64 = 3000;
pub const C_LANE_FACE: i64 = 400;
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointPair {
    pub base: String,
    pub p: String,
    pub n: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct PreferredTooth {
    pub tooth: Point,
    pub direction: Option<Direction>,
    pub layer: Option<String>,
    pub kind: Option<MoveKind>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct AskedHand {
    pub hand: i32,
    pub arriving: bool,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointInput {
    pub reference: String,
    pub pads: Vec<Pad>,
    pub bus: BTreeSet<String>,
    pub others: BTreeSet<String>,
    pub drops: BTreeSet<String>,
    pub other_layers: Vec<String>,
    pub routing_layers: Vec<String>,
    pub plane_layers: BTreeSet<String>,
    pub far: Option<Direction>,
    pub preferred: BTreeMap<String, PreferredTooth>,
    pub pairs: Vec<JointPair>,
    pub hands: BTreeMap<String, AskedHand>,
    pub bus_groups: BTreeMap<String, String>,
    pub sizes: JointSizes,
    pub segments: Vec<NetLeg>,
    pub zones: Vec<ZoneRegion>,
    pub pockets: Vec<Pocket>,
    pub only: Option<BTreeSet<String>>,
    pub vias_only: bool,
    pub climb: Option<u32>,
    pub climb_other: u32,
    pub street: u32,
    pub solve_batches: i32,
    pub solve_workers: i32,
    pub phase1_deterministic_time: f64,
    pub time_limit: Option<f64>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub enum OptionMove {
    Escape { movement: Move },
    Strap { strap: Strap },
    Drop { drop: Drop },
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointOption {
    pub option: OptionMove,
    pub piece: Piece,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointMenus {
    pub grid: Grid,
    pub items: BTreeMap<String, Pad>,
    pub menus: BTreeMap<String, Vec<Move>>,
    pub options: BTreeMap<String, Vec<JointOption>>,
    pub straps: BTreeMap<String, Vec<Strap>>,
    pub pairs: Vec<JointPair>,
    pub via_radius: BTreeMap<Member, f64>,
    pub lane_faces: BTreeSet<(i32, i32)>,
    pub menu_seconds: f64,
}
impl JointMenus {
    /// Resolve the source's per-net menu view without changing physical menu
    /// keys. A short-name collision is an error; multiple pads on the same
    /// fully-qualified net remain separate physical entries.
    pub fn physical_keys_for_net(&self, net: &str) -> Result<Vec<&str>, String> {
        let matching: Vec<_> = self
            .items
            .iter()
            .filter(|(_, pad)| em::short_name(&pad.net) == net)
            .collect();
        let full_names: BTreeSet<_> = matching.iter().map(|(_, pad)| pad.net.as_str()).collect();
        if full_names.len() > 1 {
            return Err(format!("AWX short net name collision for {net}"));
        }
        Ok(matching.into_iter().map(|(key, _)| key.as_str()).collect())
    }
}
fn short_set(names: &BTreeSet<String>) -> BTreeSet<String> {
    names.iter().map(|n| em::short_name(n).into()).collect()
}
fn off_layer(m: &Move, want: Option<&str>) -> bool {
    if m.layer == RUN {
        !m.runs_on.iter().any(|layer| Some(layer.as_str()) == want)
    } else {
        Some(m.layer.as_str()) != want
    }
}

pub fn build_menus<G: JointGeometry + ?Sized>(
    input: &JointInput,
    geometry: &G,
) -> Result<JointMenus, String> {
    let normalized = normalized_input(input);
    let input = &normalized;
    if input.pockets.iter().any(|p| p.polygon.len() != 4) {
        return Err("AWX pair pocket must be a four-point quadrilateral".into());
    }
    let start = Instant::now();
    let grid = em::grid_of(&input.pads)?;
    let climb = input
        .climb
        .unwrap_or(grid.xs.len().max(grid.ys.len()) as u32);
    let bus = short_set(&input.bus);
    let others = short_set(&input.others);
    let drops: BTreeSet<_> = short_set(&input.drops)
        .difference(&bus)
        .cloned()
        .collect::<BTreeSet<_>>()
        .difference(&others)
        .cloned()
        .collect();
    let mut items = BTreeMap::new();
    let mut menus = BTreeMap::new();
    let mut drop_menus = BTreeMap::new();
    let sz = &input.sizes;
    let runs_mode = input.routing_layers.len() > 2;
    let partner: BTreeMap<_, _> = input
        .pairs
        .iter()
        .flat_map(|p| [(p.p.clone(), p.n.clone()), (p.n.clone(), p.p.clone())])
        .collect();
    let net_ids: BTreeMap<_, _> = input
        .pads
        .iter()
        .filter(|p| p.net_id != 0)
        .map(|p| (em::short_name(&p.net).to_string(), p.net_id))
        .collect();
    let not_planned: BTreeSet<_> = input
        .pads
        .iter()
        .filter(|p| p.net_id != 0)
        .map(|p| em::short_name(&p.net).to_string())
        .filter(|n| !bus.contains(n) && !others.contains(n) && !drops.contains(n))
        .collect();
    let rays = if drops.is_empty() {
        vec![]
    } else {
        geo::lane_rays(&grid, &input.pads, &input.segments, &not_planned, 1.6)
    };
    for p in &input.pads {
        let name = em::short_name(&p.net).to_string();
        if p.net_id == 0
            || (!bus.contains(&name) && !others.contains(&name) && !drops.contains(&name))
        {
            continue;
        }
        let key = format!("{name}#{}", p.pad_number);
        if input.only.as_ref().is_some_and(|only| !only.contains(&key)) {
            continue;
        }
        let pad = p.clone();

        if items.insert(key.clone(), pad.clone()).is_some() {
            return Err(format!("duplicate AWX array ball key {key}"));
        }
        if drops.contains(&name) {
            menus.insert(key.clone(), vec![]);
            drop_menus.insert(
                key.clone(),
                geo::drops(
                    geometry,
                    &grid,
                    &pad,
                    &input.pads,
                    sz,
                    &rays,
                    Some(&input.zones),
                ),
            );
            continue;
        }
        let home = geometry
            .copper_layers()
            .iter()
            .find(|layer| pad.layers.contains(layer))
            .map(String::as_str)
            .unwrap_or("F.Cu");
        let layer_choices = if bus.contains(&name) {
            &input.routing_layers
        } else {
            &input.other_layers
        };
        let mut layers = vec![home.to_string()];
        layers.extend(
            layer_choices
                .iter()
                .filter(|layer| {
                    layer.as_str() != home && !(runs_mode && input.plane_layers.contains(*layer))
                })
                .cloned(),
        );
        let runs = if runs_mode && layers.len() > 2 {
            Some(layers[1..].to_vec())
        } else {
            None
        };
        let enumerate_layers = if runs.is_some() {
            vec![home.to_string(), RUN.into()]
        } else {
            layers
        };
        let inpad = sz
            .inpad
            .get(&p.key)
            .or_else(|| sz.inpad.get(&key))
            .copied()
            .ok_or("joint sizing lacks pad-specific via clamp")?;
        let opts = em::EscapeOptions {
            margin: 0.0,
            climb: if bus.contains(&name) {
                climb
            } else {
                climb.min(input.climb_other)
            },
            own_line: true,
            straight: true,
            street: input.street,
            street_pitch: sz.stack + 1e-4,
            street_dirs: None,
        };
        let mut moves = em::enumerate_moves(
            &pad,
            &grid,
            &enumerate_layers,
            &opts,
            |a, b, layer| {
                if layer == RUN {
                    runs.as_ref().unwrap().iter().any(|layer| {
                        geometry.segment_clear(p.net_id, layer, a, b, sz.clearance + sz.track / 2.0)
                    })
                } else {
                    geometry.segment_clear(p.net_id, layer, a, b, sz.clearance + sz.track / 2.0)
                }
            },
            |q, _| {
                geo::via_clear(
                    geometry,
                    p.net_id,
                    q,
                    if q == p.point {
                        inpad.radius
                    } else {
                        sz.via.radius
                    },
                    sz,
                )
            },
        );
        if let Some(runs) = runs {
            moves.retain_mut(|m| {
                if m.layer != RUN {
                    return true;
                }
                m.runs_on = runs
                    .iter()
                    .filter(|layer| {
                        m.legs.iter().filter(|leg| leg.layer == RUN).all(|leg| {
                            geometry.segment_clear(
                                p.net_id,
                                layer,
                                leg.a,
                                leg.b,
                                sz.clearance + sz.track / 2.0,
                            )
                        })
                    })
                    .cloned()
                    .collect();
                !m.runs_on.is_empty()
            });
        }
        moves = em::dedupe_climbs(moves);
        if bus.contains(&name) {
            if let Some(far) = input.far {
                moves.retain(|m| m.direction != far);
            }
            if input.vias_only {
                let via_moves: Vec<_> = moves
                    .iter()
                    .filter(|m| m.kind != MoveKind::Surface)
                    .cloned()
                    .collect();
                if !via_moves.is_empty() {
                    moves = via_moves;
                }
            }
        }
        if let Some(partner) = partner.get(&name).and_then(|partner| net_ids.get(partner)) {
            let mut kept = Vec::new();
            for mut m in moves.iter().cloned() {
                if m.layer == RUN {
                    m.runs_on = m
                        .runs_on
                        .iter()
                        .filter(|layer| {
                            geo::pair_exit_clear(geometry, p.net_id, *partner, &m, layer, sz)
                        })
                        .cloned()
                        .collect();
                    if !m.runs_on.is_empty() {
                        kept.push(m);
                    }
                } else if geo::pair_exit_clear(geometry, p.net_id, *partner, &m, &m.layer, sz) {
                    kept.push(m);
                }
            }
            if !kept.is_empty() {
                moves = kept;
            }
        }
        moves.retain(|m| {
            !geo::in_pockets(
                &input.pockets,
                &m.legs,
                &m.site.into_iter().collect::<Vec<_>>(),
            )
        });
        menus.insert(key.clone(), moves);
        drop_menus.insert(key, vec![]);
    }
    for drops in drop_menus.values_mut() {
        drops.retain(|drop| {
            !geo::in_pockets(
                &input.pockets,
                &drop
                    .stub
                    .map(|[a, b]| vec![Leg::new(a, b, &drop.layer)])
                    .unwrap_or_default(),
                &[drop.site],
            )
        });
    }
    let mut by_net: BTreeMap<String, Vec<String>> = BTreeMap::new();
    for (key, pad) in &items {
        by_net
            .entry(em::short_name(&pad.net).into())
            .or_default()
            .push(key.clone());
    }
    let mut straps: BTreeMap<String, Vec<Strap>> = BTreeMap::new();
    let near = |d: f64, pitch: f64| d < 0.01 || (d - pitch).abs() < 0.01;
    for keys in by_net.values() {
        if keys.len() < 2 {
            continue;
        }
        for key in keys {
            let p = &items[key];
            let home = geometry
                .copper_layers()
                .iter()
                .find(|layer| p.layers.contains(layer))
                .map(String::as_str)
                .unwrap_or("F.Cu");
            for other in keys {
                let q = &items[other];
                if key == other
                    || !q.layers.iter().any(|l| l == home)
                    || !near((p.point[0] - q.point[0]).abs(), grid.pitch_x)
                    || !near((p.point[1] - q.point[1]).abs(), grid.pitch_y)
                {
                    continue;
                }
                let leg = Leg::new(p.point, q.point, home);
                if geometry.segment_clear(
                    p.net_id,
                    home,
                    p.point,
                    q.point,
                    sz.clearance + sz.track / 2.0,
                ) && !geo::in_pockets(&input.pockets, std::slice::from_ref(&leg), &[])
                {
                    straps.entry(key.clone()).or_default().push(Strap {
                        to: other.clone(),
                        a: p.point,
                        b: q.point,
                        layer: home.into(),
                        length: leg.length(),
                    });
                }
            }
        }
    }
    let mut options = BTreeMap::new();
    let mut via_radius = BTreeMap::new();
    for (key, p) in &items {
        let name = em::short_name(&p.net).to_string();
        let mut opts = Vec::new();
        for m in &menus[key] {
            let vias = if let Some(site) = m.site {
                let size = if site == p.point {
                    sz.inpad
                        .get(&p.key)
                        .or_else(|| sz.inpad.get(key))
                        .copied()
                        .ok_or("pad via clamp missing")?
                } else {
                    sz.via
                };
                via_radius.insert(
                    Member {
                        ball: key.clone(),
                        index: opts.len(),
                    },
                    size.radius,
                );
                vec![PieceVia {
                    point: site,
                    radius: size.radius,
                    drill_radius: size.drill_radius,
                }]
            } else {
                vec![]
            };
            opts.push(JointOption {
                option: OptionMove::Escape {
                    movement: m.clone(),
                },
                piece: Piece {
                    net: name.clone(),
                    legs: m.legs.clone(),
                    vias,
                    balls: vec![p.point],
                },
            });
        }
        for strap in straps.get(key).into_iter().flatten() {
            opts.push(JointOption {
                option: OptionMove::Strap {
                    strap: strap.clone(),
                },
                piece: Piece {
                    net: name.clone(),
                    legs: vec![Leg::new(strap.a, strap.b, &strap.layer)],
                    vias: vec![],
                    balls: vec![strap.a, strap.b],
                },
            });
        }
        for drop in &drop_menus[key] {
            opts.push(JointOption {
                option: OptionMove::Drop { drop: drop.clone() },
                piece: Piece {
                    net: name.clone(),
                    legs: drop
                        .stub
                        .map(|[a, b]| vec![Leg::new(a, b, &drop.layer)])
                        .unwrap_or_default(),
                    vias: vec![PieceVia {
                        point: drop.site,
                        radius: drop.radius,
                        drill_radius: drop.drill_radius,
                    }],
                    balls: vec![p.point],
                },
            });
        }
        options.insert(key.clone(), opts);
    }
    let pairs = input
        .pairs
        .iter()
        .filter_map(|pair| {
            let p = by_net.get(&pair.p)?;
            let n = by_net.get(&pair.n)?;
            (p.len() == 1 && n.len() == 1).then(|| JointPair {
                base: pair.base.clone(),
                p: p[0].clone(),
                n: n[0].clone(),
            })
        })
        .collect();
    Ok(JointMenus {
        grid,
        items,
        menus,
        options,
        straps,
        pairs,
        via_radius,
        lane_faces: geo::lane_faces(&rays),
        menu_seconds: start.elapsed().as_secs_f64(),
    })
}

fn escape(option: &JointOption) -> Option<&Move> {
    if let OptionMove::Escape { movement } = &option.option {
        Some(movement)
    } else {
        None
    }
}
fn costs(input: &JointInput, bm: &JointMenus, key: &str, o: &JointOption) -> Result<i64, String> {
    let pad = &bm.items[key];
    let net = em::short_name(&pad.net);
    let plane =
        input.drops.contains(net) && !input.bus.contains(net) && !input.others.contains(net);
    let value = match &o.option {
        OptionMove::Strap { strap } => {
            C_MM as f64 * strap.length + if plane { C_VIA as f64 } else { 0.0 }
        }
        OptionMove::Drop { drop } => {
            if drop.inpad {
                (C_VIA + C_VIP) as f64
            } else {
                C_MM as f64 * em::distance(drop.site, pad.point)
                    + if !geo::beyond_faces(&bm.grid, drop.site).is_disjoint(&bm.lane_faces) {
                        C_LANE_FACE as f64
                    } else {
                        0.0
                    }
                    + if drop.radius < input.sizes.via.radius - 1e-9 {
                        C_FINE_VIA as f64
                    } else {
                        0.0
                    }
            }
        }
        OptionMove::Escape { movement: m } => {
            let mut length = m.legs.iter().map(Leg::length).sum::<f64>();
            if length == 0.0 {
                length = em::distance(m.exit_pt, pad.point);
            }
            let mut value = C_VIA as f64 * m.vias as f64
                + C_MM as f64 * length
                + C_PLANE_LAYER as f64
                    * m.legs
                        .iter()
                        .filter(|l| input.plane_layers.contains(&l.layer))
                        .count() as f64;
            if input.bus.contains(net) {
                if let Some(p) = input.preferred.get(net) {
                    value += C_DEV_MM as f64 * em::distance(m.exit_pt, p.tooth);
                    value += C_DEV_KIND as f64
                        * (usize::from(Some(m.direction) != p.direction)
                            + usize::from(off_layer(m, p.layer.as_deref()))
                            + usize::from(Some(m.kind) != p.kind)) as f64;
                }
            }
            value
        }
    };
    super::numeric::checked_round_i64(value)
}
fn value(response: &CpSolverResponse, var: BoolVar) -> bool {
    response.solution[var.index() as usize] != 0
}
fn feasible(response: &CpSolverResponse) -> bool {
    matches!(
        response.status(),
        CpSolverStatus::Optimal | CpSolverStatus::Feasible
    )
}
fn sum_terms(variables: impl IntoIterator<Item = BoolVar>) -> Vec<(IntVar, i64)> {
    variables.into_iter().map(|v| (v.into(), 1)).collect()
}
fn at_most(m: &mut Model, variables: &[BoolVar], cap: i64) -> Result<(), String> {
    m.le(&sum_terms(variables.iter().copied()), cap, &[])
}
fn maximize(m: &mut Model, terms: &[(IntVar, i64)]) -> Result<(), String> {
    let mut proto = cpsat_rs::proto::CpObjectiveProto::default();
    proto.vars = terms.iter().map(|(v, _)| v.index()).collect();
    proto.coeffs = terms
        .iter()
        .map(|(_, w)| w.checked_neg().ok_or("joint objective overflow"))
        .collect::<Result<_, _>>()?;
    proto.scaling_factor = -1.0;
    m.inner.raw_proto_mut().objective = Some(proto);
    Ok(())
}
fn vars_of(
    v: &BTreeMap<String, Vec<BoolVar>>,
    members: impl IntoIterator<Item = Member>,
) -> Vec<BoolVar> {
    members.into_iter().map(|m| v[&m.ball][m.index]).collect()
}
fn run_capacity(
    m: &mut Model,
    input: &JointInput,
    bm: &JointMenus,
    v: &BTreeMap<String, Vec<BoolVar>>,
    members: &[Member],
) -> Result<i64, String> {
    let mut by_set: BTreeMap<BTreeSet<String>, Vec<BoolVar>> = BTreeMap::new();
    let mut bus = Vec::new();
    let mut others = Vec::new();
    for member in members {
        let movement = escape(&bm.options[&member.ball][member.index])
            .ok_or("RUN capacity applied to non-escape")?;
        let var = v[&member.ball][member.index];
        by_set
            .entry(movement.runs_on.iter().cloned().collect())
            .or_default()
            .push(var);
        if input
            .bus
            .contains(em::short_name(&bm.items[&member.ball].net))
        {
            bus.push(var);
        } else {
            others.push(var);
        }
    }
    let all: BTreeSet<_> = by_set.keys().flat_map(|s| s.iter().cloned()).collect();
    let cap = all.len() as i64;
    if bus.len() > 1 {
        at_most(m, &bus, 1)?;
    }
    if !bus.is_empty() && !others.is_empty() {
        let mut terms: Vec<_> = bus.iter().map(|&v| (v.into(), cap)).collect();
        terms.extend(sum_terms(others));
        m.le(&terms, cap, &[])?;
    }
    let mut sets: BTreeSet<_> = by_set.keys().cloned().collect();
    sets.insert(all);
    let mut sets: Vec<_> = sets.into_iter().collect();
    sets.sort_by(|a, b| a.len().cmp(&b.len()).then(a.cmp(b)));
    for set in sets {
        let lits: Vec<_> = by_set
            .iter()
            .filter(|(s, _)| s.is_subset(&set))
            .flat_map(|(_, v)| v.iter().copied())
            .collect();
        if lits.len() > set.len() {
            at_most(m, &lits, set.len() as i64)?;
        }
    }
    Ok(cap)
}
fn exclude(
    m: &mut Model,
    lit: BoolVar,
    others: Vec<Member>,
    v: &BTreeMap<String, Vec<BoolVar>>,
    cover: &[(Vec<Member>, i64)],
    membership: &BTreeMap<Member, Vec<usize>>,
) -> Result<(), String> {
    let mut remaining: BTreeSet<_> = others.into_iter().collect();
    while !remaining.is_empty() {
        let mut counts: BTreeMap<usize, usize> = BTreeMap::new();
        for member in &remaining {
            for &g in membership.get(member).into_iter().flatten() {
                *counts.entry(g).or_default() += 1;
            }
        }
        let Some((&index, &count)) = counts.iter().max_by(|a, b| a.1.cmp(b.1).then(b.0.cmp(a.0)))
        else {
            break;
        };
        if count < 2 {
            break;
        }
        let (group, cap) = &cover[index];
        let hit: Vec<_> = group
            .iter()
            .filter(|g| remaining.contains(*g))
            .cloned()
            .collect();
        let mut terms = vec![(lit.into(), *cap)];
        terms.extend(sum_terms(vars_of(v, hit.clone())));
        m.le(&terms, *cap, &[])?;
        for member in hit {
            remaining.remove(&member);
        }
    }
    for member in remaining {
        m.or(&[!lit, !v[&member.ball][member.index]], &[]);
    }
    Ok(())
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct TierReport {
    pub name: String,
    pub balls: usize,
    pub status: String,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointHint {
    pub ball: String,
    pub at: Point,
    pub hint: serde_json::Value,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointReport {
    pub status: String,
    pub balls: usize,
    pub moves: usize,
    pub conflicts: usize,
    pub conflict_groups: usize,
    pub conflict_bicliques: usize,
    pub bus_escaped: usize,
    pub bus_balls: usize,
    pub others_escaped: usize,
    pub others_strapped: usize,
    pub others_balls: usize,
    pub dropped: usize,
    pub plane_strapped: usize,
    pub dropped_in_pad: usize,
    pub drop_vias: usize,
    pub plane_balls: usize,
    pub no_move: Vec<String>,
    pub unplanned: Vec<String>,
    pub clashes: Vec<(String, String)>,
    pub climbed: usize,
    pub streets: usize,
    pub faces: BTreeMap<String, usize>,
    pub let_go: Vec<String>,
    pub phase1: String,
    pub phase1_seconds: f64,
    pub phase1_tiers: Vec<TierReport>,
    pub uncoloured: Vec<String>,
    pub run_layers: Option<BTreeMap<String, usize>>,
    pub pairs: usize,
    pub pairs_escaped: usize,
    pub hands_held: Vec<String>,
    pub hands_free: Vec<String>,
    pub seconds: f64,
    pub objective: Option<i64>,
    pub bound: f64,
    pub wall_time: f64,
    pub branches: i64,
    pub conflicts_cp: i64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointPlan {
    pub chosen: BTreeMap<String, OptionMove>,
    pub selected_indices: BTreeMap<String, usize>,
    pub pieces: BTreeMap<String, Piece>,
    pub hints: Vec<JointHint>,
    pub report: JointReport,
}

pub fn plan_array<G: JointGeometry + ?Sized>(
    input: &JointInput,
    geometry: &G,
) -> Result<JointPlan, String> {
    let normalized = normalized_input(input);
    let input = &normalized;
    let bm = build_menus(input, geometry)?;
    solve_menus(input, bm)
}
pub fn solve_menus(input: &JointInput, bm: JointMenus) -> Result<JointPlan, String> {
    let normalized = normalized_input(input);
    let input = &normalized;
    let start = Instant::now();
    let runs_mode = input.routing_layers.len() > 2;
    let groups = cg::conflict_groups(
        &bm.menus,
        true,
        input.sizes.stack,
        |key, index, _| {
            bm.via_radius
                .get(&Member {
                    ball: key.into(),
                    index,
                })
                .copied()
                .unwrap_or(input.sizes.via.radius)
        },
        input.sizes.clearance + input.sizes.track / 2.0,
        2,
    );
    let is_run = |tags: Option<&BTreeSet<Tag>>| {
        tags.is_some_and(|tags|!tags.is_empty()&&tags.iter().all(|tag|matches!(tag,Tag::Lane(layer)|Tag::Crossing(layer)|Tag::Exit(layer)if layer==RUN)))
    };
    let cap_groups: BTreeSet<_> = groups
        .cliques
        .iter()
        .filter(|g| is_run(groups.tags.get(&GroupKey::Clique((*g).clone()))))
        .cloned()
        .collect();
    let cap_bicliques: BTreeSet<_> = groups
        .bicliques
        .iter()
        .filter(|(a, b)| is_run(groups.tags.get(&GroupKey::Biclique(a.clone(), b.clone()))))
        .cloned()
        .collect();
    let cliques: Vec<Vec<Member>> = groups
        .cliques
        .iter()
        .filter(|g| g.iter().map(|m| &m.ball).collect::<BTreeSet<_>>().len() > 1)
        .map(|g| g.iter().cloned().collect())
        .collect();
    let bicliques: Vec<_> = groups
        .bicliques
        .iter()
        .filter(|(a, b)| a.union(b).map(|m| &m.ball).collect::<BTreeSet<_>>().len() > 1)
        .cloned()
        .collect();
    let mut conflict_pairs = groups.pairs.clone();
    let mut boxes = BTreeMap::new();
    let mut index: BTreeMap<(i64, i64), Vec<Member>> = BTreeMap::new();
    for (key, options) in &bm.options {
        for (i, option) in options.iter().enumerate() {
            let member = Member {
                ball: key.clone(),
                index: i,
            };
            let bounds = geo::piece_box(&option.piece, input.sizes.grow);
            boxes.insert(member.clone(), bounds);
            for x in bounds[0].floor() as i64..=bounds[2].floor() as i64 {
                for y in bounds[1].floor() as i64..=bounds[3].floor() as i64 {
                    index.entry((x, y)).or_default().push(member.clone());
                }
            }
        }
    }
    for (key, options) in &bm.options {
        for (i, option) in options.iter().enumerate() {
            if escape(option).is_some() {
                continue;
            }
            let member = Member {
                ball: key.clone(),
                index: i,
            };
            let bounds = boxes[&member];
            let mut seen = BTreeSet::new();
            for x in bounds[0].floor() as i64..=bounds[2].floor() as i64 {
                for y in bounds[1].floor() as i64..=bounds[3].floor() as i64 {
                    for other in index.get(&(x, y)).into_iter().flatten() {
                        if other.ball == *key || !seen.insert(other.clone()) {
                            continue;
                        }
                        let second = &bm.options[&other.ball][other.index];
                        if escape(second).is_none() && *other <= member {
                            continue;
                        }
                        let b = boxes[other];
                        if bounds[0] > b[2]
                            || b[0] > bounds[2]
                            || bounds[1] > b[3]
                            || b[1] > bounds[3]
                        {
                            continue;
                        }
                        if geo::piece_hit(
                            &option.piece,
                            &second.piece,
                            option.piece.net == second.piece.net,
                            &input.sizes,
                        ) {
                            conflict_pairs.insert((member.clone(), other.clone()));
                        }
                    }
                }
            }
        }
    }
    let keys: Vec<_> = bm.options.keys().cloned().collect();
    let mut m = Model::new();
    let mut v = BTreeMap::new();
    let mut served = BTreeMap::new();
    for (k, key) in keys.iter().enumerate() {
        let variables: Vec<_> = (0..bm.options[key].len())
            .map(|i| m.bool(&format!("v{k}_{i}")))
            .collect();
        if !variables.is_empty() {
            let s = m.bool(&format!("s{k}"));
            let mut terms = sum_terms(variables.iter().copied());
            terms.push((s.into(), -1));
            m.eq(&terms, 0, &[])?;
            served.insert(key.clone(), s);
        }
        v.insert(key.clone(), variables);
    }
    let mut cover: Vec<(Vec<Member>, i64)> = Vec::new();
    let mut membership: BTreeMap<Member, Vec<usize>> = BTreeMap::new();
    let mut covers = |group: Vec<Member>, cap: i64| {
        if runs_mode {
            let index = cover.len();
            for member in &group {
                membership.entry(member.clone()).or_default().push(index);
            }
            cover.push((group, cap));
        }
    };
    for group in &cliques {
        let set: BTreeSet<_> = group.iter().cloned().collect();
        let cap = if cap_groups.contains(&set) {
            run_capacity(&mut m, input, &bm, &v, group)?
        } else {
            at_most(&mut m, &vars_of(&v, group.clone()), 1)?;
            1
        };
        covers(group.clone(), cap);
    }
    for (a, b) in &bicliques {
        if cap_bicliques.contains(&(a.clone(), b.clone())) {
            let group: Vec<_> = a.union(b).cloned().collect();
            let cap = run_capacity(&mut m, input, &bm, &v, &group)?;
            covers(group, cap);
        }
    }
    if runs_mode {
        let pair_of: BTreeMap<_, _> = bm
            .pairs
            .iter()
            .enumerate()
            .flat_map(|(i, p)| [(p.p.clone(), (i, 0)), (p.n.clone(), (i, 1))])
            .collect();
        let mut seen = BTreeSet::new();
        for group in cap_groups.iter().cloned().chain(
            cap_bicliques
                .iter()
                .map(|(a, b)| a.union(b).cloned().collect()),
        ) {
            let mut legs: BTreeMap<usize, [Vec<Member>; 2]> = BTreeMap::new();
            for member in group {
                if let Some(&(i, side)) = pair_of.get(&member.ball) {
                    legs.entry(i).or_insert_with(|| [vec![], vec![]])[side].push(member);
                }
            }
            for (i, [p, n]) in legs {
                if !p.is_empty() && !n.is_empty() && seen.insert((i, p.clone(), n.clone())) {
                    at_most(&mut m, &vars_of(&v, p.into_iter().chain(n)), 1)?;
                }
            }
        }
    }
    for (n, (a, b)) in bicliques.iter().enumerate() {
        if cap_bicliques.contains(&(a.clone(), b.clone())) {
            continue;
        }
        let both: BTreeSet<_> = a.intersection(b).cloned().collect();
        let one_site = |side: &BTreeSet<Member>| {
            let sites: BTreeSet<_> = side
                .iter()
                .filter_map(|member| {
                    escape(&bm.options[&member.ball][member.index]).and_then(cg::site_key)
                })
                .collect();
            sites.len() == 1
                && side.iter().all(|member| {
                    escape(&bm.options[&member.ball][member.index])
                        .is_some_and(|m| m.site.is_some())
                })
        };
        if runs_mode && (both.is_empty() || one_site(a) || one_site(b)) {
            let (x, y) = if if both.is_empty() {
                a.len() <= b.len()
            } else {
                one_site(a)
            } {
                (a, b)
            } else {
                (b, a)
            };
            let lit = if x.len() == 1 {
                let member = x.first().unwrap();
                v[&member.ball][member.index]
            } else {
                let lit = m.bool(&format!("bi{n}"));
                for member in x {
                    m.implication(v[&member.ball][member.index], lit);
                }
                lit
            };
            exclude(
                &mut m,
                lit,
                y.difference(x).cloned().collect(),
                &v,
                &cover,
                &membership,
            )?;
            continue;
        }
        let mut lits = vars_of(&v, both.iter().cloned());
        for (side, tag) in [(a, "a"), (b, "b")] {
            let only: Vec<_> = side.difference(&both).cloned().collect();
            if !only.is_empty() {
                let lit = m.bool(&format!("bi{n}{tag}"));
                for member in only {
                    m.implication(v[&member.ball][member.index], lit);
                }
                lits.push(lit);
            }
        }
        if lits.len() > 1 {
            at_most(&mut m, &lits, 1)?;
        }
    }
    if runs_mode {
        let mut by_option: BTreeMap<Member, Vec<Member>> = BTreeMap::new();
        for (a, b) in &conflict_pairs {
            by_option.entry(a.clone()).or_default().push(b.clone());
        }
        for (a, others) in by_option {
            exclude(&mut m, v[&a.ball][a.index], others, &v, &cover, &membership)?;
        }
    } else {
        for (a, b) in &conflict_pairs {
            m.or(&[!v[&a.ball][a.index], !v[&b.ball][b.index]], &[]);
        }
    }
    let (hands_held, hands_free) = pair_constraints(&mut m, input, &bm, &v)?;
    bus_regions(&mut m, input, &bm, &v)?;
    let mut levels = BTreeMap::new();
    for (k, key) in keys.iter().enumerate() {
        if bm.straps.get(key).is_some_and(|s| !s.is_empty()) {
            levels.insert(
                key.clone(),
                m.int(0, bm.items.len() as i64, &format!("l{k}"))?,
            );
        }
    }
    for key in &keys {
        for (i, option) in bm.options[key].iter().enumerate() {
            if let OptionMove::Strap { strap } = &option.option {
                let lit = v[key][i];
                if v.get(&strap.to).is_none_or(Vec::is_empty) {
                    m.eq(&[(lit.into(), 1)], 0, &[])?;
                    continue;
                }
                m.ge(&sum_terms(v[&strap.to].iter().copied()), 1, &[lit])?;
                if let Some(&other) = levels.get(&strap.to) {
                    m.ge(&[(levels[key], 1), (other, -1)], 1, &[lit])?;
                }
            }
        }
    }
    let mut site_representatives: BTreeMap<(String, i64, i64), Vec<Point>> = BTreeMap::new();
    let mut at_site: BTreeMap<(String, i64, i64), Vec<BoolVar>> = BTreeMap::new();
    for key in &keys {
        for (i, option) in bm.options[key].iter().enumerate() {
            if let OptionMove::Drop { drop } = &option.option {
                if !drop.inpad {
                    let site =
                        shared_site_key(&option.piece.net, drop.site, &mut site_representatives)?;
                    at_site.entry(site).or_default().push(v[key][i]);
                }
            }
        }
    }
    let mut site_used = Vec::new();
    for (i, lits) in at_site.values().enumerate() {
        if lits.len() == 1 {
            site_used.push(lits[0]);
        } else {
            let lit = m.bool(&format!("site{i}"));
            for &v in lits {
                m.implication(v, lit);
            }
            site_used.push(lit);
        }
    }
    let phase1_start = Instant::now();
    let mut let_go: Vec<String> = Vec::new();
    let mut held_keys: Vec<String> = Vec::new();
    let mut left_out = BTreeSet::new();
    let mut hint_values: BTreeMap<i32, i64> = BTreeMap::new();
    let mut tier_reports = Vec::new();
    let pair_legs: BTreeSet<_> = bm
        .pairs
        .iter()
        .flat_map(|p| [p.p.clone(), p.n.clone()])
        .collect();
    let rank = |key: &str| {
        let net = em::short_name(&bm.items[key].net);
        if input.bus.contains(net) {
            2
        } else if input.drops.contains(net) {
            0
        } else {
            1
        }
    };
    for key in &keys {
        if let Some(preferred) = input
            .preferred
            .get(em::short_name(&bm.items[key].net))
            .filter(|_| input.bus.contains(em::short_name(&bm.items[key].net)))
        {
            if let Some((best, _)) = bm.options[key]
                .iter()
                .enumerate()
                .filter_map(|(i, o)| escape(o).map(|m| (i, m)))
                .min_by(|a, b| {
                    let wrong = |m: &Move| {
                        usize::from(Some(m.direction) != preferred.direction)
                            + usize::from(off_layer(m, preferred.layer.as_deref()))
                            + usize::from(Some(m.kind) != preferred.kind)
                    };
                    wrong(a.1).cmp(&wrong(b.1)).then(
                        em::distance(a.1.exit_pt, preferred.tooth)
                            .total_cmp(&em::distance(b.1.exit_pt, preferred.tooth)),
                    )
                })
            {
                for (i, &var) in v[key].iter().enumerate() {
                    m.hint(var.into(), i64::from(i == best));
                }
            }
        }
    }
    let tiers = [
        (
            "bus and pairs",
            keys.iter()
                .filter(|k| {
                    served.contains_key(*k)
                        && (input.bus.contains(em::short_name(&bm.items[*k].net))
                            || pair_legs.contains(*k))
                })
                .cloned()
                .collect::<Vec<_>>(),
        ),
        (
            "others",
            keys.iter()
                .filter(|k| {
                    served.contains_key(*k)
                        && !pair_legs.contains(*k)
                        && input.others.contains(em::short_name(&bm.items[*k].net))
                })
                .cloned()
                .collect(),
        ),
        (
            "plane",
            keys.iter()
                .filter(|k| {
                    served.contains_key(*k)
                        && input.drops.contains(em::short_name(&bm.items[*k].net))
                })
                .cloned()
                .collect(),
        ),
    ];
    let mut phase1_status = "nothing to ask".to_string();
    let p1 = SatParameters {
        num_workers: Some(1),
        max_deterministic_time: Some(input.phase1_deterministic_time),
        max_time_in_seconds: input.time_limit,
        ..Default::default()
    };
    let served_index: BTreeMap<_, _> = served.iter().map(|(k, v)| (v.index(), k.clone())).collect();
    for (name, tier) in tiers {
        if tier.is_empty() {
            continue;
        }
        let mut response = loop {
            let assumptions: Vec<_> = held_keys
                .iter()
                .chain(&tier)
                .filter(|k| !let_go.contains(k))
                .map(|k| served[k])
                .collect();
            m.assumptions(&assumptions);
            let response = super::cp_sat_bridge::solve(&m.inner, &p1, None)?;
            if response.status() != CpSolverStatus::Infeasible {
                break response;
            }
            let core: Vec<_> = response
                .sufficient_assumptions_for_infeasibility
                .iter()
                .filter_map(|i| served_index.get(i).cloned())
                .collect();
            let mut candidates: Vec<_> =
                core.iter().filter(|k| tier.contains(k)).cloned().collect();
            if candidates.is_empty() {
                candidates = core;
            }
            let Some(remove) = candidates
                .into_iter()
                .min_by(|a, b| rank(a).cmp(&rank(b)).then(a.cmp(b)))
            else {
                break response;
            };
            let_go.push(remove);
        };
        let mut status = response.status().as_str_name().to_string();
        if response.status() == CpSolverStatus::Unknown {
            m.assumptions(&held_keys.iter().map(|k| served[k]).collect::<Vec<_>>());
            maximize(
                &mut m,
                &tier
                    .iter()
                    .filter(|k| !let_go.contains(k))
                    .map(|k| (served[k].into(), 1))
                    .collect::<Vec<_>>(),
            )?;
            response = super::cp_sat_bridge::solve(&m.inner, &p1, None)?;
            m.clear_objective();
            if feasible(&response) {
                for key in &tier {
                    if !let_go.contains(key) && !value(&response, served[key]) {
                        left_out.insert(key.clone());
                    }
                }
            }
            status.push_str(&format!(
                ", the most of it {}",
                response.status().as_str_name()
            ));
        }
        tier_reports.push(TierReport {
            name: name.into(),
            balls: tier.len(),
            status,
        });
        phase1_status = response.status().as_str_name().to_string();
        if !feasible(&response) {
            break;
        }
        held_keys.extend(
            tier.iter()
                .filter(|k| !let_go.contains(k) && !left_out.contains(*k))
                .cloned(),
        );
        for variables in v.values() {
            for &var in variables {
                hint_values.insert(var.index(), response.solution[var.index() as usize]);
            }
        }
    }
    m.clear_assumptions();
    if !held_keys.is_empty() {
        m.clear_hints();
        for variables in v.values() {
            for &var in variables {
                m.hint(var.into(), hint_values[&var.index()]);
            }
        }
        for key in &held_keys {
            m.eq(&[(served[key].into(), 1)], 1, &[])?;
        }
        if phase1_status != "OPTIMAL" && phase1_status != "FEASIBLE" {
            phase1_status = "FEASIBLE".into();
        }
    }
    let phase1_seconds = phase1_start.elapsed().as_secs_f64();
    let held: BTreeSet<_> = held_keys.into_iter().collect();
    let mut objective = Vec::new();
    for key in &keys {
        let net = em::short_name(&bm.items[key].net);
        let weight = if held.contains(key) {
            0
        } else if input.bus.contains(net) {
            W_BUS
        } else if input.drops.contains(net) {
            W_DROP
        } else {
            W_OTHER
        };
        for (i, option) in bm.options[key].iter().enumerate() {
            objective.push((v[key][i].into(), weight - costs(input, &bm, key, option)?));
        }
    }
    objective.extend(site_used.iter().map(|&lit| (lit.into(), -C_VIA)));
    maximize(&mut m, &objective)?;
    let parameters = SatParameters {
        num_workers: Some(if input.solve_workers == 0 {
            2
        } else {
            input.solve_workers
        }),
        interleave_search: Some(true),
        max_num_deterministic_batches: Some(if input.solve_batches == 0 {
            60
        } else {
            input.solve_batches
        }),
        share_glue_clauses: Some(false),
        share_binary_clauses: Some(false),
        subsolvers: if runs_mode {
            vec!["quick_restart".into()]
        } else {
            vec!["default_lp".into(), "quick_restart".into()]
        },
        max_time_in_seconds: input.time_limit,
        ..Default::default()
    };
    let response = super::cp_sat_bridge::solve(&m.inner, &parameters, None)?;
    let mut chosen = BTreeMap::new();
    let mut selected = BTreeMap::new();
    let mut pieces = BTreeMap::new();
    if feasible(&response) {
        for key in &keys {
            for (i, &var) in v[key].iter().enumerate() {
                if value(&response, var) {
                    chosen.insert(key.clone(), bm.options[key][i].option.clone());
                    selected.insert(key.clone(), i);
                    pieces.insert(key.clone(), bm.options[key][i].piece.clone());
                }
            }
        }
    }
    let uncoloured = if runs_mode {
        colour_selected_runs(
            input,
            &bm,
            &cap_groups,
            &cap_bicliques,
            &mut chosen,
            &mut selected,
            &mut pieces,
        )?
    } else {
        vec![]
    };
    finish_plan(
        input,
        bm,
        chosen,
        selected,
        pieces,
        response,
        conflict_pairs.len(),
        cliques.len(),
        bicliques.len(),
        let_go,
        phase1_status,
        phase1_seconds,
        tier_reports,
        uncoloured,
        hands_held,
        hands_free,
        start.elapsed().as_secs_f64(),
        site_representatives,
    )
}

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd)]
struct Slot {
    direction: Direction,
    layer: String,
    x: i64,
    y: i64,
}
type Position = (Direction, String, u64);
fn float_id(x: f64) -> u64 {
    if x == 0.0 {
        0.0f64.to_bits()
    } else {
        x.to_bits()
    }
}
fn slot_of(m: &Move) -> Result<Slot, String> {
    Ok(Slot {
        direction: m.direction,
        layer: m.layer.clone(),
        x: super::numeric::quantize(em::rounded(m.exit_pt[0], 6), 1e-6)?,
        y: super::numeric::quantize(em::rounded(m.exit_pt[1], 6), 1e-6)?,
    })
}
fn occupied(
    m: &mut Model,
    occ: &mut BTreeMap<Position, BoolVar>,
    at: &BTreeMap<Position, Vec<Member>>,
    position: &Position,
    v: &BTreeMap<String, Vec<BoolVar>>,
) -> BoolVar {
    if let Some(&lit) = occ.get(position) {
        return lit;
    }
    let lit = m.bool(&format!("occ{}", occ.len()));
    for member in at.get(position).into_iter().flatten() {
        m.implication(v[&member.ball][member.index], lit);
    }
    occ.insert(position.clone(), lit);
    lit
}
fn pair_constraints(
    m: &mut Model,
    input: &JointInput,
    bm: &JointMenus,
    v: &BTreeMap<String, Vec<BoolVar>>,
) -> Result<(Vec<String>, Vec<String>), String> {
    let reach = 1.3 * bm.grid.pitch_x.max(bm.grid.pitch_y);
    let mut by_class: BTreeMap<(Direction, String), Vec<(f64, Member)>> = BTreeMap::new();
    let mut at: BTreeMap<Position, Vec<Member>> = BTreeMap::new();
    for (key, options) in &bm.options {
        for (i, option) in options.iter().enumerate() {
            if let Some(movement) = escape(option) {
                let along = movement.exit_pt[movement.direction.along_axis()];
                let member = Member {
                    ball: key.clone(),
                    index: i,
                };
                by_class
                    .entry((movement.direction, movement.layer.clone()))
                    .or_default()
                    .push((along, member.clone()));
                at.entry((movement.direction, movement.layer.clone(), float_id(along)))
                    .or_default()
                    .push(member);
            }
        }
    }
    for rows in by_class.values_mut() {
        rows.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    }
    let mut occ = BTreeMap::new();
    let mut hands_held = Vec::new();
    let mut hands_free = Vec::new();
    for (n_pair, pair) in bm.pairs.iter().enumerate() {
        let kp = &pair.p;
        let kn = &pair.n;
        let mut terms = sum_terms(v[kp].iter().copied());
        terms.extend(v[kn].iter().map(|&x| (x.into(), -1)));
        m.eq(&terms, 0, &[])?;
        let mut slots: BTreeMap<String, BTreeMap<Slot, Vec<usize>>> = BTreeMap::new();
        let mut literals: BTreeMap<(String, Slot), BoolVar> = BTreeMap::new();
        for key in [kp, kn] {
            let mut side: BTreeMap<Slot, Vec<usize>> = BTreeMap::new();
            for (i, option) in bm.options[key].iter().enumerate() {
                if let Some(movement) = escape(option) {
                    side.entry(slot_of(movement)?).or_default().push(i);
                } else {
                    m.eq(&[(v[key][i].into(), 1)], 0, &[])?;
                }
            }
            for (slot, indexes) in &side {
                let lit = if indexes.len() == 1 {
                    v[key][indexes[0]]
                } else {
                    let lit = m.bool(&format!("slot{n_pair}_{}", literals.len()));
                    let mut terms: Vec<_> =
                        indexes.iter().map(|&i| (v[key][i].into(), 1)).collect();
                    terms.push((lit.into(), -1));
                    m.eq(&terms, 0, &[])?;
                    lit
                };
                literals.insert((key.clone(), slot.clone()), lit);
            }
            slots.insert(key.clone(), side);
        }
        let mut couples: Vec<(Slot, Slot)> = Vec::new();
        for p in slots[kp].keys() {
            for n in slots[kn].keys() {
                let d = ((p.x - n.x) as f64 * 1e-6).hypot((p.y - n.y) as f64 * 1e-6);
                if p.direction == n.direction && p.layer == n.layer && d > 0.05 && d <= reach {
                    couples.push((p.clone(), n.clone()));
                }
            }
        }
        if let Some(want) = input.hands.get(em::short_name(&bm.items[kp].net)) {
            let right: Vec<_> = couples
                .iter()
                .filter(|(p, n)| {
                    super::pairs::hand(
                        p.direction,
                        [p.x as f64 * 1e-6, p.y as f64 * 1e-6],
                        [n.x as f64 * 1e-6, n.y as f64 * 1e-6],
                        want.arriving,
                    ) == want.hand
                })
                .cloned()
                .collect();
            if right.is_empty() {
                hands_free.push(em::short_name(&bm.items[kp].net).into());
            } else {
                hands_held.push(em::short_name(&bm.items[kp].net).into());
                couples = right;
            }
        }
        let mut mate: BTreeMap<(String, Slot), Vec<Slot>> = BTreeMap::new();
        let mut spans: BTreeMap<Position, BoolVar> = BTreeMap::new();
        for (p, n) in couples {
            mate.entry((kp.clone(), p.clone()))
                .or_default()
                .push(n.clone());
            mate.entry((kn.clone(), n.clone()))
                .or_default()
                .push(p.clone());
            let axis = p.direction.along_axis();
            let ap = if axis == 0 { p.x } else { p.y } as f64 * 1e-6;
            let an = if axis == 0 { n.x } else { n.y } as f64 * 1e-6;
            let (lo, hi) = (ap.min(an), ap.max(an));
            let cls = (p.direction, p.layer.clone());
            let mut done = BTreeSet::new();
            for &(along, ref member) in by_class.get(&cls).into_iter().flatten() {
                if along < lo + 0.02 {
                    continue;
                }
                if along >= hi - 0.02 {
                    break;
                }
                if member.ball == *kp || member.ball == *kn || !done.insert(float_id(along)) {
                    continue;
                }
                let pos = (p.direction, p.layer.clone(), float_id(along));
                let span = if let Some(&lit) = spans.get(&pos) {
                    lit
                } else {
                    let lit = m.bool(&format!("span{n_pair}_{}", spans.len()));
                    spans.insert(pos.clone(), lit);
                    lit
                };
                m.or(
                    &[
                        !literals[&(kp.clone(), p.clone())],
                        !literals[&(kn.clone(), n.clone())],
                        span,
                    ],
                    &[],
                );
            }
        }
        for (pos, span) in spans {
            let occupied = occupied(m, &mut occ, &at, &pos, v);
            m.implication(span, !occupied);
        }
        for (key, other) in [(kp, kn), (kn, kp)] {
            for slot in slots[key].keys() {
                let alternatives = mate
                    .get(&(key.clone(), slot.clone()))
                    .cloned()
                    .unwrap_or_default();
                if alternatives.is_empty() {
                    m.eq(
                        &[(literals[&(key.clone(), slot.clone())].into(), 1)],
                        0,
                        &[],
                    )?;
                } else {
                    let mut clause = vec![!literals[&(key.clone(), slot.clone())]];
                    clause.extend(
                        alternatives
                            .into_iter()
                            .map(|mate| literals[&(other.clone(), mate)]),
                    );
                    m.or(&clause, &[]);
                }
            }
        }
    }
    Ok((hands_held, hands_free))
}
fn bus_regions(
    m: &mut Model,
    input: &JointInput,
    bm: &JointMenus,
    v: &BTreeMap<String, Vec<BoolVar>>,
) -> Result<(), String> {
    if input.bus_groups.is_empty() {
        return Ok(());
    }
    let mut by_face: BTreeMap<(Direction, String), BTreeMap<String, Vec<(i64, BoolVar)>>> =
        BTreeMap::new();
    for (key, options) in &bm.options {
        let Some(group) = input.bus_groups.get(em::short_name(&bm.items[key].net)) else {
            continue;
        };
        for (i, option) in options.iter().enumerate() {
            if let Some(move_) = escape(option) {
                let at = super::numeric::quantize(
                    em::rounded(move_.exit_pt[move_.direction.along_axis()], 3),
                    0.001,
                )?;
                by_face
                    .entry((move_.direction, move_.layer.clone()))
                    .or_default()
                    .entry(group.clone())
                    .or_default()
                    .push((at, v[key][i]));
            }
        }
    }
    let mut region = 0;
    for per in by_face.values() {
        if per.len() < 2 {
            continue;
        }
        let positions: BTreeSet<_> = per
            .values()
            .flat_map(|list| list.iter().map(|(at, _)| *at))
            .collect();
        let indexes: BTreeMap<_, _> = positions
            .into_iter()
            .enumerate()
            .map(|(i, at)| (at, i))
            .collect();
        let mut before: BTreeMap<String, Vec<BoolVar>> = BTreeMap::new();
        let mut after: BTreeMap<String, Vec<BoolVar>> = BTreeMap::new();
        for (group, exits) in per {
            let left: Vec<_> = (0..indexes.len())
                .map(|i| m.bool(&format!("rb{region}_{i}")))
                .collect();
            let right: Vec<_> = (0..indexes.len())
                .map(|i| m.bool(&format!("ra{region}_{i}")))
                .collect();
            region += 1;
            for i in 0..indexes.len() {
                if i > 0 {
                    m.implication(left[i - 1], left[i]);
                    m.implication(right[i], right[i - 1]);
                }
                for &(at, lit) in exits {
                    if indexes[&at] == i {
                        m.implication(lit, left[i]);
                        m.implication(lit, right[i]);
                    }
                }
            }
            before.insert(group.clone(), left);
            after.insert(group.clone(), right);
        }
        for (other, exits) in per {
            for &(at, lit) in exits {
                let n = indexes[&at];
                if n == 0 || n + 1 == indexes.len() {
                    continue;
                }
                for group in per.keys().filter(|g| *g != other) {
                    m.or(&[!lit, !before[group][n - 1], !after[group][n + 1]], &[]);
                }
            }
        }
    }
    Ok(())
}
fn shared_site_key(
    net: &str,
    q: Point,
    reps: &mut BTreeMap<(String, i64, i64), Vec<Point>>,
) -> Result<(String, i64, i64), String> {
    let (x, y) = (
        super::numeric::quantize(q[0], 0.01)?,
        super::numeric::quantize(q[1], 0.01)?,
    );
    for dx in [-1, 0, 1] {
        for dy in [-1, 0, 1] {
            for &r in reps
                .get(&(net.into(), x + dx, y + dy))
                .into_iter()
                .flatten()
            {
                if em::distance(r, q) < geo::SHARE_TOL {
                    return Ok((
                        net.into(),
                        super::numeric::quantize(r[0], 1e-9)?,
                        super::numeric::quantize(r[1], 1e-9)?,
                    ));
                }
            }
        }
    }
    reps.entry((net.into(), x, y)).or_default().push(q);
    Ok((
        net.into(),
        super::numeric::quantize(q[0], 1e-9)?,
        super::numeric::quantize(q[1], 1e-9)?,
    ))
}

pub fn colour_runs(
    runs: &BTreeMap<String, Vec<String>>,
    edges: &BTreeSet<(String, String)>,
    together: &[(String, String)],
    want: &BTreeMap<String, String>,
) -> Result<(BTreeMap<String, String>, Vec<String>), String> {
    if runs.is_empty() {
        return Ok((BTreeMap::new(), vec![]));
    }
    let mut m = Model::new();
    let mut colours: BTreeMap<String, BTreeMap<String, BoolVar>> = BTreeMap::new();
    let mut ok = BTreeMap::new();
    for (i, (key, layers)) in runs.iter().enumerate() {
        let vars: BTreeMap<_, _> = layers
            .iter()
            .map(|layer| (layer.clone(), m.bool(&format!("c{i}_{layer}"))))
            .collect();
        let present = m.bool(&format!("ok{i}"));
        let mut terms = sum_terms(vars.values().copied());
        terms.push((present.into(), -1));
        m.eq(&terms, 0, &[])?;
        colours.insert(key.clone(), vars);
        ok.insert(key.clone(), present);
    }
    for (a, b) in edges {
        for layer in colours[a]
            .keys()
            .filter(|layer| colours[b].contains_key(*layer))
        {
            m.or(&[!colours[a][layer], !colours[b][layer]], &[]);
        }
    }
    for (a, b) in together {
        let all: BTreeSet<_> = colours[a]
            .keys()
            .chain(colours[b].keys())
            .cloned()
            .collect();
        for layer in all {
            match (colours[a].get(&layer), colours[b].get(&layer)) {
                (Some(&x), Some(&y)) => m.eq(&[(x.into(), 1), (y.into(), -1)], 0, &[])?,
                (Some(&x), None) | (None, Some(&x)) => m.eq(&[(x.into(), 1)], 0, &[])?,
                _ => {}
            }
        }
    }
    let mut objective: Vec<_> = ok.values().map(|&lit| (lit.into(), 1000)).collect();
    for (key, vars) in &colours {
        let mut layers = runs[key].clone();
        layers.sort_by_key(|layer| {
            (
                !matches!(layer.as_str(), "F.Cu" | "B.Cu"),
                runs[key].iter().position(|l| l == layer).unwrap(),
            )
        });
        for (layer, &lit) in vars {
            let rank = if want.get(key) == Some(layer) {
                0
            } else {
                1 + layers.iter().position(|l| l == layer).unwrap() as i64
            };
            objective.push((lit.into(), -rank));
        }
    }
    maximize(&mut m, &objective)?;
    let response = super::cp_sat_bridge::solve(
        &m.inner,
        &SatParameters {
            num_workers: Some(1),
            max_deterministic_time: Some(30.0),
            ..Default::default()
        },
        None,
    )?;
    if !feasible(&response) {
        return Ok((BTreeMap::new(), runs.keys().cloned().collect()));
    }
    let out: BTreeMap<_, _> = colours
        .iter()
        .flat_map(|(key, layers)| {
            layers
                .iter()
                .filter(|(_, lit)| value(&response, **lit))
                .map(move |(layer, _)| (key.clone(), layer.clone()))
        })
        .collect();
    let missing = runs
        .keys()
        .filter(|key| !out.contains_key(*key))
        .cloned()
        .collect();
    Ok((out, missing))
}
fn colour_selected_runs(
    input: &JointInput,
    bm: &JointMenus,
    cap_groups: &BTreeSet<BTreeSet<Member>>,
    cap_bicliques: &BTreeSet<(BTreeSet<Member>, BTreeSet<Member>)>,
    chosen: &mut BTreeMap<String, OptionMove>,
    selected: &mut BTreeMap<String, usize>,
    pieces: &mut BTreeMap<String, Piece>,
) -> Result<Vec<String>, String> {
    let runs: BTreeMap<_, _> = chosen
        .iter()
        .filter_map(|(key, option)| {
            if let OptionMove::Escape { movement } = option {
                (movement.layer == RUN).then(|| (key.clone(), movement.runs_on.clone()))
            } else {
                None
            }
        })
        .collect();
    let sel: BTreeSet<_> = runs
        .keys()
        .map(|key| Member {
            ball: key.clone(),
            index: selected[key],
        })
        .collect();
    let mut edges = BTreeSet::new();
    for group in cap_groups.iter().cloned().chain(
        cap_bicliques
            .iter()
            .map(|(a, b)| a.union(b).cloned().collect()),
    ) {
        let members: Vec<_> = group
            .iter()
            .filter(|m| sel.contains(*m))
            .map(|m| m.ball.clone())
            .collect();
        for i in 0..members.len() {
            for j in i + 1..members.len() {
                edges.insert((members[i].clone(), members[j].clone()));
            }
        }
    }
    let together: Vec<_> = bm
        .pairs
        .iter()
        .filter(|p| runs.contains_key(&p.p) && runs.contains_key(&p.n))
        .map(|p| (p.p.clone(), p.n.clone()))
        .collect();
    let want: BTreeMap<_, _> = runs
        .keys()
        .filter(|key| input.bus.contains(em::short_name(&bm.items[*key].net)))
        .filter_map(|key| {
            input
                .preferred
                .get(em::short_name(&bm.items[key].net))
                .and_then(|p| p.layer.clone())
                .map(|layer| (key.clone(), layer))
        })
        .collect();
    let (colours, uncoloured) = colour_runs(&runs, &edges, &together, &want)?;
    for (key, layer) in colours {
        if let OptionMove::Escape { movement } = &mut chosen.get_mut(&key).unwrap() {
            movement.layer = layer.clone();
            for leg in &mut movement.legs {
                if leg.layer == RUN {
                    leg.layer = layer.clone();
                }
            }
            pieces.get_mut(&key).unwrap().legs = movement.legs.clone();
        }
    }
    for key in &uncoloured {
        chosen.remove(key);
        selected.remove(key);
        pieces.remove(key);
    }
    Ok(uncoloured)
}

#[allow(clippy::too_many_arguments)]
fn finish_plan(
    input: &JointInput,
    bm: JointMenus,
    chosen: BTreeMap<String, OptionMove>,
    selected: BTreeMap<String, usize>,
    pieces: BTreeMap<String, Piece>,
    response: CpSolverResponse,
    n_conflicts: usize,
    n_groups: usize,
    n_bicliques: usize,
    let_go: Vec<String>,
    phase1: String,
    phase1_seconds: f64,
    phase1_tiers: Vec<TierReport>,
    uncoloured: Vec<String>,
    hands_held: Vec<String>,
    hands_free: Vec<String>,
    seconds: f64,
    mut site_representatives: BTreeMap<(String, i64, i64), Vec<Point>>,
) -> Result<JointPlan, String> {
    let mut clashes = Vec::new();
    let keys: Vec<_> = chosen.keys().cloned().collect();
    for (i, a) in keys.iter().enumerate() {
        let bounds = geo::piece_box(&pieces[a], input.sizes.grow);
        for b in &keys[i + 1..] {
            let other = geo::piece_box(&pieces[b], input.sizes.grow);
            if bounds[0] > other[2]
                || other[0] > bounds[2]
                || bounds[1] > other[3]
                || other[1] > bounds[3]
            {
                continue;
            }
            if geo::piece_hit(
                &pieces[a],
                &pieces[b],
                pieces[a].net == pieces[b].net,
                &input.sizes,
            ) {
                clashes.push((a.clone(), b.clone()));
            }
        }
    }
    let mut gap_sites = BTreeSet::new();
    let mut counts: BTreeMap<(&str, &str), usize> = BTreeMap::new();
    let mut hints = Vec::new();
    let pair_of: BTreeMap<_, _> = bm
        .pairs
        .iter()
        .flat_map(|p| [(p.p.clone(), p.base.clone()), (p.n.clone(), p.base.clone())])
        .collect();
    let mut faces = BTreeMap::new();
    let mut run_layers = BTreeMap::new();
    let mut climbed = 0;
    let mut streets = 0;
    let mut dropped_in_pad = 0;
    let mut objective = 0i64;
    for (key, option) in &chosen {
        let net = em::short_name(&bm.items[key].net);
        let category = if input.bus.contains(net) {
            "bus"
        } else if input.drops.contains(net) {
            "plane"
        } else {
            "other"
        };
        let name = match option {
            OptionMove::Escape { .. } => "escape",
            OptionMove::Strap { .. } => "strap",
            OptionMove::Drop { .. } => "drop",
        };
        *counts.entry((category, name)).or_default() += 1;
        let joint_option = JointOption {
            option: option.clone(),
            piece: pieces[key].clone(),
        };
        let weight = if category == "bus" {
            W_BUS
        } else if category == "plane" {
            W_DROP
        } else {
            W_OTHER
        };
        objective = objective
            .checked_add(weight - costs(input, &bm, key, &joint_option)?)
            .ok_or("joint objective overflow")?;
        let mut hint = match option {
            OptionMove::Escape { movement: m } => {
                climbed += usize::from(m.climb > 0);
                streets += usize::from(m.street);
                let direction = match m.direction {
                    Direction::Left => "left",
                    Direction::Right => "right",
                    Direction::Up => "up",
                    Direction::Down => "down",
                };
                let kind = match m.kind {
                    MoveKind::Surface => "surface",
                    MoveKind::Dogbone => "dogbone",
                    MoveKind::ViaInPad => "via_in_pad",
                };
                *faces
                    .entry(format!(
                        "{} {direction} {} {kind}",
                        if category == "bus" { "bus" } else { "other" },
                        m.layer
                    ))
                    .or_default() += 1;
                if m.kind != MoveKind::Surface {
                    *run_layers.entry(m.layer.clone()).or_default() += 1;
                }
                let mut hint = super::source_realize::full_move(m, false);
                hint["legs"] = serde_json::to_value(
                    m.legs
                        .iter()
                        .map(|leg| (leg.a, leg.b, leg.layer.clone()))
                        .collect::<Vec<_>>(),
                )
                .map_err(|e| e.to_string())?;
                hint
            }
            OptionMove::Strap { strap } => {
                serde_json::json!({"kind":"strap","to":strap.b,"layer":strap.layer})
            }
            OptionMove::Drop { drop } => {
                if drop.inpad {
                    dropped_in_pad += 1;
                } else {
                    gap_sites.insert(shared_site_key(net, drop.site, &mut site_representatives)?);
                }
                let mut hint = serde_json::json!({"kind":"drop","site":drop.site,"layer":drop.layer,"inpad":drop.inpad});
                if !drop.inpad && drop.radius < input.sizes.via.radius - 1e-9 {
                    hint["via"] = serde_json::json!([
                        em::rounded(2.0 * drop.radius, 4),
                        em::rounded(2.0 * drop.drill_radius, 4)
                    ]);
                }
                hint
            }
        };
        hint["strict"] = serde_json::Value::Bool(true);
        if matches!(option, OptionMove::Escape { .. }) {
            if let Some(pair) = pair_of.get(key) {
                hint["pair"] = serde_json::Value::String(pair.clone());
            }
        }
        hints.push(JointHint {
            ball: key.clone(),
            at: [
                em::rounded(bm.items[key].point[0], 3),
                em::rounded(bm.items[key].point[1], 3),
            ],
            hint,
        });
    }
    objective = objective
        .checked_sub(C_VIA * gap_sites.len() as i64)
        .ok_or("joint objective overflow")?;
    let count = |category, kind| *counts.get(&(category, kind)).unwrap_or(&0);
    let report = JointReport {
        status: response.status().as_str_name().to_string(),
        balls: bm.items.len(),
        moves: bm.menus.values().map(Vec::len).sum(),
        conflicts: n_conflicts,
        conflict_groups: n_groups,
        conflict_bicliques: n_bicliques,
        bus_escaped: count("bus", "escape"),
        bus_balls: bm
            .items
            .values()
            .filter(|p| input.bus.contains(em::short_name(&p.net)))
            .count(),
        others_escaped: count("other", "escape"),
        others_strapped: count("other", "strap"),
        others_balls: bm
            .items
            .values()
            .filter(|p| input.others.contains(em::short_name(&p.net)))
            .count(),
        dropped: count("plane", "drop"),
        plane_strapped: count("plane", "strap"),
        dropped_in_pad,
        drop_vias: gap_sites.len() + dropped_in_pad,
        plane_balls: bm
            .items
            .values()
            .filter(|p| input.drops.contains(em::short_name(&p.net)))
            .count(),
        no_move: bm
            .items
            .keys()
            .filter(|k| bm.options[*k].is_empty())
            .cloned()
            .collect(),
        unplanned: bm
            .items
            .keys()
            .filter(|k| !chosen.contains_key(*k))
            .cloned()
            .collect(),
        clashes,
        climbed,
        streets,
        faces,
        let_go,
        phase1,
        phase1_seconds,
        phase1_tiers,
        uncoloured,
        run_layers: runs_mode(input).then_some(run_layers),
        pairs: bm.pairs.len(),
        pairs_escaped: bm
            .pairs
            .iter()
            .filter(|p| chosen.contains_key(&p.p) && chosen.contains_key(&p.n))
            .count(),
        hands_held,
        hands_free,
        seconds: seconds + bm.menu_seconds,
        objective: (!chosen.is_empty()).then_some(objective),
        bound: response.best_objective_bound,
        wall_time: response.wall_time,
        branches: response.num_branches,
        conflicts_cp: response.num_conflicts,
    };
    Ok(JointPlan {
        chosen,
        selected_indices: selected,
        pieces,
        hints,
        report,
    })
}
fn runs_mode(input: &JointInput) -> bool {
    input.routing_layers.len() > 2
}
fn normalized_input(input: &JointInput) -> JointInput {
    let mut out = input.clone();
    out.bus = short_set(&input.bus);
    out.others = short_set(&input.others);
    out.drops = short_set(&input.drops)
        .difference(&out.bus)
        .cloned()
        .collect::<BTreeSet<_>>()
        .difference(&out.others)
        .cloned()
        .collect();
    out.preferred = input
        .preferred
        .iter()
        .map(|(n, p)| (em::short_name(n).into(), p.clone()))
        .collect();
    out.hands = input
        .hands
        .iter()
        .map(|(n, h)| (em::short_name(n).into(), h.clone()))
        .collect();
    out.bus_groups = input
        .bus_groups
        .iter()
        .map(|(n, g)| (em::short_name(n).into(), g.clone()))
        .collect();
    let here: BTreeSet<_> = input
        .pads
        .iter()
        .filter(|p| p.net_id != 0)
        .map(|p| em::short_name(&p.net).to_string())
        .filter(|n| out.bus.contains(n) || out.others.contains(n))
        .collect();
    out.pairs = input
        .pairs
        .iter()
        .map(|p| JointPair {
            base: em::short_name(&p.base).into(),
            p: em::short_name(&p.p).into(),
            n: em::short_name(&p.n).into(),
        })
        .filter(|p| !here.contains(&p.base) && here.contains(&p.p) && here.contains(&p.n))
        .collect();
    out
}
