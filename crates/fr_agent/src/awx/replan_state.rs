//! Native state, actual-board verdicts, and accepted-move transitions for replan.
use super::{
    board_snapshot::BoardSnapshot,
    escape_moves::Move,
    evolution_native::{self, NativeWorld},
    replan_types::{AcceptedMove, EndState, NetVerdict, ProbeContext, ReplanVerdict},
    route_evolution::RouteGrade,
};
use crate::board_adapter::BoardAdapter;
use std::{
    collections::{BTreeMap, BTreeSet},
    time::{Duration, Instant},
};

#[derive(Clone)]
pub struct ReplanState {
    pub world: NativeWorld,
    /// Exact operator environment, merged by the coordinator and retained for
    /// model rebuilds after accepted/finalized worlds.
    pub environment_overrides: BTreeMap<String, String>,
    /// Nets named by the fanout sidecar, in RouteSpec order where possible.
    pub names: Vec<String>,
    pub current_grade: RouteGrade,
    pub ends: BTreeMap<String, EndState>,
    pub verdicts: BTreeMap<String, NetVerdict>,
    /// Model output is retained only when present in native seed metadata.
    /// The source Python model's corridor schedule is not serialized in its plan sidecar.
    pub predicted: BTreeMap<String, f64>,
    pub residual: BTreeMap<String, f64>,
    pub worst: Vec<String>,
    pub gatekeepers: Vec<String>,
    /// Per-net provenance for strip/apply semantics. Unchanged nets start at F.
    pub fan_src: BTreeMap<String, BoardAdapter>,
    pub fanout_seed: BoardAdapter,
    /// Captured once so native operators can rebuild geometry from the exact seed.
    pub snapshot: BoardSnapshot,
    pub probe: ProbeContext,
}

fn read_point(value: &serde_json::Value) -> Option<[f64; 2]> {
    let values = value.as_array()?;
    Some([values.first()?.as_f64()?, values.get(1)?.as_f64()?])
}

fn move_value(value: &serde_json::Value, keys: &[&str]) -> Option<Move> {
    keys.iter()
        .find_map(|key| value.get(*key))
        .and_then(|v| serde_json::from_value(v.clone()).ok())
}

fn end_states(world: &NativeWorld) -> BTreeMap<String, EndState> {
    let mut out = BTreeMap::new();
    for (name, value) in world.plan_ends.iter() {
        // NativeWorld stores the `ends` map itself, not the enclosing plan.
        let point_pair = value.as_array();
        let (Some(pair), Some(tooth), Some(exit)) = (
            point_pair,
            point_pair.and_then(|p| p.first()).and_then(read_point),
            point_pair.and_then(|p| p.get(1)).and_then(read_point),
        ) else {
            continue;
        };
        let _ = pair;
        let tooth_layer = world
            .plan_sidecar
            .get("tooth_layer")
            .and_then(|m| m.get(name))
            .and_then(serde_json::Value::as_str)
            .unwrap_or("F.Cu")
            .to_owned();
        let dest_layer = world
            .plan_sidecar
            .get("dest_layer")
            .and_then(|m| m.get(name))
            .and_then(serde_json::Value::as_str)
            .unwrap_or("F.Cu")
            .to_owned();
        out.insert(
            name.clone(),
            EndState {
                tooth,
                exit,
                tooth_layer,
                dest_layer,
                src_move: move_value(value, &["src_move", "source_move"]),
                dst_move: move_value(value, &["dst_move", "destination_move"]),
            },
        );
    }
    out
}

fn net_vias(board: &BoardAdapter, net: &str) -> Result<usize, String> {
    let number = board.net_number(net)?;
    Ok(board
        .board
        .get_vias()
        .iter()
        .filter(|&&key| board.board.item(key).net_numbers().contains(&number))
        .count())
}

fn model_prediction(world: &NativeWorld, net: &str) -> Option<f64> {
    let lane = world.pack_sidecar.get("lanes")?.get(net)?;
    ["predicted_vias", "predicted", "model_vias"]
        .iter()
        .find_map(|key| lane.get(*key).and_then(serde_json::Value::as_f64))
}

fn set_preferences(
    input: &mut super::joint_escape::JointInput,
    ends: &BTreeMap<String, EndState>,
    source: bool,
) {
    use super::joint_escape::PreferredTooth;
    for net in &input.bus {
        let Some(end) = ends.get(net) else { continue };
        let movement = if source {
            end.src_move.as_ref()
        } else {
            end.dst_move.as_ref()
        };
        input.preferred.insert(
            net.clone(),
            PreferredTooth {
                tooth: if source { end.tooth } else { end.exit },
                direction: movement.map(|m| m.direction),
                layer: Some(if source {
                    end.tooth_layer.clone()
                } else {
                    end.dest_layer.clone()
                }),
                kind: movement.map(|m| m.kind),
            },
        );
    }
}

fn build_probe_context(
    world: &NativeWorld,
    snapshot: &BoardSnapshot,
    names: &[String],
    ends: &BTreeMap<String, EndState>,
) -> Result<ProbeContext, String> {
    let related = super::integration::related_groups(&world.spec)?;
    let named: BTreeSet<_> = names.iter().cloned().collect();
    let selected = related
        .into_iter()
        .find(|groups| {
            let group_nets: BTreeSet<_> =
                groups.iter().flat_map(|g| g.nets.iter().cloned()).collect();
            named.is_subset(&group_nets)
        })
        .ok_or("native replan nets do not belong to one related source/destination group")?;
    let first = selected
        .first()
        .and_then(|g| g.awx.as_ref())
        .ok_or("native replan group has no AWX end configuration")?;
    let source_part = first.source.clone();
    let destination_part = first.destination.clone();
    let group_ids = selected.iter().map(|g| g.id.clone()).collect();
    let chi = world
        .plan_sidecar
        .get("chi")
        .and_then(serde_json::Value::as_i64)
        .map(|v| v as i32)
        .filter(|v| *v == 1 || *v == -1)
        .ok_or("fanout plan lacks valid source chirality")?;
    let deadline = Instant::now() + Duration::from_millis(world.spec.execution.budget_ms.max(1));
    let mut source_input = super::integration::joint_input(
        snapshot,
        &world.spec,
        &selected,
        &source_part,
        false,
        deadline,
    )?;
    let mut destination_input = super::integration::joint_input(
        snapshot,
        &world.spec,
        &selected,
        &destination_part,
        true,
        deadline,
    )?;
    set_preferences(&mut source_input, ends, true);
    set_preferences(&mut destination_input, ends, false);
    let source_menus = super::joint_escape::build_menus(&source_input, snapshot)?;
    let destination_menus = super::joint_escape::build_menus(&destination_input, snapshot)?;
    let source_plan = super::joint_escape::solve_menus(&source_input, source_menus.clone())?;
    let destination_plan =
        super::joint_escape::solve_menus(&destination_input, destination_menus.clone())?;
    let rules = world
        .spec
        .awx
        .as_ref()
        .ok_or("native replan requires AWX routing rules")?
        .rules
        .clone();
    Ok(ProbeContext {
        source_part,
        destination_part,
        groups: group_ids,
        chi,
        source_input,
        destination_input,
        source_menus,
        destination_menus,
        source_plan,
        destination_plan,
        rules,
        deadline,
        probe_attempts: 1,
        probe_budget_x: 4.0,
        probe_ladder: "open".into(),
        probe_smooth: false,
        coupled: "census".into(),
        grade_mode: "sub".into(),
        exact_pages: false,
        swimmer_page_price: 2.0,
        swim_vias: 2.0,
        plan_judge: String::new(),
    })
}

fn infer_current_moves(ends: &mut BTreeMap<String, EndState>, context: &ProbeContext) {
    use super::escape_moves;
    for (name, end) in ends.iter_mut() {
        let short = escape_moves::short_name(name);
        let nearest = |menus: &BTreeMap<String, Vec<Move>>, point: [f64; 2], layer: &str| {
            menus
                .get(name)
                .or_else(|| menus.get(short))?
                .iter()
                .filter(|m| m.layer == layer || m.layer == escape_moves::RUN || layer == "F.Cu")
                .min_by(|a, b| {
                    let ad = escape_moves::distance(a.exit_pt, point);
                    let bd = escape_moves::distance(b.exit_pt, point);
                    ad.total_cmp(&bd)
                        .then_with(|| (a.layer != layer).cmp(&(b.layer != layer)))
                })
                .filter(|m| escape_moves::distance(m.exit_pt, point) <= 0.05)
                .cloned()
        };
        end.src_move = nearest(&context.source_menus.menus, end.tooth, &end.tooth_layer);
        end.dst_move = nearest(&context.destination_menus.menus, end.exit, &end.dest_layer);
    }
}

impl ReplanState {
    pub fn new(world: NativeWorld) -> Result<Self, String> {
        let mut names: Vec<_> = world
            .spec
            .nets
            .iter()
            .filter(|n| world.plan_ends.contains_key(&n.name))
            .map(|n| n.name.clone())
            .collect();
        for name in world.plan_ends.keys() {
            if !names.contains(name) {
                names.push(name.clone());
            }
        }
        if names.is_empty() {
            return Err("native replan sidecar names no routed nets".into());
        }
        let awx = world
            .spec
            .awx
            .as_ref()
            .ok_or("native replan requires AWX spec")?;
        let snapshot = BoardSnapshot::capture(&world.fanout, &awx.components, &awx.via_drills)?;
        let current_grade = evolution_native::grade_board(&world.routed, &world.spec)?;
        let fanout_seed = world.fanout.clone();
        let fan_src = names
            .iter()
            .map(|name| (name.clone(), fanout_seed.clone()))
            .collect();
        let verdict_evidence = world.verdict_evidence.clone();
        let mut ends = end_states(&world);
        let probe = build_probe_context(&world, &snapshot, &names, &ends)?;
        infer_current_moves(&mut ends, &probe);
        let probe = build_probe_context(&world, &snapshot, &names, &ends)?;
        let mut state = Self {
            world,
            environment_overrides: BTreeMap::new(),
            names,
            current_grade,
            ends,
            verdicts: BTreeMap::new(),
            predicted: BTreeMap::new(),
            residual: BTreeMap::new(),
            worst: Vec::new(),
            gatekeepers: Vec::new(),
            fan_src,
            fanout_seed,
            snapshot,
            probe,
        };
        state.merge_verdict_evidence(verdict_evidence);
        state.refresh_verdict(3, 3, 2, 3)?;
        Ok(state)
    }

    pub fn from_evolution_world(
        world: super::route_evolution::EvolutionWorld<NativeWorld>,
    ) -> Result<Self, String> {
        let mut state = Self::new(world.data)?;
        state.current_grade = world.grade;
        Ok(state)
    }

    pub fn set_environment_overrides(&mut self, environment_overrides: BTreeMap<String, String>) {
        self.environment_overrides = environment_overrides;
    }

    /// Rebuild derived state after a coordinator installs a finalized world.
    /// Per-net fanout provenance remains owned by the finalizer.
    pub fn refresh_world(&mut self) -> Result<(), String> {
        let prior_ends = self.ends.clone();
        let probe_config = (
            self.probe.probe_attempts,
            self.probe.probe_budget_x,
            self.probe.probe_ladder.clone(),
            self.probe.probe_smooth,
            self.probe.coupled.clone(),
            self.probe.grade_mode.clone(),
            self.probe.exact_pages,
            self.probe.swimmer_page_price,
            self.probe.swim_vias,
            self.probe.plan_judge.clone(),
        );
        let awx = self
            .world
            .spec
            .awx
            .as_ref()
            .ok_or("native replan requires AWX spec")?;
        self.snapshot =
            BoardSnapshot::capture(&self.world.fanout, &awx.components, &awx.via_drills)?;
        self.current_grade = evolution_native::grade_board(&self.world.routed, &self.world.spec)?;
        let mut rebuilt_ends = end_states(&self.world);
        for (name, current) in &prior_ends {
            if let Some(rebuilt) = rebuilt_ends.get_mut(name) {
                if let Some(movement) = &current.src_move {
                    rebuilt.src_move = Some(movement.clone());
                    rebuilt.tooth_layer = current.tooth_layer.clone();
                }
                if let Some(movement) = &current.dst_move {
                    rebuilt.dst_move = Some(movement.clone());
                    rebuilt.dest_layer = current.dest_layer.clone();
                }
            }
        }
        self.ends = rebuilt_ends;
        let first_probe =
            build_probe_context(&self.world, &self.snapshot, &self.names, &self.ends)?;
        infer_current_moves(&mut self.ends, &first_probe);
        self.probe = build_probe_context(&self.world, &self.snapshot, &self.names, &self.ends)?;
        self.apply_probe_config(
            probe_config.0,
            probe_config.1,
            &probe_config.2,
            probe_config.3,
        )?;
        self.apply_replan_modes(&probe_config.4, &probe_config.5)?;
        self.apply_cost_config(probe_config.6, probe_config.7, probe_config.8)?;
        self.apply_plan_judge(&probe_config.9)?;
        self.refresh_verdict(3, 3, 2, 3)?;
        Ok(())
    }

    pub fn apply_probe_config(
        &mut self,
        attempts: usize,
        budget_x: f64,
        ladder: &str,
        smooth: bool,
    ) -> Result<(), String> {
        if attempts == 0 {
            return Err("replan probe attempts must be positive".into());
        }
        if !budget_x.is_finite() || budget_x <= 0.0 {
            return Err("replan probe budget multiplier must be finite and positive".into());
        }
        if !matches!(ladder, "open" | "full") {
            return Err(format!("unsupported replan probe ladder {ladder}"));
        }
        self.probe.probe_attempts = attempts;
        self.probe.probe_budget_x = budget_x;
        self.probe.probe_ladder = ladder.to_owned();
        self.probe.probe_smooth = smooth;
        Ok(())
    }

    pub fn apply_replan_modes(&mut self, coupled: &str, grade_mode: &str) -> Result<(), String> {
        if coupled != "census" {
            return Err(format!("unsupported replan coupling mode {coupled}"));
        }
        if !matches!(grade_mode, "sub" | "inproc") {
            return Err(format!("unsupported replan grade mode {grade_mode}"));
        }
        self.probe.coupled = coupled.to_owned();
        self.probe.grade_mode = grade_mode.to_owned();
        Ok(())
    }

    pub fn apply_cost_config(
        &mut self,
        exact_pages: bool,
        swimmer_page_price: f64,
        swim_vias: f64,
    ) -> Result<(), String> {
        if !swimmer_page_price.is_finite() || swimmer_page_price < 0.0 {
            return Err("replan swimmer page price must be finite and nonnegative".into());
        }
        if !swim_vias.is_finite() || swim_vias < 0.0 {
            return Err("replan swim vias price must be finite and nonnegative".into());
        }
        self.probe.exact_pages = exact_pages;
        self.probe.swimmer_page_price = swimmer_page_price;
        self.probe.swim_vias = swim_vias;
        Ok(())
    }

    pub fn apply_plan_judge(&mut self, plan_judge: &str) -> Result<(), String> {
        if !matches!(plan_judge, "" | "count" | "flat" | "ends") {
            return Err(format!("unsupported PLAN_JUDGE={plan_judge:?}"));
        }
        self.probe.plan_judge = plan_judge.to_owned();
        Ok(())
    }

    /// Install predictions produced by the native braid-plan evaluator.
    /// Residuals are always measured against the actual lane vias on R.
    pub fn set_model_predictions(&mut self, predictions: BTreeMap<String, f64>) {
        self.predicted = predictions;
        for (name, predicted) in &self.predicted {
            if let Some(verdict) = self.verdicts.get_mut(name) {
                verdict.predicted_vias = Some(*predicted);
            }
            if let Some(verdict) = self.verdicts.get(name) {
                if !verdict.refused {
                    self.residual
                        .insert(name.clone(), verdict.actual_vias as f64 - predicted);
                }
            }
        }
    }

    pub fn set_model_output(&mut self, prediction: super::replan_cost::ReplanPrediction) {
        self.predicted.clear();
        self.residual.clear();
        for (name, model) in prediction.per_net {
            self.predicted.insert(name.clone(), model.predicted_vias);
            let verdict = self.verdicts.entry(name.clone()).or_default();
            verdict.model_page = model.page;
            verdict.predicted_vias = Some(model.predicted_vias);
            if !verdict.refused {
                self.residual
                    .insert(name, verdict.actual_vias as f64 - model.predicted_vias);
            }
        }
    }

    pub fn current_model_prediction(
        &self,
    ) -> Result<BTreeMap<String, super::replan_cost::ReplanNetPrediction>, String> {
        let mut result = BTreeMap::new();
        for name in &self.names {
            if !self.ends.contains_key(name) {
                continue;
            }
            let verdict = self
                .verdicts
                .get(name)
                .ok_or_else(|| format!("current model verdict missing for {name}"))?;
            let predicted_vias = verdict
                .predicted_vias
                .ok_or_else(|| format!("current B.model prediction missing for {name}"))?;
            result.insert(
                name.clone(),
                super::replan_cost::ReplanNetPrediction {
                    page: verdict.model_page.clone(),
                    predicted_vias,
                },
            );
        }
        Ok(result)
    }

    /// Add braid verdict facts that are delivered separately from the seed
    /// NativeWorld (the native session format itself does not contain a log).
    pub fn merge_verdict_evidence(&mut self, evidence: BTreeMap<String, NetVerdict>) {
        for (name, incoming) in evidence {
            let target = self.verdicts.entry(name).or_default();
            if incoming.class.is_some() {
                target.class = incoming.class;
            }
            if incoming.predicted_vias.is_some() {
                target.predicted_vias = incoming.predicted_vias;
                target.model_page = incoming.model_page;
            }
            if incoming.refused {
                target.refused = true;
            }
            if incoming.walled_at.is_some() {
                target.walled_at = incoming.walled_at;
            }
            if incoming.launch_rank.is_some() {
                target.launch_rank = incoming.launch_rank;
            }
            if incoming.target_rank.is_some() {
                target.target_rank = incoming.target_rank;
            }
            if incoming.last_call.is_some() {
                target.last_call = incoming.last_call;
            }
            if !incoming.rip.is_empty() {
                target.rip = incoming.rip;
            }
            if !incoming.frontier.is_empty() {
                target.frontier = incoming.frontier;
            }
            if !incoming.mincut.is_empty() {
                target.mincut = incoming.mincut;
            }
            for blocker in incoming.census {
                if !target.census.contains(&blocker) {
                    target.census.push(blocker);
                }
            }
        }
    }

    /// Rebuild actual per-net via prices from routed R relative to the fanout
    /// copper on F, then carry the blocker census into the gatekeeper ranking.
    pub fn refresh_verdict(
        &mut self,
        worst_limit: usize,
        min_lane_vias: usize,
        gate_min: usize,
        gate_limit: usize,
    ) -> Result<ReplanVerdict, String> {
        for name in &self.names {
            let actual_vias = net_vias(&self.world.routed, name)?;
            let fanout_vias = net_vias(&self.world.fanout, name)?;
            let entry = self.verdicts.entry(name.clone()).or_default();
            entry.actual_vias = actual_vias;
            entry.lane_vias = actual_vias.saturating_sub(fanout_vias);
        }
        let named: BTreeSet<_> = self.names.iter().cloned().collect();
        let prediction = super::replan_prediction::evaluate_model(self, &named)?;
        self.set_model_output(prediction);

        let mut verdicts = BTreeMap::new();
        let mut predicted = BTreeMap::new();
        let mut residual = BTreeMap::new();
        for name in &self.names {
            let actual_vias = net_vias(&self.world.routed, name)?;
            let fanout_vias = net_vias(&self.world.fanout, name)?;
            let lane_vias = actual_vias.saturating_sub(fanout_vias);
            let evidence = self
                .world
                .pack_sidecar
                .get("lanes")
                .and_then(|v| v.get(name));
            let carried = self.verdicts.get(name).cloned().unwrap_or_default();
            let mut census = carried.mincut.clone();
            let mut frontier = carried.frontier.clone();
            frontier.sort_by_key(|(_, cells)| std::cmp::Reverse(*cells));
            census.extend(frontier.into_iter().map(|(net, _)| net));
            census.extend(
                self.world
                    .census_sidecar
                    .as_ref()
                    .and_then(|v| v.get(name))
                    .and_then(serde_json::Value::as_array)
                    .into_iter()
                    .flatten()
                    .filter_map(serde_json::Value::as_str)
                    .map(str::to_owned),
            );
            let mut seen_census = BTreeSet::new();
            census.retain(|blocker| blocker != name && seen_census.insert(blocker.clone()));
            census.truncate(4);
            let prediction = self
                .predicted
                .get(name)
                .copied()
                .or_else(|| model_prediction(&self.world, name))
                .or(carried.predicted_vias);
            let entry = NetVerdict {
                class: evidence
                    .and_then(|v| v.get("class").or_else(|| v.get("page")))
                    .and_then(serde_json::Value::as_str)
                    .map(str::to_owned)
                    .or(carried.class),
                predicted_vias: prediction,
                model_page: carried.model_page,
                lane_vias,
                actual_vias,
                refused: evidence
                    .and_then(|v| v.get("refused"))
                    .and_then(serde_json::Value::as_bool)
                    .unwrap_or(carried.refused),
                walled_at: evidence
                    .and_then(|v| v.get("walled_at"))
                    .and_then(serde_json::Value::as_str)
                    .map(str::to_owned)
                    .or(carried.walled_at),
                launch_rank: evidence
                    .and_then(|v| v.get("launch_rank"))
                    .and_then(serde_json::Value::as_u64)
                    .map(|v| v as usize)
                    .or(carried.launch_rank),
                target_rank: evidence
                    .and_then(|v| v.get("target_rank"))
                    .and_then(serde_json::Value::as_u64)
                    .map(|v| v as usize)
                    .or(carried.target_rank),
                last_call: evidence
                    .and_then(|v| v.get("last_call"))
                    .and_then(serde_json::Value::as_u64)
                    .map(|v| v as usize)
                    .or(carried.last_call),
                rip: carried.rip,
                frontier: carried.frontier,
                mincut: carried.mincut,
                census: if census.is_empty() {
                    carried.census
                } else {
                    census
                },
            };
            if let Some(value) = prediction {
                predicted.insert(name.clone(), value);
                if !entry.refused {
                    residual.insert(name.clone(), actual_vias as f64 - value);
                }
            }
            verdicts.insert(name.clone(), entry);
        }

        let mut worst: Vec<_> = self
            .names
            .iter()
            .filter(|n| verdicts.get(*n).is_some_and(|v| v.refused))
            .cloned()
            .collect();
        let mut non_refused: Vec<_> = self
            .names
            .iter()
            .filter(|n| {
                verdicts
                    .get(*n)
                    .is_some_and(|v| !v.refused && v.lane_vias >= min_lane_vias)
            })
            .cloned()
            .collect();
        non_refused.sort_by_key(|name| {
            let v = &verdicts[name];
            (
                std::cmp::Reverse(v.lane_vias),
                v.class.as_deref() != Some("swim"),
                name.clone(),
            )
        });
        worst.extend(non_refused.into_iter().take(worst_limit));

        let bad: BTreeSet<_> = worst.iter().cloned().collect();
        let mut gate_scores = BTreeMap::<String, usize>::new();
        for name in &worst {
            if let Some(v) = verdicts.get(name) {
                for blocker in &v.census {
                    if blocker != name && !bad.contains(blocker) {
                        *gate_scores.entry(blocker.clone()).or_default() += 1;
                    }
                }
            }
        }
        let mut gatekeepers: Vec<_> = gate_scores
            .iter()
            .filter(|(_, count)| **count >= gate_min)
            .map(|(name, _)| name.clone())
            .collect();
        gatekeepers.sort_by_key(|name| (std::cmp::Reverse(gate_scores[name]), name.clone()));
        gatekeepers.truncate(gate_limit);

        self.verdicts = verdicts.clone();
        self.predicted = predicted.clone();
        self.residual = residual.clone();
        self.worst = worst.clone();
        self.gatekeepers = gatekeepers.clone();
        Ok(ReplanVerdict {
            grade: self.current_grade.clone(),
            nets: verdicts,
            predicted,
            residual,
            worst,
            gatekeepers,
        })
    }

    /// Commit a previously accepted physical probe atomically to the active
    /// routed world, then make subsequent decisions see its achieved geometry.
    pub fn advance(
        &mut self,
        accepted: AcceptedMove,
        routed_after: BoardAdapter,
        grade_after: RouteGrade,
        fanout_after: Option<BoardAdapter>,
        relaid: &[String],
    ) -> Result<(), String> {
        if !self.names.contains(&accepted.net) {
            return Err(format!(
                "accepted replan net {} is not active",
                accepted.net
            ));
        }
        let probe_config = (
            self.probe.probe_attempts,
            self.probe.probe_budget_x,
            self.probe.probe_ladder.clone(),
            self.probe.probe_smooth,
            self.probe.coupled.clone(),
            self.probe.grade_mode.clone(),
            self.probe.exact_pages,
            self.probe.swimmer_page_price,
            self.probe.swim_vias,
            self.probe.plan_judge.clone(),
        );
        self.world.routed = routed_after;
        self.current_grade = grade_after;
        if let Some(fanout) = fanout_after {
            self.world.fanout = fanout.clone();
            self.fanout_seed = fanout.clone();
            if let Some(awx) = self.world.spec.awx.as_ref() {
                self.snapshot = BoardSnapshot::capture(&fanout, &awx.components, &awx.via_drills)?;
            }
            let touched: BTreeSet<_> = std::iter::once(accepted.net.clone())
                .chain(relaid.iter().cloned())
                .chain(accepted.comove_got.keys().cloned())
                .collect();
            for name in touched {
                if self.names.contains(&name) {
                    self.fan_src.insert(name, fanout.clone());
                }
            }
        }
        if let Some(movement) = accepted.src_got {
            let end = self.ends.entry(accepted.net.clone()).or_default();
            end.tooth = movement.exit_pt;
            end.tooth_layer = movement.layer.clone();
            end.src_move = Some(movement);
        }
        if let Some(movement) = accepted.dst_got {
            let end = self.ends.entry(accepted.net.clone()).or_default();
            end.exit = movement.exit_pt;
            end.dest_layer = movement.layer.clone();
            end.dst_move = Some(movement);
        }
        for (name, movement) in accepted.comove_got {
            let end = self.ends.entry(name).or_default();
            end.exit = movement.exit_pt;
            end.dest_layer = movement.layer.clone();
            end.dst_move = Some(movement);
        }
        for (name, end) in &self.ends {
            self.world
                .plan_ends
                .insert(name.clone(), serde_json::json!([end.tooth, end.exit]));
        }
        self.probe = build_probe_context(&self.world, &self.snapshot, &self.names, &self.ends)?;
        self.apply_probe_config(
            probe_config.0,
            probe_config.1,
            &probe_config.2,
            probe_config.3,
        )?;
        self.apply_replan_modes(&probe_config.4, &probe_config.5)?;
        self.apply_cost_config(probe_config.6, probe_config.7, probe_config.8)?;
        self.apply_plan_judge(&probe_config.9)?;
        self.refresh_verdict(3, 3, 2, 3)?;
        Ok(())
    }
}
