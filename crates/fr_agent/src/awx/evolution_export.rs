//! Export a complete native evolution seed bundle from an actual routed world.

use super::{
    evolution_native::{NativeWorld, ensure_same_layout_with_spec, grade_board},
    replan_finalize::{update_pack_sidecar, update_plan_sidecar},
    replan_state::ReplanState,
    replan_types::NetVerdict,
    whole_context::WholeContext,
    whole_frame::FlowFrame,
};
use crate::{
    board_adapter::BoardAdapter,
    spec::{AdvancedConstraints, AwxSpec, Channel, EndpointGroup, NetRule, RouteGroup, RouteSpec},
};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::{
    collections::{BTreeMap, BTreeSet},
    fs::{self, OpenOptions},
    io::Write,
    path::{Path, PathBuf},
};

pub struct NativeSeedExportInput<'a> {
    /// The immutable Specctra board used to load both actual board states.
    pub board_dsn: &'a Path,
    /// Must be a new directory under an existing parent.
    pub output_dir: &'a Path,
    pub routed: &'a BoardAdapter,
    pub fanout: &'a BoardAdapter,
    pub spec: &'a RouteSpec,
    pub requested_k: i64,
    pub destination: &'a str,
    pub names: &'a [String],
    pub context: &'a WholeContext,
    pub flow: FlowFrame,
    /// Actual plan seed carrying chi and per-net berth signatures. D16 adds
    /// current board-space ends, layers, and directions to this value.
    pub plan_seed: &'a Value,
    /// Optional prior blocker census. Missing means there was no census file.
    pub census: Option<&'a Value>,
    /// Captured per-net facts; no absent values are filled with synthetic defaults.
    pub verdicts: &'a BTreeMap<String, NetVerdict>,
}

#[derive(Serialize)]
struct Manifest {
    format: &'static str,
    requested_k: i64,
    destination: String,
    active_nets: Vec<String>,
    runtime_scope: NativeSeedScope,
    routed_dsn: String,
    routed_ses: &'static str,
    fanout_dsn: String,
    fanout_ses: &'static str,
    pack: &'static str,
    plan: &'static str,
    #[serde(skip_serializing_if = "Option::is_none")]
    census: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    verdicts: Option<&'static str>,
    #[serde(skip_serializing_if = "Option::is_none")]
    native_vias: Option<Vec<NativeViaDrill>>,
    grade: SeedGrade,
}

/// Explicit drill data for native-created padstacks. SES carries their copper
/// shapes, while this manifest preserves the exact drill across seed reloads.
#[derive(Clone, Debug, Deserialize, Serialize)]
pub(super) struct NativeViaDrill {
    pub padstack: String,
    pub diameter_mm: f64,
    pub drill_mm: f64,
    /// Nets on which this explicit native via stack occurs in the exported
    /// routed/fanout boards. This is the authorization scope for SES replay.
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub nets: Vec<String>,
}

/// Exact route scope captured from the native execution, without source or
/// destination file paths. The importing job supplies its own I/O paths and
/// runtime orchestration settings; this restores the actual route model.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(super) struct NativeSeedScope {
    pub version: u32,
    pub route_remaining: bool,
    pub allow_partial_output: bool,
    pub protected_nets: Vec<String>,
    pub nets: Vec<NetRule>,
    pub endpoint_groups: Vec<EndpointGroup>,
    pub channels: Vec<Channel>,
    pub groups: Vec<RouteGroup>,
    pub advanced: Option<AdvancedConstraints>,
    pub awx: AwxSpec,
}

#[derive(Serialize)]
struct SeedGrade {
    opens: Vec<String>,
    drc: usize,
    vias: usize,
}

fn write_new(
    directory: &Path,
    name: &str,
    bytes: &[u8],
    created: &mut Vec<String>,
) -> Result<(), String> {
    let path = directory.join(name);
    let mut file = OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&path)
        .map_err(|e| format!("create native seed {}: {e}", path.display()))?;
    let result = file.write_all(bytes).and_then(|_| file.sync_all());
    drop(file);
    if let Err(error) = result {
        let _ = fs::remove_file(&path);
        return Err(format!("write native seed {}: {error}", path.display()));
    }
    created.push(name.to_owned());
    Ok(())
}

fn native_via_drills(board: &BoardAdapter) -> Result<Vec<NativeViaDrill>, String> {
    let mut by_stack: BTreeMap<String, NativeViaDrill> = BTreeMap::new();
    for key in board.board.get_vias() {
        let item = board.board.item(key);
        let stack = item.padstack(&board.board).ok_or("native via has no padstack")?;
        let Some(drill_radius) = stack.get_explicit_drill_radius() else {
            continue;
        };
        let diameter = (stack.from_layer()..=stack.to_layer())
            .filter_map(|layer| stack.get_shape(layer))
            .map(|shape| shape.max_width())
            .fold(0.0_f64, f64::max);
        if diameter <= 0.0 {
            return Err(format!("native via {} has no copper diameter", stack.name));
        }
        let mut nets = BTreeSet::new();
        for &net_id in item.net_numbers() {
            let net = board.board.rules.nets.get(net_id)
                .ok_or("native via refers to an unknown net")?;
            nets.insert(net.name.clone());
        }
        if nets.is_empty() {
            return Err(format!("native via {} has no net association", stack.name));
        }
        let via = by_stack.entry(stack.name.clone()).or_insert_with(|| NativeViaDrill {
            padstack: stack.name.clone(),
            diameter_mm: diameter / board.units_per_mm,
            drill_mm: 2.0 * drill_radius / board.units_per_mm,
            nets: Vec::new(),
        });
        if (via.diameter_mm - diameter / board.units_per_mm).abs() > 1e-9
            || (via.drill_mm - 2.0 * drill_radius / board.units_per_mm).abs() > 1e-9
        {
            return Err(format!("native via {} has inconsistent physical dimensions", stack.name));
        }
        via.nets.extend(nets);
    }
    for via in by_stack.values_mut() {
        via.nets.sort();
        via.nets.dedup();
    }
    Ok(by_stack.into_values().collect())
}

fn session_bytes(board: &BoardAdapter) -> Result<Vec<u8>, String> {
    let mut bytes = Vec::new();
    fr_io::ses_writer::write_ses(&board.board, &mut bytes, "fastroute_native_seed")
        .map_err(|e| format!("write native SES: {e}"))?;
    Ok(bytes)
}

fn validate_plan_seed(plan: &Value, names: &[String]) -> Result<(), String> {
    let root = plan
        .as_object()
        .ok_or("native seed plan seed must be an object")?;
    if !root.get("chi").is_some_and(Value::is_number) {
        return Err("native seed plan seed is missing actual chi".into());
    }
    let signatures = root
        .get("berth_sig")
        .and_then(Value::as_object)
        .ok_or("native seed plan seed is missing actual berth_sig map")?;
    for name in names {
        if !signatures.contains_key(name) {
            return Err(format!(
                "native seed plan seed has no actual berth signature for {name}"
            ));
        }
    }
    Ok(())
}

fn fresh_pack_seed(context: &WholeContext) -> Value {
    serde_json::json!({
        "lanes": {},
        "layers": context.layers,
        "board_edge_clearance": context.cfg.board_edge_clearance,
        "hole_to_hole_clearance": context.cfg.hole_to_hole_clearance,
    })
}

fn native_scope(
    spec: &RouteSpec,
    names: &[String],
    context: &WholeContext,
) -> Result<NativeSeedScope, String> {
    let active: BTreeSet<_> = names.iter().cloned().collect();
    let net_rules: Vec<_> = names
        .iter()
        .map(|name| {
            spec.nets
                .iter()
                .find(|rule| &rule.name == name)
                .cloned()
                .ok_or_else(|| format!("native seed spec has no actual net rule for {name}"))
        })
        .collect::<Result<_, _>>()?;
    let mut active_groups = Vec::new();
    let mut covered = BTreeSet::new();
    for source in &spec.groups {
        let Some(group_awx) = source.awx.as_ref() else {
            continue;
        };
        if group_awx.source != context.source || group_awx.destination != context.destination {
            continue;
        }
        let mut group = source.clone();
        group.nets.retain(|name| active.contains(name));
        if group.nets.is_empty() {
            continue;
        }
        covered.extend(group.nets.iter().cloned());
        if let Some(awx) = group.awx.as_mut() {
            awx.pairs
                .retain(|pair| active.contains(&pair.p) && active.contains(&pair.n));
        }
        active_groups.push(group);
    }
    if covered != active {
        let missing: Vec<_> = active.difference(&covered).cloned().collect();
        return Err(format!(
            "native seed source/destination groups do not describe active nets: {}",
            missing.join(", ")
        ));
    }
    let group_ids: BTreeSet<_> = active_groups.iter().map(|group| group.id.clone()).collect();
    for group in &mut active_groups {
        group.depends_on.retain(|id| group_ids.contains(id));
    }

    let mut awx = spec
        .awx
        .clone()
        .ok_or("native seed spec has no actual AWX configuration")?;
    // Seed bundles carry the physical AWX route scope, not the producing
    // process's filesystem paths for future evolution/export orchestration.
    awx.evolve.tag = None;
    awx.evolve.seeds.clear();
    awx.evolve.board = None;
    awx.evolve.descend_env.clear();
    awx.evolve.jump_env.clear();
    awx.seed_export.directory = None;
    Ok(NativeSeedScope {
        version: spec.version,
        route_remaining: spec.route_remaining,
        allow_partial_output: spec.allow_partial_output,
        protected_nets: spec.protected_nets.clone(),
        nets: net_rules,
        endpoint_groups: spec.endpoint_groups.clone(),
        channels: spec.channels.clone(),
        groups: active_groups,
        advanced: spec.advanced.clone(),
        awx,
    })
}

fn check_sidecar_scope(value: &Value, field: &str, names: &BTreeSet<String>) -> Result<(), String> {
    let object = value
        .as_object()
        .ok_or_else(|| format!("native seed {field} must be a JSON object"))?;
    if let Some(unknown) = object.keys().find(|name| !names.contains(*name)) {
        return Err(format!(
            "native seed {field} contains out-of-scope net {unknown}"
        ));
    }
    Ok(())
}

fn export_inner(
    input: &NativeSeedExportInput<'_>,
    routed_board_dsn: &Path,
    fanout_board_dsn: &Path,
    directory: &Path,
    created: &mut Vec<String>,
) -> Result<PathBuf, String> {
    let routed_source = fs::canonicalize(routed_board_dsn).map_err(|e| {
        format!(
            "read-only native board DSN {}: {e}",
            routed_board_dsn.display()
        )
    })?;
    let fanout_source = fs::canonicalize(fanout_board_dsn).map_err(|e| {
        format!(
            "read-only native fanout DSN {}: {e}",
            fanout_board_dsn.display()
        )
    })?;
    let routed_dsn = fs::read(&routed_source).map_err(|e| {
        format!(
            "read-only native board DSN {}: {e}",
            routed_source.display()
        )
    })?;
    let fanout_dsn = if routed_source == fanout_source {
        routed_dsn.clone()
    } else {
        fs::read(&fanout_source).map_err(|e| {
            format!(
                "read-only native fanout DSN {}: {e}",
                fanout_source.display()
            )
        })?
    };
    let routed_base = BoardAdapter::load(&routed_dsn)
        .map_err(|e| format!("configured board is not a supported native DSN: {e}"))?;
    let fanout_base = if routed_dsn == fanout_dsn {
        routed_base.clone()
    } else {
        BoardAdapter::load(&fanout_dsn)
            .map_err(|e| format!("configured fanout board is not a supported native DSN: {e}"))?
    };
    ensure_same_layout_with_spec(&routed_base, input.routed, input.spec, "routed export")?;
    ensure_same_layout_with_spec(&fanout_base, input.fanout, input.spec, "fanout export")?;
    ensure_same_layout_with_spec(&routed_base, &fanout_base, input.spec, "routed/fanout base")?;

    let requested: BTreeSet<_> = input.names.iter().cloned().collect();
    if input.names.is_empty() {
        return Err("native seed export requires at least one active net".into());
    }
    if input.names.iter().collect::<BTreeSet<_>>().len() != input.names.len() {
        return Err("native seed export has duplicate active nets".into());
    }
    let configured: BTreeSet<_> = input.spec.nets.iter().map(|net| net.name.clone()).collect();
    if requested.iter().any(|name| !configured.contains(name)) {
        return Err("native seed active names are absent from the captured route spec".into());
    }
    if input.destination != input.context.destination {
        return Err("native seed destination differs from its realized fanout context".into());
    }
    let runtime_scope = native_scope(input.spec, input.names, input.context)?;
    for name in input.names {
        input.routed.net_number(name)?;
        input.fanout.net_number(name)?;
    }
    validate_plan_seed(input.plan_seed, input.names)?;

    let plan = update_plan_sidecar(input.plan_seed, input.context, input.flow)?;
    let ends = plan
        .get("ends")
        .and_then(Value::as_object)
        .ok_or("native seed plan producer did not return ends")?;
    for name in input.names {
        if !ends.contains_key(name) {
            return Err(format!("native seed plan is missing active net {name}"));
        }
    }

    let pack = update_pack_sidecar(
        &fresh_pack_seed(input.context),
        input.routed,
        input.fanout,
        input.names,
    )?;
    let lanes = pack
        .get("lanes")
        .and_then(Value::as_object)
        .ok_or("native pack producer did not return lanes")?;
    for name in input.names {
        let lane = lanes
            .get(name)
            .ok_or_else(|| format!("native pack producer omitted active net {name}"))?;
        if lane.get("segs").and_then(Value::as_array).is_none()
            || lane.get("vias").and_then(Value::as_array).is_none()
        {
            return Err(format!("native pack lane {name} lacks copper arrays"));
        }
    }

    let mut census = input
        .census
        .cloned()
        .filter(|v| !v.as_object().is_some_and(|m| m.is_empty()));
    if let Some(value) = census.as_ref() {
        check_sidecar_scope(value, "census", &requested)?;
    }
    let verdicts: BTreeMap<_, _> = input
        .verdicts
        .iter()
        .filter(|(name, _)| requested.contains(*name))
        .map(|(name, evidence)| (name.clone(), evidence.clone()))
        .collect();
    if input.verdicts.keys().any(|name| !requested.contains(name)) {
        return Err("native seed verdict evidence contains out-of-scope nets".into());
    }
    if let Some(Value::Object(census_map)) = census.as_mut() {
        census_map.retain(|name, _| requested.contains(name));
    }

    let grade = grade_board(input.routed, input.spec)?;
    let (routed_dsn_name, fanout_dsn_name) = if routed_dsn == fanout_dsn {
        ("board.dsn".to_owned(), "board.dsn".to_owned())
    } else {
        ("routed.dsn".to_owned(), "fanout.dsn".to_owned())
    };
    let manifest = Manifest {
        format: "fastroute-native-seed-v2",
        requested_k: input.requested_k,
        destination: input.destination.to_owned(),
        active_nets: input.names.to_vec(),
        runtime_scope,
        routed_dsn: routed_dsn_name.clone(),
        routed_ses: "routed.ses",
        fanout_dsn: fanout_dsn_name.clone(),
        fanout_ses: "fanout.ses",
        pack: "pack.json",
        plan: "plan.json",
        census: census.as_ref().map(|_| "census.json"),
        verdicts: (!verdicts.is_empty()).then_some("verdicts.json"),
        native_vias: {
            let mut by_name: BTreeMap<String, NativeViaDrill> = BTreeMap::new();
            for via in native_via_drills(input.routed)?
                .into_iter()
                .chain(native_via_drills(input.fanout)?)
            {
                if let Some(old) = by_name.get_mut(&via.padstack) {
                    if (old.diameter_mm - via.diameter_mm).abs() > 1e-9
                        || (old.drill_mm - via.drill_mm).abs() > 1e-9
                    {
                        return Err(format!(
                            "native via {} differs between routed and fanout boards",
                            via.padstack
                        ));
                    }
                    old.nets.extend(via.nets);
                    old.nets.sort();
                    old.nets.dedup();
                } else {
                    by_name.insert(via.padstack.clone(), via);
                }
            }
            (!by_name.is_empty()).then(|| by_name.into_values().collect())
        },
        grade: SeedGrade {
            opens: grade.opens,
            drc: grade.drc,
            vias: grade.vias,
        },
    };
    let manifest_bytes = toml::to_string_pretty(&manifest)
        .map_err(|e| format!("serialize native seed manifest: {e}"))?
        .into_bytes();

    write_new(directory, &routed_dsn_name, &routed_dsn, created)?;
    if fanout_dsn_name != routed_dsn_name {
        write_new(directory, &fanout_dsn_name, &fanout_dsn, created)?;
    }
    write_new(
        directory,
        "routed.ses",
        &session_bytes(input.routed)?,
        created,
    )?;
    write_new(
        directory,
        "fanout.ses",
        &session_bytes(input.fanout)?,
        created,
    )?;
    write_new(
        directory,
        "pack.json",
        &serde_json::to_vec_pretty(&pack).map_err(|e| e.to_string())?,
        created,
    )?;
    write_new(
        directory,
        "plan.json",
        &serde_json::to_vec_pretty(&plan).map_err(|e| e.to_string())?,
        created,
    )?;
    if let Some(census) = census {
        write_new(
            directory,
            "census.json",
            &serde_json::to_vec_pretty(&census).map_err(|e| e.to_string())?,
            created,
        )?;
    }
    if !verdicts.is_empty() {
        write_new(
            directory,
            "verdicts.json",
            &serde_json::to_vec_pretty(&verdicts).map_err(|e| e.to_string())?,
            created,
        )?;
    }
    write_new(directory, "seed.toml", &manifest_bytes, created)?;
    Ok(directory.join("seed.toml"))
}

/// Write all bundle members into a newly created directory, with the TOML
/// manifest committed last. Existing outputs are always rejected.
fn export_seed_bundle_with_sources(
    input: NativeSeedExportInput<'_>,
    routed_board_dsn: &Path,
    fanout_board_dsn: &Path,
) -> Result<PathBuf, String> {
    let directory = input.output_dir;
    if directory.exists() {
        return Err(format!(
            "native seed export target already exists: {}",
            directory.display()
        ));
    }
    let parent = directory
        .parent()
        .filter(|path| !path.as_os_str().is_empty())
        .unwrap_or_else(|| Path::new("."));
    if !parent.is_dir() {
        return Err(format!(
            "native seed export parent does not exist: {}",
            parent.display()
        ));
    }
    let parent = fs::canonicalize(parent)
        .map_err(|e| format!("native seed export parent {}: {e}", parent.display()))?;
    let name = directory
        .file_name()
        .ok_or("native seed export path has no final directory name")?;
    let target = parent.join(name);
    fs::create_dir(&target)
        .map_err(|e| format!("create native seed export {}: {e}", target.display()))?;

    let mut created = Vec::new();
    match export_inner(
        &input,
        routed_board_dsn,
        fanout_board_dsn,
        &target,
        &mut created,
    ) {
        Ok(path) => Ok(path),
        Err(error) => {
            for name in created.into_iter().rev() {
                let _ = fs::remove_file(target.join(name));
            }
            let _ = fs::remove_dir(&target);
            Err(error)
        }
    }
}

pub fn export_native_seed_bundle(input: NativeSeedExportInput<'_>) -> Result<PathBuf, String> {
    let board_dsn = input.board_dsn;
    export_seed_bundle_with_sources(input, board_dsn, board_dsn)
}

/// Export a final evolution world through the same real board/sidecar path as
/// an initial fanout capture. Current berth signatures are read from the
/// realized fanout menus; missing measurements fail instead of being filled.
pub fn export_native_world_bundle(
    world: &NativeWorld,
    output_dir: &Path,
) -> Result<PathBuf, String> {
    let state = ReplanState::new(world.clone())?;
    let context = super::replan_finalize::fanout_context(&state, &world.fanout)?;
    let mut plan_seed = world.plan_sidecar.clone();
    let root = plan_seed
        .as_object_mut()
        .ok_or("native world plan sidecar is not an object")?;
    let signatures = root
        .entry("berth_sig")
        .or_insert_with(|| serde_json::json!({}))
        .as_object_mut()
        .ok_or("native world plan berth_sig is not an object")?;
    for name in &state.names {
        let movement = state
            .ends
            .get(name)
            .and_then(|end| end.dst_move.as_ref())
            .ok_or_else(|| format!("native world has no measured destination move for {name}"))?;
        let signature = super::whole_ends::move_signature(movement)?;
        let signature = serde_json::from_str(&signature).map_err(|e| e.to_string())?;
        signatures.insert(name.clone(), signature);
    }
    let export_input = NativeSeedExportInput {
        board_dsn: &world.source_dsn,
        output_dir,
        routed: &world.routed,
        fanout: &world.fanout,
        spec: &world.spec,
        requested_k: world.requested_k,
        destination: &world.destination,
        names: &state.names,
        context: &context.context,
        flow: context.flow,
        plan_seed: &plan_seed,
        census: world.census_sidecar.as_ref(),
        verdicts: &state.verdicts,
    };
    export_seed_bundle_with_sources(export_input, &world.source_dsn, &world.fanout_source_dsn)
}
