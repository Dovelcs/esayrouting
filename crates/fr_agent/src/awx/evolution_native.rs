//! Native world loading, grading, copper identity, and adapter glue for evolution.
//!
//! Each seed entry names a TOML manifest with routed/fanout DSN files, optional
//! SES overlays, and required pack/plan JSON plus optional census JSON. Paths
//! inside the manifest are relative to that manifest. This native input contract
//! replaces the original KiCad PCB format, which this Rust toolchain cannot read.

use super::replan_types::NetVerdict;
use super::route_evolution::{DescentOptions, EvolutionOperators, EvolutionWorld, RouteGrade};
use crate::{board_adapter::BoardAdapter, spec::RouteSpec};
use fr_engine::drc::{DesignRulesChecker, all_clearance_violations};
use serde::Deserialize;
use serde_json::Value;
use sha1::{Digest, Sha1};
use std::{
    fs,
    path::{Path, PathBuf},
};

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct NativeSeedManifest {
    #[serde(default)]
    format: Option<String>,
    #[serde(default)]
    requested_k: Option<i64>,
    #[serde(default)]
    destination: Option<String>,
    /// Ordered active run names. `requested_k` is a distinct source quantity
    /// and is not assumed to equal this list's length.
    #[serde(default)]
    active_nets: Option<Vec<String>>,
    /// Captured route scope lets a native seed be replayed when the importing
    /// controller has no source groups of its own. Absent in v1 bundles.
    #[serde(default)]
    runtime_scope: Option<super::evolution_export::NativeSeedScope>,
    #[serde(default)]
    native_vias: Option<Vec<super::evolution_export::NativeViaDrill>>,
    routed_dsn: PathBuf,
    #[serde(default)]
    routed_ses: Option<PathBuf>,
    fanout_dsn: PathBuf,
    #[serde(default)]
    fanout_ses: Option<PathBuf>,
    pack: PathBuf,
    plan: PathBuf,
    #[serde(default)]
    census: Option<PathBuf>,
    #[serde(default)]
    log: Option<PathBuf>,
    #[serde(default)]
    refusals: Option<PathBuf>,
    #[serde(default)]
    verdicts: Option<PathBuf>,
    #[serde(default)]
    grade: Option<NativeSeedGrade>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct NativeSeedGrade {
    opens: Vec<String>,
    drc: usize,
    vias: usize,
}

#[derive(Clone)]
pub struct NativeWorld {
    /// Fully materialized routed board, including source geometry and imported copper.
    pub routed: BoardAdapter,
    /// Fully materialized fanout board, including the fanout copper state.
    pub fanout: BoardAdapter,
    /// Routing configuration needed by native replan operators.
    pub spec: RouteSpec,
    pub requested_k: i64,
    pub destination: String,
    pub seed_stem: PathBuf,
    /// Resolved immutable DSN used by routed/fanout SES overlays. This is the
    /// actual board source needed to export another native seed bundle.
    pub source_dsn: PathBuf,
    /// Resolved native DSN on which the fanout SES was authored.
    pub fanout_source_dsn: PathBuf,
    /// Parsed braid lane geometry used by pack/replan operations.
    pub pack_sidecar: Value,
    /// Parsed fanout-plan ends used by replan operations.
    pub plan_ends: serde_json::Map<String, Value>,
    /// Full plan metadata (layers, directions, chi and berth signatures).
    pub plan_sidecar: Value,
    /// Source SHA-1 values are retained because the closed-world key includes
    /// exact sidecar bytes in addition to their parsed semantics.
    pub plan_sidecar_hash: String,
    /// Parsed prior census data; absent when the source run had none.
    pub census_sidecar: Option<Value>,
    pub census_sidecar_hash: Option<String>,
    /// Braid verdict facts reconstructed from optional log/refusal evidence and
    /// the required pack/census metadata.
    pub verdict_evidence: std::collections::BTreeMap<String, NetVerdict>,
}

/// Actual native route operations are supplied by the dedicated descent and
/// probe modules. Keeping this boundary explicit prevents a generic reroute from
/// being mistaken for the reference's incremental operators.
pub trait NativeEvolutionBackend: Sync {
    fn descend(
        &self,
        world: &EvolutionWorld<NativeWorld>,
        options: DescentOptions,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String>;

    fn jump_near(
        &self,
        world: &EvolutionWorld<NativeWorld>,
        n_nets: usize,
        seed: u32,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String>;

    fn cross_probe(
        &self,
        a: &EvolutionWorld<NativeWorld>,
        b: &EvolutionWorld<NativeWorld>,
        seed: u32,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String>;
}

pub struct NativeEvolutionOperators<B> {
    pub spec: RouteSpec,
    /// Base directory for manifest-relative file references.
    pub seed_base: PathBuf,
    pub backend: B,
}

/// Native implementation of the frozen controller boundary. All route
/// mutations are delegated to the dedicated native operator modules.
pub struct NativeOperationBackend;

impl<B> NativeEvolutionOperators<B> {
    pub fn new(spec: RouteSpec, seed_base: PathBuf, backend: B) -> Self {
        Self {
            spec,
            seed_base,
            backend,
        }
    }
}

impl NativeEvolutionOperators<NativeOperationBackend> {
    pub fn native(spec: RouteSpec, seed_base: PathBuf) -> Self {
        Self::new(spec, seed_base, NativeOperationBackend)
    }
}

impl NativeEvolutionBackend for NativeOperationBackend {
    fn descend(
        &self,
        world: &EvolutionWorld<NativeWorld>,
        options: DescentOptions,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
        super::evolution_descent::descend(world, options, env)
    }

    fn jump_near(
        &self,
        world: &EvolutionWorld<NativeWorld>,
        n_nets: usize,
        seed: u32,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
        super::evolution_probes::jump_near(world, n_nets, seed, env)
    }

    fn cross_probe(
        &self,
        a: &EvolutionWorld<NativeWorld>,
        b: &EvolutionWorld<NativeWorld>,
        seed: u32,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
        super::evolution_probes::cross_probe(a, b, seed, env)
    }
}

/// Build a controller world from a real native board state and calculate its
/// grade from the imported routed board.
pub fn world_from_native(
    id: impl Into<String>,
    data: NativeWorld,
) -> Result<EvolutionWorld<NativeWorld>, String> {
    let grade = grade_board(&data.routed, &data.spec)?;
    Ok(EvolutionWorld {
        id: id.into(),
        name: String::new(),
        origin: String::new(),
        grade,
        data,
    })
}

fn read_required(path: &Path, label: &str) -> Result<Vec<u8>, String> {
    fs::read(path).map_err(|e| format!("native seed {label} {}: {e}", path.display()))
}

fn resolve_manifest_path(base: &Path, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_owned()
    } else {
        base.join(path)
    }
}

fn import_board(
    dsn_path: &Path,
    ses_path: Option<&Path>,
    base: &Path,
    label: &str,
    native_vias: &[super::evolution_export::NativeViaDrill],
) -> Result<BoardAdapter, String> {
    let dsn_path = resolve_manifest_path(base, dsn_path);
    let dsn = read_required(&dsn_path, &format!("{label} DSN"))?;
    let source_board = BoardAdapter::load(&dsn)
        .map_err(|e| format!("native seed {label} {}: {e}", dsn_path.display()))?;
    let via_candidates = native_via_candidates(&source_board, native_vias)?;
    let Some(ses_path) = ses_path else {
        return Ok(source_board);
    };
    let ses_path = resolve_manifest_path(base, ses_path);
    let ses = read_required(&ses_path, &format!("{label} session"))?;
    BoardAdapter::load_session_with_via_candidates(&dsn, &ses, &via_candidates)
        .map_err(|e| format!("native seed {label} {}: {e}", ses_path.display()))
}

fn restore_native_via_drills(
    board: &mut BoardAdapter,
    vias: &[super::evolution_export::NativeViaDrill],
    label: &str,
) -> Result<(), String> {
    for via in vias {
        if !via.diameter_mm.is_finite()
            || !via.drill_mm.is_finite()
            || via.diameter_mm <= via.drill_mm
            || via.drill_mm <= 0.0
        {
            return Err(format!(
                "native seed {label} has invalid explicit drill metadata for {}",
                via.padstack
            ));
        }
        let Some(id) = board
            .board
            .library
            .padstacks
            .get_by_name(&via.padstack)
            .map(|stack| stack.id)
        else {
            // A stack used only by the other state can be absent from this SES.
            continue;
        };
        let stack = board
            .board
            .library
            .padstacks
            .get(id)
            .ok_or("native seed explicit drill padstack lookup failed")?;
        let actual_diameter = (stack.from_layer()..=stack.to_layer())
            .filter_map(|layer| stack.get_shape(layer))
            .map(|shape| shape.max_width())
            .fold(0.0_f64, f64::max)
            / board.units_per_mm;
        if (actual_diameter - via.diameter_mm).abs() > 1.0 / board.units_per_mm {
            return Err(format!(
                "native seed {label} padstack {} copper diameter differs from explicit metadata",
                via.padstack
            ));
        }
        let drill_radius = via.drill_mm * board.units_per_mm / 2.0;
        let actual_drill = stack.get_explicit_drill_radius();
        if actual_drill.is_some_and(|actual| (actual - drill_radius).abs() > 1.0) {
            return Err(format!(
                "native seed {label} padstack {} drill differs from explicit metadata",
                via.padstack
            ));
        }
        if actual_drill.is_none() {
            board
                .board
                .library_mut()
                .padstacks
                .get_mut(id)
                .ok_or("native seed explicit drill padstack mutation failed")?
                .set_explicit_drill_radius(Some(drill_radius))?;
        }
    }
    Ok(())
}

fn native_via_candidates(
    board: &BoardAdapter,
    vias: &[super::evolution_export::NativeViaDrill],
) -> Result<Vec<(String, f64, f64)>, String> {
    let mut candidates = Vec::new();
    let mut padstacks = std::collections::BTreeSet::new();
    for via in vias {
        if !padstacks.insert(via.padstack.as_str()) {
            return Err(format!("native seed repeats via padstack metadata for {}", via.padstack));
        }
        if !via.diameter_mm.is_finite()
            || !via.drill_mm.is_finite()
            || via.diameter_mm <= via.drill_mm
            || via.drill_mm <= 0.0
        {
            return Err(format!("native seed has invalid explicit drill metadata for {}", via.padstack));
        }
        if via.nets.is_empty() {
            return Err("legacy native seed via metadata lacks verified per-net authorization".into());
        }
        let radius = (via.diameter_mm * board.units_per_mm / 2.0).round();
        let drill_diameter = (via.drill_mm * board.units_per_mm).round();
        if !radius.is_finite()
            || !drill_diameter.is_finite()
            || radius <= 0.0
            || radius > (i32::MAX / 2) as f64
            || drill_diameter <= 0.0
            || drill_diameter > i32::MAX as f64
            || 2.0 * radius <= drill_diameter
        {
            return Err(format!("native seed via {} dimensions exceed board resolution", via.padstack));
        }
        let generated_name = format!("awx_via_r{}_d{}", radius as i32, drill_diameter as i32);
        let existing = board.board.library.padstacks.get_by_name(&via.padstack);
        if via.padstack != generated_name {
            let Some(stack) = existing else {
                return Err(format!(
                    "native seed via padstack {} does not match its verified native dimensions",
                    via.padstack
                ));
            };
            let stack = board.board.library.padstacks.get(stack.id)
                .ok_or("native seed source via padstack lookup failed")?;
            let actual_diameter = (stack.from_layer()..=stack.to_layer())
                .filter_map(|layer| stack.get_shape(layer))
                .map(|shape| shape.max_width())
                .fold(0.0_f64, f64::max) / board.units_per_mm;
            let actual_drill = stack.get_explicit_drill_radius()
                .ok_or_else(|| format!("native seed source via {} has no verified drill", via.padstack))?
                * 2.0 / board.units_per_mm;
            if (actual_diameter - via.diameter_mm).abs() > 1.0 / board.units_per_mm
                || (actual_drill - via.drill_mm).abs() > 1.0 / board.units_per_mm
            {
                return Err(format!("native seed source padstack {} differs from explicit metadata", via.padstack));
            }
            continue;
        }
        if let Some(id) = existing {
            let stack = board.board.library.padstacks.get(id.id)
                .ok_or("native seed native via padstack lookup failed")?;
            if let Some(actual) = stack.get_explicit_drill_radius() {
                let expected = via.drill_mm * board.units_per_mm / 2.0;
                if (actual - expected).abs() > 1.0 {
                    return Err(format!("native seed native padstack {} has a conflicting drill", via.padstack));
                }
            }
        }
        let mut seen = std::collections::BTreeSet::new();
        for net in &via.nets {
            if net.trim().is_empty() || !seen.insert(net) {
                return Err(format!("native seed via {} has invalid net authorization", via.padstack));
            }
            candidates.push((net.clone(), via.diameter_mm, via.drill_mm));
        }
    }
    Ok(candidates)
}

#[cfg(test)]
mod seed_scope_regression {
    use super::native_via_candidates;
    use crate::board_adapter::BoardAdapter;
    use crate::awx::evolution_export::NativeViaDrill;

    fn source_board() -> BoardAdapter {
        BoardAdapter::load(include_bytes!("../../tests/fixtures/two_chip.dsn")).unwrap()
    }

    #[test]
    fn prior_inactive_bus_via_is_authorized_only_for_its_exported_net() {
        let via: NativeViaDrill = toml::from_str(
            r#"padstack = "awx_via_r300_d300"
diameter_mm = 0.6
drill_mm = 0.3
nets = ["BUS_A"]"#,
        )
        .unwrap();
        let candidates = native_via_candidates(&source_board(), &[via]).unwrap();
        assert_eq!(candidates, vec![("BUS_A".into(), 0.6, 0.3)]);
        assert!(!candidates.iter().any(|(net, _, _)| net == "BUS_B"));
    }

    #[test]
    fn legacy_drill_only_manifest_does_not_authorize_any_net() {
        let legacy: NativeViaDrill = toml::from_str(
            r#"padstack = "awx_via_r300_d300"
diameter_mm = 0.6
drill_mm = 0.3"#,
        )
        .unwrap();
        assert!(native_via_candidates(&source_board(), &[legacy])
            .unwrap_err()
            .contains("lacks verified per-net authorization"));
    }
}

fn parse_pack(path: &Path) -> Result<Value, String> {
    let bytes = read_required(path, "pack metadata")?;
    let value: Value = serde_json::from_slice(&bytes)
        .map_err(|e| format!("native seed pack metadata {}: {e}", path.display()))?;
    let lanes = value
        .get("lanes")
        .and_then(Value::as_object)
        .ok_or_else(|| {
            format!(
                "native seed pack metadata {} lacks lanes object",
                path.display()
            )
        })?;
    for (name, lane) in lanes {
        for (field, tuple_len) in [("segs", 6usize), ("vias", 2usize)] {
            let entries = lane
                .get(field)
                .and_then(Value::as_array)
                .ok_or_else(|| format!("pack lane {name} lacks {field} array"))?;
            for entry in entries {
                let tuple = entry
                    .as_array()
                    .ok_or_else(|| format!("pack lane {name} {field} entry is not an array"))?;
                if tuple.len() != tuple_len
                    || tuple
                        .iter()
                        .any(|v| !v.as_f64().is_some_and(f64::is_finite))
                {
                    return Err(format!("pack lane {name} has malformed {field} tuple"));
                }
            }
        }
    }
    Ok(value)
}

fn parse_plan(path: &Path) -> Result<Value, String> {
    let bytes = read_required(path, "fanout plan")?;
    let value: Value = serde_json::from_slice(&bytes)
        .map_err(|e| format!("native seed fanout plan {}: {e}", path.display()))?;
    if value.get("ends").and_then(Value::as_object).is_none() {
        return Err(format!(
            "native seed fanout plan {} lacks ends object",
            path.display()
        ));
    }
    Ok(value)
}

fn active_names(
    manifest: &NativeSeedManifest,
    spec: &RouteSpec,
    pack: &Value,
    plan_ends: &serde_json::Map<String, Value>,
) -> Result<Vec<String>, String> {
    let lanes = pack
        .get("lanes")
        .and_then(Value::as_object)
        .ok_or("native seed pack metadata lacks lanes object")?;
    let names = if let Some(names) = &manifest.active_nets {
        names.clone()
    } else {
        // v1 bundles did not persist the ordered active-name list. The plan
        // and pack intersection defines that scope; use caller order only if
        // it represents the same complete set.
        let sidecar_names: Vec<_> = plan_ends
            .keys()
            .filter(|name| lanes.contains_key(*name))
            .cloned()
            .collect();
        let sidecar_set: std::collections::BTreeSet<_> = sidecar_names.iter().cloned().collect();
        let configured: Vec<_> = spec
            .nets
            .iter()
            .filter(|net| plan_ends.contains_key(&net.name) && lanes.contains_key(&net.name))
            .map(|net| net.name.clone())
            .collect();
        let configured_set: std::collections::BTreeSet<_> = configured.iter().cloned().collect();
        if configured_set == sidecar_set && !configured.is_empty() {
            configured
        } else {
            sidecar_names
        }
    };
    if names.is_empty() {
        return Err("native seed plan and pack identify no active nets".into());
    }
    if names
        .iter()
        .collect::<std::collections::BTreeSet<_>>()
        .len()
        != names.len()
    {
        return Err("native seed active net list contains duplicates".into());
    }
    for name in &names {
        if !plan_ends.contains_key(name) || !lanes.contains_key(name) {
            return Err(format!(
                "native seed active net {name} is missing from plan ends or pack lanes"
            ));
        }
    }
    Ok(names)
}

fn filtered_spec(
    base: &RouteSpec,
    manifest: &NativeSeedManifest,
    names: &[String],
) -> Result<RouteSpec, String> {
    let active: std::collections::BTreeSet<_> = names.iter().cloned().collect();
    let mut spec = base.clone();
    if let Some(scope) = &manifest.runtime_scope {
        let scope_names: Vec<_> = scope.nets.iter().map(|net| net.name.clone()).collect();
        if scope_names != names {
            return Err("native seed runtime scope order differs from active_nets".into());
        }
        let group_nets: std::collections::BTreeSet<_> = scope
            .groups
            .iter()
            .flat_map(|group| group.nets.iter().cloned())
            .collect();
        if group_nets != active {
            return Err("native seed runtime groups differ from active_nets".into());
        }
        spec.version = scope.version;
        spec.route_remaining = scope.route_remaining;
        spec.allow_partial_output = scope.allow_partial_output;
        spec.protected_nets = scope.protected_nets.clone();
        spec.nets = scope.nets.clone();
        spec.endpoint_groups = scope.endpoint_groups.clone();
        spec.channels = scope.channels.clone();
        spec.groups = scope.groups.clone();
        spec.advanced = scope.advanced.clone();
        spec.awx = Some(scope.awx.clone());
        return Ok(spec);
    }

    // Legacy manifests omit route scope. Restrict only metadata actually
    // present in the caller's spec; never invent missing rules or groups.
    spec.nets.retain(|net| active.contains(&net.name));
    if spec
        .nets
        .iter()
        .map(|net| net.name.clone())
        .collect::<std::collections::BTreeSet<_>>()
        != active
    {
        let missing: Vec<_> = active
            .iter()
            .filter(|name| !spec.nets.iter().any(|net| &net.name == *name))
            .cloned()
            .collect();
        return Err(format!(
            "legacy native seed scope lacks configured net rules: {}",
            missing.join(", ")
        ));
    }
    for group in &mut spec.groups {
        group.nets.retain(|name| active.contains(name));
        if let Some(awx) = group.awx.as_mut() {
            awx.pairs
                .retain(|pair| active.contains(&pair.p) && active.contains(&pair.n));
        }
    }
    Ok(spec)
}

fn scope_sidecar_map(value: &mut Value, field: &str, names: &std::collections::BTreeSet<String>) {
    if let Some(map) = value.get_mut(field).and_then(Value::as_object_mut) {
        map.retain(|name, _| names.contains(name));
    }
}

fn filter_pack_to_names(
    value: &mut Value,
    names: &std::collections::BTreeSet<String>,
) -> Result<(), String> {
    let lanes = value
        .get_mut("lanes")
        .and_then(Value::as_object_mut)
        .ok_or("native seed pack metadata lacks lanes object")?;
    lanes.retain(|name, _| names.contains(name));
    Ok(())
}

fn parse_census(path: &Path) -> Result<Value, String> {
    let bytes = read_required(path, "census metadata")?;
    let value: Value = serde_json::from_slice(&bytes)
        .map_err(|e| format!("native seed census metadata {}: {e}", path.display()))?;
    if !value.is_object() {
        return Err(format!(
            "native seed census metadata {} must be an object",
            path.display()
        ));
    }
    Ok(value)
}

fn parsed_names(value: &str) -> Vec<String> {
    value
        .split(',')
        .map(|name| name.trim().trim_matches('\'').to_owned())
        .filter(|name| !name.is_empty())
        .collect()
}

fn bracket_value(line: &str) -> Option<&str> {
    let (_, rest) = line.split_once('[')?;
    rest.split_once(']').map(|(inside, _)| inside)
}

fn verdict_slot<'a>(
    facts: &'a mut std::collections::BTreeMap<String, NetVerdict>,
    name: &str,
) -> &'a mut NetVerdict {
    facts.entry(name.to_owned()).or_default()
}

fn parse_log_evidence(
    path: &Path,
) -> Result<std::collections::BTreeMap<String, NetVerdict>, String> {
    let log =
        fs::read_to_string(path).map_err(|e| format!("native seed log {}: {e}", path.display()))?;
    let mut facts = std::collections::BTreeMap::new();
    let mut launch_orders = Vec::new();
    let mut target_orders = Vec::new();
    for line in log.lines() {
        if let Some(names) = line.strip_prefix("  page F: ").and_then(bracket_value) {
            for name in parsed_names(names) {
                verdict_slot(&mut facts, &name).class = Some("F".into());
            }
        } else if let Some(rest) = line.strip_prefix("  page B: ") {
            if let Some(names) = bracket_value(rest) {
                for name in parsed_names(names) {
                    verdict_slot(&mut facts, &name).class = Some("B".into());
                }
            }
            if let Some((_, swimmers)) = rest.split_once("swimmers:") {
                if let Some(names) = bracket_value(swimmers) {
                    for name in parsed_names(names) {
                        verdict_slot(&mut facts, &name).class = Some("swim".into());
                    }
                }
            }
        } else if let Some(names) = line
            .strip_prefix("  launch order: ")
            .and_then(bracket_value)
        {
            launch_orders.push(parsed_names(names));
        } else if let Some(names) = line
            .strip_prefix("  target order: ")
            .and_then(bracket_value)
        {
            target_orders.push(parsed_names(names));
        } else if let Some(rest) = line.split_once("last call routed: ").map(|(_, rest)| rest) {
            if let Some((name, tail)) = rest.split_once(" (") {
                if let Some(count) = tail.split_whitespace().next().and_then(|v| v.parse().ok()) {
                    verdict_slot(&mut facts, name).last_call = Some(count);
                }
            }
        } else if let Some(rest) = line.split_once("rip [").map(|(_, rest)| rest) {
            if let Some((names, tail)) = rest.split_once("]:") {
                if let Some(name) = tail.trim().split_whitespace().next() {
                    verdict_slot(&mut facts, name).rip = parsed_names(names);
                }
            }
        } else if let Some(rest) = line.split_once("rip for ").map(|(_, rest)| rest) {
            if let Some((name, tail)) = rest.split_once(": frontier ") {
                if let Some((_, lanes)) = tail.split_once("lanes of this run on it: ") {
                    let lanes = lanes.split(" (").next().unwrap_or(lanes);
                    for entry in lanes.split(',').map(str::trim).filter(|s| !s.is_empty()) {
                        if let Some((blocker, count)) = entry.rsplit_once('(').and_then(|(n, c)| {
                            c.strip_suffix(')')
                                .and_then(|c| c.parse::<usize>().ok().map(|v| (n, v)))
                        }) {
                            verdict_slot(&mut facts, name)
                                .frontier
                                .push((blocker.to_owned(), count));
                        }
                    }
                }
            }
            if let Some((name, tail)) = rest.split_once(": min-cut probe ") {
                if let Some((_, crossed)) = tail.split_once("crosses ") {
                    if let Some(names) = bracket_value(crossed) {
                        verdict_slot(&mut facts, name).mincut = parsed_names(names);
                    }
                }
            }
        }
    }
    for (launch, target) in launch_orders.iter().zip(target_orders.iter()) {
        for (rank, name) in launch.iter().enumerate() {
            verdict_slot(&mut facts, name).launch_rank = Some(rank);
        }
        for (rank, name) in target.iter().enumerate() {
            verdict_slot(&mut facts, name).target_rank = Some(rank);
        }
    }
    Ok(facts)
}

fn merge_refusals(
    facts: &mut std::collections::BTreeMap<String, NetVerdict>,
    path: &Path,
) -> Result<(), String> {
    let bytes = read_required(path, "refusal metadata")?;
    let value: Value = serde_json::from_slice(&bytes)
        .map_err(|e| format!("native seed refusal metadata {}: {e}", path.display()))?;
    let records = value.as_object().ok_or_else(|| {
        format!(
            "native seed refusal metadata {} must be an object",
            path.display()
        )
    })?;
    for (name, record) in records {
        let fact = verdict_slot(facts, name);
        fact.refused = true;
        fact.walled_at = record
            .get("walled_at")
            .and_then(Value::as_str)
            .map(str::to_owned);
    }
    Ok(())
}

fn merge_verdicts(
    target: &mut std::collections::BTreeMap<String, NetVerdict>,
    path: &Path,
) -> Result<(), String> {
    let bytes = read_required(path, "typed verdict evidence")?;
    let incoming: std::collections::BTreeMap<String, NetVerdict> =
        serde_json::from_slice(&bytes)
            .map_err(|e| format!("native seed verdict evidence {}: {e}", path.display()))?;
    for (name, source) in incoming {
        let destination = verdict_slot(target, &name);
        if source.class.is_some() {
            destination.class = source.class;
        }
        if source.predicted_vias.is_some() {
            destination.predicted_vias = source.predicted_vias;
        }
        if source.model_page.is_some() {
            destination.model_page = source.model_page;
        }
        if source.actual_vias != 0 {
            destination.actual_vias = source.actual_vias;
        }
        if source.lane_vias != 0 {
            destination.lane_vias = source.lane_vias;
        }
        if source.refused {
            destination.refused = true;
        }
        if source.walled_at.is_some() {
            destination.walled_at = source.walled_at;
        }
        if source.launch_rank.is_some() {
            destination.launch_rank = source.launch_rank;
        }
        if source.target_rank.is_some() {
            destination.target_rank = source.target_rank;
        }
        if source.last_call.is_some() {
            destination.last_call = source.last_call;
        }
        if !source.rip.is_empty() {
            destination.rip = source.rip;
        }
        if !source.frontier.is_empty() {
            destination.frontier = source.frontier;
        }
        if !source.mincut.is_empty() {
            destination.mincut = source.mincut;
        }
        if !source.census.is_empty() {
            destination.census = source.census;
        }
    }
    Ok(())
}

fn sha1_hex(bytes: &[u8]) -> String {
    format!("{:x}", Sha1::digest(bytes))
}

fn source_hash(bytes: &[u8], digits: usize) -> String {
    sha1_hex(bytes).chars().take(digits).collect()
}

fn knob_name(key: &str) -> bool {
    (key.starts_with("BRAID_")
        || key.starts_with("PROBE_")
        || key.starts_with("PLAN_")
        || key.starts_with("TAUT_")
        || key == "DST_CLIMB"
        || key == "SRC_CLIMB")
        && !matches!(key, "PROBE_MEMO" | "PROBE_MEMO_DIR" | "PROBE_MEMO_CODE")
}

fn native_code_hash() -> String {
    let mut hash = Sha1::new();
    for (name, bytes) in [
        (
            "route_evolution.rs",
            include_bytes!("route_evolution.rs").as_slice(),
        ),
        (
            "evolution_native.rs",
            include_bytes!("evolution_native.rs").as_slice(),
        ),
        (
            "evolution_descent.rs",
            include_bytes!("evolution_descent.rs").as_slice(),
        ),
        (
            "evolution_probes.rs",
            include_bytes!("evolution_probes.rs").as_slice(),
        ),
        (
            "replan_state.rs",
            include_bytes!("replan_state.rs").as_slice(),
        ),
        (
            "replan_types.rs",
            include_bytes!("replan_types.rs").as_slice(),
        ),
        (
            "replan_probes.rs",
            include_bytes!("replan_probes.rs").as_slice(),
        ),
        (
            "replan_cost.rs",
            include_bytes!("replan_cost.rs").as_slice(),
        ),
        (
            "replan_finalize.rs",
            include_bytes!("replan_finalize.rs").as_slice(),
        ),
        (
            "native_replan.rs",
            include_bytes!("native_replan.rs").as_slice(),
        ),
        (
            "integration.rs",
            include_bytes!("integration.rs").as_slice(),
        ),
        (
            "fanout_driver.rs",
            include_bytes!("fanout_driver.rs").as_slice(),
        ),
        (
            "whole_route.rs",
            include_bytes!("whole_route.rs").as_slice(),
        ),
        (
            "native_execution.rs",
            include_bytes!("native_execution.rs").as_slice(),
        ),
        (
            "joint_escape.rs",
            include_bytes!("joint_escape.rs").as_slice(),
        ),
        (
            "joint_geometry.rs",
            include_bytes!("joint_geometry.rs").as_slice(),
        ),
        (
            "source_realize.rs",
            include_bytes!("source_realize.rs").as_slice(),
        ),
        (
            "destination_realize.rs",
            include_bytes!("destination_realize.rs").as_slice(),
        ),
        (
            "route_recovery.rs",
            include_bytes!("route_recovery.rs").as_slice(),
        ),
        (
            "whole_context.rs",
            include_bytes!("whole_context.rs").as_slice(),
        ),
        (
            "escape_moves.rs",
            include_bytes!("escape_moves.rs").as_slice(),
        ),
        (
            "board_snapshot.rs",
            include_bytes!("board_snapshot.rs").as_slice(),
        ),
        ("fab_rules.rs", include_bytes!("fab_rules.rs").as_slice()),
        (
            "route_ladder.rs",
            include_bytes!("route_ladder.rs").as_slice(),
        ),
        ("relayer.rs", include_bytes!("relayer.rs").as_slice()),
        (
            "board_adapter.rs",
            include_bytes!("../board_adapter.rs").as_slice(),
        ),
        ("execution.rs", include_bytes!("../execution.rs").as_slice()),
        ("planner.rs", include_bytes!("../planner.rs").as_slice()),
        ("spec.rs", include_bytes!("../spec.rs").as_slice()),
    ] {
        hash.update(name.as_bytes());
        hash.update(bytes);
    }
    format!("{:x}", hash.finalize())[..16].to_owned()
}

fn board_layout(
    board: &BoardAdapter,
) -> Result<
    (
        Vec<String>,
        Vec<Vec<[i32; 2]>>,
        Vec<Vec<[i32; 2]>>,
        Vec<Vec<[i32; 2]>>,
        Vec<(String, String, String, String, i64, i64, Vec<String>)>,
    ),
    String,
> {
    board_layout_with_movable(board, &std::collections::BTreeSet::new())
}

fn board_layout_with_movable(
    board: &BoardAdapter,
    movable: &std::collections::BTreeSet<String>,
) -> Result<
    (
        Vec<String>,
        Vec<Vec<[i32; 2]>>,
        Vec<Vec<[i32; 2]>>,
        Vec<Vec<[i32; 2]>>,
        Vec<(String, String, String, String, i64, i64, Vec<String>)>,
    ),
    String,
> {
    let layers = board
        .board
        .layer_structure
        .layers
        .iter()
        .map(|layer| layer.name.clone())
        .collect();
    let mut pads = Vec::new();
    for pad in board.pads() {
        let point = if movable.contains(&pad.chip) {
            board
                .board
                .components
                .get_by_name(&pad.chip)
                .and_then(|component| component.get_location())
                .map(|origin| {
                    let origin = origin.to_float();
                    [
                        pad.point[0] - origin.x / board.units_per_mm,
                        pad.point[1] - origin.y / board.units_per_mm,
                    ]
                })
                .unwrap_or(pad.point)
        } else {
            pad.point
        };
        let x = (point[0] * 10_000.0).round();
        let y = (point[1] * 10_000.0).round();
        if !x.is_finite()
            || !y.is_finite()
            || x < i64::MIN as f64
            || x > i64::MAX as f64
            || y < i64::MIN as f64
            || y > i64::MAX as f64
        {
            return Err("board pad outside topology range".into());
        }
        pads.push((
            pad.reference,
            pad.chip,
            pad.pad,
            pad.net,
            x as i64,
            y as i64,
            pad.layers,
        ));
    }
    pads.sort();
    Ok((
        layers,
        board.board_outline_polygons.clone(),
        board.board_cutouts.clone(),
        board.board_edge_contours.clone(),
        pads,
    ))
}

pub(super) fn ensure_same_layout(
    base: &BoardAdapter,
    world: &BoardAdapter,
    label: &str,
) -> Result<(), String> {
    if board_layout(base)? != board_layout(world)? {
        return Err(format!(
            "native seed {label} DSN layout differs from configured board"
        ));
    }
    Ok(())
}

pub(super) fn ensure_same_layout_with_spec(
    base: &BoardAdapter,
    world: &BoardAdapter,
    spec: &RouteSpec,
    label: &str,
) -> Result<(), String> {
    let mut movable = std::collections::BTreeSet::new();
    if let Some(awx) = &spec.awx {
        movable.extend(
            awx.components
                .iter()
                .filter(|component| component.movable)
                .map(|component| component.reference.clone()),
        );
        movable.extend(
            awx.passive_geometry
                .iter()
                .filter(|(_, geometry)| !geometry.locked)
                .map(|(reference, _)| reference.clone()),
        );
    }
    if movable.is_empty() {
        return ensure_same_layout(base, world, label);
    }
    if board_layout_with_movable(base, &movable)? != board_layout_with_movable(world, &movable)? {
        return Err(format!(
            "native seed {label} DSN layout differs from configured board"
        ));
    }
    Ok(())
}

/// Grade the configured run nets, all board clearance violations, and their vias.
/// This mirrors grade_k.py's net-scoped open/via census and board-wide DRC count.
pub fn grade_board(board: &BoardAdapter, spec: &RouteSpec) -> Result<RouteGrade, String> {
    let selected: Vec<_> = spec
        .nets
        .iter()
        .map(|net| {
            board
                .net_number(&net.name)
                .map(|number| (net.name.clone(), number))
        })
        .collect::<Result<_, _>>()?;
    let mut checker = DesignRulesChecker::new();
    let open_numbers = checker.incomplete_net_numbers(&board.board);
    let mut opens: Vec<_> = selected
        .iter()
        .filter(|(_, number)| open_numbers.contains(number))
        .map(|(name, _)| name.clone())
        .collect();
    opens.sort();
    let drc = all_clearance_violations(&board.board).len();
    let vias = selected
        .iter()
        .map(|(_, number)| board.board.net_via_count(*number).max(0) as usize)
        .sum();
    Ok(RouteGrade { opens, drc, vias })
}

fn quantize_mm(value: f64, units_per_mm: f64) -> Result<i64, String> {
    let scaled = value / units_per_mm * 10_000.0;
    if !scaled.is_finite() || scaled < i64::MIN as f64 || scaled > i64::MAX as f64 {
        return Err("copper coordinate outside fingerprint range".into());
    }
    Ok(scaled.round() as i64)
}

/// Hash routed copper only, matching dedupe_boards.fingerprint: segment and
/// via geometry, dimensions, layers and net identity; board UUIDs are excluded.
pub fn copper_fingerprint(board: &BoardAdapter) -> Result<String, String> {
    let mut segments = Vec::<(i64, i64, i64, i64, i32, i32, i64)>::new();
    // DSN padstacks do not carry explicit drill/plating data. Preserve the
    // actual stack identity instead of hashing Padstack::get_drill_radius(),
    // which is only a name/shape-based estimate.
    let mut vias = Vec::<(i64, i64, i64, i32, String)>::new();
    for key in board.board.get_items() {
        let item = board.board.item(key);
        let net = item.net_numbers().first().copied().unwrap_or(0);
        if let Some(trace) = item.as_trace() {
            let polyline = trace.polyline();
            for index in 0..polyline.lines.len().saturating_sub(2) {
                let a = polyline.corner(index as i32).to_float();
                let b = polyline.corner(index as i32 + 1).to_float();
                segments.push((
                    quantize_mm(a.x, board.units_per_mm)?,
                    quantize_mm(a.y, board.units_per_mm)?,
                    quantize_mm(b.x, board.units_per_mm)?,
                    quantize_mm(b.y, board.units_per_mm)?,
                    trace.layer(),
                    net,
                    quantize_mm(trace.half_width() as f64 * 2.0, board.units_per_mm)?,
                ));
            }
        } else if item.as_via().is_some() {
            let center = item.center(&board.board).to_float();
            let padstack = item
                .padstack(&board.board)
                .ok_or("via has no native padstack")?;
            let shape = (0..board.board.layer_count())
                .find_map(|layer| padstack.get_shape(layer))
                .ok_or("via has no copper shape")?;
            let bounds = shape.bounding_box();
            let size = bounds.width().min(bounds.height()) as f64;
            vias.push((
                quantize_mm(center.x, board.units_per_mm)?,
                quantize_mm(center.y, board.units_per_mm)?,
                quantize_mm(size, board.units_per_mm)?,
                net,
                padstack.name.clone(),
            ));
        }
    }
    segments.sort_unstable();
    vias.sort_unstable();
    let mut hash = Sha1::new();
    hash.update(b"fastroute-native-copper-v1\0");
    hash.update(format!("{segments:?}|{vias:?}").as_bytes());
    Ok(format!("{:x}", hash.finalize()))
}

impl<B: NativeEvolutionBackend> EvolutionOperators<NativeWorld> for NativeEvolutionOperators<B> {
    fn import_seed(
        &self,
        seed: &Path,
        requested_k: i64,
        board: &Path,
        destination: &str,
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
        let manifest_path = if seed.is_absolute() {
            seed.to_owned()
        } else {
            self.seed_base.join(seed)
        };
        let manifest_dir = manifest_path
            .parent()
            .ok_or("native seed manifest has no parent directory")?;
        let manifest_text = fs::read_to_string(&manifest_path)
            .map_err(|e| format!("native seed manifest {}: {e}", manifest_path.display()))?;
        let manifest: NativeSeedManifest = toml::from_str(&manifest_text)
            .map_err(|e| format!("native seed manifest {}: {e}", manifest_path.display()))?;
        if manifest.format.as_deref().is_some_and(|value| {
            !matches!(
                value,
                "fastroute-native-seed-v1" | "fastroute-native-seed-v2"
            )
        }) {
            return Err("unsupported native seed manifest format".into());
        }
        if manifest.format.as_deref() == Some("fastroute-native-seed-v2")
            && (manifest.active_nets.is_none() || manifest.runtime_scope.is_none())
        {
            return Err("native seed v2 manifest lacks its captured active route scope".into());
        }
        if manifest.requested_k.is_some_and(|k| k != requested_k) {
            return Err(format!(
                "native seed manifest K {:?} differs from requested K {requested_k}",
                manifest.requested_k
            ));
        }
        if manifest
            .destination
            .as_deref()
            .is_some_and(|value| value != destination)
        {
            return Err(
                "native seed manifest destination differs from configured destination".into(),
            );
        }
        let routed_dsn = resolve_manifest_path(manifest_dir, &manifest.routed_dsn);
        let fanout_dsn = resolve_manifest_path(manifest_dir, &manifest.fanout_dsn);
        let source_dsn = fs::canonicalize(&routed_dsn)
            .map_err(|e| format!("native seed routed DSN {}: {e}", routed_dsn.display()))?;
        let fanout_source_dsn = fs::canonicalize(&fanout_dsn)
            .map_err(|e| format!("native seed fanout DSN {}: {e}", fanout_dsn.display()))?;
        // Authorize only net/padstack associations captured from the complete
        // exported routed/fanout boards. Legacy drill metadata without these
        // associations remains parseable, but cannot authorize new SES copper.
        let native_vias = manifest.native_vias.as_deref().unwrap_or(&[]);
        let mut routed = import_board(
            &routed_dsn,
            manifest.routed_ses.as_deref(),
            manifest_dir,
            "routed",
            native_vias,
        )?;
        let mut fanout = import_board(
            &fanout_dsn,
            manifest.fanout_ses.as_deref(),
            manifest_dir,
            "fanout",
            native_vias,
        )?;
        if let Some(native_vias) = manifest.native_vias.as_deref() {
            restore_native_via_drills(&mut routed, native_vias, "routed")?;
            restore_native_via_drills(&mut fanout, native_vias, "fanout")?;
        }
        // The configured board remains an explicit compatibility anchor; reject KiCad or
        // unrelated input early rather than implying that it was imported.
        let base_bytes = read_required(board, "configured board DSN")?;
        let base_board = BoardAdapter::load(&base_bytes)
            .map_err(|e| format!("configured board is not a supported native DSN: {e}"))?;
        let mut pack_sidecar = parse_pack(&resolve_manifest_path(manifest_dir, &manifest.pack))?;
        let plan_path = resolve_manifest_path(manifest_dir, &manifest.plan);
        let mut plan_sidecar = parse_plan(&plan_path)?;
        let initial_plan_ends = plan_sidecar
            .get("ends")
            .and_then(Value::as_object)
            .cloned()
            .ok_or("validated native plan lost ends object")?;
        let names = active_names(&manifest, &self.spec, &pack_sidecar, &initial_plan_ends)?;
        let configured: std::collections::BTreeSet<_> = names.iter().cloned().collect();
        let runtime_spec = filtered_spec(&self.spec, &manifest, &names)?;
        ensure_same_layout_with_spec(&base_board, &routed, &runtime_spec, "routed")?;
        ensure_same_layout_with_spec(&base_board, &fanout, &runtime_spec, "fanout")?;
        for name in &names {
            base_board.net_number(name)?;
            routed.net_number(name)?;
            fanout.net_number(name)?;
        }
        filter_pack_to_names(&mut pack_sidecar, &configured)?;
        for field in [
            "ends",
            "tooth_layer",
            "dest_layer",
            "tooth_dir",
            "stub_dir",
            "berth_sig",
            "dest_chain",
        ] {
            scope_sidecar_map(&mut plan_sidecar, field, &configured);
        }
        let plan_ends = plan_sidecar
            .get("ends")
            .and_then(Value::as_object)
            .cloned()
            .ok_or("scoped native seed plan lost ends object")?;
        let lanes = pack_sidecar
            .get("lanes")
            .and_then(Value::as_object)
            .expect("parse_pack validates lanes");
        if lanes.len() != names.len() || plan_ends.len() != names.len() {
            return Err("native seed pack/plan scope differs from active_nets".into());
        }
        let mut census_sidecar = manifest
            .census
            .as_deref()
            .map(|p| parse_census(&resolve_manifest_path(manifest_dir, p)))
            .transpose()?;
        if let Some(census) = census_sidecar.as_mut() {
            if let Some(census_map) = census.as_object_mut() {
                census_map.retain(|name, _| configured.contains(name));
            }
        }
        let plan_sidecar_hash = source_hash(&read_required(&plan_path, "fanout plan")?, 16);
        let census_path = manifest
            .census
            .as_deref()
            .map(|p| resolve_manifest_path(manifest_dir, p));
        let census_sidecar_hash = census_path
            .as_deref()
            .map(|path| read_required(path, "census metadata"))
            .transpose()?
            .map(|bytes| source_hash(&bytes, 16));
        let mut verdict_evidence = match manifest.log.as_deref() {
            Some(path) => parse_log_evidence(&resolve_manifest_path(manifest_dir, path))?,
            None => std::collections::BTreeMap::new(),
        };
        if let Some(path) = manifest.refusals.as_deref() {
            merge_refusals(
                &mut verdict_evidence,
                &resolve_manifest_path(manifest_dir, path),
            )?;
        }
        if let Some(path) = manifest.verdicts.as_deref() {
            merge_verdicts(
                &mut verdict_evidence,
                &resolve_manifest_path(manifest_dir, path),
            )?;
        }
        verdict_evidence.retain(|name, _| configured.contains(name));
        for (name, lane) in lanes {
            let fact = verdict_slot(&mut verdict_evidence, name);
            fact.lane_vias = lane
                .get("vias")
                .and_then(Value::as_array)
                .map(Vec::len)
                .ok_or_else(|| format!("pack lane {name} lacks vias array"))?;
            fact.predicted_vias = ["predicted_vias", "predicted", "model_vias"]
                .iter()
                .find_map(|key| lane.get(*key).and_then(Value::as_f64));
            fact.model_page = lane
                .get("model_page")
                .or_else(|| lane.get("page"))
                .and_then(Value::as_str)
                .map(str::to_owned);
        }
        for name in &names {
            let number = routed.net_number(name)?;
            verdict_slot(&mut verdict_evidence, name).actual_vias =
                routed.board.net_via_count(number).max(0) as usize;
        }
        if let Some(census) = census_sidecar.as_ref().and_then(Value::as_object) {
            for (name, evidence) in census {
                let fact = verdict_slot(&mut verdict_evidence, name);
                fact.census.push(evidence.to_string());
            }
        }
        let grade = grade_board(&routed, &runtime_spec)?;
        if let Some(recorded) = manifest.grade {
            if recorded.opens != grade.opens
                || recorded.drc != grade.drc
                || recorded.vias != grade.vias
            {
                return Err(
                    "native seed manifest grade does not match imported routed board".into(),
                );
            }
        }
        let id = seed.display().to_string();
        Ok(Some(EvolutionWorld {
            id,
            name: String::new(),
            origin: String::new(),
            grade,
            data: NativeWorld {
                routed,
                fanout,
                spec: runtime_spec,
                requested_k,
                destination: destination.to_owned(),
                seed_stem: manifest_path,
                source_dsn,
                fanout_source_dsn,
                pack_sidecar,
                plan_ends,
                plan_sidecar,
                plan_sidecar_hash,
                census_sidecar,
                census_sidecar_hash,
                verdict_evidence,
            },
        }))
    }

    fn descend(
        &self,
        world: &EvolutionWorld<NativeWorld>,
        options: DescentOptions,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
        self.backend.descend(world, options, env)
    }

    fn jump_near(
        &self,
        world: &EvolutionWorld<NativeWorld>,
        n_nets: usize,
        seed: u32,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
        self.backend.jump_near(world, n_nets, seed, env)
    }

    fn cross_probe(
        &self,
        a: &EvolutionWorld<NativeWorld>,
        b: &EvolutionWorld<NativeWorld>,
        seed: u32,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
        self.backend.cross_probe(a, b, seed, env)
    }

    fn fingerprint(&self, world: &EvolutionWorld<NativeWorld>) -> Result<String, String> {
        copper_fingerprint(&world.data.routed)
    }

    fn descent_key(
        &self,
        world: &EvolutionWorld<NativeWorld>,
        options: &DescentOptions,
        env: &[(String, String)],
    ) -> Result<String, String> {
        let routed = copper_fingerprint(&world.data.routed)?;
        let fanout = copper_fingerprint(&world.data.fanout)?;
        let mut knobs: std::collections::BTreeMap<String, String> =
            std::env::vars().filter(|(key, _)| knob_name(key)).collect();
        for (key, value) in env {
            if knob_name(key) {
                knobs.insert(key.clone(), value.clone());
            }
        }
        let arguments = serde_json::json!({
            "rounds": options.rounds,
            "worst": options.worst,
            "probes": options.probes,
            "min_vias": options.min_vias,
            "coupled": options.coupled,
            "grade": options.grade,
        });
        let knobs = source_hash(&serde_json::to_vec(&knobs).map_err(|e| e.to_string())?, 12);
        let parts = serde_json::json!({
            "code": native_code_hash(),
            "knobs": knobs,
            "K": world.data.requested_k,
            "args": arguments,
            "R": routed,
            "F": fanout,
            "sidecar": world.data.plan_sidecar_hash,
            "census": world.data.census_sidecar_hash,
        });
        Ok(sha1_hex(
            &serde_json::to_vec(&parts).map_err(|e| e.to_string())?,
        ))
    }
}
