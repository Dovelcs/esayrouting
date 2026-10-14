use serde::{Deserialize, Serialize};
use std::{
    collections::{BTreeSet, HashMap, HashSet},
    fmt, fs,
    path::{Component, Path, PathBuf},
};

#[path = "awx/config_options.rs"]
pub mod config_options;
#[path = "awx/smoothing_metadata.rs"]
pub mod smoothing_metadata;

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RouteSpec {
    pub version: u32,
    #[serde(default = "default_true")]
    pub route_remaining: bool,
    #[serde(default)]
    pub allow_partial_output: bool,
    #[serde(default)]
    pub protected_nets: Vec<String>,
    pub input: PathBuf,
    pub output: PathBuf,
    pub nets: Vec<NetRule>,
    #[serde(default)]
    pub endpoint_groups: Vec<EndpointGroup>,
    #[serde(default)]
    pub channels: Vec<Channel>,
    pub groups: Vec<RouteGroup>,
    #[serde(default)]
    pub execution: Execution,
    pub advanced: Option<AdvancedConstraints>,
    pub awx: Option<AwxSpec>,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct NetRule {
    pub name: String,
    pub width_mm: f64,
    pub clearance_mm: f64,
    pub allowed_layers: Vec<String>,
    pub via_budget: Option<u32>,
    pub max_length_mm: Option<f64>,
    #[serde(default)]
    pub protected: bool,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Endpoint {
    pub net: String,
    pub chip: String,
    pub pad: String,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct EndpointGroup {
    pub id: String,
    pub endpoints: Vec<Endpoint>,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Channel {
    pub id: String,
    pub region: ChannelRegion,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ChannelRegion {
    Rectangle {
        min: [f64; 2],
        max: [f64; 2],
    },
    Polyline {
        points: Vec<[f64; 2]>,
        width_mm: f64,
    },
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct RouteGroup {
    pub id: String,
    pub nets: Vec<String>,
    #[serde(default)]
    pub endpoint_groups: Vec<String>,
    #[serde(default)]
    pub channels: Vec<String>,
    #[serde(default)]
    pub priority: i32,
    #[serde(default)]
    pub depends_on: Vec<String>,
    #[serde(default)]
    pub joint_escape: bool,
    pub awx: Option<AwxGroup>,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Execution {
    pub threads: usize,
    pub budget_ms: u64,
    pub seed: u64,
}
impl Default for Execution {
    fn default() -> Self {
        Self {
            threads: 1,
            budget_ms: 30_000,
            seed: 0,
        }
    }
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AdvancedConstraints {
    pub differential_coupling: Option<bool>,
    pub impedance_ohms: Option<f64>,
    pub cross_net_paths: Option<bool>,
}
#[derive(Debug, Clone, Deserialize, Serialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum RuleStatus {
    Supported,
    Executed,
    Unsupported,
    Unmet,
}
#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct RuleReport {
    pub rule: String,
    pub status: RuleStatus,
    pub detail: String,
}
#[derive(Debug, Clone)]
pub struct LoadedSpec {
    pub spec: RouteSpec,
    pub config_path: PathBuf,
    /// Resolved native evolution artifact directory derived from the source tag.
    pub evolve_output_directory: Option<PathBuf>,
}
#[derive(Debug)]
pub struct SpecError(pub String);
impl fmt::Display for SpecError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for SpecError {}
fn default_true() -> bool {
    true
}
fn err(s: impl Into<String>) -> SpecError {
    SpecError(s.into())
}
fn positive(v: f64, what: &str) -> Result<(), SpecError> {
    if !v.is_finite() || v <= 0.0 {
        Err(err(format!("{what} must be finite and positive")))
    } else {
        Ok(())
    }
}
fn unique<'a>(
    values: impl IntoIterator<Item = &'a str>,
    what: &str,
) -> Result<HashSet<String>, SpecError> {
    let mut seen = HashSet::new();
    for v in values {
        if v.trim().is_empty() || !seen.insert(v.to_owned()) {
            return Err(err(format!("empty or duplicate {what}: {v}")));
        }
    }
    Ok(seen)
}
fn resolve(p: &Path, base: &Path) -> Result<PathBuf, SpecError> {
    if p.as_os_str().is_empty() {
        return Err(err("empty file path"));
    }
    let absolute = if p.is_absolute() {
        p.to_owned()
    } else {
        base.join(p)
    };
    // Resolve existing ancestors before processing `..`, preserving symlink semantics.
    let mut resolved = PathBuf::new();
    for component in absolute.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                resolved.pop();
            }
            _ => {
                resolved.push(component.as_os_str());
                match fs::symlink_metadata(&resolved) {
                    Ok(_) => {
                        resolved = fs::canonicalize(&resolved)
                            .map_err(|e| err(format!("{}: {e}", resolved.display())))?;
                    }
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                    Err(e) => return Err(err(e.to_string())),
                }
            }
        }
    }
    Ok(resolved)
}
fn paths_overlap(a: &Path, b: &Path) -> bool {
    a == b || a.starts_with(b) || b.starts_with(a)
}
impl LoadedSpec {
    pub fn load(path: &Path) -> Result<Self, SpecError> {
        Self::parse(
            &fs::read_to_string(path).map_err(|e| err(format!("{}: {e}", path.display())))?,
            path,
        )
    }
    pub fn parse(text: &str, path: &Path) -> Result<Self, SpecError> {
        let cwd = std::env::current_dir().map_err(|e| err(e.to_string()))?;
        let config_path = resolve(path, &cwd)?;
        let base = config_path
            .parent()
            .ok_or_else(|| err("config has no parent"))?;
        let mut spec: RouteSpec = toml::from_str(text).map_err(|e| err(e.to_string()))?;
        spec.validate()?;
        spec.input = resolve(&spec.input, base)?;
        spec.output = resolve(&spec.output, base)?;
        let mut evolve_output_directory = None;
        if let Some(awx) = spec.awx.as_mut() {
            if let Some(board) = awx.evolve.board.as_mut() {
                *board = resolve(board, base)?;
            }
            for seed in &mut awx.evolve.seeds {
                *seed = resolve(seed, base)?;
            }
            if let Some(directory) = awx.seed_export.directory.as_mut() {
                *directory = resolve(directory, base)?;
            }
            let evolve_enabled = awx.evolve.enabled;
            let export_target = if evolve_enabled {
                let root = awx
                    .seed_export
                    .directory
                    .clone()
                    .unwrap_or_else(|| base.join("tmp"));
                if let Ok(metadata) = fs::symlink_metadata(&root) {
                    if !metadata.is_dir() {
                        return Err(err(format!(
                            "AWX evolve output root is not a directory: {}",
                            root.display()
                        )));
                    }
                }
                let tag = awx.evolve.tag.as_deref().ok_or_else(|| {
                    err("awx.evolve.tag is required when evolve is enabled")
                })?;
                Some(resolve(&root.join(tag), base)?)
            } else {
                awx.seed_export.directory.clone()
            };
            if let Some(target) = export_target {
                for (name, path) in [
                    ("configuration", config_path.as_path()),
                    ("input DSN", spec.input.as_path()),
                    ("session output", spec.output.as_path()),
                ] {
                    if paths_overlap(&target, path) {
                        return Err(err(format!(
                            "AWX seed output aliases or contains the {name} path"
                        )));
                    }
                }
                if let Some(board) = &awx.evolve.board {
                    if paths_overlap(&target, board) {
                        return Err(err("AWX seed output aliases or contains awx.evolve.board"));
                    }
                }
                for seed in &awx.evolve.seeds {
                    if paths_overlap(&target, seed) {
                        return Err(err("AWX seed output aliases or contains an evolve seed manifest"));
                    }
                }
                if fs::symlink_metadata(&target).is_ok() {
                    return Err(err(format!(
                        "refusing to overwrite existing AWX seed output directory: {}",
                        target.display()
                    )));
                }
                if evolve_enabled {
                    evolve_output_directory = Some(target);
                } else if awx.seed_export.directory.is_some() {
                    awx.seed_export.directory = Some(target);
                }
            }
        }
        if spec.input == spec.output || spec.output == config_path {
            return Err(err("output must differ from input and configuration"));
        }
        if fs::symlink_metadata(&spec.output).is_ok() {
            return Err(err(format!(
                "refusing to overwrite existing output: {}",
                spec.output.display()
            )));
        }
        Ok(Self {
            spec,
            config_path,
            evolve_output_directory,
        })
    }
}
impl RouteSpec {
    pub fn protected_names(&self) -> BTreeSet<String> {
        self.protected_nets
            .iter()
            .cloned()
            .chain(
                self.nets
                    .iter()
                    .filter(|n| n.protected)
                    .map(|n| n.name.clone()),
            )
            .collect()
    }
    /// Configuration support only; execution reports must replace these statuses after verification.
    pub fn structure_reports(&self) -> Vec<RuleReport> {
        let mut reports = Vec::new();
        let mut add = |rule: String| {
            reports.push(RuleReport {
                rule,
                status: RuleStatus::Supported,
                detail: "configuration accepted; board validation and execution pending".into(),
            })
        };
        for n in &self.nets {
            for rule in ["width_mm", "clearance_mm", "allowed_layers"] {
                add(format!("net.{}.{}", n.name, rule));
            }
            if n.via_budget.is_some() {
                add(format!("net.{}.via_budget", n.name));
            }
            if n.max_length_mm.is_some() {
                add(format!("net.{}.max_length_mm", n.name));
            }
            if n.protected {
                add(format!("net.{}.protected", n.name));
            }
        }
        for name in &self.protected_nets {
            add(format!("protected_nets.{name}.geometry"));
        }
        for e in &self.endpoint_groups {
            add(format!("endpoint_group.{}", e.id));
        }
        for c in &self.channels {
            add(format!("channel.{}", c.id));
        }
        for g in &self.groups {
            add(format!("group.{}.priority", g.id));
            if !g.depends_on.is_empty() {
                add(format!("group.{}.dependencies", g.id));
            }
            if g.joint_escape {
                add(format!("group.{}.joint_escape", g.id));
            }
        }
        add("route_remaining".into());
        if self.allow_partial_output {
            add("allow_partial_output".into());
        }
        for key in ["threads", "budget_ms", "seed"] {
            add(format!("execution.{key}"));
        }
        reports
    }
    pub fn validate(&self) -> Result<(), SpecError> {
        let evolve_seed_mode = self
            .awx
            .as_ref()
            .is_some_and(|awx| awx.evolve.enabled);
        let discovery_only = self.groups.is_empty()
            && !self.route_remaining
            && self
                .awx
                .as_ref()
                .is_some_and(|awx| awx.bus_discovery.enabled);
        if self.version != 1 {
            return Err(err("only version = 1 is supported"));
        }
        if self.groups.is_empty() && !evolve_seed_mode && !discovery_only {
            if !self.route_remaining || self.nets.iter().any(|n| !n.protected) {
                return Err(err(
                    "empty groups require route_remaining = true and only protected nets",
                ));
            }
        } else if !self.groups.is_empty() && self.nets.is_empty() {
            return Err(err("route groups require declared nets"));
        }
        if discovery_only
            && (!self.nets.is_empty()
                || !self.endpoint_groups.is_empty()
                || !self.channels.is_empty())
        {
            return Err(err(
                "bus-discovery-only jobs do not accept route nets, endpoints, or channels",
            ));
        }
        if self.execution.threads == 0
            || self.execution.threads > 256
            || self.execution.budget_ms == 0
            || self.execution.budget_ms > 86_400_000
        {
            return Err(err("threads must be 1..256 and budget_ms 1..86400000"));
        }
        if let Some(a) = &self.advanced {
            if a.differential_coupling.is_some()
                || a.impedance_ohms.is_some()
                || a.cross_net_paths.is_some()
            {
                return Err(err("unsupported hard constraint: differential_coupling, impedance_ohms, or cross_net_paths"));
            }
        }
        if let Some(awx) = &self.awx {
            awx.rules.validate().map_err(err)?;
            awx.bus_discovery.validate().map_err(err)?;
            awx.route_ladder.validate().map_err(err)?;
            awx.evolve.validate().map_err(err)?;
            awx.seed_export.validate().map_err(err)?;
            if evolve_seed_mode && awx.bus_discovery.enabled {
                return Err(err(
                    "awx.bus_discovery.enabled is a separate discovery entry point and cannot be combined with awx.evolve",
                ));
            }
            if evolve_seed_mode && awx.board_smoothing.enabled {
                return Err(err(
                    "awx.board_smoothing.enabled applies to routed AWX lane copper and cannot be combined with awx.evolve",
                ));
            }
            if evolve_seed_mode
                && (awx.route_ladder.requested_k.is_some()
                    || awx.route_ladder.rounds != 3
                    || awx.via_nudge.max_shift_mm != default_via_nudge_max_shift_mm()
                    || !awx.fabrication_overrides.is_empty()
                    || !awx.passive_geometry.is_empty()
                    || !awx.component_body_bounds.is_empty()
                    || self.execution.seed != 0)
            {
                return Err(err(
                    "AWX evolve uses its own K/seed and native replan operators; execution.seed, route_ladder, via_nudge, fabrication overrides and passive-placement metadata do not apply",
                ));
            }
            if self.groups.is_empty() && !evolve_seed_mode {
                if awx.seed_export.directory.is_some() {
                    return Err(err(
                        "awx.seed_export.directory requires AWX route groups to capture routed seeds",
                    ));
                }
                if awx.board_smoothing.enabled {
                    return Err(err(
                        "awx.board_smoothing.enabled requires routed AWX lane copper",
                    ));
                }
                if awx.route_ladder.requested_k.is_some() || awx.route_ladder.rounds != 3 {
                    return Err(err(
                        "awx.route_ladder options require AWX route groups",
                    ));
                }
            }
            if let Some(metadata) = &awx.smoothing_metadata {
                metadata.validate().map_err(err)?;
            }
            if awx.board_smoothing.enabled && awx.smoothing_metadata.is_none() {
                return Err(err(
                    "awx.board_smoothing.enabled requires explicit awx.smoothing_metadata",
                ));
            }
            if !awx.via_nudge.max_shift_mm.is_finite() || awx.via_nudge.max_shift_mm < 0.0 {
                return Err(err(
                    "awx.via_nudge.max_shift_mm must be finite and non-negative",
                ));
            }
            for group in &self.groups {
                let route = group
                    .awx
                    .as_ref()
                    .ok_or_else(|| err("AWX groups require explicit source/destination/layers"))?;
                if route.source == route.destination
                    || route.routing_layers.len() < 2
                    || route.fanout_rounds == 0
                    || route.geometry_rounds == 0
                {
                    return Err(err("invalid AWX group source/layers/round counts"));
                }
                unique(route.routing_layers.iter().map(String::as_str), "AWX layer")?;
            }
        } else if self.groups.iter().any(|g| g.awx.is_some()) {
            return Err(err(
                "AWX group needs global physical component metadata and routing rules",
            ));
        }
        let protected = unique(
            self.protected_nets.iter().map(String::as_str),
            "protected net",
        )?;
        if self.nets.iter().any(|n| n.protected) {
            return Err(err("unsupported numeric protected NetRule; use top-level protected_nets for geometry freezing"));
        }
        let nets = unique(self.nets.iter().map(|n| n.name.as_str()), "net")?;
        if nets.iter().any(|n| protected.contains(n)) {
            return Err(err(
                "protected net cannot also have an active electrical NetRule",
            ));
        }
        let endpoints = unique(
            self.endpoint_groups.iter().map(|g| g.id.as_str()),
            "endpoint group",
        )?;
        let channels = unique(self.channels.iter().map(|c| c.id.as_str()), "channel")?;
        let groups = unique(self.groups.iter().map(|g| g.id.as_str()), "route group")?;
        for n in &self.nets {
            positive(n.width_mm, "width_mm")?;
            positive(n.clearance_mm, "clearance_mm")?;
            if let Some(v) = n.max_length_mm {
                positive(v, "max_length_mm")?;
            }
            if n.allowed_layers.is_empty() {
                return Err(err("allowed_layers must not be empty"));
            }
            unique(n.allowed_layers.iter().map(String::as_str), "layer")?;
        }
        for g in &self.endpoint_groups {
            if g.endpoints.is_empty() {
                return Err(err("endpoint group must not be empty"));
            }
            let mut seen = HashSet::new();
            for e in &g.endpoints {
                if !nets.contains(&e.net)
                    || e.chip.trim().is_empty()
                    || e.pad.trim().is_empty()
                    || !seen.insert((&e.net, &e.chip, &e.pad))
                {
                    return Err(err(format!("invalid or duplicate endpoint in {}", g.id)));
                }
            }
        }
        for c in &self.channels {
            match &c.region {
                ChannelRegion::Rectangle { min, max } => {
                    if min.iter().chain(max).any(|x| !x.is_finite())
                        || min[0] >= max[0]
                        || min[1] >= max[1]
                    {
                        return Err(err("rectangle must have finite increasing bounds"));
                    }
                }
                ChannelRegion::Polyline { points, width_mm } => {
                    positive(*width_mm, "channel width_mm")?;
                    if points.len() < 2
                        || points.iter().flatten().any(|x| !x.is_finite())
                        || points.windows(2).any(|p| p[0] == p[1])
                    {
                        return Err(err("polyline requires finite distinct consecutive points"));
                    }
                }
            }
        }
        let mut assigned = HashSet::new();
        let mut used_endpoints = HashSet::new();
        let mut used_channels = HashSet::new();
        for g in &self.groups {
            if g.nets.is_empty() {
                return Err(err(format!("group {} is empty", g.id)));
            }
            for (refs, set, kind) in [
                (&g.nets, &nets, "net"),
                (&g.endpoint_groups, &endpoints, "endpoint group"),
                (&g.channels, &channels, "channel"),
                (&g.depends_on, &groups, "dependency"),
            ] {
                unique(refs.iter().map(String::as_str), kind)?;
                for r in refs {
                    if !set.contains(r) {
                        return Err(err(format!("unknown {kind}: {r}")));
                    }
                }
            }
            for n in &g.nets {
                if !assigned.insert(n)
                    || self
                        .nets
                        .iter()
                        .any(|rule| &rule.name == n && rule.protected)
                {
                    return Err(err(format!(
                        "net {n} duplicated across groups or protected"
                    )));
                }
            }
            let mut group_pins = HashSet::new();
            used_channels.extend(g.channels.iter().cloned());
            for id in &g.endpoint_groups {
                used_endpoints.insert(id.clone());
                let eg = self.endpoint_groups.iter().find(|e| &e.id == id).unwrap();
                for e in &eg.endpoints {
                    if !group_pins.insert((&e.chip, &e.pad)) {
                        return Err(err(format!(
                            "duplicate or conflicting pad {}:{} in group {}",
                            e.chip, e.pad, g.id
                        )));
                    }
                }
                if eg.endpoints.iter().any(|e| !g.nets.contains(&e.net)) {
                    return Err(err("endpoint group refers outside route group's nets"));
                }
            }
        }
        if !discovery_only && (endpoints != used_endpoints || channels != used_channels) {
            return Err(err("unused endpoint group or channel definition"));
        }
        let map: HashMap<&str, &RouteGroup> =
            self.groups.iter().map(|g| (g.id.as_str(), g)).collect();
        fn visit<'a>(
            id: &'a str,
            map: &HashMap<&'a str, &'a RouteGroup>,
            active: &mut HashSet<&'a str>,
            done: &mut HashSet<&'a str>,
        ) -> Result<(), SpecError> {
            if done.contains(id) {
                return Ok(());
            }
            if !active.insert(id) {
                return Err(err(format!("dependency cycle at {id}")));
            }
            for d in &map[id].depends_on {
                visit(d, map, active, done)?;
            }
            active.remove(id);
            done.insert(id);
            Ok(())
        }
        let mut active = HashSet::new();
        let mut done = HashSet::new();
        for g in &self.groups {
            visit(&g.id, &map, &mut active, &mut done)?;
        }
        for n in &self.nets {
            if !evolve_seed_mode && !discovery_only && !n.protected && !assigned.contains(&n.name) {
                return Err(err(format!("unassigned net: {}", n.name)));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AwxSpec {
    pub components: Vec<crate::awx::board_snapshot::LogicalComponent>,
    pub via_drills: std::collections::BTreeMap<String, f64>,
    pub rules: crate::awx::whole_context::RoutingRules,
    pub fabrication: crate::awx::fab_rules::FabTier,
    pub escalation: crate::awx::fab_rules::EscalationPolicy,
    #[serde(default)]
    pub via_nudge: ViaNudgeSpec,
    #[serde(default)]
    pub bus_discovery: config_options::BusDiscoveryOptions,
    #[serde(default)]
    pub route_ladder: config_options::RouteLadderOptions,
    #[serde(default)]
    pub board_smoothing: config_options::BoardSmoothingOptions,
    /// Exact source pad metadata for the smoothing dense-field guard. Optional
    /// while smoothing is disabled; required and board-coverage-checked when enabled.
    #[serde(default)]
    pub smoothing_metadata: Option<smoothing_metadata::SmoothingMetadata>,
    #[serde(default)]
    pub evolve: config_options::EvolveOptions,
    #[serde(default)]
    pub seed_export: config_options::SeedExportOptions,
    #[serde(default)]
    pub fabrication_overrides: std::collections::BTreeMap<String, f64>,
    /// Explicit local courtyard/footprint geometry for movable passives. Native flat DSN
    /// component records do not carry enough package geometry to infer these safely.
    #[serde(default)]
    pub passive_geometry:
        std::collections::BTreeMap<String, crate::awx::passive_clearance::ComponentGeometry>,
    /// Component-local body bounds used when the native DSN has no component outline geometry.
    #[serde(default)]
    pub component_body_bounds: std::collections::BTreeMap<String, [f64; 4]>,
}
/// Configuration for the final via relocation pass. The 0.6 mm default matches
/// placement/fanout_clearance.py::nudge_vias_for_unresolved.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ViaNudgeSpec {
    #[serde(default = "default_via_nudge_max_shift_mm")]
    pub max_shift_mm: f64,
}
impl Default for ViaNudgeSpec {
    fn default() -> Self {
        Self {
            max_shift_mm: default_via_nudge_max_shift_mm(),
        }
    }
}
fn default_via_nudge_max_shift_mm() -> f64 {
    0.6
}
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct AwxGroup {
    pub source: String,
    pub destination: String,
    pub routing_layers: Vec<String>,
    #[serde(default)]
    pub pairs: Vec<crate::awx::pairs::Pair>,
    pub fanout_rounds: usize,
    pub geometry_rounds: usize,
    #[serde(default)]
    pub allow_unproved: bool,
}
