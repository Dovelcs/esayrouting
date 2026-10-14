//! Human-readable options for the AWX modes ported from the fixed reference.
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::PathBuf};

/// Optional destination for native seed-bundle export. When omitted, routing
/// produces no extra files beyond its configured session output.
#[derive(Debug, Clone, Default, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct SeedExportOptions {
    /// New directory for routed/fanout data and its `seed.toml` manifest.
    pub directory: Option<PathBuf>,
}
impl SeedExportOptions {
    pub fn validate(&self) -> Result<(), String> {
        if self
            .directory
            .as_ref()
            .is_some_and(|path| path.as_os_str().is_empty())
        {
            return Err("awx.seed_export.directory must not be empty".into());
        }
        Ok(())
    }
}

fn default_min_bus_nets() -> usize {
    8
}
fn default_rounds() -> usize {
    3
}
fn default_evolve_dest() -> String {
    "DU1".into()
}
fn default_population() -> usize {
    4
}
fn default_generations() -> usize {
    3
}
fn default_jumps() -> usize {
    2
}
fn default_crossovers() -> usize {
    1
}
fn default_jobs() -> usize {
    2
}
fn default_jump_nets() -> usize {
    2
}
fn default_seed() -> u64 {
    1
}
fn default_descent_rounds() -> usize {
    2
}
fn default_descent_worst() -> usize {
    6
}
fn default_descent_probes() -> usize {
    2
}
fn default_descent_min_vias() -> usize {
    2
}

/// Options for `route_bus.find_buses`. The producer needs an existing board
/// snapshot and the already resolved via rule; it does not infer components.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct BusDiscoveryOptions {
    pub enabled: bool,
    /// Source default: `MIN_BUS_NETS = 8`.
    pub min_bus_nets: usize,
}
impl Default for BusDiscoveryOptions {
    fn default() -> Self {
        Self {
            enabled: false,
            min_bus_nets: default_min_bus_nets(),
        }
    }
}
impl BusDiscoveryOptions {
    pub fn validate(&self) -> Result<(), String> {
        if self.min_bus_nets == 0 {
            return Err("awx.bus_discovery.min_bus_nets must be positive".into());
        }
        Ok(())
    }
}

/// Source `route_bus` ladder prefix and whole-route rounds.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct RouteLadderOptions {
    /// Python slice-compatible K: absent or zero selects the full ladder;
    /// positive values take a prefix and negative values omit a suffix.
    pub requested_k: Option<i64>,
    /// Source default: 3 whole-route rounds; rounds are not ladder rungs.
    pub rounds: usize,
}
impl Default for RouteLadderOptions {
    fn default() -> Self {
        Self {
            requested_k: None,
            rounds: default_rounds(),
        }
    }
}
impl RouteLadderOptions {
    pub fn validate(&self) -> Result<(), String> {
        if self.rounds == 0 {
            return Err("awx.route_ladder.rounds must be positive".into());
        }
        Ok(())
    }
}

/// `smooth_board.py` has no user tuning flags. Its inputs are the routed
/// board, corresponding fanout board, and selected lane nets from execution.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct BoardSmoothingOptions {
    pub enabled: bool,
}
impl Default for BoardSmoothingOptions {
    fn default() -> Self {
        Self { enabled: false }
    }
}

/// Evolve mode's published controls. `tag`, `requested_k`, `seeds`, and
/// `board` are required when enabled. Each seed is a native TOML manifest
/// resolved relative to the job configuration; it describes routed/fanout DSN
/// inputs and optional SES data, plus the pack/plan/census data used by evolve.
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct EvolveOptions {
    pub enabled: bool,
    pub tag: Option<String>,
    pub requested_k: Option<i64>,
    pub seeds: Vec<PathBuf>,
    pub board: Option<PathBuf>,
    /// Source default: `DU1`.
    pub destination: String,
    /// Source defaults: pop=4, gens=3, jumps=2, cross=1, jobs=2,
    /// jump-nets=2, seed=1.
    pub population: usize,
    pub generations: usize,
    pub jumps: usize,
    pub crossovers: usize,
    pub jobs: usize,
    pub jump_nets: usize,
    pub seed: u64,
    pub descent: EvolveDescentOptions,
    /// Extra environment values passed to the descent and jump operators.
    pub descend_env: BTreeMap<String, String>,
    pub jump_env: BTreeMap<String, String>,
}
impl Default for EvolveOptions {
    fn default() -> Self {
        let mut descend_env = BTreeMap::new();
        descend_env.insert("DST_CLIMB".into(), "2".into());
        let mut jump_env = BTreeMap::new();
        jump_env.insert("DST_CLIMB".into(), "2".into());
        jump_env.insert("SRC_CLIMB".into(), "4".into());
        Self {
            enabled: false,
            tag: None,
            requested_k: None,
            seeds: Vec::new(),
            board: None,
            destination: default_evolve_dest(),
            population: default_population(),
            generations: default_generations(),
            jumps: default_jumps(),
            crossovers: default_crossovers(),
            jobs: default_jobs(),
            jump_nets: default_jump_nets(),
            seed: default_seed(),
            descent: EvolveDescentOptions::default(),
            descend_env,
            jump_env,
        }
    }
}
impl EvolveOptions {
    pub fn validate(&self) -> Result<(), String> {
        if self.population == 0 || self.jobs == 0 {
            return Err("awx.evolve.population and awx.evolve.jobs must be positive".into());
        }
        if self.destination.trim().is_empty() {
            return Err("awx.evolve.destination must not be empty".into());
        }
        validate_env("awx.evolve.descend_env", &self.descend_env)?;
        validate_env("awx.evolve.jump_env", &self.jump_env)?;
        self.descent.validate()?;
        if self.enabled {
            let tag = self.tag.as_deref().unwrap_or_default();
            if tag.trim().is_empty()
                || tag.contains('/')
                || tag.contains('\\')
                || tag == "."
                || tag == ".."
            {
                return Err(
                    "awx.evolve.tag is required and must be a single path component".into(),
                );
            }
            if self.requested_k.is_none() {
                return Err("awx.evolve.requested_k is required when evolve is enabled".into());
            }
            if self.seeds.is_empty() {
                return Err(
                    "awx.evolve.seeds must contain at least one native seed manifest".into(),
                );
            }
            if self.board.as_ref().is_none_or(|p| p.as_os_str().is_empty()) {
                return Err("awx.evolve.board is required when evolve is enabled".into());
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(default, deny_unknown_fields)]
pub struct EvolveDescentOptions {
    /// Defaults copied from evolve.py's `--descend` value.
    pub rounds: usize,
    pub worst: usize,
    pub probes: usize,
    pub min_vias: usize,
    pub coupled: CoupledMode,
    pub grade: GradeMode,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum CoupledMode {
    Census,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum GradeMode {
    Sub,
    Inproc,
}
impl Default for EvolveDescentOptions {
    fn default() -> Self {
        Self {
            rounds: default_descent_rounds(),
            worst: default_descent_worst(),
            probes: default_descent_probes(),
            min_vias: default_descent_min_vias(),
            coupled: CoupledMode::Census,
            grade: GradeMode::Inproc,
        }
    }
}
impl EvolveDescentOptions {
    fn validate(&self) -> Result<(), String> {
        if self.rounds == 0 || self.worst == 0 || self.probes == 0 {
            return Err("awx.evolve.descent rounds/worst/probes must be positive".into());
        }
        Ok(())
    }
}

fn validate_env(name: &str, values: &BTreeMap<String, String>) -> Result<(), String> {
    for (key, value) in values {
        if key.trim().is_empty() || key.contains('=') || key.contains('\0') || value.contains('\0')
        {
            return Err(format!("{name} contains an invalid environment entry"));
        }
    }
    Ok(())
}
