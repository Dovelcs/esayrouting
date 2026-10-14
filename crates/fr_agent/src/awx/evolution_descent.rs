//! Native implementation of `evolve.py::descend`.
//!
//! A descent is the incremental replan producer (worst-net selection followed
//! by the native candidate probes), not a general whole-board reroute.

use super::{
    evolution_native::NativeWorld,
    native_replan::{run_replan, NativeReplanOptions, ReplanMode},
    replan_state::ReplanState,
    route_evolution::{DescentOptions, EvolutionWorld},
};

/// Run one source-equivalent incremental descent from `world`.
///
/// `run_replan` owns per-round candidate acceptance and stopping. The returned
/// world is the coordinator's materialized routed/fanout state, even when no
/// candidate improved it; the evolution controller applies the source's final
/// strict-improvement check against the parent grade.
pub fn descend(
    world: &EvolutionWorld<NativeWorld>,
    options: DescentOptions,
    env: &[(String, String)],
) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
    let state = ReplanState::new(world.data.clone())?;
    let native_options = NativeReplanOptions {
        rounds: options.rounds,
        worst: options.worst,
        probes: options.probes,
        min_vias: options.min_vias,
        // These are replan.py's defaults for gates, census depth, and joint
        // tooth/berth probing; the descent options expose only the reference
        // CLI's rounds/worst/probes/min-vias/coupled/grade controls.
        gate_min: 2,
        gates: 3,
        census: 4,
        joint: true,
        coupled: options.coupled,
        grade: options.grade,
        mode: ReplanMode::Incremental,
        apply_strip: true,
        seed: 1,
        jobs: 1,
        perturb_nets: 0,
        perturb_seed: 1,
        perturb_tries: 1,
        cross_fraction: 0.5,
        cross_parent: None,
        source_climb: 14,
        descend_env: env.to_vec(),
        probe_env: env.to_vec(),
    };
    let result = run_replan(state, native_options)?;
    let state = result.state;
    Ok(Some(EvolutionWorld {
        id: format!("{}:descend", world.id),
        name: world.name.clone(),
        origin: world.origin.clone(),
        grade: state.current_grade,
        data: state.world,
    }))
}
