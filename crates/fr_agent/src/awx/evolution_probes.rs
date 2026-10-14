//! Evolution's near-jump and crossover operators over native incremental probes.

use super::{
    evolution_native::{world_from_native, NativeWorld},
    native_replan::{run_replan, AcceptedKind, NativeReplanOptions, ReplanMode},
    replan_state::ReplanState,
    route_evolution::EvolutionWorld,
};

fn operation_options(seed: u32, env: &[(String, String)]) -> Result<NativeReplanOptions, String> {
    let source_climb = env
        .iter()
        .find(|(key, _)| key == "SRC_CLIMB")
        .map(|(_, value)| {
            value
                .parse::<u32>()
                .map_err(|e| format!("invalid SRC_CLIMB value {value:?}: {e}"))
        })
        .transpose()?
        .unwrap_or(14);
    Ok(NativeReplanOptions {
        rounds: 1,
        worst: usize::MAX,
        probes: 3,
        min_vias: 3,
        gate_min: 2,
        gates: 3,
        census: 4,
        joint: true,
        coupled: "census".into(),
        grade: "inproc".into(),
        mode: ReplanMode::Incremental,
        apply_strip: true,
        seed: seed as u64,
        jobs: 1,
        perturb_nets: 0,
        perturb_seed: seed as u64,
        perturb_tries: 3,
        cross_fraction: 0.5,
        cross_parent: None,
        source_climb,
        descend_env: env.to_vec(),
        probe_env: env.to_vec(),
    })
}

/// Produce a near jump by perturbing the requested number of nets through
/// native replan probes. A world exists only when a probe actually landed.
pub fn jump_near(
    world: &EvolutionWorld<NativeWorld>,
    n_nets: usize,
    seed: u32,
    env: &[(String, String)],
) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
    if n_nets == 0 {
        return Ok(None);
    }
    let state = ReplanState::from_evolution_world(world.clone())?;
    let mut options = operation_options(seed, env)?;
    options.perturb_nets = n_nets;
    let result = run_replan(state, options)?;

    // Accepted records are emitted only after the native probe reports an
    // achieved end and its board has been installed in the replan state.
    if !result
        .accepted
        .iter()
        .any(|record| record.kind == AcceptedKind::Jumped)
    {
        return Ok(None);
    }
    world_from_native(format!("{}:jump:{seed}", world.id), result.state.world).map(Some)
}

/// Recombine differing per-net ends from B onto A and keep the result only
/// when at least one crossover probe physically landed.
pub fn cross_probe(
    a: &EvolutionWorld<NativeWorld>,
    b: &EvolutionWorld<NativeWorld>,
    seed: u32,
    env: &[(String, String)],
) -> Result<Option<EvolutionWorld<NativeWorld>>, String> {
    let state = ReplanState::from_evolution_world(a.clone())?;
    let parent = ReplanState::from_evolution_world(b.clone())?;
    let mut options = operation_options(seed, env)?;
    options.cross_parent = Some(Box::new(parent));
    let result = run_replan(state, options)?;

    if !result.accepted.iter().any(|record| {
        record.kind == AcceptedKind::Crossed
            && (record.achieved_src.is_some() || record.achieved_dst.is_some())
    }) {
        return Ok(None);
    }
    world_from_native(
        format!("{}:cross:{}:{seed}", a.id, b.id),
        result.state.world,
    )
    .map(Some)
}
