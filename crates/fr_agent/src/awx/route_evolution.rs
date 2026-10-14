//! Population controller ported from the pinned `awx/evolve.py` reference.
//!
//! Board parsing, staging, grading and the three route operators are provided
//! by `EvolutionOperators`; this module owns the reference generation,
//! acceptance, rollback, deduplication and elitist-selection policy.

use std::{cmp::Ordering, path::Path, thread};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RouteGrade {
    /// Names of unrouted nets (the reference orders by their count).
    pub opens: Vec<String>,
    pub drc: usize,
    pub vias: usize,
}

impl RouteGrade {
    fn population_key(&self) -> (usize, bool, usize) {
        (self.opens.len(), self.drc != 0, self.vias)
    }

    /// `plan_loop.better`: a candidate must be DRC-clean; then fewer opens,
    /// or (at equal opens) fewer vias, is an improvement.
    pub fn better_than(&self, reference: &Self) -> bool {
        self.drc == 0
            && (self.opens.len() < reference.opens.len()
                || (self.opens.len() == reference.opens.len() && self.vias < reference.vias))
    }
}

#[derive(Debug, Clone)]
pub struct EvolutionWorld<D> {
    /// Stable operator identifier (normally the board/world stem).
    pub id: String,
    pub name: String,
    pub origin: String,
    pub grade: RouteGrade,
    /// Adapter-owned routed board, fanout board, sidecars and other world data.
    pub data: D,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DescentOptions {
    pub rounds: usize,
    pub worst: usize,
    pub probes: usize,
    pub min_vias: usize,
    /// Source `--coupled=census` and `--grade=inproc` defaults.
    pub coupled: String,
    pub grade: String,
}

impl Default for DescentOptions {
    fn default() -> Self {
        Self {
            rounds: 2,
            worst: 6,
            probes: 2,
            min_vias: 2,
            coupled: "census".into(),
            grade: "inproc".into(),
        }
    }
}

#[derive(Debug, Clone)]
pub struct EvolutionConfig {
    pub population: usize,
    pub generations: usize,
    pub jumps: usize,
    pub crossovers: usize,
    pub jobs: usize,
    pub jump_nets: usize,
    pub seed: u64,
    pub descent: DescentOptions,
}

impl Default for EvolutionConfig {
    fn default() -> Self {
        Self {
            population: 4,
            generations: 3,
            jumps: 2,
            crossovers: 1,
            jobs: 2,
            jump_nets: 2,
            seed: 1,
            descent: DescentOptions::default(),
        }
    }
}

/// Route-specific I/O and operations intentionally stay behind this boundary.
/// These are the source functions `import_seed`, `descend`, `jump_near`,
/// `cross_probe`, and `dedupe_boards.fingerprint`, respectively.
pub trait EvolutionOperators<D>: Sync {
    fn import_seed(
        &self,
        seed: &Path,
        requested_k: i64,
        board: &Path,
        destination: &str,
    ) -> Result<Option<EvolutionWorld<D>>, String>;

    fn descend(
        &self,
        world: &EvolutionWorld<D>,
        options: DescentOptions,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<D>>, String>;

    /// Stable equivalent of `world_key(...)`; it covers the routed and
    /// fanout copper, plan/census sidecars, K, descent arguments and knobs.
    fn descent_key(
        &self,
        world: &EvolutionWorld<D>,
        options: &DescentOptions,
        env: &[(String, String)],
    ) -> Result<String, String>;

    fn jump_near(
        &self,
        world: &EvolutionWorld<D>,
        n_nets: usize,
        seed: u32,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<D>>, String>;

    fn cross_probe(
        &self,
        a: &EvolutionWorld<D>,
        b: &EvolutionWorld<D>,
        seed: u32,
        env: &[(String, String)],
    ) -> Result<Option<EvolutionWorld<D>>, String>;

    fn fingerprint(&self, world: &EvolutionWorld<D>) -> Result<String, String>;
}

#[derive(Debug, Clone)]
pub struct EvolutionResult<D> {
    pub best: EvolutionWorld<D>,
    pub population: Vec<EvolutionWorld<D>>,
    pub generations: Vec<Vec<EvolutionWorld<D>>>,
}

#[derive(Debug, Clone)]
enum Operation<D> {
    Descend(EvolutionWorld<D>),
    Jump(EvolutionWorld<D>, u32),
    Cross(EvolutionWorld<D>, EvolutionWorld<D>, u32),
    DescendFresh(EvolutionWorld<D>),
}

#[derive(Debug)]
enum OperationResult<D> {
    Descend(Result<Option<EvolutionWorld<D>>, String>),
    Jump(Result<Option<EvolutionWorld<D>>, String>),
    Cross(Result<Option<EvolutionWorld<D>>, String>),
}

/// Run the fixed reference's multi-world loop. Each seed is imported in order;
/// an invalid/unreadable seed is skipped, and an empty initial population is an
/// error. Candidate callbacks return `None` to signal no produced world.
pub fn evolve<D, O>(
    operators: &O,
    config: &EvolutionConfig,
    seeds: &[std::path::PathBuf],
    requested_k: i64,
    board: &Path,
    destination: &str,
    descend_env: &[(String, String)],
    jump_env: &[(String, String)],
) -> Result<EvolutionResult<D>, String>
where
    D: Clone + Send + Sync,
    O: EvolutionOperators<D>,
{
    if config.population == 0 || config.jobs == 0 {
        return Err("evolution population and jobs must be positive".into());
    }

    let mut population = Vec::new();
    for (index, seed) in seeds.iter().enumerate() {
        match operators.import_seed(seed, requested_k, board, destination) {
            Ok(Some(mut world)) => {
                world.name = format!("s{index}");
                world.origin = format!("seed {}", seed.display());
                population.push(world);
            }
            Ok(None) | Err(_) => {}
        }
    }
    if population.is_empty() {
        return Err("no seed world".into());
    }
    population.sort_by(world_cmp);
    let mut best = population[0].clone();
    let mut rng = PythonRandom::new(config.seed);
    let mut world_count = population.len();
    let mut history = Vec::with_capacity(config.generations);
    let mut closed_descents = std::collections::HashSet::new();

    for generation in 1..=config.generations {
        let parents = &population[..population.len().min(config.population)];
        let mut jobs = Vec::new();
        for parent in parents {
            jobs.push(Operation::Descend(parent.clone()));
        }
        for index in 0..config.jumps {
            let parent = if index < population.len() {
                population[index].clone()
            } else {
                population[rng.randrange(parents.len())].clone()
            };
            jobs.push(Operation::Jump(parent, rng.randint(1, 1_000_000)));
        }
        for _ in 0..config.crossovers {
            if parents.len() < 2 { break; }
            let (a, b) = rng.sample_two(parents.len());
            jobs.push(Operation::Cross(
                parents[a].clone(), parents[b].clone(), rng.randint(1, 1_000_000),
            ));
        }

        jobs.retain(|operation| match operation {
            Operation::Descend(world) => operators
                .descent_key(world, &config.descent, descend_env)
                .map(|key| !closed_descents.contains(&key))
                .unwrap_or(true),
            _ => true,
        });
        let mut new_worlds = Vec::new();
        for (operation, outcome) in execute_batch(operators, jobs, config.jobs, config, descend_env, jump_env) {
            match (operation, outcome) {
                (Operation::Descend(parent), OperationResult::Descend(Ok(Some(mut child)))) => {
                    if child.grade.better_than(&parent.grade) {
                        world_count += 1;
                        child.name = format!("g{generation}d{world_count}");
                        child.origin = format!("descend<{}>", parent.name);
                        new_worlds.push(child);
                    } else if let Ok(key) = operators.descent_key(&parent, &config.descent, descend_env) {
                        closed_descents.insert(key);
                    }
                }
                (Operation::Jump(parent, _), OperationResult::Jump(Ok(Some(mut child)))) => {
                    world_count += 1;
                    child.name = format!("g{generation}j{world_count}");
                    child.origin = format!("jump<{}>", parent.name);
                    new_worlds.push(child);
                }
                (Operation::Cross(a, b, _), OperationResult::Cross(Ok(Some(mut child)))) => {
                    world_count += 1;
                    child.name = format!("g{generation}x{world_count}");
                    child.origin = format!("cross<{} x {}>", a.name, b.name);
                    new_worlds.push(child);
                }
                _ => {}
            }
        }

        // A jump/crossover's landing grade is deliberately ignored for
        // selection: descend every new basin once before comparing it.
        let fresh: Vec<_> = new_worlds.iter()
            .filter(|world| !world.origin.starts_with("descend"))
            .cloned().collect();
        let descend_jobs: Vec<_> = fresh.into_iter().map(Operation::DescendFresh).collect();
        let descend_jobs: Vec<_> = descend_jobs.into_iter().filter(|operation| match operation {
            Operation::DescendFresh(world) => operators
                .descent_key(world, &config.descent, descend_env)
                .map(|key| !closed_descents.contains(&key))
                .unwrap_or(true),
            _ => true,
        }).collect();
        for (operation, outcome) in execute_batch(operators, descend_jobs, config.jobs, config, descend_env, jump_env) {
            if let (Operation::DescendFresh(parent), OperationResult::Descend(Ok(Some(mut child)))) = (operation, outcome) {
                if child.grade.better_than(&parent.grade) {
                    world_count += 1;
                    child.name = format!("g{generation}d{world_count}");
                    child.origin = format!("descend<{}: {}>", parent.name, parent.origin);
                    new_worlds.push(child);
                } else if let Ok(key) = operators.descent_key(&parent, &config.descent, descend_env) {
                    closed_descents.insert(key);
                }
            }
        }

        // Source selection is elitist and removes duplicate copper worlds.
        let mut candidates = population;
        candidates.extend(new_worlds);
        candidates.sort_by(world_cmp);
        let mut fingerprints = std::collections::HashSet::new();
        population = candidates.into_iter().filter(|world| {
            let fingerprint = operators.fingerprint(world).unwrap_or_else(|_| world.id.clone());
            fingerprints.insert(fingerprint)
        }).collect();
        if population.is_empty() { return Err("evolution lost all worlds".into()); }
        if world_cmp(&population[0], &best) == Ordering::Less {
            best = population[0].clone();
        }
        history.push(population[..population.len().min(config.population)].to_vec());
    }

    Ok(EvolutionResult { best, population, generations: history })
}

fn world_cmp<D>(a: &EvolutionWorld<D>, b: &EvolutionWorld<D>) -> Ordering {
    a.grade.population_key().cmp(&b.grade.population_key())
}

fn execute_batch<D, O>(
    operators: &O,
    jobs: Vec<Operation<D>>,
    concurrency: usize,
    config: &EvolutionConfig,
    descend_env: &[(String, String)],
    jump_env: &[(String, String)],
) -> Vec<(Operation<D>, OperationResult<D>)>
where
    D: Clone + Send + Sync,
    O: EvolutionOperators<D>,
{
    let mut results = Vec::with_capacity(jobs.len());
    for chunk in jobs.chunks(concurrency.max(1)) {
        thread::scope(|scope| {
            let handles: Vec<_> = chunk.iter().cloned().map(|job| {
                scope.spawn(move || {
                    let result = match &job {
                        Operation::Descend(world) | Operation::DescendFresh(world) => {
                            OperationResult::Descend(operators.descend(world, config.descent.clone(), descend_env))
                        }
                        Operation::Jump(world, seed) => OperationResult::Jump(
                            operators.jump_near(world, config.jump_nets, *seed, jump_env),
                        ),
                        Operation::Cross(a, b, seed) => OperationResult::Cross(
                            operators.cross_probe(a, b, *seed, jump_env),
                        ),
                    };
                    (job, result)
                })
            }).collect();
            for handle in handles {
                if let Ok(value) = handle.join() { results.push(value); }
            }
        });
    }
    results
}

/// Python's `random.Random` MT19937 subset used by evolve.py for seeded
/// parent choice and operator seeds. `getrandbits` uses Python's little-endian
/// 32-bit word order and `_randbelow` rejection sampling.
struct PythonRandom { mt: [u32; 624], index: usize }

impl PythonRandom {
    fn new(seed: u64) -> Self {
        let mut state = seed;
        let mut key = Vec::new();
        loop { key.push(state as u32); state >>= 32; if state == 0 { break; } }
        let mut rng = Self { mt: [0; 624], index: 624 };
        rng.seed_by_array(&key);
        rng
    }

    fn seed_by_array(&mut self, key: &[u32]) {
        self.seed(19_650_218);
        let mut i = 1usize;
        let mut j = 0usize;
        for _ in 0..624.max(key.len()) {
            self.mt[i] = (self.mt[i] ^ ((self.mt[i - 1] ^ (self.mt[i - 1] >> 30)).wrapping_mul(1_664_525)))
                .wrapping_add(key[j]).wrapping_add(j as u32);
            i += 1; j += 1;
            if i >= 624 { self.mt[0] = self.mt[623]; i = 1; }
            if j >= key.len() { j = 0; }
        }
        for _ in 0..623 {
            self.mt[i] = (self.mt[i] ^ ((self.mt[i - 1] ^ (self.mt[i - 1] >> 30)).wrapping_mul(1_566_083_941)))
                .wrapping_sub(i as u32);
            i += 1;
            if i >= 624 { self.mt[0] = self.mt[623]; i = 1; }
        }
        self.mt[0] = 0x8000_0000;
        self.index = 624;
    }

    fn seed(&mut self, seed: u32) {
        self.mt[0] = seed;
        for i in 1..624 {
            self.mt[i] = 1_812_433_253u32
                .wrapping_mul(self.mt[i - 1] ^ (self.mt[i - 1] >> 30))
                .wrapping_add(i as u32);
        }
        self.index = 624;
    }

    fn next_u32(&mut self) -> u32 {
        if self.index >= 624 {
            for i in 0..624 {
                let y = (self.mt[i] & 0x8000_0000) | (self.mt[(i + 1) % 624] & 0x7fff_ffff);
                self.mt[i] = self.mt[(i + 397) % 624] ^ (y >> 1) ^ if y & 1 != 0 { 0x9908_b0df } else { 0 };
            }
            self.index = 0;
        }
        let mut y = self.mt[self.index];
        self.index += 1;
        y ^= y >> 11;
        y ^= (y << 7) & 0x9d2c_5680;
        y ^= (y << 15) & 0xefc6_0000;
        y ^= y >> 18;
        y
    }

    fn getrandbits(&mut self, bits: u32) -> u32 {
        if bits == 0 { 0 } else if bits <= 32 { self.next_u32() >> (32 - bits) } else { self.next_u32() }
    }

    fn randbelow(&mut self, n: usize) -> usize {
        let bits = usize::BITS - n.leading_zeros();
        loop { let value = self.getrandbits(bits) as usize; if value < n { return value; } }
    }

    fn randrange(&mut self, n: usize) -> usize { self.randbelow(n) }
    fn randint(&mut self, low: u32, high: u32) -> u32 { low + self.randbelow((high - low + 1) as usize) as u32 }
    fn sample_two(&mut self, n: usize) -> (usize, usize) {
        let i = self.randbelow(n);
        let j = self.randbelow(n - 1);
        let j = if j == i { n - 1 } else { j };
        (i, j)
    }
}
