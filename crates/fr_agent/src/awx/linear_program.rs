//! Sparse continuous LP assembled in Rust, solved by bundled native HiGHS, matching SciPy linprog(method="highs").
use serde::{Deserialize, Serialize};
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Variable {
    pub lower: f64,
    pub upper: f64,
    pub cost: f64,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Row {
    pub terms: Vec<(usize, f64)>,
    pub lower: f64,
    pub upper: f64,
}
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct LinearProgram {
    pub variables: Vec<Variable>,
    pub rows: Vec<Row>,
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct LpSolution {
    pub values: Vec<f64>,
    pub objective: f64,
    pub proved: bool,
    pub row_duals: Vec<f64>,
}
extern "C" {
    fn awx_highs_solve(
        n: i32,
        m: i32,
        lower: *const f64,
        upper: *const f64,
        cost: *const f64,
        starts: *const i32,
        columns: *const i32,
        values: *const f64,
        row_lower: *const f64,
        row_upper: *const f64,
        milliseconds: i64,
        solution: *mut f64,
        row_dual: *mut f64,
        objective: *mut f64,
        mode: i32,
    ) -> i32;
}
impl LinearProgram {
    pub fn variable(&mut self, lower: f64, upper: f64, cost: f64) -> usize {
        let i = self.variables.len();
        self.variables.push(Variable { lower, upper, cost });
        i
    }
    pub fn le(&mut self, terms: Vec<(usize, f64)>, upper: f64) -> usize {
        let i = self.rows.len();
        self.rows.push(Row {
            terms,
            lower: f64::NEG_INFINITY,
            upper,
        });
        i
    }
    pub fn equal(&mut self, terms: Vec<(usize, f64)>, value: f64) -> usize {
        let i = self.rows.len();
        self.rows.push(Row {
            terms,
            lower: value,
            upper: value,
        });
        i
    }
    pub fn elastic_le(
        &mut self,
        mut terms: Vec<(usize, f64)>,
        upper: f64,
        cost: f64,
    ) -> (usize, usize) {
        let slack = self.variable(0., f64::INFINITY, cost);
        terms.push((slack, -1.));
        let row = self.le(terms, upper);
        (row, slack)
    }
    pub fn solve(&self, milliseconds: u64) -> Result<LpSolution, String> {
        self.solve_mode(milliseconds, 0)
    }
    fn solve_mode(&self, milliseconds: u64, mode: i32) -> Result<LpSolution, String> {
        if self.variables.is_empty() {
            return Err("AWX LP has no variables".into());
        }
        if milliseconds == 0 {
            return Err("AWX LP budget exhausted".into());
        }
        for v in &self.variables {
            if v.lower.is_nan() || v.upper.is_nan() || v.lower > v.upper || !v.cost.is_finite() {
                return Err("invalid AWX LP variable".into());
            }
        }
        let lower: Vec<_> = self.variables.iter().map(|v| v.lower).collect();
        let upper: Vec<_> = self.variables.iter().map(|v| v.upper).collect();
        let cost: Vec<_> = self.variables.iter().map(|v| v.cost).collect();
        let mut starts = vec![0i32];
        let mut columns = Vec::new();
        let mut coefficients = Vec::new();
        let mut row_lower = Vec::new();
        let mut row_upper = Vec::new();
        for row in &self.rows {
            if row.lower.is_nan() || row.upper.is_nan() || row.lower > row.upper {
                return Err("invalid AWX LP row".into());
            }
            let mut combined = std::collections::BTreeMap::<usize, f64>::new();
            for &(j, a) in &row.terms {
                if j >= self.variables.len() || !a.is_finite() {
                    return Err("invalid AWX LP coefficient".into());
                }
                *combined.entry(j).or_default() += a;
            }
            for (j, a) in combined {
                columns.push(i32::try_from(j).map_err(|_| "LP variable overflow")?);
                coefficients.push(a);
            }
            starts.push(i32::try_from(columns.len()).map_err(|_| "LP matrix overflow")?);
            row_lower.push(row.lower);
            row_upper.push(row.upper);
        }
        let mut values = vec![0.; self.variables.len()];
        let mut row_duals = vec![0.; self.rows.len()];
        let mut objective = 0.;
        let status = unsafe {
            awx_highs_solve(
                i32::try_from(values.len()).map_err(|_| "LP size overflow")?,
                i32::try_from(self.rows.len()).map_err(|_| "LP row overflow")?,
                lower.as_ptr(),
                upper.as_ptr(),
                cost.as_ptr(),
                starts.as_ptr(),
                columns.as_ptr(),
                coefficients.as_ptr(),
                row_lower.as_ptr(),
                row_upper.as_ptr(),
                i64::try_from(milliseconds).map_err(|_| "LP time overflow")?,
                values.as_mut_ptr(),
                row_duals.as_mut_ptr(),
                &mut objective,
                mode,
            )
        };
        match status {
            0 | 1 => Ok(LpSolution {
                values,
                objective,
                proved: status == 0,
                row_duals,
            }),
            _ => Err(format!("AWX continuous geometry LP status {status}")),
        }
    }
}

impl LinearProgram {
    /// AWX whole_geo.lp_by_dual: IPM (500 iterations), then dual simplex, primal offsets from row marginals.
    pub fn solve_by_dual(&self, budget_ms: u64) -> Result<LpSolution, String> {
        let start = std::time::Instant::now();
        let mut upper_rows = Vec::<(Vec<(usize, f64)>, f64)>::new();
        for row in &self.rows {
            if row.upper.is_finite() {
                upper_rows.push((row.terms.clone(), row.upper));
            }
            if row.lower.is_finite() {
                upper_rows.push((
                    row.terms.iter().map(|&(j, a)| (j, -a)).collect(),
                    -row.lower,
                ));
            }
        }
        let mut dual = LinearProgram::default();
        for (_, rhs) in &upper_rows {
            dual.variable(0., f64::INFINITY, *rhs);
        }
        let mut terms = vec![Vec::new(); self.variables.len()];
        for (i, (row, _)) in upper_rows.iter().enumerate() {
            for &(j, a) in row {
                terms[j].push((i, -a));
            }
        }
        for (j, v) in self.variables.iter().enumerate() {
            let plain = v.lower == 0. && !v.upper.is_finite();
            if plain {
                dual.le(terms[j].clone(), v.cost);
            } else {
                if v.lower.is_finite() {
                    let k = dual.variable(0., f64::INFINITY, -v.lower);
                    terms[j].push((k, 1.));
                }
                if v.upper.is_finite() {
                    let k = dual.variable(0., f64::INFINITY, v.upper);
                    terms[j].push((k, -1.));
                }
                dual.equal(terms[j].clone(), v.cost);
            }
        }
        let first = dual.solve_mode(budget_ms, 1);
        let solved = match first {
            Ok(s) if s.proved => s,
            _ => {
                let left = budget_ms.saturating_sub(start.elapsed().as_millis() as u64);
                let s = dual.solve_mode(left, 2)?;
                if !s.proved {
                    return Err("AWX dual geometry LP did not prove optimum".into());
                }
                s
            }
        };
        let quantum = 2_f64.powi(-24);
        let values: Vec<_> = solved
            .row_duals
            .iter()
            .map(|&v| (-v / quantum).round_ties_even() * quantum)
            .collect();
        if values.len() != self.variables.len() {
            return Err("AWX dual multiplier cardinality mismatch".into());
        }
        let objective = self
            .variables
            .iter()
            .zip(&values)
            .map(|(v, &x)| v.cost * x)
            .sum();
        Ok(LpSolution {
            values,
            objective,
            proved: true,
            row_duals: Vec::new(),
        })
    }
}
