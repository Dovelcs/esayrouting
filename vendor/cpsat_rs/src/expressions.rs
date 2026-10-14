//! Linear expressions and bounded linear expressions for constraints.

use crate::vars::{BoolVar, IntVar};
use std::ops::{Add, Mul, Neg, Sub};

/// A linear expression: sum(coeff_i * var_i) + constant.
#[derive(Debug, Clone, Default)]
#[must_use]
pub struct LinearExpr {
    pub(crate) terms: Vec<(IntVar, i64)>,
    pub(crate) constant: i64,
}

impl LinearExpr {
    /// Create an expression equal to a constant.
    pub fn constant(val: i64) -> Self {
        Self {
            terms: vec![],
            constant: val,
        }
    }

    /// Create a sum of variables with equal coefficients of 1.
    pub fn sum(vars: &[IntVar]) -> Self {
        Self {
            terms: vars.iter().map(|&v| (v, 1)).collect(),
            constant: 0,
        }
    }

    /// Create a weighted sum of variables.
    ///
    /// # Panics
    ///
    /// Panics if `vars` and `coeffs` have different lengths.
    pub fn weighted_sum(vars: &[IntVar], coeffs: &[i64]) -> Self {
        assert_eq!(
            vars.len(),
            coeffs.len(),
            "weighted_sum: vars ({}) and coeffs ({}) must have the same length",
            vars.len(),
            coeffs.len()
        );
        Self {
            terms: vars.iter().zip(coeffs.iter()).map(|(&v, &c)| (v, c)).collect(),
            constant: 0,
        }
    }

    /// Add a term to this expression.
    pub fn add_term(&mut self, var: IntVar, coeff: i64) {
        self.terms.push((var, coeff));
    }

    /// Constrain this expression to be <= upper_bound.
    pub fn le(self, ub: i64) -> BoundedLinearExpr {
        BoundedLinearExpr {
            expr: self,
            lb: i64::MIN,
            ub,
        }
    }

    /// Constrain this expression to be >= lower_bound.
    pub fn ge(self, lb: i64) -> BoundedLinearExpr {
        BoundedLinearExpr {
            expr: self,
            lb,
            ub: i64::MAX,
        }
    }

    /// Constrain this expression to be == value.
    pub fn eq(self, val: i64) -> BoundedLinearExpr {
        BoundedLinearExpr {
            expr: self,
            lb: val,
            ub: val,
        }
    }

    /// Constrain this expression to be in [lb, ub].
    ///
    /// # Panics
    ///
    /// Panics if `lb > ub`.
    pub fn between(self, lb: i64, ub: i64) -> BoundedLinearExpr {
        assert!(lb <= ub, "between: lb ({lb}) must be <= ub ({ub})");
        BoundedLinearExpr {
            expr: self,
            lb,
            ub,
        }
    }

    /// Convert to the proto representation.
    pub(crate) fn to_proto(&self) -> crate::proto::LinearExpressionProto {
        crate::proto::LinearExpressionProto {
            vars: self.terms.iter().map(|(v, _)| v.0).collect(),
            coeffs: self.terms.iter().map(|(_, c)| *c).collect(),
            offset: self.constant,
        }
    }
}

// --- From conversions ---

impl From<IntVar> for LinearExpr {
    fn from(v: IntVar) -> Self {
        Self {
            terms: vec![(v, 1)],
            constant: 0,
        }
    }
}

impl From<BoolVar> for LinearExpr {
    fn from(b: BoolVar) -> Self {
        Self::from(b.as_int_var())
    }
}

impl From<i64> for LinearExpr {
    fn from(val: i64) -> Self {
        Self::constant(val)
    }
}

impl From<i32> for LinearExpr {
    fn from(val: i32) -> Self {
        Self::constant(i64::from(val))
    }
}

// --- Arithmetic operators ---

impl Add for LinearExpr {
    type Output = LinearExpr;
    fn add(mut self, rhs: LinearExpr) -> LinearExpr {
        self.terms.extend(rhs.terms);
        self.constant = self.constant.checked_add(rhs.constant)
            .expect("LinearExpr constant overflow on addition");
        self
    }
}

impl Add<IntVar> for LinearExpr {
    type Output = LinearExpr;
    fn add(mut self, rhs: IntVar) -> LinearExpr {
        self.terms.push((rhs, 1));
        self
    }
}

impl Add<i64> for LinearExpr {
    type Output = LinearExpr;
    fn add(mut self, rhs: i64) -> LinearExpr {
        self.constant = self.constant.checked_add(rhs)
            .expect("LinearExpr constant overflow on addition");
        self
    }
}

impl Sub for LinearExpr {
    type Output = LinearExpr;
    fn sub(mut self, rhs: LinearExpr) -> LinearExpr {
        for (v, c) in rhs.terms {
            self.terms.push((v, c.checked_neg().expect("LinearExpr coefficient overflow on negation")));
        }
        self.constant = self.constant.checked_sub(rhs.constant)
            .expect("LinearExpr constant overflow on subtraction");
        self
    }
}

impl Sub<IntVar> for LinearExpr {
    type Output = LinearExpr;
    fn sub(mut self, rhs: IntVar) -> LinearExpr {
        self.terms.push((rhs, -1));
        self
    }
}

impl Sub<i64> for LinearExpr {
    type Output = LinearExpr;
    fn sub(mut self, rhs: i64) -> LinearExpr {
        self.constant = self.constant.checked_sub(rhs)
            .expect("LinearExpr constant overflow on subtraction");
        self
    }
}

impl Neg for LinearExpr {
    type Output = LinearExpr;
    fn neg(mut self) -> LinearExpr {
        for term in &mut self.terms {
            term.1 = term.1.checked_neg().expect("LinearExpr coefficient overflow on negation");
        }
        self.constant = self.constant.checked_neg()
            .expect("LinearExpr constant overflow on negation");
        self
    }
}

impl Mul<i64> for LinearExpr {
    type Output = LinearExpr;
    fn mul(mut self, rhs: i64) -> LinearExpr {
        for term in &mut self.terms {
            term.1 = term.1.checked_mul(rhs).expect("LinearExpr coefficient overflow on multiplication");
        }
        self.constant = self.constant.checked_mul(rhs)
            .expect("LinearExpr constant overflow on multiplication");
        self
    }
}

// coeff * IntVar → LinearExpr
impl Mul<IntVar> for i64 {
    type Output = LinearExpr;
    fn mul(self, rhs: IntVar) -> LinearExpr {
        LinearExpr {
            terms: vec![(rhs, self)],
            constant: 0,
        }
    }
}

// IntVar + IntVar → LinearExpr
impl Add for IntVar {
    type Output = LinearExpr;
    fn add(self, rhs: IntVar) -> LinearExpr {
        LinearExpr {
            terms: vec![(self, 1), (rhs, 1)],
            constant: 0,
        }
    }
}

// IntVar + i64 → LinearExpr
impl Add<i64> for IntVar {
    type Output = LinearExpr;
    fn add(self, rhs: i64) -> LinearExpr {
        LinearExpr {
            terms: vec![(self, 1)],
            constant: rhs,
        }
    }
}

// IntVar - IntVar → LinearExpr
impl Sub for IntVar {
    type Output = LinearExpr;
    fn sub(self, rhs: IntVar) -> LinearExpr {
        LinearExpr {
            terms: vec![(self, 1), (rhs, -1)],
            constant: 0,
        }
    }
}

// IntVar * i64 → LinearExpr
impl Mul<i64> for IntVar {
    type Output = LinearExpr;
    fn mul(self, rhs: i64) -> LinearExpr {
        LinearExpr {
            terms: vec![(self, rhs)],
            constant: 0,
        }
    }
}

/// A bounded linear expression: lb <= expr <= ub.
/// Created by calling `.le()`, `.ge()`, `.eq()`, or `.between()` on a `LinearExpr`.
#[must_use]
pub struct BoundedLinearExpr {
    pub(crate) expr: LinearExpr,
    pub(crate) lb: i64,
    pub(crate) ub: i64,
}

// Convenience: IntVar <= i64 → BoundedLinearExpr
// We can't impl PartialOrd for foreign types, so use free functions.

/// Create a bounded expression: var <= ub.
pub fn leq(var: impl Into<LinearExpr>, ub: i64) -> BoundedLinearExpr {
    var.into().le(ub)
}

/// Create a bounded expression: var >= lb.
pub fn geq(var: impl Into<LinearExpr>, lb: i64) -> BoundedLinearExpr {
    var.into().ge(lb)
}

/// Create a bounded expression: expr == val.
pub fn eq(var: impl Into<LinearExpr>, val: i64) -> BoundedLinearExpr {
    var.into().eq(val)
}
