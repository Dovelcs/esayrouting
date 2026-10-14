//! Variable handle types.
//!
//! These are `Copy` index newtypes, not borrows. They can be freely
//! stored in `Vec`, `HashMap`, passed to functions, etc. Using a handle
//! from one model in another model is a logic error (debug assertions
//! may catch this in the future).

/// Handle to an integer variable in a `CpModel`.
/// Inner value is the index into `CpModelProto.variables`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntVar(pub(crate) i32);

impl IntVar {
    /// Get the raw variable index.
    #[must_use]
    pub fn index(self) -> i32 {
        self.0
    }
}

/// Handle to a Boolean variable in a `CpModel`.
/// A `BoolVar` is an `IntVar` with domain `[0, 1]`.
///
/// # Negation
///
/// Use `.negated()` or `!b` to get the negated literal for use in
/// Boolean constraints (enforcement literals, `add_bool_or`, etc.).
///
/// **Warning:** Negated `BoolVar` handles should NOT be converted to
/// `IntVar` or used in `LinearExpr`. In CP-SAT, a negated literal
/// index means boolean complement in constraint contexts but arithmetic
/// negation in variable contexts. Use `1 - b.as_int_var()` for
/// the arithmetic complement in linear expressions.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct BoolVar(pub(crate) i32);

impl BoolVar {
    /// Get the negated literal for Boolean constraints.
    ///
    /// This is valid for enforcement literals, `add_bool_or`, `add_bool_and`,
    /// `add_exactly_one`, `add_at_most_one`, and `add_implication`.
    ///
    /// Do NOT use the result in `LinearExpr` or `value()`. For arithmetic
    /// complement, use `1 - b.as_int_var()` instead.
    #[must_use]
    pub fn negated(self) -> Self {
        Self(!self.0)
    }

    /// Whether this is a negated literal.
    #[must_use]
    pub fn is_negated(self) -> bool {
        self.0 < 0
    }

    /// Convert to the underlying integer variable.
    ///
    /// # Panics
    ///
    /// Panics if this is a negated literal. Use the non-negated variable
    /// for arithmetic expressions.
    #[must_use]
    pub fn as_int_var(self) -> IntVar {
        assert!(
            self.0 >= 0,
            "Cannot convert negated BoolVar to IntVar. \
             Use the non-negated variable for linear expressions."
        );
        IntVar(self.0)
    }

    /// Get the raw literal index (may be negative for negated literals).
    /// Only valid for Boolean constraint contexts.
    #[must_use]
    pub fn index(self) -> i32 {
        self.0
    }
}

impl std::ops::Not for BoolVar {
    type Output = Self;
    fn not(self) -> Self {
        self.negated()
    }
}

impl From<BoolVar> for IntVar {
    /// Convert a `BoolVar` to `IntVar`.
    ///
    /// # Panics
    ///
    /// Panics if the `BoolVar` is negated.
    fn from(b: BoolVar) -> Self {
        b.as_int_var()
    }
}

/// Handle to an interval variable in a `CpModel`.
/// Inner value is the index into `CpModelProto.constraints`
/// (the constraint that defines the interval).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct IntervalVar(pub(crate) i32);
