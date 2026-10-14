//! Complete CP-SAT protobuf constraint surface used by the fixed AWX model.
use cpsat_rs::{
    prelude::*,
    proto::{
        constraint_proto, BoolArgumentProto, ConstraintProto, IntervalConstraintProto,
        LinearConstraintProto, LinearExpressionProto,
    },
};
pub use cpsat_rs::{
    prelude::{BoolVar, IntVar, IntervalVar, LinearExpr},
    proto::{CpSolverResponse, CpSolverStatus, SatParameters},
};
#[derive(Clone, Debug)]
pub struct Affine {
    pub terms: Vec<(IntVar, i64)>,
    pub offset: i64,
}
impl Affine {
    pub fn constant(offset: i64) -> Self {
        Self {
            terms: vec![],
            offset,
        }
    }
    pub fn proto(&self) -> LinearExpressionProto {
        LinearExpressionProto {
            vars: self.terms.iter().map(|(v, _)| v.index()).collect(),
            coeffs: self.terms.iter().map(|(_, c)| *c).collect(),
            offset: self.offset,
        }
    }
}
pub struct Model {
    pub inner: CpModel,
    constants: std::collections::BTreeMap<i64, IntVar>,
}
impl Default for Model {
    fn default() -> Self {
        Self::new()
    }
}
impl Model {
    pub fn new() -> Self {
        Self {
            inner: CpModel::new(),
            constants: std::collections::BTreeMap::new(),
        }
    }
    pub fn int(&mut self, lo: i64, hi: i64, name: &str) -> Result<IntVar, String> {
        if lo > hi {
            return Err("AWX empty integer domain".into());
        }
        if self.inner.to_proto().variables.len() >= i32::MAX as usize {
            return Err("AWX variable index overflow".into());
        }
        Ok(self.inner.new_int_var(lo..=hi, name))
    }
    pub fn bool(&mut self, name: &str) -> BoolVar {
        self.inner.new_bool_var(name)
    }
    pub fn constant(&mut self, value: i64) -> Result<IntVar, String> {
        if let Some(v) = self.constants.get(&value) {
            return Ok(*v);
        }
        let v = self.int(value, value, "")?;
        self.constants.insert(value, v);
        Ok(v)
    }
    pub fn linear(
        &mut self,
        terms: &[(IntVar, i64)],
        lo: i64,
        hi: i64,
        enforced: &[BoolVar],
    ) -> Result<(), String> {
        if lo > hi {
            return Err("AWX empty constraint domain".into());
        }
        self.inner
            .raw_proto_mut()
            .constraints
            .push(ConstraintProto {
                name: String::new(),
                enforcement_literal: enforced.iter().map(|x| x.index()).collect(),
                constraint: Some(constraint_proto::Constraint::Linear(
                    LinearConstraintProto {
                        vars: terms.iter().map(|(v, _)| v.index()).collect(),
                        coeffs: terms.iter().map(|(_, c)| *c).collect(),
                        domain: vec![lo, hi],
                    },
                )),
            });
        Ok(())
    }
    pub fn eq(
        &mut self,
        terms: &[(IntVar, i64)],
        rhs: i64,
        enforced: &[BoolVar],
    ) -> Result<(), String> {
        self.linear(terms, rhs, rhs, enforced)
    }
    pub fn le(
        &mut self,
        terms: &[(IntVar, i64)],
        rhs: i64,
        enforced: &[BoolVar],
    ) -> Result<(), String> {
        self.linear(terms, i64::MIN, rhs, enforced)
    }
    pub fn ge(
        &mut self,
        terms: &[(IntVar, i64)],
        rhs: i64,
        enforced: &[BoolVar],
    ) -> Result<(), String> {
        self.linear(terms, rhs, i64::MAX, enforced)
    }
    pub fn ne(
        &mut self,
        terms: &[(IntVar, i64)],
        rhs: i64,
        enforced: &[BoolVar],
    ) -> Result<(), String> {
        let lower = rhs.checked_sub(1).ok_or("AWX exclusion bound overflow")?;
        let upper = rhs.checked_add(1).ok_or("AWX exclusion bound overflow")?;
        self.inner
            .raw_proto_mut()
            .constraints
            .push(ConstraintProto {
                name: String::new(),
                enforcement_literal: enforced.iter().map(|v| v.index()).collect(),
                constraint: Some(constraint_proto::Constraint::Linear(
                    LinearConstraintProto {
                        vars: terms.iter().map(|(v, _)| v.index()).collect(),
                        coeffs: terms.iter().map(|(_, c)| *c).collect(),
                        domain: vec![i64::MIN, lower, upper, i64::MAX],
                    },
                )),
            });
        Ok(())
    }
    pub fn or(&mut self, lits: &[BoolVar], enforced: &[BoolVar]) {
        self.boolean(lits, enforced, false, false)
    }
    pub fn and(&mut self, lits: &[BoolVar], enforced: &[BoolVar]) {
        self.boolean(lits, enforced, true, false)
    }
    pub fn xor(&mut self, lits: &[BoolVar]) {
        self.boolean(lits, &[], false, true)
    }
    pub fn xor_with_one(&mut self, lits: &[BoolVar], one: bool) -> Result<(), String> {
        let mut indices: Vec<_> = lits.iter().map(|v| v.index()).collect();
        if one {
            indices.push(self.constant(1)?.index());
        }
        self.inner
            .raw_proto_mut()
            .constraints
            .push(ConstraintProto {
                name: String::new(),
                enforcement_literal: vec![],
                constraint: Some(constraint_proto::Constraint::BoolXor(BoolArgumentProto {
                    literals: indices,
                })),
            });
        Ok(())
    }
    fn boolean(&mut self, lits: &[BoolVar], enforced: &[BoolVar], and: bool, xor: bool) {
        let args = BoolArgumentProto {
            literals: lits.iter().map(|v| v.index()).collect(),
        };
        let kind = if xor {
            constraint_proto::Constraint::BoolXor(args)
        } else if and {
            constraint_proto::Constraint::BoolAnd(args)
        } else {
            constraint_proto::Constraint::BoolOr(args)
        };
        self.inner
            .raw_proto_mut()
            .constraints
            .push(ConstraintProto {
                name: String::new(),
                enforcement_literal: enforced.iter().map(|v| v.index()).collect(),
                constraint: Some(kind),
            });
    }
    pub fn implication(&mut self, a: BoolVar, b: BoolVar) {
        self.or(&[!a, b], &[])
    }
    pub fn optional_interval(
        &mut self,
        start: IntVar,
        width: i64,
        present: BoolVar,
    ) -> Result<i32, String> {
        if width <= 0 {
            return Err("AWX interval width must be positive".into());
        }
        let proto = self.inner.raw_proto_mut();
        let index =
            i32::try_from(proto.constraints.len()).map_err(|_| "AWX interval index overflow")?;
        proto.constraints.push(ConstraintProto {
            name: String::new(),
            enforcement_literal: vec![present.index()],
            constraint: Some(constraint_proto::Constraint::Interval(
                IntervalConstraintProto {
                    start: Some(LinearExpressionProto {
                        vars: vec![start.index()],
                        coeffs: vec![1],
                        offset: 0,
                    }),
                    size: Some(LinearExpressionProto {
                        vars: vec![],
                        coeffs: vec![],
                        offset: width,
                    }),
                    end: Some(LinearExpressionProto {
                        vars: vec![start.index()],
                        coeffs: vec![1],
                        offset: width,
                    }),
                },
            )),
        });
        // IntervalVar's constructor is private; the native builder offers handles only.
        // Keep direct-proto interval indexes in this AWX wrapper instead.
        Ok(index)
    }
    pub fn no_overlap(&mut self, intervals: &[i32]) {
        if intervals.len() > 1 {
            self.inner
                .raw_proto_mut()
                .constraints
                .push(ConstraintProto {
                    name: String::new(),
                    enforcement_literal: vec![],
                    constraint: Some(constraint_proto::Constraint::NoOverlap(
                        cpsat_rs::proto::NoOverlapConstraintProto {
                            intervals: intervals.to_vec(),
                        },
                    )),
                });
        }
    }
    pub fn cumulative(
        &mut self,
        intervals: &[i32],
        demands: &[Affine],
        capacity: i64,
    ) -> Result<(), String> {
        if intervals.len() != demands.len() || capacity < 0 {
            return Err("AWX invalid cumulative constraint".into());
        }
        if intervals.len() > 1 {
            let expressions = demands.iter().map(Affine::proto).collect();
            self.inner
                .raw_proto_mut()
                .constraints
                .push(ConstraintProto {
                    name: String::new(),
                    enforcement_literal: vec![],
                    constraint: Some(constraint_proto::Constraint::Cumulative(
                        cpsat_rs::proto::CumulativeConstraintProto {
                            capacity: Some(LinearExpressionProto {
                                vars: vec![],
                                coeffs: vec![],
                                offset: capacity,
                            }),
                            intervals: intervals.to_vec(),
                            demands: expressions,
                        },
                    )),
                });
        }
        Ok(())
    }
    pub fn element(
        &mut self,
        index: IntVar,
        array: &[IntVar],
        target: IntVar,
    ) -> Result<(), String> {
        if array.is_empty() {
            return Err("AWX empty element array".into());
        }
        self.inner.add_element(index, array, target);
        Ok(())
    }
    pub fn max(&mut self, target: IntVar, variables: &[IntVar]) -> Result<(), String> {
        if variables.is_empty() {
            return Err("AWX empty maximum".into());
        }
        self.inner.add_max_equality(target, variables);
        Ok(())
    }
    pub fn hint(&mut self, var: IntVar, value: i64) {
        self.inner.add_hint(var, value)
    }
    pub fn clear_hints(&mut self) {
        self.inner.raw_proto_mut().solution_hint = None
    }
    pub fn assumptions(&mut self, lits: &[BoolVar]) {
        self.inner.raw_proto_mut().assumptions = lits.iter().map(|v| v.index()).collect()
    }
    pub fn clear_assumptions(&mut self) {
        self.inner.raw_proto_mut().assumptions.clear()
    }
    pub fn minimize(&mut self, terms: &[(IntVar, i64)]) {
        let mut e = LinearExpr::constant(0);
        for (v, c) in terms {
            e = e + LinearExpr::from(*v) * *c;
        }
        self.inner.minimize(e)
    }
    pub fn clear_objective(&mut self) {
        self.inner.raw_proto_mut().objective = None;
        self.inner.raw_proto_mut().floating_point_objective = None;
    }
}
