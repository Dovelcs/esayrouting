//! # cpsat-rs — Rust bindings for Google OR-Tools CP-SAT solver.
//!
//! Build constraint programming models in safe Rust, solve with the full
//! power of OR-Tools CP-SAT (CDCL, LP relaxation, LNS, FeasibilityJump).
//!
//! ## Architecture
//!
//! All model construction happens in pure safe Rust. The only FFI call
//! is a single `extern "C"` function that passes serialized protobuf
//! bytes to OR-Tools and receives response bytes back. Zero C++ object
//! lifetime management.
//!
//! ## Quick Start
//!
//! ```no_run
//! use cpsat_rs::prelude::*;
//!
//! let mut model = CpModel::new();
//! let x = model.new_int_var(0..=10, "x");
//! let y = model.new_int_var(0..=10, "y");
//! model.add((x + y).le(15));
//! model.minimize(x + y);
//!
//! let response = CpSolver::solve(&model).unwrap();
//! if response.is_optimal() {
//!     println!("x={}, y={}", response.value(x), response.value(y));
//! }
//! ```
//!
//! ## Requirements
//!
//! Requires Google OR-Tools C++ library installed:
//! ```sh
//! brew install or-tools  # macOS
//! ```

#![deny(unsafe_op_in_unsafe_fn)]
#![warn(clippy::pedantic)]
#![allow(clippy::must_use_candidate)]
#![allow(clippy::return_self_not_must_use)]
#![allow(clippy::module_name_repetitions)]
#![allow(clippy::cast_possible_truncation)]
#![allow(clippy::cast_sign_loss)]
#![allow(clippy::cast_possible_wrap)]
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::doc_markdown)]
#![allow(clippy::similar_names)]

pub mod error;
pub mod expressions;
mod ffi;
pub mod model;
pub mod params;
pub mod solver;
pub mod vars;

#[allow(missing_docs, clippy::pedantic, clippy::all)]
pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/operations_research.sat.rs"));
}

/// Prelude for convenient imports.
pub mod prelude {
    pub use crate::error::SolveError;
    pub use crate::expressions::{BoundedLinearExpr, LinearExpr};
    pub use crate::model::CpModel;
    pub use crate::proto::{CpSolverStatus, SatParameters};
    pub use crate::solver::{CpSolver, SolveResponse};
    pub use crate::vars::{BoolVar, IntVar, IntervalVar};
}
