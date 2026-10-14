//! Error types for the cpsat-rs crate.

/// Errors that can occur during solving.
#[derive(Debug, thiserror::Error)]
pub enum SolveError {
    /// FFI call returned a non-zero error code.
    #[error("FFI call returned error code {0}")]
    FfiError(i32),

    /// Failed to decode the solver response protobuf.
    #[error("Failed to decode solver response: {0}")]
    DecodeError(#[from] prost::DecodeError),

    /// Serialized model exceeds the 2 GB limit for i32 length fields.
    #[error("Serialized model too large ({0} bytes, max 2 GB)")]
    ModelTooLarge(usize),
}
