//! The single unsafe FFI bridge to OR-Tools CP-SAT.
//!
//! Uses the official C API from `ortools/sat/c_api/cp_solver_c.h`.
//! This module contains the ONLY unsafe code in the crate.

use crate::error::SolveError;
use std::ptr;

extern "C" {
    fn SolveCpModelWithParameters(
        creq: *const u8,
        creq_len: i32,
        cparams: *const u8,
        cparams_len: i32,
        cres: *mut *mut u8,
        cres_len: *mut i32,
    );
}

// OR-Tools C API allocates response with malloc (confirmed in cp_solver_c.cc).
// We use libc::free to match. On Windows with multiple CRTs this could mismatch,
// but OR-Tools' own C API examples use free() for deallocation.
extern "C" {
    #[cfg(not(target_os = "windows"))]
    fn free(ptr: *mut std::ffi::c_void);
}

#[cfg(target_os = "windows")]
extern "C" {
    // On Windows, use _aligned_free or the CRT-matched free.
    // For now we use the same free — OR-Tools links the same CRT.
    fn free(ptr: *mut std::ffi::c_void);
}

/// Safely convert a `usize` to `i32`, returning an error if it overflows.
fn usize_to_i32(val: usize) -> Result<i32, SolveError> {
    i32::try_from(val).map_err(|_| SolveError::ModelTooLarge(val))
}

/// Serialize a model and parameters, call OR-Tools, return response bytes.
///
/// # Errors
///
/// Returns `SolveError::FfiError` if the C API returns a null response buffer.
/// Returns `SolveError::ModelTooLarge` if the serialized model exceeds 2 GB.
pub(crate) fn solve_raw(
    model_bytes: &[u8],
    params_bytes: Option<&[u8]>,
) -> Result<Vec<u8>, SolveError> {
    let model_len = usize_to_i32(model_bytes.len())?;

    let (params_ptr, params_len) = match params_bytes {
        Some(p) => (p.as_ptr(), usize_to_i32(p.len())?),
        None => (ptr::null(), 0),
    };

    let mut response_buf: *mut u8 = ptr::null_mut();
    let mut response_len: i32 = 0;

    // SAFETY: All pointers are valid for the duration of the call.
    // The C API allocates the response buffer which we must free.
    unsafe {
        SolveCpModelWithParameters(
            model_bytes.as_ptr(),
            model_len,
            params_ptr,
            params_len,
            ptr::addr_of_mut!(response_buf),
            ptr::addr_of_mut!(response_len),
        );
    }

    if response_buf.is_null() {
        return Err(SolveError::FfiError(-1));
    }

    // SAFETY: OR-Tools allocated response_buf with malloc, length is response_len.
    // We copy into a Vec immediately and free the buffer.
    // A response_len of 0 is valid (empty protobuf message).
    let response = unsafe {
        let len = if response_len >= 0 { response_len as usize } else { 0 };
        let owned = if len > 0 {
            let slice = std::slice::from_raw_parts(response_buf, len);
            slice.to_vec()
        } else {
            Vec::new()
        };
        free(response_buf.cast::<std::ffi::c_void>());
        owned
    };

    Ok(response)
}
