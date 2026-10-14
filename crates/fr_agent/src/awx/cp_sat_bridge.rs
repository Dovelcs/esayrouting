//! Scoped official OR-Tools solve/log/cancellation boundary. No routing model lives in C++.
use cpsat_rs::{
    prelude::CpModel,
    proto::{CpSolverResponse, SatParameters},
};
use prost::Message;
use std::{
    ffi::c_void,
    panic::{catch_unwind, AssertUnwindSafe},
};
type Callback = unsafe extern "C" fn(*const u8, usize, *mut c_void) -> bool;
unsafe extern "C" {
    fn awx_solve_cp_sat(
        request: *const u8,
        request_len: usize,
        parameters: *const u8,
        parameters_len: usize,
        callback: Option<Callback>,
        context: *mut c_void,
        response: *mut *mut u8,
        response_len: *mut usize,
    ) -> i32;
    fn awx_cp_sat_free(response: *mut c_void);
}
struct CallbackContext<'a> {
    callback: &'a mut (dyn FnMut(&str) -> bool + Send),
    panicked: bool,
}
unsafe extern "C" fn on_log(pointer: *const u8, length: usize, opaque: *mut c_void) -> bool {
    // SAFETY: The blocking native solve retains this scoped context and serializes callbacks with a mutex.
    let state = unsafe { &mut *(opaque as *mut CallbackContext<'_>) };
    if state.panicked {
        return true;
    }
    let result = catch_unwind(AssertUnwindSafe(|| {
        let bytes = unsafe { std::slice::from_raw_parts(pointer, length) };
        match std::str::from_utf8(bytes) {
            Ok(line) => (state.callback)(line),
            Err(_) => true,
        }
    }));
    match result {
        Ok(stop) => stop,
        Err(_) => {
            state.panicked = true;
            true
        }
    }
}
pub fn solve(
    model: &CpModel,
    parameters: &SatParameters,
    progress: Option<&mut (dyn FnMut(&str) -> bool + Send)>,
) -> Result<CpSolverResponse, String> {
    let request = model.to_bytes();
    let parameters = parameters.encode_to_vec();
    if request.len() > i32::MAX as usize || parameters.len() > i32::MAX as usize {
        return Err("AWX serialized model exceeds native protobuf size".into());
    }
    let mut state = progress.map(|callback| CallbackContext {
        callback,
        panicked: false,
    });
    let opaque = state
        .as_mut()
        .map_or(std::ptr::null_mut(), |state| state as *mut _ as *mut c_void);
    let mut response = std::ptr::null_mut();
    let mut length = 0usize;
    // SAFETY: All request buffers and callback state outlive this blocking call; output is allocated by this bridge.
    let code = unsafe {
        awx_solve_cp_sat(
            request.as_ptr(),
            request.len(),
            parameters.as_ptr(),
            parameters.len(),
            state.as_ref().map(|_| on_log as Callback),
            opaque,
            &mut response,
            &mut length,
        )
    };
    if code != 0 {
        if !response.is_null() {
            unsafe { awx_cp_sat_free(response.cast()) }
        }
        return Err(format!("AWX native CP-SAT bridge error {code}"));
    }
    let decoded = if length == 0 {
        CpSolverResponse::decode(&[][..]).map_err(|e| e.to_string())
    } else if response.is_null() || length > i32::MAX as usize {
        Err("AWX invalid native response buffer".into())
    } else {
        CpSolverResponse::decode(unsafe { std::slice::from_raw_parts(response, length) })
            .map_err(|e| e.to_string())
    };
    if !response.is_null() {
        unsafe { awx_cp_sat_free(response.cast()) }
    }
    if state.as_ref().is_some_and(|state| state.panicked) {
        return Err("AWX progress callback panicked".into());
    }
    decoded
}
