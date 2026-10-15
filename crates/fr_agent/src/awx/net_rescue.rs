use super::routing_defaults::{
    RESCUE_GRID_STEP_MM, RESCUE_MAX_ITERATIONS, RESCUE_MAX_WINDOW_CELLS,
    RESCUE_MIN_WINDOW_HALF_MM, RESCUE_WINDOW_MARGIN_MM,
};

pub(crate) struct RescuePlan {
    pub grid_step_mm: f64,
    pub max_iterations: usize,
    pub window_mm: [f64; 4],
}

pub(crate) fn plan_rescue(
    base_grid_mm: f64,
    base_iterations: usize,
    a: [f64; 2],
    b: [f64; 2],
) -> Result<RescuePlan, String> {
    if !base_grid_mm.is_finite() || base_grid_mm <= 0.0 {
        return Err("base grid step must be finite and positive".to_string());
    }
    if base_iterations == 0 {
        return Err("base iteration limit must be positive".to_string());
    }
    if !a.iter().chain(b.iter()).all(|value| value.is_finite()) {
        return Err("gap endpoints must be finite".to_string());
    }

    let distance = (b[0] - a[0]).hypot(b[1] - a[1]);
    if !distance.is_finite() {
        return Err("gap distance is not finite".to_string());
    }

    let center_x = a[0] * 0.5 + b[0] * 0.5;
    let center_y = a[1] * 0.5 + b[1] * 0.5;
    let half = (distance * 0.5 + RESCUE_WINDOW_MARGIN_MM)
        .max(RESCUE_MIN_WINDOW_HALF_MM);

    let mut grid_step_mm = base_grid_mm.min(RESCUE_GRID_STEP_MM);
    while (2.0 * half / grid_step_mm).powi(2) > RESCUE_MAX_WINDOW_CELLS as f64 {
        grid_step_mm *= 2.0;
        if !grid_step_mm.is_finite() {
            return Err("rescue grid step overflowed while fitting the cell budget".to_string());
        }
    }

    let window_mm = [
        center_x - half,
        center_y - half,
        center_x + half,
        center_y + half,
    ];
    if !center_x.is_finite() || !center_y.is_finite() || !half.is_finite()
        || !window_mm.iter().all(|value| value.is_finite())
    {
        return Err("rescue window bounds are not finite".to_string());
    }

    Ok(RescuePlan {
        grid_step_mm,
        max_iterations: base_iterations.max(RESCUE_MAX_ITERATIONS),
        window_mm,
    })
}
