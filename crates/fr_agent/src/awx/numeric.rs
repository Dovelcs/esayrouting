//! Python numeric semantics shared by the fixed-reference AWX port.
pub fn checked_round_i64(value: f64) -> Result<i64, String> {
    let rounded = value.round_ties_even();
    if !rounded.is_finite() || rounded < i64::MIN as f64 || rounded >= -(i64::MIN as f64) {
        return Err("AWX integer coordinate overflow".into());
    }
    Ok(rounded as i64)
}
pub fn quantize(value: f64, step: f64) -> Result<i64, String> {
    if !step.is_finite() || step <= 0.0 {
        return Err("AWX quantization step must be positive".into());
    }
    checked_round_i64(value / step)
}
pub fn quantize_up(value: f64, step: f64) -> Result<i64, String> {
    if !step.is_finite() || step <= 0.0 {
        return Err("AWX quantization step must be positive".into());
    }
    let rounded = (value / step - 1e-9).ceil();
    if !rounded.is_finite() || rounded < i64::MIN as f64 || rounded >= -(i64::MIN as f64) {
        return Err("AWX upward quantization overflow".into());
    }
    Ok(rounded as i64)
}
pub fn round_digits(value: f64, digits: i32) -> Result<f64, String> {
    if !value.is_finite() {
        return Err("AWX nonfinite rounded coordinate".into());
    }
    if digits > 323 {
        return Ok(value);
    }
    if digits < -308 {
        return Ok(0.0_f64.copysign(value));
    }
    if digits >= 0 {
        // Decimal formatting rounds the exact binary float, unlike multiplying first:
        // a scale-first implementation changes Python round(2.675,2)'s decision.
        return format!("{value:.precision$}", precision = digits as usize)
            .parse::<f64>()
            .map_err(|_| "AWX decimal rounding overflow".into());
    }
    let scale = 10_f64.powi(-digits);
    let result = (value / scale).round_ties_even() * scale;
    if !result.is_finite() {
        return Err("AWX decimal rounding overflow".into());
    }
    Ok(result)
}
pub fn floor_div(a: i64, b: i64) -> Result<i64, String> {
    if b == 0 {
        return Err("AWX division by zero".into());
    }
    let q = a.checked_div(b).ok_or("AWX integer division overflow")?;
    let r = a.checked_rem(b).ok_or("AWX integer remainder overflow")?;
    if r != 0 && ((r < 0) != (b < 0)) {
        q.checked_sub(1).ok_or("AWX floor division overflow".into())
    } else {
        Ok(q)
    }
}
pub fn modulo(a: f64, b: f64) -> Result<f64, String> {
    if !a.is_finite() || !b.is_finite() || b == 0.0 {
        return Err("AWX invalid floating remainder".into());
    }
    let mut r = a % b;
    if r != 0.0 && ((r < 0.0) != (b < 0.0)) {
        r += b;
    }
    if r == 0.0 {
        r = 0.0_f64.copysign(b)
    }
    Ok(r)
}
pub fn checked_add(a: i64, b: i64) -> Result<i64, String> {
    a.checked_add(b)
        .ok_or("AWX integer addition overflow".into())
}
pub fn checked_mul(a: i64, b: i64) -> Result<i64, String> {
    a.checked_mul(b)
        .ok_or("AWX integer multiplication overflow".into())
}
