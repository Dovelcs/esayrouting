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

/// CPython 3.12 `math.fsum` for finite inputs.
///
/// Transcribed from `Modules/mathmodule.c` (`math_fsum`, the Shewchuk exact
/// summation). Each addend is folded into a running list of non-overlapping
/// partials kept in ascending magnitude order: for each live partial `y` the
/// operands are ordered so `x` has the larger magnitude, then the exact
/// two-sum is split with `hi = x + y; yr = hi - x; lo = y - yr` (this exact
/// sequencing, *not* `x - hi + y`, is what makes the split error-free). The
/// non-zero low part overwrites the consumed slot in place and the surviving
/// high part `x` is appended only when it is non-zero (`if (x != 0.0)`).
///
/// The final correction mirrors the source's tail exactly: the largest partial
/// is popped as `hi`, the remaining partials are summed *descending* by
/// `x = hi; y = p[--n]; hi = x + y; yr = hi - x; lo = y - yr`, stopping at the
/// first non-zero `lo`; if the next remaining partial has the same sign as `lo`
/// the sum is nudged by `y = lo * 2.0; x = hi + y; yr = x - hi; if (y == yr)
/// hi = x` so half-even rounding carries across multiple partials (this is what
/// makes `fsum([1e16, 1, 1e-16])` round the last digit up to two). The result is
/// `hi`, never a plain `sum(partials, 0.0)`.
///
/// This is the interpreter's algorithm, not a Kahan/Neumaier stand-in: operand
/// order is preserved, there is no reassociation and no fused multiply add, so
/// the result is bit-identical to `math.fsum` on finite inputs.
///
/// Non-finite inputs (NaN / ±inf) and finite-input intermediate overflow have no
/// finite value the router can use, so — matching this module's other helpers —
/// they are surfaced through `Result` instead of a panic or a silently wrong
/// sum. The callers that already return `Result` propagate the error.
pub fn fsum<I: IntoIterator<Item = f64>>(values: I) -> Result<f64, String> {
    let mut partials: Vec<f64> = Vec::new();
    for value in values {
        if !value.is_finite() {
            return Err("AWX nonfinite fsum input".into());
        }
        let mut x = value;
        // `count` snapshots the live partial list; `write` is the compacted index
        // the source reuses for surviving low parts (`partials[i++] = lo`).
        let count = partials.len();
        let mut write = 0usize;
        for read in 0..count {
            let mut y = partials[read];
            if x.abs() < y.abs() {
                std::mem::swap(&mut x, &mut y);
            }
            let hi = x + y; // source `hi`
            if !hi.is_finite() {
                return Err("AWX intermediate overflow in fsum".into());
            }
            let yr = hi - x; // source `yr`
            let lo = y - yr; // source `lo = y - (hi - x)`, not `x - hi + y`
            x = hi;
            if lo != 0.0 {
                partials[write] = lo;
                write += 1;
            }
        }
        partials.truncate(write);
        // Source `n = i; if (x != 0.0) p[n++] = x;`
        if x != 0.0 {
            partials.push(x);
        }
    }
    // Final correction: pop the largest partial, then sum the remaining partials
    // descending with the exact two-sum, stopping at the first non-zero low part.
    let mut n = partials.len();
    if n == 0 {
        return Ok(0.0);
    }
    n -= 1;
    let mut hi = partials[n];
    let mut lo = 0.0_f64;
    while n > 0 {
        let x = hi;
        n -= 1;
        let y = partials[n];
        hi = x + y;
        let yr = hi - x;
        lo = y - yr;
        if lo != 0.0 {
            break;
        }
    }
    // Half-even correction across multiple partials: if the next remaining
    // partial has the same sign as the residual, nudge the result up so a
    // halfway case rounds away from even.
    if n > 0 && ((lo < 0.0 && partials[n - 1] < 0.0) || (lo > 0.0 && partials[n - 1] > 0.0)) {
        let y = lo * 2.0;
        let x = hi + y;
        let yr = x - hi;
        if y == yr {
            hi = x;
        }
    }
    Ok(hi)
}
