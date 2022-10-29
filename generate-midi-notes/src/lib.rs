use std::sync::atomic::{AtomicUsize, Ordering};

use lazy_static::lazy_static;

pub const CLOCK_FREQ: f64 = 125_000_000_f64;

lazy_static! {
    static ref DIV_VALUES: Vec<(f64, u8, u8)> = {
        let mut vec = Vec::with_capacity(255 * 16);
        for i in 1..=255 {
            for f in 0..16 {
                let float = i as f64 + f as f64 / 16.0;
                vec.push((float, i, f));
            }
        }
        vec
    };
}

/// (div_int, div_frac, top, abs_error)
/// TODO: optimize this. it currently checks every possible combination
fn best_div_top_for_product(goal: f64) -> (u8, u8, u16, f64) {
    DIV_VALUES
        .iter()
        .copied()
        .flat_map(|(div_float, div_int, div_frac)| {
            // Start closer to max to not change the duty cycle too much if not necessary
            let start: u16 = if div_int > 3 {
                65200
            } else if div_int > 2 {
                65000
            } else if div_int > 1 {
                64000
            } else {
                0
            };
            (start..=u16::MAX)
                .map(move |top| (div_int, div_frac, top, error(goal, top, div_float)))
        })
        .min_by_key(|(_, _, _, error)| float_ord::FloatOrd(error.abs()))
        .unwrap()
}

/// signed
fn error(goal: f64, top: u16, div: f64) -> f64 {
    let actual = (top as f64 + 1.0) * div;
    actual - goal
}

/// See rp-2040 datasheet section 4.5.2.6
/// f_pwm = f_sys / ((top+1) * (is_phasecorrect + 1) * (div_int + div_frac / 16))
/// Maximum of 1 count per cycle, so div_int >= 1.
/// (assume phase-correct is false for now)
/// (top+1) * (div_int + div_frac / 16) = f_sys / f_pwm
fn clkdiv_top_and_error_for_freq(freq: f64) -> (u8, u8, u16, f64) {
    if freq <= 0.0 {
        panic!()
    }
    let product: f64 = CLOCK_FREQ / freq;

    best_div_top_for_product(product)
}

/// (div_int, div_frac (4 bits), top, requested_freq, error)
pub fn midi_notes() -> impl Iterator<Item = (u8, u8, u16, f64, f64)> {
    static COUNT: AtomicUsize = AtomicUsize::new(0);
    ctrlc::set_handler(|| {
        eprintln!("{} notes generated", COUNT.load(Ordering::Relaxed));
    })
    .unwrap();
    (0..128).map(|i| {
        let half_steps_from_a4 = i as i32 - 69;
        let freq = 444.0 * 2.0_f64.powf(half_steps_from_a4 as f64 / 12.0);
        COUNT.fetch_add(1, Ordering::Relaxed);
        let (div_int, div_frac, top, error) = clkdiv_top_and_error_for_freq(freq);
        (div_int, div_frac, top, freq, error)
    })
}
