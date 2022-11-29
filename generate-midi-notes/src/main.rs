use generate_midi_notes::CLOCK_FREQ;

fn main() {
    println!("//! Generated by `generate-midi-notes`.");
    println!();
    println!("pub static MIDI_NOTES: [(u8, u8, u16); 128] = [");
    for (div_int, div_frac, top, freq, product_error) in generate_midi_notes::midi_notes() {
        let product = CLOCK_FREQ / freq;
        let real_freq = CLOCK_FREQ / (product + product_error);
        let freq_error = real_freq - freq;
        println!("    ({div_int}, {div_frac}, {top}), // intended frequency: {freq}, actual frequency: {real_freq}, frequency error = {freq_error}");
    }
    println!("];");
}