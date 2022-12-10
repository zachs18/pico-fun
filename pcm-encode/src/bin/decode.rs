use std::io::Write;

fn bit_puller(mut data: &[[u8; 4]]) -> impl Iterator<Item = bool> + '_ {
    let mut bit_idx = 0;
    let mut word = 0;
    std::iter::from_fn(move || {
        if bit_idx == 0 {
            let word_bytes;
            (word_bytes, data) = data.split_first()?;
            word = bytemuck::cast::<_, u32>(*word_bytes);
            let bit = word & 1 != 0;
            bit_idx = 1;
            Some(bit)
        } else {
            let bit = (word >> bit_idx) & 1 != 0;
            bit_idx = (bit_idx + 1) % 32;
            Some(bit)
        }
    })
}

fn s16le(data: &[[u8; 4]]) -> Vec<i16> {
    // let mut cur_error = 0_i64;
    let mut output_data = vec![];
    let mut bit_puller = bit_puller(&data);
    for bit in bit_puller {
        output_data.push(if bit { 32767 } else { -32768 });
    }
    output_data
}

fn main() {
    let filename = std::env::args()
        .nth(1)
        .expect("Usage: cargo run [filename] [source_encoding]");
    let encoding = std::env::args()
        .nth(2)
        .expect("Usage: cargo run [filename] [source_encoding]");

    let mut data = std::fs::read(&filename).expect("failed to read pcm file");
    data.resize(
        {
            let this = data.len();
            let rhs = 4;
            match this % rhs {
                0 => this,
                r => this + (rhs - r),
            }
        },
        0,
    );
    let data = bytemuck::allocation::cast_vec(data);
    let output_data = match &*encoding {
        "s16le" => s16le(&data),
        _ => panic!("unrecognized encoding: {encoding}"),
    };
    let output_data = bytemuck::cast_slice::<i16, u8>(&output_data);
    std::io::stdout().lock().write_all(output_data).unwrap();
}
