use std::io::Write;

fn bit_pusher(data: &mut Vec<u32>) -> impl FnMut(bool) + '_ {
    let mut bit_idx = 0;
    move |bit| {
        if bit_idx == 0 {
            data.push(bit as u32);
            bit_idx = 1;
        } else {
            if bit {
                *data.last_mut().unwrap() |= 1 << bit_idx;
            }
            bit_idx = (bit_idx + 1) % 32;
        }
    }
}

fn s16le(data: &[u8]) -> Vec<u32> {
    // let mut cur_error = 0_i64;
    let mut output_data = vec![];
    let mut bit_pusher = bit_pusher(&mut output_data);
    for word in data
        .chunks_exact(2)
        .map(|chunk| bytemuck::cast::<[u8; 2], i16>(chunk.try_into().unwrap()))
    {
        bit_pusher(word > 0);
    }
    drop(bit_pusher);
    output_data
}

fn main() {
    let filename = std::env::args()
        .nth(1)
        .expect("Usage: cargo run [filename] [source_encoding]");
    let encoding = std::env::args()
        .nth(2)
        .expect("Usage: cargo run [filename] [source_encoding]");

    let data = std::fs::read(&filename).expect("failed to read pcm file");
    let output_data = match &*encoding {
        "s16le" => s16le(&data),
        _ => panic!("unrecognized encoding: {encoding}"),
    };
    let output_data = bytemuck::cast_slice::<u32, u8>(&output_data);
    std::io::stdout().lock().write_all(output_data).unwrap();
}
