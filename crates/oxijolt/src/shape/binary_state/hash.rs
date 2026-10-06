//! XXH64, the checksum and the build id of shape binary state, after the xxHash specification
//! (<https://github.com/Cyan4973/xxHash/blob/dev/doc/xxhash_spec.md>). A checksum detects damage,
//! not forgery: anyone can compute it.

const PRIME_1: u64 = 0x9E37_79B1_85EB_CA87;
const PRIME_2: u64 = 0xC2B2_AE3D_27D4_EB4F;
const PRIME_3: u64 = 0x1656_67B1_9E37_79F9;
const PRIME_4: u64 = 0x85EB_CA77_C2B2_AE63;
const PRIME_5: u64 = 0x27D4_EB2F_1656_67C5;

/// Bytes the four accumulators consume at a time.
const STRIPE: usize = 32;

/// An XXH64 hash fed in pieces; the result equals the hash of the concatenated pieces.
pub(super) struct Xxh64 {
    seed: u64,
    accumulators: [u64; 4],
    buffer: [u8; STRIPE],
    buffered: usize,
    total: u64,
}

impl Xxh64 {
    pub(super) fn new(seed: u64) -> Self {
        Self {
            seed,
            accumulators: [
                seed.wrapping_add(PRIME_1).wrapping_add(PRIME_2),
                seed.wrapping_add(PRIME_2),
                seed,
                seed.wrapping_sub(PRIME_1),
            ],
            buffer: [0; STRIPE],
            buffered: 0,
            total: 0,
        }
    }

    pub(super) fn update(&mut self, mut data: &[u8]) {
        self.total = self.total.wrapping_add(data.len() as u64);
        if self.buffered > 0 {
            let take = (STRIPE - self.buffered).min(data.len());
            self.buffer[self.buffered..self.buffered + take].copy_from_slice(&data[..take]);
            self.buffered += take;
            data = &data[take..];
            if self.buffered < STRIPE {
                return;
            }
            let stripe = self.buffer;
            self.consume(&stripe);
            self.buffered = 0;
        }
        let (stripes, rest) = data.as_chunks::<STRIPE>();
        for stripe in stripes {
            self.consume(stripe);
        }
        self.buffer[..rest.len()].copy_from_slice(rest);
        self.buffered = rest.len();
    }

    pub(super) fn finish(&self) -> u64 {
        let mut hash = if self.total >= STRIPE as u64 {
            let [a, b, c, d] = self.accumulators;
            let mut hash = a
                .rotate_left(1)
                .wrapping_add(b.rotate_left(7))
                .wrapping_add(c.rotate_left(12))
                .wrapping_add(d.rotate_left(18));
            for accumulator in self.accumulators {
                hash = (hash ^ round(0, accumulator))
                    .wrapping_mul(PRIME_1)
                    .wrapping_add(PRIME_4);
            }
            hash
        } else {
            self.seed.wrapping_add(PRIME_5)
        };
        hash = hash.wrapping_add(self.total);

        let mut rest = &self.buffer[..self.buffered];
        while rest.len() >= 8 {
            hash ^= round(0, read_u64(rest));
            hash = hash
                .rotate_left(27)
                .wrapping_mul(PRIME_1)
                .wrapping_add(PRIME_4);
            rest = &rest[8..];
        }
        if rest.len() >= 4 {
            hash ^= u64::from(read_u32(rest)).wrapping_mul(PRIME_1);
            hash = hash
                .rotate_left(23)
                .wrapping_mul(PRIME_2)
                .wrapping_add(PRIME_3);
            rest = &rest[4..];
        }
        for &byte in rest {
            hash ^= u64::from(byte).wrapping_mul(PRIME_5);
            hash = hash.rotate_left(11).wrapping_mul(PRIME_1);
        }
        avalanche(hash)
    }

    fn consume(&mut self, stripe: &[u8; STRIPE]) {
        let (lanes, _) = stripe.as_chunks::<8>();
        for (accumulator, lane) in self.accumulators.iter_mut().zip(lanes) {
            *accumulator = round(*accumulator, u64::from_le_bytes(*lane));
        }
    }
}

/// The XXH64 of `data` with `seed`.
pub(super) fn xxh64(data: &[u8], seed: u64) -> u64 {
    let mut hasher = Xxh64::new(seed);
    hasher.update(data);
    hasher.finish()
}

fn round(accumulator: u64, lane: u64) -> u64 {
    accumulator
        .wrapping_add(lane.wrapping_mul(PRIME_2))
        .rotate_left(31)
        .wrapping_mul(PRIME_1)
}

fn avalanche(mut hash: u64) -> u64 {
    hash ^= hash >> 33;
    hash = hash.wrapping_mul(PRIME_2);
    hash ^= hash >> 29;
    hash = hash.wrapping_mul(PRIME_3);
    hash ^ (hash >> 32)
}

fn read_u64(bytes: &[u8]) -> u64 {
    let mut lane = [0; 8];
    lane.copy_from_slice(&bytes[..8]);
    u64::from_le_bytes(lane)
}

fn read_u32(bytes: &[u8]) -> u32 {
    let mut lane = [0; 4];
    lane.copy_from_slice(&bytes[..4]);
    u32::from_le_bytes(lane)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The test input of the reference vectors: byte `i` is `7 i + 3` modulo 256.
    fn data() -> Vec<u8> {
        (0..300u32).map(|i| (i * 7 + 3) as u8).collect()
    }

    /// Values of the reference implementation (python-xxhash 3.5.0, `xxh64(data, seed)`).
    #[test]
    fn matches_the_reference_vectors() {
        let data = data();
        let cases: [(&[u8], u64, u64); 9] = [
            (b"", 0, 0xEF46_DB37_51D8_E999),
            (b"a", 0, 0xD24E_C4F1_A98C_6E5B),
            (b"abc", 0, 0x44BC_2CF5_AD77_0999),
            (b"abc", 1, 0xBEA9_CA81_9932_8908),
            (&data[..31], 0, 0xA2AA_5F33_CC4A_6119),
            (&data[..32], 0, 0x23C3_C17E_F790_FD97),
            (&data[..100], 7, 0x6281_D896_ACB8_0D7C),
            (&data, 0, 0x2400_04DB_EE0B_A6DC),
            (&data, 0x9E37_79B9_7F4A_7C15, 0x4724_CCB3_6F2A_EE62),
        ];
        for (input, seed, expected) in cases {
            assert_eq!(
                xxh64(input, seed),
                expected,
                "{} bytes, seed {seed}",
                input.len()
            );
        }
    }

    #[test]
    fn pieces_hash_like_the_whole() {
        let data = data();
        let whole = xxh64(&data, 3);
        for split in [0, 1, 5, 31, 32, 33, 64, 299, 300] {
            for second in [split, (split + 40).min(300)] {
                let mut hasher = Xxh64::new(3);
                hasher.update(&data[..split]);
                hasher.update(&data[split..second]);
                hasher.update(&data[second..]);
                assert_eq!(hasher.finish(), whole, "split at {split} and {second}");
            }
        }
    }
}
