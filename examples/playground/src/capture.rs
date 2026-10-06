//! Turning rendered frames into small looping GIFs and PNG stills: a fixed palette, row flips,
//! downsampling, and frames that store only the rectangle that changed.
//!
//! Everything here works on top-down RGBA rows; OpenGL reads render targets bottom row first,
//! so the recorder flips them once on the way in.

use std::io::Write;

use gif::{DisposalMethod, Encoder, Frame, Repeat};

/// The palette index that marks a pixel unchanged from the previous frame. Quantizing never
/// produces it.
pub const TRANSPARENT: u8 = 255;

/// Frame delays in hundredths of a second, cycled by frame index: 30 frames per second.
const DELAYS: [u16; 3] = [3, 3, 4];

/// The fixed 256-entry palette, RGB: the 6 x 6 x 6 colour cube, 39 greys between the cube's
/// own greys, and [`TRANSPARENT`] last (black, never produced by quantizing).
pub fn palette() -> Vec<[u8; 3]> {
    let mut colours = Vec::with_capacity(256);
    for r in 0..6u8 {
        for g in 0..6u8 {
            for b in 0..6u8 {
                colours.push([r * 51, g * 51, b * 51]);
            }
        }
    }
    for k in 1..=39u32 {
        // (k + 1/2) * 255 / 40 never lands on a multiple of 51, the cube's greys.
        let grey = ((2 * k + 1) * 255 / 80) as u8;
        colours.push([grey; 3]);
    }
    colours.push([0; 3]);
    colours
}

/// Maps 5 bits per channel to the nearest palette entry.
pub struct Lut {
    entries: Vec<u8>,
}

impl Lut {
    /// The table for [`palette`]: each of the 32 x 32 x 32 bins maps to the entry nearest its
    /// centre (Euclidean in RGB), the lower index on a tie, never to [`TRANSPARENT`].
    pub fn new() -> Self {
        let palette = palette();
        let opaque = &palette[..TRANSPARENT as usize];
        let mut entries = Vec::with_capacity(32 * 32 * 32);
        for r in 0..32u32 {
            for g in 0..32u32 {
                for b in 0..32u32 {
                    let centre = [r * 8 + 4, g * 8 + 4, b * 8 + 4].map(|c| c as i32);
                    let distance = |colour: &[u8; 3]| -> i32 {
                        (0..3)
                            .map(|i| (i32::from(colour[i]) - centre[i]).pow(2))
                            .sum()
                    };
                    let nearest = (0..opaque.len())
                        .min_by_key(|&index| (distance(&opaque[index]), index))
                        .expect("the palette has colours");
                    entries.push(nearest as u8);
                }
            }
        }
        Self { entries }
    }

    /// The palette index of an RGB colour.
    pub fn index(&self, [r, g, b]: [u8; 3]) -> u8 {
        let bin = (usize::from(r >> 3) << 10) | (usize::from(g >> 3) << 5) | usize::from(b >> 3);
        self.entries[bin]
    }

    /// The palette indices of RGBA pixels; alpha is ignored.
    pub fn quantize(&self, rgba: &[u8]) -> Vec<u8> {
        rgba.chunks_exact(4)
            .map(|pixel| self.index([pixel[0], pixel[1], pixel[2]]))
            .collect()
    }
}

impl Default for Lut {
    fn default() -> Self {
        Self::new()
    }
}

/// The image of `width` x `height` pixels of 4 bytes with its rows in reverse order.
pub fn flip_rows(rgba: &[u8], width: usize, height: usize) -> Vec<u8> {
    let row = 4 * width;
    (0..height)
        .rev()
        .flat_map(|y| &rgba[y * row..(y + 1) * row])
        .copied()
        .collect()
}

/// The image at half the width and height, each pixel the mean of a 2 x 2 block.
pub fn downsample_2x(rgba: &[u8], width: usize, height: usize) -> Vec<u8> {
    let (half_width, half_height) = (width / 2, height / 2);
    let mut out = Vec::with_capacity(4 * half_width * half_height);
    for y in 0..half_height {
        for x in 0..half_width {
            for channel in 0..4 {
                let at = |dx: usize, dy: usize| {
                    u32::from(rgba[4 * ((2 * y + dy) * width + 2 * x + dx) + channel])
                };
                let sum = at(0, 0) + at(1, 0) + at(0, 1) + at(1, 1);
                out.push(((sum + 2) / 4) as u8);
            }
        }
    }
    out
}

/// The image scaled to `target_width` pixels wide with its aspect kept, each pixel the mean of
/// the source pixels whose centres fall in it.
pub fn thumbnail(
    rgba: &[u8],
    width: usize,
    height: usize,
    target_width: usize,
) -> (Vec<u8>, usize) {
    let target_height = (height * target_width + width / 2) / width;
    let mut out = Vec::with_capacity(4 * target_width * target_height);
    for ty in 0..target_height {
        let (y0, y1) = (
            ty * height / target_height,
            ((ty + 1) * height / target_height).max(ty * height / target_height + 1),
        );
        for tx in 0..target_width {
            let (x0, x1) = (
                tx * width / target_width,
                ((tx + 1) * width / target_width).max(tx * width / target_width + 1),
            );
            let mut sum = [0u32; 4];
            for y in y0..y1 {
                for x in x0..x1 {
                    for (channel, total) in sum.iter_mut().enumerate() {
                        *total += u32::from(rgba[4 * (y * width + x) + channel]);
                    }
                }
            }
            let count = ((y1 - y0) * (x1 - x0)) as u32;
            out.extend(sum.map(|total| ((total + count / 2) / count) as u8));
        }
    }
    (out, target_height)
}

/// A rectangle of pixels.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rect {
    /// First column.
    pub left: usize,
    /// First row.
    pub top: usize,
    /// Columns.
    pub width: usize,
    /// Rows.
    pub height: usize,
}

/// The smallest rectangle holding every pixel that differs between two index images of
/// `width` columns, or `None` when they are equal.
pub fn changed_rect(previous: &[u8], next: &[u8], width: usize) -> Option<Rect> {
    let mut bounds: Option<(usize, usize, usize, usize)> = None;
    for (index, (a, b)) in previous.iter().zip(next).enumerate() {
        if a != b {
            let (x, y) = (index % width, index / width);
            bounds = Some(match bounds {
                None => (x, y, x, y),
                Some((x0, y0, x1, y1)) => (x0.min(x), y0.min(y), x1.max(x), y1.max(y)),
            });
        }
    }
    bounds.map(|(x0, y0, x1, y1)| Rect {
        left: x0,
        top: y0,
        width: x1 - x0 + 1,
        height: y1 - y0 + 1,
    })
}

/// A frame written once its delay is known.
struct Pending {
    rect: Rect,
    indices: Vec<u8>,
    delay: u16,
}

/// Writes index images as a looping GIF with the fixed palette: each frame stores only the
/// rectangle that changed, with unchanged pixels transparent over the kept previous frame, and
/// an unchanged frame lengthens the one before it.
pub struct GifWriter<W: Write> {
    encoder: Encoder<W>,
    width: usize,
    previous: Option<Vec<u8>>,
    pending: Option<Pending>,
    frames: usize,
}

impl<W: Write> GifWriter<W> {
    /// A GIF of `width` x `height` pixels into `out`.
    pub fn new(out: W, width: u16, height: u16) -> Result<Self, gif::EncodingError> {
        let colours: Vec<u8> = palette().into_iter().flatten().collect();
        let mut encoder = Encoder::new(out, width, height, &colours)?;
        encoder.set_repeat(Repeat::Infinite)?;
        Ok(Self {
            encoder,
            width: usize::from(width),
            previous: None,
            pending: None,
            frames: 0,
        })
    }

    /// Adds the next frame, `width * height` palette indices, top row first.
    pub fn push(&mut self, indices: Vec<u8>) -> Result<(), gif::EncodingError> {
        let delay = DELAYS[self.frames % DELAYS.len()];
        self.frames += 1;
        let height = indices.len() / self.width;
        let rect = match &self.previous {
            None => Some(Rect {
                left: 0,
                top: 0,
                width: self.width,
                height,
            }),
            Some(previous) => changed_rect(previous, &indices, self.width),
        };
        let Some(rect) = rect else {
            if let Some(pending) = &mut self.pending {
                pending.delay += delay;
            }
            return Ok(());
        };
        let mut cropped = Vec::with_capacity(rect.width * rect.height);
        for y in rect.top..rect.top + rect.height {
            for x in rect.left..rect.left + rect.width {
                let index = y * self.width + x;
                let unchanged = self
                    .previous
                    .as_ref()
                    .is_some_and(|previous| previous[index] == indices[index]);
                cropped.push(if unchanged {
                    TRANSPARENT
                } else {
                    indices[index]
                });
            }
        }
        self.flush()?;
        self.pending = Some(Pending {
            rect,
            indices: cropped,
            delay,
        });
        self.previous = Some(indices);
        Ok(())
    }

    fn flush(&mut self) -> Result<(), gif::EncodingError> {
        if let Some(pending) = self.pending.take() {
            let frame = Frame {
                delay: pending.delay,
                dispose: DisposalMethod::Keep,
                transparent: Some(TRANSPARENT),
                left: pending.rect.left as u16,
                top: pending.rect.top as u16,
                width: pending.rect.width as u16,
                height: pending.rect.height as u16,
                buffer: pending.indices.into(),
                ..Frame::default()
            };
            self.encoder.write_frame(&frame)?;
        }
        Ok(())
    }

    /// Writes the last frame and returns the output.
    pub fn finish(mut self) -> Result<W, gif::EncodingError> {
        self.flush()?;
        self.encoder.into_inner()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_palette_has_no_duplicates_and_colours_map_near_themselves() {
        let palette = palette();
        assert_eq!(palette.len(), 256);
        let mut sorted = palette[..255].to_vec();
        sorted.sort();
        sorted.dedup();
        assert_eq!(sorted.len(), 255);
        let lut = Lut::new();
        // A bin is 8 levels wide, so a grey of the cube can share its bin with a nearer grey of
        // the ramp; every other cube colour maps to itself.
        for (index, colour) in palette[..216].iter().enumerate() {
            if colour[0] != colour[1] || colour[1] != colour[2] {
                assert_eq!(lut.index(*colour), index as u8, "{colour:?}");
            }
        }
        for colour in &palette[..255] {
            let mapped = palette[usize::from(lut.index(*colour))];
            for channel in 0..3 {
                let error = i32::from(mapped[channel]) - i32::from(colour[channel]);
                assert!(error.abs() <= 8, "{colour:?} maps to {mapped:?}");
            }
        }
    }

    #[test]
    fn no_opaque_colour_maps_to_the_transparent_index() {
        let lut = Lut::new();
        for r in (0..=255u8).step_by(5) {
            for g in (0..=255u8).step_by(5) {
                for b in (0..=255u8).step_by(5) {
                    assert_ne!(lut.index([r, g, b]), TRANSPARENT);
                }
            }
        }
    }

    #[test]
    fn rows_flip_and_halve() {
        // Two rows of two pixels: red, green / blue, white.
        let image = [
            255, 0, 0, 255, 0, 255, 0, 255, //
            0, 0, 255, 255, 255, 255, 255, 255,
        ];
        let flipped = flip_rows(&image, 2, 2);
        assert_eq!(&flipped[..8], &image[8..]);
        assert_eq!(flip_rows(&flipped, 2, 2), image);
        assert_eq!(downsample_2x(&image, 2, 2), vec![128, 128, 128, 255]);
        let (small, height) = thumbnail(&image, 2, 2, 1);
        assert_eq!((small, height), (vec![128, 128, 128, 255], 1));
    }

    /// Decodes a GIF into its full frames of palette indices, compositing each frame's
    /// rectangle over the previous canvas as `Keep` does, with the delays.
    fn decode(bytes: &[u8]) -> (Vec<Vec<u8>>, Vec<u16>, Repeat) {
        let mut options = gif::DecodeOptions::new();
        options.set_color_output(gif::ColorOutput::Indexed);
        let mut decoder = options.read_info(bytes).unwrap();
        let width = usize::from(decoder.width());
        let mut canvas = vec![0u8; width * usize::from(decoder.height())];
        let (mut frames, mut delays) = (Vec::new(), Vec::new());
        while let Some(frame) = decoder.read_next_frame().unwrap() {
            for y in 0..usize::from(frame.height) {
                for x in 0..usize::from(frame.width) {
                    let value = frame.buffer[y * usize::from(frame.width) + x];
                    if Some(value) != frame.transparent {
                        let at = (y + usize::from(frame.top)) * width + x + usize::from(frame.left);
                        canvas[at] = value;
                    }
                }
            }
            frames.push(canvas.clone());
            delays.push(frame.delay);
        }
        (frames, delays, decoder.repeat())
    }

    /// An 8 x 6 index image of background 10 with a 2 x 2 square of 20 at `square`.
    fn image(square: Option<(usize, usize)>) -> Vec<u8> {
        let mut pixels = vec![10u8; 8 * 6];
        if let Some((x, y)) = square {
            for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                pixels[(y + dy) * 8 + x + dx] = 20;
            }
        }
        pixels
    }

    fn encode(frames: &[Vec<u8>]) -> Vec<u8> {
        let mut writer = GifWriter::new(Vec::new(), 8, 6).unwrap();
        for frame in frames {
            writer.push(frame.clone()).unwrap();
        }
        writer.finish().unwrap()
    }

    #[test]
    fn a_square_that_appears_moves_and_disappears_decodes_to_the_source_frames() {
        let source = [
            image(None),
            image(Some((1, 1))),
            image(Some((4, 2))),
            image(Some((4, 2))),
            image(None),
        ];
        let bytes = encode(&source);
        let (frames, delays, repeat) = decode(&bytes);
        assert_eq!(repeat, Repeat::Infinite);
        // The unchanged fourth frame lengthens the third.
        let distinct = [&source[0], &source[1], &source[2], &source[4]];
        assert_eq!(frames.len(), distinct.len());
        for (decoded, expected) in frames.iter().zip(distinct) {
            assert_eq!(decoded, expected);
        }
        assert_eq!(delays, vec![3, 3, 4 + 3, 3]);
        assert_eq!(
            bytes,
            encode(&source),
            "encoding twice gives the same bytes"
        );
    }

    #[test]
    fn identical_frames_add_their_delays() {
        for count in [1, 2, 7] {
            let frames = vec![image(Some((2, 2))); count];
            let (decoded, delays, _) = decode(&encode(&frames));
            assert_eq!(decoded.len(), 1);
            let expected: u16 = (0..count).map(|i| DELAYS[i % 3]).sum();
            assert_eq!(delays, vec![expected], "{count} frames");
        }
    }
}
