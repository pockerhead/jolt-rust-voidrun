//! The geometry a study scene really has, decoded from its shapes, and the distances the law
//! predicates measure against it, in `f64`.
//!
//! Heightfields are read back sample by sample with [`Shape::height_field_position`], so the
//! oracle sees Jolt's quantised surface and Jolt's own cell diagonal (from sample `(x, y)` to
//! `(x + 1, y + 1)`, `HeightFieldShape::GetSubShapeCoordinates`). Boxes are kept with their
//! convex radius: a box of half extents `h` and radius `r` is the inner box `h - r` grown by a
//! sphere of radius `r`.

use oxijolt::*;

use super::frame::Frame;
use crate::common::math::{add, cross, dot, f3, norm, scale, sub, V3};

/// Half the angle, degrees, around the 45 degree limit inside which a face is "boundary" and no
/// law predicate judges it.
pub const ANGLE_MARGIN_DEG: f64 = 0.1;
/// The slope limit of the game, degrees.
pub const LIMIT_DEG: f64 = 45.0;
/// Elements within this distance, metres, of the closest one belong to the support set.
pub const SUPPORT_BAND: f64 = 0.05;

/// A face's class against an up.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FaceClass {
    /// At most 45 - margin degrees from up.
    Walkable,
    /// Within the margin of 45 degrees.
    Boundary,
    /// At least 45 + margin degrees from up.
    Steep,
}

/// The class of a face with unit normal `normal` against unit `up`.
pub fn classify(normal: V3, up: V3) -> FaceClass {
    let angle = dot(normal, up).clamp(-1.0, 1.0).acos().to_degrees();
    if angle <= LIMIT_DEG - ANGLE_MARGIN_DEG {
        FaceClass::Walkable
    } else if angle >= LIMIT_DEG + ANGLE_MARGIN_DEG {
        FaceClass::Steep
    } else {
        FaceClass::Boundary
    }
}

/// What kind of geometry an element belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Terrain,
    Box,
}

/// One triangle of a heightfield or one face of a box, seen from a point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Element {
    pub kind: Kind,
    /// Outward unit normal in world space.
    pub normal: V3,
    /// Signed distance from the point to the element's surface, metres; negative inside.
    pub distance: f64,
}

/// A heightfield as Jolt stores it, in the local space of its body.
#[derive(Clone, Debug)]
pub struct Field {
    pub frame: Frame,
    /// Samples per side as given (before Jolt's padding).
    pub samples: usize,
    /// Decoded sample positions, row-major (`y * samples + x`), `None` for holes.
    pub positions: Vec<Option<V3>>,
    /// Sample spacing along x and z, metres.
    pub spacing: f64,
    /// Local position of sample (0, 0) at height 0.
    pub offset: V3,
    /// The heights given to Jolt, row-major, for the quantisation check.
    pub intended: Vec<f64>,
    /// Bits per sample of the field.
    pub bits: u32,
}

impl Field {
    /// Reads `shape`'s samples back. `intended` are the heights it was made from.
    pub fn decode(
        shape: &Shape,
        frame: Frame,
        samples: usize,
        spacing: f64,
        offset: V3,
        intended: Vec<f64>,
        bits: u32,
    ) -> Self {
        let positions = (0..samples * samples)
            .map(|i| {
                shape
                    .height_field_position((i % samples) as u32, (i / samples) as u32)
                    .map(f3)
            })
            .collect();
        Self {
            frame,
            samples,
            positions,
            spacing,
            offset,
            intended,
            bits,
        }
    }

    fn position(&self, x: usize, y: usize) -> Option<V3> {
        self.positions[y * self.samples + x]
    }

    /// The two triangles of cell `(x, y)` in local space, as Jolt splits it.
    fn cell_triangles(&self, x: usize, y: usize) -> [Option<[V3; 3]>; 2] {
        let x1y1 = self.position(x, y);
        let x2y2 = self.position(x + 1, y + 1);
        let x1y2 = self.position(x, y + 1);
        let x2y1 = self.position(x + 1, y);
        let triangle = |a: Option<V3>, b: Option<V3>, c: Option<V3>| Some([a?, b?, c?]);
        [triangle(x1y1, x1y2, x2y2), triangle(x1y1, x2y2, x2y1)]
    }

    /// The cell index range around local coordinate `c` (x or z), `reach` cells each way.
    fn cells_around(&self, c: f64, axis: usize, reach: f64) -> std::ops::Range<usize> {
        let cells = (self.samples - 1) as f64;
        let at = (c - self.offset[axis]) / self.spacing;
        let low = (at - reach).floor().clamp(0.0, cells) as usize;
        let high = (at + reach).ceil().clamp(0.0, cells) as usize;
        low..high
    }

    /// The surface height at local `(x, z)`, if the field covers it.
    pub fn height_at(&self, x: f64, z: f64) -> Option<f64> {
        let cells = (self.samples - 1) as f64;
        let (u, v) = (
            (x - self.offset[0]) / self.spacing,
            (z - self.offset[2]) / self.spacing,
        );
        if !(0.0..=cells).contains(&u) || !(0.0..=cells).contains(&v) {
            return None;
        }
        let (cx, cy) = (
            (u.floor() as usize).min(self.samples - 2),
            (v.floor() as usize).min(self.samples - 2),
        );
        let (fu, fv) = (u - cx as f64, v - cy as f64);
        // Triangle (x1y1, x1y2, x2y2) holds the points with fv >= fu.
        let [upper, lower] = self.cell_triangles(cx, cy);
        let [a, b, c] = if fv >= fu { upper? } else { lower? };
        let n = cross(sub(b, a), sub(c, a));
        if n[1].abs() < 1e-12 {
            return None;
        }
        Some(a[1] - (n[0] * (x - a[0]) + n[2] * (z - a[2])) / n[1])
    }

    /// Elements of the field within `reach` metres (horizontally, roughly) of world point `p`.
    fn elements(&self, p: V3, reach: f64, out: &mut Vec<Element>) {
        let local = self.frame.to_local_point(p);
        let cells = reach / self.spacing + 1.0;
        let below = self
            .height_at(local[0], local[2])
            .is_some_and(|h| local[1] < h);
        let first = out.len();
        for y in self.cells_around(local[2], 2, cells) {
            for x in self.cells_around(local[0], 0, cells) {
                for [a, b, c] in self.cell_triangles(x, y).into_iter().flatten() {
                    let mut normal = cross(sub(b, a), sub(c, a));
                    normal = scale(normal, 1.0 / norm(normal));
                    out.push(Element {
                        kind: Kind::Terrain,
                        normal: self.frame.to_world_dir(normal),
                        distance: point_triangle_distance(local, a, b, c),
                    });
                }
            }
        }
        if below {
            flip_inside(&mut out[first..]);
        }
    }

    /// The largest difference between a decoded sample height and the height it was made from.
    pub fn worst_decoding_error(&self) -> f64 {
        self.positions
            .iter()
            .zip(&self.intended)
            .map(|(decoded, &h)| decoded.map_or(f64::INFINITY, |p| (p[1] - h).abs()))
            .fold(0.0, f64::max)
    }

    /// The quantisation error Jolt may introduce: one 16-bit step of the whole height range for
    /// the block bounds, plus half a step of the block's own range at the field's bits per
    /// sample (`HeightFieldShape.cpp`), plus `f32` rounding of the stored positions.
    pub fn quantisation_bound(&self) -> f64 {
        let (min, max) = self
            .intended
            .iter()
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), &h| {
                (lo.min(h), hi.max(h))
            });
        let range = max - min;
        let step16 = range / 65535.0;
        let n = self.samples;
        let mut block_range: f64 = 0.0;
        for by in (0..n - 1).step_by(2) {
            for bx in (0..n - 1).step_by(2) {
                let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
                for y in by..(by + 3).min(n) {
                    for x in bx..(bx + 3).min(n) {
                        lo = lo.min(self.intended[y * n + x]);
                        hi = hi.max(self.intended[y * n + x]);
                    }
                }
                block_range = block_range.max(hi - lo + 2.0 * step16);
            }
        }
        let magnitude = min.abs().max(max.abs());
        step16
            + block_range / (f64::from(1_u32 << self.bits) - 1.0) / 2.0
            + 4.0 * magnitude * f64::from(f32::EPSILON)
    }
}

/// A box in world space.
#[derive(Clone, Copy, Debug)]
pub struct Block {
    pub frame: Frame,
    pub half: V3,
    pub convex_radius: f64,
}

impl Block {
    /// The six faces of the inner box seen from `p`, grown by the convex radius.
    fn elements(&self, p: V3, out: &mut Vec<Element>) {
        let local = self.frame.to_local_point(p);
        let r = self.convex_radius;
        let inner = self.half.map(|h| h - r);
        let inside = (0..3).all(|i| local[i].abs() <= inner[i]);
        let first = out.len();
        for axis in 0..3 {
            for sign in [-1.0, 1.0] {
                let mut normal = [0.0; 3];
                normal[axis] = sign;
                let (j, k) = ((axis + 1) % 3, (axis + 2) % 3);
                let mut closest = local;
                closest[axis] = sign * inner[axis];
                closest[j] = local[j].clamp(-inner[j], inner[j]);
                closest[k] = local[k].clamp(-inner[k], inner[k]);
                out.push(Element {
                    kind: Kind::Box,
                    normal: self.frame.to_world_dir(normal),
                    distance: norm(sub(local, closest)),
                });
            }
        }
        if inside {
            flip_inside(&mut out[first..]);
        }
        for element in &mut out[first..] {
            element.distance -= r;
        }
    }

    /// Distance from the capsule segment `[a, b]` to this box's surface, sampled at 41 points,
    /// metres; negative when the segment is inside.
    pub fn segment_distance(&self, a: V3, b: V3) -> f64 {
        let mut elements = Vec::new();
        let mut best = f64::INFINITY;
        for i in 0..=40 {
            let p = add(a, scale(sub(b, a), f64::from(i) / 40.0));
            elements.clear();
            self.elements(p, &mut elements);
            let d = elements
                .iter()
                .map(|e| e.distance)
                .fold(f64::INFINITY, f64::min);
            best = best.min(d);
        }
        best
    }
}

/// Everything solid in a scene.
#[derive(Clone, Debug, Default)]
pub struct Surface {
    pub fields: Vec<Field>,
    pub blocks: Vec<Block>,
}

impl Surface {
    /// The elements around world point `p` (heightfield cells within about one metre, every
    /// box face), in a fixed order.
    pub fn elements(&self, p: V3) -> Vec<Element> {
        let mut out = Vec::new();
        for field in &self.fields {
            field.elements(p, 1.0, &mut out);
        }
        for block in &self.blocks {
            block.elements(p, &mut out);
        }
        out
    }

    /// The signed distance from `p` to the closest element, if any element is near.
    pub fn distance(&self, p: V3) -> Option<f64> {
        self.elements(p)
            .iter()
            .map(|e| e.distance)
            .min_by(f64::total_cmp)
    }

    /// The elements within `SUPPORT_BAND` of the closest one, closest first.
    pub fn support_set(&self, p: V3) -> Vec<Element> {
        let mut elements = self.elements(p);
        elements.sort_by(|a, b| a.distance.total_cmp(&b.distance));
        let Some(first) = elements.first().map(|e| e.distance) else {
            return elements;
        };
        elements.retain(|e| e.distance <= first + SUPPORT_BAND);
        elements
    }
}

/// Turns the unsigned distances of a point inside a solid into signed ones: the nearest element
/// gets minus its distance, every other one stays further by as much as it was.
fn flip_inside(elements: &mut [Element]) {
    let nearest = elements
        .iter()
        .map(|e| e.distance)
        .fold(f64::INFINITY, f64::min);
    for element in elements {
        element.distance -= 2.0 * nearest;
    }
}

/// Distance from `p` to triangle `abc` (closest point by Voronoi regions, Ericson 5.1.5).
pub fn point_triangle_distance(p: V3, a: V3, b: V3, c: V3) -> f64 {
    let (ab, ac, ap) = (sub(b, a), sub(c, a), sub(p, a));
    let (d1, d2) = (dot(ab, ap), dot(ac, ap));
    if d1 <= 0.0 && d2 <= 0.0 {
        return norm(ap);
    }
    let bp = sub(p, b);
    let (d3, d4) = (dot(ab, bp), dot(ac, bp));
    if d3 >= 0.0 && d4 <= d3 {
        return norm(bp);
    }
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return norm(sub(p, add(a, scale(ab, v))));
    }
    let cp = sub(p, c);
    let (d5, d6) = (dot(ab, cp), dot(ac, cp));
    if d6 >= 0.0 && d5 <= d6 {
        return norm(cp);
    }
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return norm(sub(p, add(a, scale(ac, w))));
    }
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return norm(sub(p, add(b, scale(sub(c, b), w))));
    }
    let denom = 1.0 / (va + vb + vc);
    let (v, w) = (vb * denom, vc * denom);
    norm(sub(p, add(a, add(scale(ab, v), scale(ac, w)))))
}
