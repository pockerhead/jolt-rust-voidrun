//! What a validation run measures every tick: a digest of every body's state, and how well the
//! engine kept the scene's rules (no tunnelling, stacks standing, joints holding).

use crate::engine::BodyState;
use crate::measure::percentile;
use crate::scene::{JointKind, Motion, SceneClass, SceneSpec, Shape, V3};

/// FNV-1a over the bits of every body's position, rotation, both velocities and sleep flag, in
/// the scene's body order.
pub fn digest(states: &[BodyState]) -> u64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    let mut hash = OFFSET;
    let mut eat = |bytes: [u8; 4]| {
        for byte in bytes {
            hash ^= u64::from(byte);
            hash = hash.wrapping_mul(PRIME);
        }
    };
    for state in states {
        let floats = state
            .position
            .iter()
            .chain(&state.rotation)
            .chain(&state.linear_velocity)
            .chain(&state.angular_velocity);
        for value in floats {
            eat(value.to_bits().to_le_bytes());
        }
        eat(u32::from(state.sleeping).to_le_bytes());
    }
    hash
}

/// The first tick (1-based) at which two digest streams differ, or the shorter length plus one
/// when one is a prefix of the other; `None` when they are equal.
pub fn first_difference(a: &[u64], b: &[u64]) -> Option<usize> {
    a.iter()
        .zip(b)
        .position(|(x, y)| x != y)
        .or_else(|| (a.len() != b.len()).then(|| a.len().min(b.len())))
        .map(|index| index + 1)
}

/// The validity bounds. They separate a simulation that broke (a stack that fell, a body under
/// the ground, a joint that came apart) from one that is merely soft, so measures that peak during
/// impacts are bounded at the last tick or averaged over ticks 61-600, and their maxima are
/// reported without a bound. A run outside one is reported as "below bound" next to its time.
pub mod bounds {
    /// Stacks: the highest dynamic body keeps at least this fraction of its height at tick 60,
    /// when the drop of the first second is over.
    pub const MIN_HEIGHT_RATIO: f32 = 0.95;
    /// Stacks and piles: deepest penetration of a body into the ground at the last tick, metres.
    pub const MAX_GROUND_PENETRATION: f32 = 0.05;
    /// Stacks: 99th percentile of the dynamic bodies' speed at the last tick, m/s.
    pub const MAX_FINAL_SPEED_P99: f32 = 0.1;
    /// Balls: deepest overlap of two balls at the last tick, metres.
    pub const MAX_BALL_OVERLAP: f32 = 0.05;
    /// Joints: anchor separation, metres (per-tick 99th percentile over the joints, averaged over
    /// ticks 61-600).
    pub const MAX_ANCHOR_P99: f32 = 0.1;
    /// Joints: angle error, radians (the same statistic).
    pub const MAX_ANGLE_P99: f32 = 0.1;
    /// Sliders: distance beyond the limits, metres (the same statistic).
    pub const MAX_LIMIT_P99: f32 = 0.1;
    /// Some dynamic body must move faster than this during the run, m/s.
    pub const MIN_PEAK_SPEED: f32 = 0.1;
}

/// The first tick of the warm window, after the drop and the first impacts.
const WARM_FROM: usize = 61;

/// The quality of one validation run. Measures that do not apply to the scene are `None`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Quality {
    /// The first tick with a non-finite position, rotation or velocity.
    pub non_finite_tick: Option<usize>,
    /// Highest dynamic body above the ground at the last tick over the same at tick 60 (stacks).
    pub height_ratio: Option<f32>,
    /// Deepest a body's lowest point is below the ground's top at the last tick.
    pub ground_penetration: Option<f32>,
    /// The same, deepest over the run, impacts included; reported, not bounded.
    pub ground_penetration_max: Option<f32>,
    /// Bodies whose centre ended more than 1 m below the ground's top.
    pub fallen: Option<usize>,
    /// Median and 99th percentile of the dynamic bodies' speed at the last tick.
    pub final_speed_p50: Option<f32>,
    pub final_speed_p99: Option<f32>,
    /// Whether the final speed is bounded: for stacks, not for piles.
    pub final_speed_bounded: bool,
    /// Deepest overlap of two balls at the last tick.
    pub ball_overlap: Option<f32>,
    /// The same, deepest over the run; reported, not bounded.
    pub ball_overlap_max: Option<f32>,
    /// Joint anchor separation: the largest over the run, and the per-tick 99th percentile over
    /// the joints averaged over ticks 61-600 (over every tick in shorter runs).
    pub anchor_max: Option<f32>,
    pub anchor_p99: Option<f32>,
    /// Joint angle error (fixed and slider: relative rotation; hinge: angle between the hinge
    /// axes), the same two statistics.
    pub angle_max: Option<f32>,
    pub angle_p99: Option<f32>,
    /// Slider travel beyond the limits, the same two statistics.
    pub limit_max: Option<f32>,
    pub limit_p99: Option<f32>,
    /// Fastest dynamic body over the run, m/s.
    pub peak_speed: f32,
    /// Bodies awake at the last tick.
    pub awake_at_end: usize,
}

impl Quality {
    /// The bounds this run broke, by name; empty when it is within all of them.
    pub fn violations(&self) -> Vec<&'static str> {
        use bounds::*;
        let above = |value: Option<f32>, bound: f32| value.is_some_and(|v| v > bound);
        let checks = [
            ("non_finite", self.non_finite_tick.is_some()),
            (
                "height_ratio",
                self.height_ratio.is_some_and(|v| v < MIN_HEIGHT_RATIO),
            ),
            (
                "ground_penetration",
                above(self.ground_penetration, MAX_GROUND_PENETRATION),
            ),
            ("fallen", self.fallen.is_some_and(|v| v > 0)),
            (
                "final_speed_p99",
                self.final_speed_bounded && above(self.final_speed_p99, MAX_FINAL_SPEED_P99),
            ),
            ("ball_overlap", above(self.ball_overlap, MAX_BALL_OVERLAP)),
            ("anchor_p99", above(self.anchor_p99, MAX_ANCHOR_P99)),
            ("angle_p99", above(self.angle_p99, MAX_ANGLE_P99)),
            ("limit_p99", above(self.limit_p99, MAX_LIMIT_P99)),
        ];
        checks
            .into_iter()
            .filter(|&(_, bad)| bad)
            .map(|(name, _)| name)
            .collect()
    }
}

fn sub(a: V3, b: V3) -> V3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn add(a: V3, b: V3) -> V3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn dot(a: V3, b: V3) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn length(a: V3) -> f32 {
    dot(a, a).sqrt()
}

fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Rotates `v` by the unit quaternion `q = [x, y, z, w]`.
pub fn rotate(q: [f32; 4], v: V3) -> V3 {
    let u = [q[0], q[1], q[2]];
    let t = cross(u, v).map(|c| 2.0 * c);
    add(add(v, t.map(|c| q[3] * c)), cross(u, t))
}

/// Rotation angle of the unit quaternion `q`, in `[0, π]`.
fn rotation_angle(q: [f32; 4]) -> f32 {
    2.0 * q[3].abs().min(1.0).acos()
}

/// `conj(a) * b`: the rotation from frame `a` to frame `b`.
fn relative_rotation(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    let [ax, ay, az, aw] = [-a[0], -a[1], -a[2], a[3]];
    let [bx, by, bz, bw] = b;
    [
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ]
}

/// The lowest point of `shape` at `state`, as a height.
fn lowest_point(shape: &Shape, state: &BodyState) -> f32 {
    let y = state.position[1];
    match *shape {
        Shape::Ball { radius } => y - radius,
        Shape::Cuboid { half_extents } => {
            let reach: f32 = (0..3)
                .map(|axis| {
                    let mut basis = [0.0; 3];
                    basis[axis] = 1.0;
                    rotate(state.rotation, basis)[1].abs() * half_extents[axis]
                })
                .sum();
            y - reach
        }
        Shape::CapsuleY {
            half_height,
            radius,
        } => y - rotate(state.rotation, [0.0, 1.0, 0.0])[1].abs() * half_height - radius,
    }
}

/// Collects the quality measures of one run tick by tick.
pub struct QualityTracker<'a> {
    spec: &'a SceneSpec,
    class: SceneClass,
    ground_top: Option<f32>,
    /// Highest dynamic body above the ground at the start, then at tick 60.
    reference_height: Option<f32>,
    quality: Quality,
    joint_errors: JointErrors,
    /// Anchor, angle and limit errors.
    joint_stats: [JointStat; 3],
    last: Vec<BodyState>,
    tick: usize,
}

/// Per-tick joint errors, reused across ticks.
#[derive(Default)]
struct JointErrors {
    anchor: Vec<f32>,
    angle: Vec<f32>,
    limit: Vec<f32>,
}

/// One joint error's largest value, and its per-tick 99th percentiles summed over every tick and
/// over the warm window.
#[derive(Clone, Copy, Default)]
struct JointStat {
    max: f32,
    sum_all: f64,
    ticks_all: u32,
    sum_warm: f64,
    ticks_warm: u32,
}

impl JointStat {
    fn add(&mut self, errors: &mut [f32], tick: usize) {
        if errors.is_empty() {
            return;
        }
        errors.sort_by(f32::total_cmp);
        self.max = self.max.max(*errors.last().unwrap());
        let p99 = f64::from(percentile(errors, 99.0));
        self.sum_all += p99;
        self.ticks_all += 1;
        if tick >= WARM_FROM {
            self.sum_warm += p99;
            self.ticks_warm += 1;
        }
    }

    /// The largest value and the mean per-tick 99th percentile.
    fn finish(self) -> (f32, f32) {
        let (sum, ticks) = if self.ticks_warm > 0 {
            (self.sum_warm, self.ticks_warm)
        } else {
            (self.sum_all, self.ticks_all.max(1))
        };
        (self.max, (sum / f64::from(ticks)) as f32)
    }
}

impl<'a> QualityTracker<'a> {
    pub fn new(spec: &'a SceneSpec, class: SceneClass) -> Self {
        let ground_top = spec.ground_top();
        let reference_height = (class == SceneClass::Stack)
            .then(|| highest_dynamic(spec, spec.bodies.iter().map(|b| b.position[1])))
            .flatten()
            .zip(ground_top)
            .map(|(top, ground)| top - ground);
        Self {
            spec,
            class,
            ground_top,
            reference_height,
            quality: Quality::default(),
            joint_errors: JointErrors::default(),
            joint_stats: [JointStat::default(); 3],
            last: Vec::new(),
            tick: 0,
        }
    }

    /// Takes the states after one more tick.
    pub fn observe(&mut self, states: &[BodyState]) {
        self.tick += 1;
        if self.quality.non_finite_tick.is_none() && states.iter().any(|s| !is_finite(s)) {
            self.quality.non_finite_tick = Some(self.tick);
        }
        for (body, state) in self.spec.bodies.iter().zip(states) {
            if body.motion == Motion::Dynamic {
                let speed = length(state.linear_velocity);
                if speed > self.quality.peak_speed {
                    self.quality.peak_speed = speed;
                }
            }
        }
        match self.class {
            SceneClass::Stack | SceneClass::Pile => self.observe_ground(states),
            SceneClass::Balls => self.observe_balls(states),
            SceneClass::Joints => self.observe_joints(states),
        }
        if self.class == SceneClass::Stack && self.tick == WARM_FROM - 1 {
            let top = highest_dynamic(self.spec, states.iter().map(|s| s.position[1]));
            if let (Some(top), Some(ground)) = (top, self.ground_top) {
                self.reference_height = Some(top - ground);
            }
        }
        self.last.clear();
        self.last.extend_from_slice(states);
    }

    fn observe_ground(&mut self, states: &[BodyState]) {
        let Some(ground) = self.ground_top else {
            return;
        };
        let deepest = self
            .spec
            .bodies
            .iter()
            .zip(states)
            .filter(|(body, _)| body.motion == Motion::Dynamic)
            .map(|(body, state)| ground - lowest_point(&body.shape, state))
            .fold(0.0f32, f32::max);
        let max = self.quality.ground_penetration_max.get_or_insert(0.0);
        *max = max.max(deepest);
        self.quality.ground_penetration = Some(deepest);
    }

    fn observe_balls(&mut self, states: &[BodyState]) {
        let overlap = max_ball_overlap(self.spec, states);
        let max = self.quality.ball_overlap_max.get_or_insert(0.0);
        *max = max.max(overlap);
        self.quality.ball_overlap = Some(overlap);
    }

    fn observe_joints(&mut self, states: &[BodyState]) {
        let errors = &mut self.joint_errors;
        errors.anchor.clear();
        errors.angle.clear();
        errors.limit.clear();
        for joint in &self.spec.joints {
            let (s1, s2) = (&states[joint.body1], &states[joint.body2]);
            let a1 = add(s1.position, rotate(s1.rotation, joint.local_anchor1));
            let a2 = add(s2.position, rotate(s2.rotation, joint.local_anchor2));
            let d = sub(a2, a1);
            match joint.kind {
                JointKind::Spherical => errors.anchor.push(length(d)),
                JointKind::Fixed => {
                    errors.anchor.push(length(d));
                    errors
                        .angle
                        .push(rotation_angle(relative_rotation(s1.rotation, s2.rotation)));
                }
                JointKind::Revolute { axis } => {
                    errors.anchor.push(length(d));
                    let (x1, x2) = (rotate(s1.rotation, axis), rotate(s2.rotation, axis));
                    errors.angle.push(dot(x1, x2).clamp(-1.0, 1.0).acos());
                }
                JointKind::Prismatic { axis, limits } => {
                    let along = rotate(s1.rotation, axis);
                    let travel = dot(d, along);
                    errors
                        .anchor
                        .push(length(sub(d, along.map(|c| c * travel))));
                    errors
                        .angle
                        .push(rotation_angle(relative_rotation(s1.rotation, s2.rotation)));
                    errors
                        .limit
                        .push((limits[0] - travel).max(travel - limits[1]).max(0.0));
                }
            }
        }
        let [anchor, angle, limit] = &mut self.joint_stats;
        anchor.add(&mut errors.anchor, self.tick);
        angle.add(&mut errors.angle, self.tick);
        limit.add(&mut errors.limit, self.tick);
    }

    /// The measures of the whole run; `awake_at_end` comes from the engine.
    pub fn finish(mut self, awake_at_end: usize) -> Quality {
        self.quality.awake_at_end = awake_at_end;
        match self.class {
            SceneClass::Stack | SceneClass::Pile => self.finish_ground(),
            SceneClass::Joints => self.finish_joints(),
            SceneClass::Balls => {}
        }
        self.quality
    }

    fn finish_joints(&mut self) {
        let kinds = || self.spec.joints.iter().map(|j| j.kind);
        let has_angle = kinds().any(|k| !matches!(k, JointKind::Spherical));
        let has_limit = kinds().any(|k| matches!(k, JointKind::Prismatic { .. }));
        let [anchor, angle, limit] = self.joint_stats;
        let q = &mut self.quality;
        (q.anchor_max, q.anchor_p99) = split(Some(anchor.finish()));
        (q.angle_max, q.angle_p99) = split(has_angle.then(|| angle.finish()));
        (q.limit_max, q.limit_p99) = split(has_limit.then(|| limit.finish()));
    }

    fn finish_ground(&mut self) {
        let Some(ground) = self.ground_top else {
            return;
        };
        let states = &self.last;
        if self.class == SceneClass::Stack {
            let end = highest_dynamic(self.spec, states.iter().map(|s| s.position[1]));
            self.quality.height_ratio = end
                .zip(self.reference_height)
                .map(|(top, reference)| (top - ground) / reference);
            self.quality.final_speed_bounded = true;
        }
        let fallen = states
            .iter()
            .filter(|s| s.position[1] < ground - 1.0)
            .count();
        self.quality.fallen = Some(fallen);
        let mut speeds: Vec<f32> = self
            .spec
            .bodies
            .iter()
            .zip(states)
            .filter(|(body, _)| body.motion == Motion::Dynamic)
            .map(|(_, state)| length(state.linear_velocity))
            .collect();
        if !speeds.is_empty() {
            speeds.sort_by(f32::total_cmp);
            self.quality.final_speed_p50 = Some(percentile(&speeds, 50.0));
            self.quality.final_speed_p99 = Some(percentile(&speeds, 99.0));
        }
    }
}

/// A pair of statistics as two optional measures.
fn split(value: Option<(f32, f32)>) -> (Option<f32>, Option<f32>) {
    value.map_or((None, None), |(a, b)| (Some(a), Some(b)))
}

fn is_finite(state: &BodyState) -> bool {
    state
        .position
        .iter()
        .chain(&state.rotation)
        .chain(&state.linear_velocity)
        .chain(&state.angular_velocity)
        .all(|v| v.is_finite())
}

/// The largest height among the dynamic bodies' `heights`.
fn highest_dynamic(spec: &SceneSpec, heights: impl Iterator<Item = f32>) -> Option<f32> {
    spec.bodies
        .iter()
        .zip(heights)
        .filter(|(body, _)| body.motion == Motion::Dynamic)
        .map(|(_, y)| y)
        .reduce(f32::max)
}

/// The deepest overlap between two balls, found on a uniform grid of cells as wide as the
/// largest ball.
fn max_ball_overlap(spec: &SceneSpec, states: &[BodyState]) -> f32 {
    use std::collections::HashMap;
    let radius = |i: usize| match spec.bodies[i].shape {
        Shape::Ball { radius } => Some(radius),
        _ => None,
    };
    let cell = 2.0
        * (0..spec.bodies.len())
            .filter_map(radius)
            .fold(0.0f32, f32::max);
    if cell == 0.0 {
        return 0.0;
    }
    let key = |p: V3| p.map(|c| (c / cell).floor() as i64);
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
    for (i, state) in states.iter().enumerate() {
        if radius(i).is_some() && is_finite(state) {
            grid.entry(key(state.position)).or_default().push(i);
        }
    }
    let mut deepest = 0.0f32;
    for (i, state) in states.iter().enumerate() {
        let (Some(ri), true) = (radius(i), is_finite(state)) else {
            continue;
        };
        let [cx, cy, cz] = key(state.position);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let Some(others) = grid.get(&[cx + dx, cy + dy, cz + dz]) else {
                        continue;
                    };
                    for &j in others.iter().filter(|&&j| j > i) {
                        let rj = radius(j).unwrap();
                        let gap = length(sub(states[j].position, state.position)) - ri - rj;
                        deepest = deepest.max(-gap);
                    }
                }
            }
        }
    }
    deepest
}
