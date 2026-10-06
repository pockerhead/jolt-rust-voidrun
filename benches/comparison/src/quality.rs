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

/// The validity bounds, fixed before any published run. A run outside one is reported as
/// "quality below bound" next to its time.
pub mod bounds {
    /// Stacks: the highest dynamic body keeps at least this fraction of its start height.
    pub const MIN_HEIGHT_RATIO: f32 = 0.95;
    /// Stacks: deepest penetration of a body into the ground, metres.
    pub const MAX_GROUND_PENETRATION: f32 = 0.05;
    /// Stacks: 99th percentile of the dynamic bodies' speed at the last tick, m/s.
    pub const MAX_FINAL_SPEED_P99: f32 = 0.1;
    /// Balls: deepest overlap of two balls, metres.
    pub const MAX_BALL_OVERLAP: f32 = 0.05;
    /// Joints: anchor separation, metres.
    pub const MAX_ANCHOR_P99: f32 = 0.05;
    /// Joints: angle error, radians.
    pub const MAX_ANGLE_P99: f32 = 0.05;
    /// Sliders: distance beyond the limits, metres.
    pub const MAX_LIMIT_P99: f32 = 0.05;
    /// Some dynamic body must move faster than this during the run, m/s.
    pub const MIN_PEAK_SPEED: f32 = 0.1;
}

/// The quality of one validation run. Measures that do not apply to the scene are `None`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Quality {
    /// The first tick with a non-finite position, rotation or velocity.
    pub non_finite_tick: Option<usize>,
    /// Highest dynamic body above the ground at the end over the same at the start.
    pub height_ratio: Option<f32>,
    /// Deepest a body's lowest point went below the ground's top, over the run.
    pub ground_penetration: Option<f32>,
    /// Bodies whose centre ended more than 1 m below the ground's top.
    pub fallen: Option<usize>,
    /// Median and 99th percentile of the dynamic bodies' speed at the last tick.
    pub final_speed_p50: Option<f32>,
    pub final_speed_p99: Option<f32>,
    /// Deepest overlap of two balls over the run.
    pub ball_overlap: Option<f32>,
    /// Joint anchor separation: the largest, and the largest per-tick 99th percentile over the
    /// joints.
    pub anchor_max: Option<f32>,
    pub anchor_p99: Option<f32>,
    /// Joint angle error (fixed and slider: relative rotation; hinge: angle between the hinge
    /// axes), largest and largest per-tick 99th percentile.
    pub angle_max: Option<f32>,
    pub angle_p99: Option<f32>,
    /// Slider travel beyond the limits, largest and largest per-tick 99th percentile.
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
        let mut broken = Vec::new();
        let mut check = |name, bad: bool| {
            if bad {
                broken.push(name);
            }
        };
        check("non_finite", self.non_finite_tick.is_some());
        check(
            "height_ratio",
            self.height_ratio.is_some_and(|v| v < MIN_HEIGHT_RATIO),
        );
        check(
            "ground_penetration",
            self.ground_penetration
                .is_some_and(|v| v > MAX_GROUND_PENETRATION),
        );
        check("fallen", self.fallen.is_some_and(|v| v > 0));
        check(
            "final_speed_p99",
            self.final_speed_p99
                .is_some_and(|v| v > MAX_FINAL_SPEED_P99),
        );
        check(
            "ball_overlap",
            self.ball_overlap.is_some_and(|v| v > MAX_BALL_OVERLAP),
        );
        check(
            "anchor_p99",
            self.anchor_p99.is_some_and(|v| v > MAX_ANCHOR_P99),
        );
        check(
            "angle_p99",
            self.angle_p99.is_some_and(|v| v > MAX_ANGLE_P99),
        );
        check(
            "limit_p99",
            self.limit_p99.is_some_and(|v| v > MAX_LIMIT_P99),
        );
        broken
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
    start_height: Option<f32>,
    quality: Quality,
    joint_errors: JointErrors,
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

impl<'a> QualityTracker<'a> {
    pub fn new(spec: &'a SceneSpec, class: SceneClass) -> Self {
        let ground_top = spec.ground_top();
        let start_height = (class == SceneClass::Stack)
            .then(|| highest_dynamic(spec, spec.bodies.iter().map(|b| b.position[1])))
            .flatten()
            .zip(ground_top)
            .map(|(top, ground)| top - ground);
        let mut quality = Quality::default();
        match class {
            SceneClass::Stack => quality.ground_penetration = Some(0.0),
            SceneClass::Balls => quality.ball_overlap = Some(0.0),
            SceneClass::Joints => {
                quality.anchor_max = Some(0.0);
                quality.anchor_p99 = Some(0.0);
                let kinds = || spec.joints.iter().map(|j| j.kind);
                if kinds().any(|k| !matches!(k, JointKind::Spherical)) {
                    quality.angle_max = Some(0.0);
                    quality.angle_p99 = Some(0.0);
                }
                if kinds().any(|k| matches!(k, JointKind::Prismatic { .. })) {
                    quality.limit_max = Some(0.0);
                    quality.limit_p99 = Some(0.0);
                }
            }
        }
        Self {
            spec,
            class,
            ground_top,
            start_height,
            quality,
            joint_errors: JointErrors::default(),
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
            SceneClass::Stack => self.observe_ground(states),
            SceneClass::Balls => self.observe_balls(states),
            SceneClass::Joints => self.observe_joints(states),
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
        let penetration = self.quality.ground_penetration.get_or_insert(0.0);
        *penetration = penetration.max(deepest);
    }

    fn observe_balls(&mut self, states: &[BodyState]) {
        let overlap = max_ball_overlap(self.spec, states);
        let max = self.quality.ball_overlap.get_or_insert(0.0);
        *max = max.max(overlap);
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
        let q = &mut self.quality;
        fold_errors(&mut errors.anchor, &mut q.anchor_max, &mut q.anchor_p99);
        fold_errors(&mut errors.angle, &mut q.angle_max, &mut q.angle_p99);
        fold_errors(&mut errors.limit, &mut q.limit_max, &mut q.limit_p99);
    }

    /// The measures of the whole run; `awake_at_end` comes from the engine.
    pub fn finish(mut self, awake_at_end: usize) -> Quality {
        self.quality.awake_at_end = awake_at_end;
        if self.class == SceneClass::Stack {
            self.finish_stack();
        }
        self.quality
    }

    fn finish_stack(&mut self) {
        let states = &self.last;
        if let (Some(ground), Some(start)) = (self.ground_top, self.start_height) {
            let end = highest_dynamic(self.spec, states.iter().map(|s| s.position[1]));
            self.quality.height_ratio = end.map(|top| (top - ground) / start);
            let fallen = states
                .iter()
                .filter(|s| s.position[1] < ground - 1.0)
                .count();
            self.quality.fallen = Some(fallen);
        }
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

/// Folds one tick's errors into the run's largest value and largest per-tick 99th percentile.
fn fold_errors(errors: &mut [f32], max: &mut Option<f32>, p99: &mut Option<f32>) {
    if errors.is_empty() {
        return;
    }
    errors.sort_by(f32::total_cmp);
    let (Some(max), Some(p99)) = (max.as_mut(), p99.as_mut()) else {
        return;
    };
    *max = max.max(*errors.last().unwrap());
    *p99 = p99.max(percentile(errors, 99.0));
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
