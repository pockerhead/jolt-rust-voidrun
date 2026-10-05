use super::*;
use crate::limits::{
    MAX_ANGULAR_VELOCITY, MAX_LINEAR_VELOCITY, MAX_MASS, MAX_SHAPE_EXTENT, MIN_MASS,
};

/// A unit cube of 1 kg, half under water, at rest, under Earth gravity at 60 Hz.
fn cube() -> BuoyancyInputs {
    BuoyancyInputs {
        buoyancy: 1.0,
        linear_drag: 0.5,
        angular_drag: 0.01,
        fluid_velocity: Vec3::ZERO,
        gravity: Vec3::new(0.0, -9.81, 0.0),
        gravity_factor: 1.0,
        delta_time: 1.0 / 60.0,
        total_volume: 1.0,
        submerged_volume: 0.5,
        center_of_buoyancy: Vec3::new(0.0, -0.25, 0.0),
        inverse_mass: 1.0,
        largest_inverse_inertia: 6.0,
        linear_velocity: Vec3::ZERO,
        angular_velocity: Vec3::ZERO,
        bounds_size: Vec3::new(1.0, 1.0, 1.0),
    }
}

/// The largest value whose outward-rounded bound still passes `within(_, headroom)`, and the
/// smallest that does not, both scaled by `per_unit` (the rest of the chain).
fn around(headroom: f64, per_unit: f64) -> (f32, f32) {
    let edge = headroom / (1.0 + 64.0 * F32_UNIT_ROUNDOFF) / per_unit;
    (
        (edge * (1.0 - 1.0e-6)) as f32,
        (edge * (1.0 + 1.0e-6)) as f32,
    )
}

/// Asserts that `inputs` with `low` pass `rule` (they may break a later one) and with `high`
/// break it.
fn assert_boundary(
    rule: BuoyancyRule,
    set: impl Fn(&mut BuoyancyInputs, f32),
    (low, high): (f32, f32),
) {
    let mut inputs = cube();
    set(&mut inputs, low);
    assert_ne!(check(&inputs), Err(rule), "{rule:?} at {low}");
    set(&mut inputs, high);
    assert_eq!(check(&inputs), Err(rule), "{rule:?} at {high}");
}

#[test]
fn ordinary_floating_is_accepted() {
    assert_eq!(check(&cube()), Ok(()));
    let mut moving = cube();
    moving.linear_velocity = Vec3::new(3.0, -2.0, 0.5);
    moving.angular_velocity = Vec3::new(0.0, 5.0, 1.0);
    moving.fluid_velocity = Vec3::new(1.0, 0.0, 0.0);
    moving.buoyancy = 3.0;
    assert_eq!(check(&moving), Ok(()));
}

#[test]
fn the_reviewed_counterexamples_are_refused() {
    // A 20 m box at the mass bound with gravity factor 0: Jolt's `-ρ · Vs · 0` is `inf · 0`.
    let heavy = BuoyancyInputs {
        buoyancy: 7.0e34,
        total_volume: 8000.0,
        submerged_volume: 8000.0,
        inverse_mass: 1.0e-6,
        gravity_factor: 0.0,
        linear_drag: 0.0,
        bounds_size: Vec3::new(20.0, 20.0, 20.0),
        center_of_buoyancy: Vec3::ZERO,
        ..cube()
    };
    assert_eq!(check(&heavy), Err(BuoyancyRule::Density));
    // The same body at rest in still water with a large drag: Jolt's area is 0.
    let resting = BuoyancyInputs {
        linear_drag: 100.0,
        ..heavy
    };
    assert_eq!(check(&resting), Err(BuoyancyRule::Density));
}

#[test]
fn each_chain_has_its_boundary() {
    let h = F32_PRODUCT_HEADROOM;
    let h2 = SQUARED_HEADROOM;
    // ρ = b / (V / m) with a 1e6 kg body of 1 m³.
    assert_boundary(
        BuoyancyRule::Density,
        |i, b| {
            i.inverse_mass = 1.0e-6;
            i.buoyancy = b;
        },
        around(h, 1.0e6),
    );
    // The buoyant impulse of a neutral 1e6 kg body under a huge gravity; the velocity change
    // rule would refuse far earlier, so it is out of the way here only for this chain.
    assert_boundary(
        BuoyancyRule::BuoyantImpulse,
        |i, g| {
            i.inverse_mass = 1.0e-6;
            i.submerged_volume = 1.0;
            i.delta_time = 1.0;
            i.gravity = Vec3::new(0.0, -g, 0.0);
        },
        around(h, 1.0e6),
    );
    // A spinning body with its centre of buoyancy 1 m off: `|ω| · |r|` is squared.
    assert_boundary(
        BuoyancyRule::RelativeVelocity,
        |i, w| {
            i.center_of_buoyancy = Vec3::new(1.0, 0.0, 0.0);
            i.angular_velocity = Vec3::new(0.0, w, 0.0);
        },
        around(h2, 1.0),
    );
    // A huge bounding box moving at 1e12 m/s.
    assert_boundary(
        BuoyancyRule::Area,
        |i, side| {
            i.linear_velocity = Vec3::new(1.0e12, 0.0, 0.0);
            i.bounds_size = Vec3::new(side, side, side);
            i.linear_drag = 0.0;
        },
        {
            let (low, high) = around(h, 1.0e12 * 3.0_f64.sqrt());
            (
                f64::from(low).sqrt() as f32,
                f64::from(high).sqrt() as f32 * (1.0 + 1.0e-6),
            )
        },
    );
    let dt = f64::from(cube().delta_time);
    // Quadratic drag at 1e6 m/s relative velocity: its velocity change
    // `0.5 · b / V · Cd · ‖q‖ · dt · ‖vrel‖²` is squared.
    assert_boundary(
        BuoyancyRule::Drag,
        |i, drag| {
            i.fluid_velocity = Vec3::new(1.0e6, 0.0, 0.0);
            i.linear_drag = drag;
        },
        around(h2, 0.5 * 3.0_f64.sqrt() * dt * 1.0e12),
    );
    // Angular drag on a 1e6 kg body whose inverse inertia is 6, spinning at 1 rad/s: its change
    // `λ · Cda · Vs / V · dt · l² / invM · ‖ω‖` is squared.
    assert_boundary(
        BuoyancyRule::AngularDrag,
        |i, drag| {
            i.inverse_mass = 1.0e-6;
            i.angular_velocity = Vec3::new(0.0, 1.0, 0.0);
            i.angular_drag = drag;
        },
        around(h2, 6.0 * 0.5 * dt * 1.0e6),
    );
    // A light body whose inverse inertia turns the buoyant impulse `b · Vs / V · |g| · dt / invM`
    // at a 1 m lever.
    assert_boundary(
        BuoyancyRule::Lever,
        |i, inverse_inertia| {
            i.center_of_buoyancy = Vec3::new(1.0, 0.0, 0.0);
            i.largest_inverse_inertia = inverse_inertia;
        },
        around(h2, 0.5 * f64::from(9.81_f32) * dt),
    );
    // A spin close to the squared headroom, with no lever to speak of.
    assert_boundary(
        BuoyancyRule::NewVelocity,
        |i, w| {
            i.center_of_buoyancy = Vec3::new(1.0e-20, 0.0, 0.0);
            i.largest_inverse_inertia = 0.5;
            i.angular_velocity = Vec3::new(0.0, w, 0.0);
            i.angular_drag = 0.0;
        },
        around(h2, 2.0),
    );
    // The buoyant velocity change `b · Vs / V · |gf| · |g| · dt`.
    assert_boundary(
        BuoyancyRule::BuoyantVelocityChange,
        |i, b| {
            i.submerged_volume = 1.0;
            i.delta_time = 1.0;
            i.gravity = Vec3::new(0.0, -10.0, 0.0);
            i.buoyancy = b;
        },
        around(f64::from(MAX_VELOCITY_CHANGE), 10.0),
    );
}

#[test]
fn both_signs_of_the_gravity_factor_are_bounded_alike() {
    for gravity_factor in [1000.0, -1000.0] {
        let mut inputs = cube();
        inputs.gravity_factor = gravity_factor;
        // 12 · 0.5 · 1000 · 9.81 / 60 = 981 m/s; 13 gives 1063.
        inputs.buoyancy = 12.0;
        assert_eq!(check(&inputs), Ok(()), "{gravity_factor}");
        inputs.buoyancy = 13.0;
        assert_eq!(
            check(&inputs),
            Err(BuoyancyRule::BuoyantVelocityChange),
            "{gravity_factor}"
        );
    }
}

#[test]
fn zero_factors_do_not_hide_large_ones() {
    let mut inputs = cube();
    inputs.gravity = Vec3::ZERO;
    inputs.buoyancy = 1.0e37;
    assert_eq!(check(&inputs), Err(BuoyancyRule::Density));
    let mut inputs = cube();
    inputs.gravity_factor = 0.0;
    inputs.gravity = Vec3::new(0.0, -2.0e37, 0.0);
    assert_eq!(check(&inputs), Err(BuoyancyRule::BuoyantImpulse));
}

#[test]
fn nan_in_any_input_is_refused() {
    let setters: [fn(&mut BuoyancyInputs); 15] = [
        |i| i.buoyancy = f32::NAN,
        |i| i.linear_drag = f32::NAN,
        |i| i.angular_drag = f32::NAN,
        |i| i.fluid_velocity.x = f32::NAN,
        |i| i.gravity.z = f32::NAN,
        |i| i.gravity_factor = f32::NAN,
        |i| i.delta_time = f32::NAN,
        |i| i.total_volume = f32::NAN,
        |i| i.submerged_volume = f32::NAN,
        |i| i.center_of_buoyancy.y = f32::NAN,
        |i| i.inverse_mass = f32::NAN,
        |i| i.largest_inverse_inertia = f32::NAN,
        |i| i.linear_velocity.z = f32::NAN,
        |i| i.angular_velocity.x = f32::NAN,
        |i| i.bounds_size.y = f32::NAN,
    ];
    for (index, set) in setters.iter().enumerate() {
        let mut inputs = cube();
        set(&mut inputs);
        assert!(check(&inputs).is_err(), "input {index}");
    }
}

/// SplitMix64, for seeded inputs.
struct SplitMix64(u64);

impl SplitMix64 {
    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn unit(&mut self) -> f64 {
        (self.next() >> 11) as f64 / (1_u64 << 53) as f64
    }

    /// A magnitude spread log-uniformly over `10^low..10^high`, but over `10^low..10^extreme`
    /// one time in eight and 0 one time in sixteen.
    fn magnitude(&mut self, low: f64, high: f64, extreme: f64) -> f32 {
        let top = if self.next().is_multiple_of(8) {
            extreme
        } else {
            high
        };
        if self.next().is_multiple_of(16) {
            0.0
        } else {
            10_f64.powf(low + (top - low) * self.unit()) as f32
        }
    }

    fn signed(&mut self, low: f64, high: f64, extreme: f64) -> f32 {
        let value = self.magnitude(low, high, extreme);
        if self.next().is_multiple_of(2) {
            value
        } else {
            -value
        }
    }

    fn vector(&mut self, low: f64, high: f64, extreme: f64) -> Vec3 {
        Vec3::new(
            self.signed(low, high, extreme),
            self.signed(low, high, extreme),
            self.signed(low, high, extreme),
        )
    }
}

/// How the replay associates Jolt's products.
#[derive(Clone, Copy, Debug)]
enum Order {
    /// Left to right, as written in `Body.cpp`.
    Source,
    /// Right to left.
    Reversed,
    /// Fused multiply-adds in dot and cross products, right to left elsewhere.
    Fused,
}

fn dot(a: Vec3, b: Vec3, order: Order) -> f32 {
    match order {
        Order::Fused => a.x.mul_add(b.x, a.y.mul_add(b.y, a.z * b.z)),
        _ => a.x * b.x + a.y * b.y + a.z * b.z,
    }
}

fn cross(a: Vec3, b: Vec3, order: Order) -> Vec3 {
    match order {
        Order::Fused => Vec3::new(
            a.y.mul_add(b.z, -(a.z * b.y)),
            a.z.mul_add(b.x, -(a.x * b.z)),
            a.x.mul_add(b.y, -(a.y * b.x)),
        ),
        _ => Vec3::new(
            a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x,
        ),
    }
}

fn scale(v: Vec3, s: f32) -> Vec3 {
    Vec3::new(v.x * s, v.y * s, v.z * s)
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x + b.x, a.y + b.y, a.z + b.z)
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x - b.x, a.y - b.y, a.z - b.z)
}

/// A product of scalars in `order`.
fn product(factors: &[f32], order: Order) -> f32 {
    match order {
        Order::Source => factors.iter().fold(1.0, |p, &f| p * f),
        _ => factors.iter().rev().fold(1.0, |p, &f| f * p),
    }
}

/// The columns of Jolt's world inverse inertia `R · diag(d) · Rᵀ` for the rotation matrix of
/// columns `r`, in `f32` as `MotionProperties::GetInverseInertiaForRotation` forms it.
fn world_inverse_inertia(r: [Vec3; 3], d: [f32; 3], order: Order) -> [Vec3; 3] {
    let row = |i: usize| {
        let c = |k: usize| [r[k].x, r[k].y, r[k].z][i];
        [c(0), c(1), c(2)]
    };
    let rows = [row(0), row(1), row(2)];
    let entry = |i: usize, j: usize| {
        let a = Vec3::new(rows[i][0], rows[i][1], rows[i][2]);
        let b = Vec3::new(d[0] * rows[j][0], d[1] * rows[j][1], d[2] * rows[j][2]);
        dot(a, b, order)
    };
    [0, 1, 2].map(|j| Vec3::new(entry(0, j), entry(1, j), entry(2, j)))
}

/// `m · v` for a matrix of columns `m`, as Jolt's `Mat44 * Vec3` sums the columns.
fn transform(m: [Vec3; 3], v: Vec3, order: Order) -> Vec3 {
    match order {
        Order::Fused => {
            let fused = |a: f32, b: f32, c: f32| c.mul_add(v.z, b.mul_add(v.y, a * v.x));
            Vec3::new(
                fused(m[0].x, m[1].x, m[2].x),
                fused(m[0].y, m[1].y, m[2].y),
                fused(m[0].z, m[1].z, m[2].z),
            )
        }
        _ => add(add(scale(m[0], v.x), scale(m[1], v.y)), scale(m[2], v.z)),
    }
}

/// Jolt's volume overload of `Body::ApplyBuoyancyImpulse` in `f32` for an unrotated body whose
/// inverse inertia is the largest moment on every axis.
fn replay(i: &BuoyancyInputs, order: Order) -> Option<f32> {
    let lambda = i.largest_inverse_inertia;
    let diagonal = [
        Vec3::new(lambda, 0.0, 0.0),
        Vec3::new(0.0, lambda, 0.0),
        Vec3::new(0.0, 0.0, lambda),
    ];
    replay_with_inertia(i, diagonal, order)
}

/// Jolt's volume overload of `Body::ApplyBuoyancyImpulse` in `f32` with the world inverse
/// inertia of columns `inverse_inertia`, the intermediates it squares included: the largest
/// magnitude among them, or `None` when one is not finite.
fn replay_with_inertia(
    i: &BuoyancyInputs,
    inverse_inertia: [Vec3; 3],
    order: Order,
) -> Option<f32> {
    let mut values = Vec::new();
    let inverse_mass = i.inverse_mass;
    let density = i.buoyancy / (i.total_volume * inverse_mass);
    let factor = product(&[-density, i.submerged_volume, i.gravity_factor], order);
    let buoyant = match order {
        Order::Source => scale(scale(i.gravity, factor), i.delta_time),
        _ => scale(i.gravity, factor * i.delta_time),
    };
    let center_velocity = add(
        i.linear_velocity,
        cross(i.angular_velocity, i.center_of_buoyancy, order),
    );
    let relative = sub(i.fluid_velocity, center_velocity);
    let relative_sq = dot(relative, relative, order);
    let s = i.bounds_size;
    let facing = Vec3::new(s.y * s.z, s.z * s.x, s.x * s.y);
    let area = if relative_sq > 1.0e-12 {
        let abs = Vec3::new(relative.x.abs(), relative.y.abs(), relative.z.abs());
        dot(abs, facing, order) / relative_sq.sqrt()
    } else {
        0.0
    };
    let drag_factor = product(&[0.5, density, i.linear_drag, area, i.delta_time], order);
    let mut drag = scale(scale(relative, drag_factor), relative_sq.sqrt());
    let speed_sq = dot(i.linear_velocity, i.linear_velocity, order);
    let drag_change = scale(drag, inverse_mass);
    let drag_change_sq = dot(drag_change, drag_change, order);
    if drag_change_sq > speed_sq {
        drag = scale(drag, (speed_sq / drag_change_sq).sqrt());
    }
    let linear = add(i.linear_velocity, scale(add(drag, buoyant), inverse_mass));
    let width = (s.x + s.y + s.z) / 3.0;
    let angular_factor = match order {
        Order::Source => {
            -i.angular_drag * i.submerged_volume / i.total_volume * i.delta_time * (width * width)
                / inverse_mass
        }
        _ => {
            (width * width) / inverse_mass
                * i.delta_time
                * (i.submerged_volume / i.total_volume)
                * -i.angular_drag
        }
    };
    let mut angular_drag = transform(
        inverse_inertia,
        scale(i.angular_velocity, angular_factor),
        order,
    );
    let spin_sq = dot(i.angular_velocity, i.angular_velocity, order);
    let angular_drag_sq = dot(angular_drag, angular_drag, order);
    if angular_drag_sq > spin_sq {
        angular_drag = scale(angular_drag, (spin_sq / angular_drag_sq).sqrt());
    }
    let lever = transform(
        inverse_inertia,
        cross(i.center_of_buoyancy, add(buoyant, drag), order),
        order,
    );
    let angular = add(i.angular_velocity, add(angular_drag, lever));
    values.extend([
        density,
        relative_sq,
        area,
        drag_change_sq,
        angular_factor,
        angular_drag_sq,
        dot(linear, linear, order),
        dot(angular, angular, order),
    ]);
    for v in [
        buoyant,
        relative,
        drag,
        linear,
        angular_drag,
        lever,
        angular,
    ] {
        values.extend([v.x, v.y, v.z]);
    }
    values.iter().try_fold(0.0_f32, |largest, v| {
        v.is_finite().then(|| largest.max(v.abs()))
    })
}

/// Seeded inputs, mostly of everyday size and now and then over many orders of magnitude, each
/// inside the ranges the caller checks first (finite, volumes and inverse mass positive, step in `1e-6..=1`).
fn seeded_inputs(rng: &mut SplitMix64) -> BuoyancyInputs {
    let total_volume = rng.magnitude(-6.0, 3.0, 12.0).max(1.0e-12);
    BuoyancyInputs {
        buoyancy: rng.magnitude(-3.0, 1.0, 38.0),
        linear_drag: rng.magnitude(-3.0, 2.0, 38.0),
        angular_drag: rng.magnitude(-3.0, 2.0, 38.0),
        fluid_velocity: rng.vector(-3.0, 2.0, 20.0),
        gravity: rng.vector(-3.0, 2.0, 20.0),
        gravity_factor: rng.signed(-3.0, 3.0, 3.0),
        delta_time: 10_f64.powf(-6.0 * rng.unit()) as f32,
        total_volume,
        submerged_volume: total_volume * rng.unit() as f32,
        center_of_buoyancy: rng.vector(-6.0, 1.0, 20.0),
        inverse_mass: rng.magnitude(-6.0, 3.0, 12.0).max(1.0e-12),
        largest_inverse_inertia: rng.magnitude(-6.0, 6.0, 25.0),
        linear_velocity: rng.vector(-3.0, 2.0, 20.0),
        angular_velocity: rng.vector(-3.0, 2.0, 20.0),
        bounds_size: Vec3::new(
            rng.magnitude(-6.0, 1.0, 12.0),
            rng.magnitude(-6.0, 1.0, 12.0),
            rng.magnitude(-6.0, 1.0, 12.0),
        ),
    }
}

#[test]
fn accepted_inputs_stay_finite_in_any_association() {
    let mut rng = SplitMix64(0x5EED_B0A7);
    let (mut accepted, mut refused) = (0, 0);
    let mut largest = 0.0_f32;
    for case in 0..200_000 {
        let inputs = seeded_inputs(&mut rng);
        if check(&inputs).is_err() {
            refused += 1;
            continue;
        }
        accepted += 1;
        for order in [Order::Source, Order::Reversed, Order::Fused] {
            let value = replay(&inputs, order);
            assert!(value.is_some(), "case {case} {order:?}: {inputs:?}");
            largest = largest.max(value.unwrap_or_default());
        }
    }
    assert!(accepted > 1000 && refused > 1000, "{accepted} {refused}");
    // Accepted inputs reach far beyond everyday sizes.
    assert!(largest > 1.0e30, "{largest}");
}

/// The largest value of the input that `set` writes that `check` accepts with the rest of
/// `inputs`, which it accepts with `start`; every rule refuses `1e38` in any of these inputs.
fn largest_accepted(
    inputs: &BuoyancyInputs,
    start: f32,
    set: impl Fn(&mut BuoyancyInputs, f32),
) -> f32 {
    let accepts = |value: f64| {
        let mut probe = *inputs;
        set(&mut probe, value as f32);
        check(&probe).is_ok()
    };
    let (mut low, mut high) = (f64::from(start), 1.0e38);
    assert!(accepts(low) && !accepts(high));
    for _ in 0..80 {
        let middle = if low > 0.0 {
            (low * high).sqrt()
        } else {
            high / 2.0
        };
        if accepts(middle) {
            low = middle;
        } else {
            high = middle;
        }
    }
    low as f32
}

/// The columns of the rotation matrix of a seeded unit quaternion, in `f32`.
fn seeded_rotation(rng: &mut SplitMix64) -> [Vec3; 3] {
    let q = [0; 4].map(|_| rng.unit() * 2.0 - 1.0);
    let n = q.iter().map(|c| c * c).sum::<f64>().sqrt().max(1.0e-9);
    let [x, y, z, w] = q.map(|c| c / n);
    let column = |a: f64, b: f64, c: f64| Vec3::new(a as f32, b as f32, c as f32);
    [
        column(
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y + w * z),
            2.0 * (x * z - w * y),
        ),
        column(
            2.0 * (x * y - w * z),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z + w * x),
        ),
        column(
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        ),
    ]
}

#[test]
fn rotated_anisotropic_inertia_at_the_bound_stays_finite() {
    // Accepted inputs with the largest inverse inertia the rules allow, spread over three
    // principal axes (the largest on a seeded axis, the others smaller) and rotated: Jolt's
    // world matrix and its products with the angular drag and the lever stay finite.
    let mut rng = SplitMix64(0x1E7E_12A5);
    let (mut cases, mut largest) = (0, 0.0_f32);
    while cases < 4000 {
        let mut inputs = seeded_inputs(&mut rng);
        if check(&inputs).is_err() {
            continue;
        }
        cases += 1;
        let lambda = largest_accepted(&inputs, inputs.largest_inverse_inertia, |i, lambda| {
            i.largest_inverse_inertia = lambda;
        });
        inputs.largest_inverse_inertia = lambda;
        let inertias = seeded_inertia(&mut rng, lambda);
        for (order, inertia) in [Order::Source, Order::Reversed, Order::Fused]
            .into_iter()
            .zip(inertias)
        {
            let value = replay_with_inertia(&inputs, inertia, order);
            assert!(value.is_some(), "case {cases} {order:?}: {inputs:?}");
            largest = largest.max(value.unwrap_or_default());
        }
    }
    // The angular velocity's square reaches the squared headroom's scale.
    assert!(largest > 1.0e35, "{largest}");
}

/// Seeded rotated world inverse inertia with the largest principal moment `lambda` on a seeded
/// axis, in `order`.
fn seeded_inertia(rng: &mut SplitMix64, lambda: f32) -> [[Vec3; 3]; 3] {
    let mut moments = [0; 3].map(|_| lambda * rng.unit() as f32);
    moments[(rng.next() % 3) as usize] = lambda;
    let rotation = seeded_rotation(rng);
    [Order::Source, Order::Reversed, Order::Fused]
        .map(|order| world_inverse_inertia(rotation, moments, order))
}

#[test]
fn drag_coefficients_at_their_bound_stay_finite() {
    // Accepted inputs with the largest linear or angular drag coefficient the rules allow: the
    // lengths Jolt squares sit at the squared rules' bound, and every order stays finite.
    let mut rng = SplitMix64(0xD7A6_B0B5);
    let (mut cases, mut largest) = (0, 0.0_f32);
    while cases < 4000 {
        let mut inputs = seeded_inputs(&mut rng);
        if check(&inputs).is_err() {
            continue;
        }
        cases += 1;
        if cases % 2 == 0 {
            inputs.linear_drag = largest_accepted(&inputs, inputs.linear_drag, |i, drag| {
                i.linear_drag = drag;
            });
        } else {
            inputs.angular_drag = largest_accepted(&inputs, inputs.angular_drag, |i, drag| {
                i.angular_drag = drag;
            });
        }
        let inertias = seeded_inertia(&mut rng, inputs.largest_inverse_inertia);
        for (order, inertia) in [Order::Source, Order::Reversed, Order::Fused]
            .into_iter()
            .zip(inertias)
        {
            let value = replay_with_inertia(&inputs, inertia, order);
            assert!(value.is_some(), "case {cases} {order:?}: {inputs:?}");
            largest = largest.max(value.unwrap_or_default());
        }
    }
    // The squares of the drag changes reach the squared headroom's scale.
    assert!(largest > 1.0e35, "{largest}");
}

/// A box of half extent `a` and `mass` in `cube()`'s fluid (Jolt's default drags), half under
/// water, moving at `velocity` and spinning at `spin`: what Jolt gives the check.
fn half_submerged_box(a: f32, mass: f32, velocity: Vec3, spin: Vec3) -> BuoyancyInputs {
    let side = 2.0 * a;
    let volume = side * side * side;
    BuoyancyInputs {
        total_volume: volume,
        submerged_volume: 0.5 * volume,
        center_of_buoyancy: Vec3::new(0.0, -0.5 * a, 0.0),
        inverse_mass: 1.0 / mass,
        // A solid box's moment of inertia is `mass · side² / 6` about every axis.
        largest_inverse_inertia: 6.0 / (mass * side * side),
        linear_velocity: velocity,
        angular_velocity: spin,
        bounds_size: Vec3::new(side, side, side),
        ..cube()
    }
}

#[test]
fn large_calm_bodies_are_accepted() {
    // Every box within the shape extent and mass bounds, at rest or at both velocity bounds,
    // half under default water. Among them the boxes of half extent 152 m at the mass bound,
    // 376 m at 1e4 kg and 591 m at 1e3 kg, which a rule comparing whole factor chains with the
    // squared headroom would refuse although their angular drag change is at most 0.03 rad/s.
    let moving = (
        Vec3::new(MAX_LINEAR_VELOCITY, 0.0, 0.0),
        Vec3::new(0.0, 0.0, MAX_ANGULAR_VELOCITY),
    );
    let mut boxes = vec![(152.0, MAX_MASS), (376.0, 1.0e4), (591.0, 1.0e3)];
    for size in 0..=12 {
        for mass in 0..=9 {
            let a = 0.005 * (MAX_SHAPE_EXTENT / 0.005).powf(size as f32 / 12.0);
            let mass = MIN_MASS * (MAX_MASS / MIN_MASS).powf(mass as f32 / 9.0);
            boxes.push((a.min(MAX_SHAPE_EXTENT), mass.min(MAX_MASS)));
        }
    }
    for (a, mass) in boxes {
        for (velocity, spin) in [(Vec3::ZERO, Vec3::ZERO), moving] {
            let inputs = half_submerged_box(a, mass, velocity, spin);
            assert_eq!(check(&inputs), Ok(()), "{a} m, {mass} kg: {inputs:?}");
            for order in [Order::Source, Order::Reversed, Order::Fused] {
                assert!(
                    replay(&inputs, order).is_some(),
                    "{a} m, {mass} kg {order:?}"
                );
            }
        }
    }
}

#[test]
fn the_replay_overflows_without_the_rules() {
    // The counterexamples above, which the rules refuse, are NaN in Jolt's arithmetic.
    let heavy = BuoyancyInputs {
        buoyancy: 7.0e34,
        total_volume: 8000.0,
        submerged_volume: 8000.0,
        inverse_mass: 1.0e-6,
        gravity_factor: 0.0,
        linear_drag: 0.0,
        bounds_size: Vec3::new(20.0, 20.0, 20.0),
        center_of_buoyancy: Vec3::ZERO,
        ..cube()
    };
    assert_eq!(replay(&heavy, Order::Source), None);
}
