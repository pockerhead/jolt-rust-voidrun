//! Bodies for constraint impulse tests, and every impulse readout of a constraint in a fixed
//! order.

use oxijolt::*;

/// Half extent of a [`cube`], metres.
pub const HALF: f32 = 0.25;

/// A small static box at `position`, the world end of a constraint.
pub fn anchor(world: &mut PhysicsWorld, position: RVec3) -> BodyId {
    let shape = Shape::new_box(Vec3::new(0.1, 0.1, 0.1)).unwrap();
    world
        .create_body(&shape, &BodySettings::new_static().position(position))
        .unwrap()
}

/// A dynamic cube of half extent [`HALF`] and `mass` kg centred at `centre`, without damping,
/// so a held cube carries exactly its load, and kept awake.
pub fn cube(world: &mut PhysicsWorld, centre: RVec3, mass: f32) -> BodyId {
    world
        .create_body(
            &cube_shape(),
            &cube_settings(centre, mass).allow_sleeping(false),
        )
        .unwrap()
}

/// A [`cube`] that may fall asleep.
pub fn sleepy_cube(world: &mut PhysicsWorld, centre: RVec3, mass: f32) -> BodyId {
    world
        .create_body(&cube_shape(), &cube_settings(centre, mass))
        .unwrap()
}

fn cube_shape() -> Shape {
    Shape::new_box(Vec3::new(HALF, HALF, HALF)).unwrap()
}

fn cube_settings(centre: RVec3, mass: f32) -> BodySettings {
    BodySettings::new_dynamic()
        .position(centre)
        .mass(mass)
        .linear_damping(0.0)
        .angular_damping(0.0)
}

/// Every `total_lambda*` readout of the constraint `id`, in the order its kind declares them;
/// vectors and arrays component by component.
pub fn readouts(world: &PhysicsWorld, id: AnyConstraintId) -> Vec<f32> {
    fn typed<K: ConstraintKind>(world: &PhysicsWorld, id: AnyConstraintId) -> ConstraintRef<'_, K> {
        world.constraint(id.downcast::<K>().unwrap()).unwrap()
    }
    let v = |v: Vec3| [v.x, v.y, v.z];
    let mut values = Vec::new();
    match id.kind() {
        ConstraintType::Fixed => {
            let c = typed::<FixedConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.extend(v(c.total_lambda_rotation()));
        }
        ConstraintType::Point => {
            let c = typed::<PointConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
        }
        ConstraintType::Distance => {
            let c = typed::<DistanceConstraint>(world, id);
            values.push(c.total_lambda_position());
        }
        ConstraintType::Hinge => {
            let c = typed::<HingeConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.extend(c.total_lambda_rotation());
            values.push(c.total_lambda_motor());
            values.push(c.total_lambda_rotation_limits());
        }
        ConstraintType::Slider => {
            let c = typed::<SliderConstraint>(world, id);
            values.extend(c.total_lambda_position());
            values.push(c.total_lambda_motor());
            values.push(c.total_lambda_position_limits());
            values.extend(v(c.total_lambda_rotation()));
        }
        ConstraintType::Cone => {
            let c = typed::<ConeConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.push(c.total_lambda_rotation());
        }
        ConstraintType::SwingTwist => {
            let c = typed::<SwingTwistConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.extend(c.total_lambda_limits());
            values.extend(v(c.total_lambda_motor()));
        }
        ConstraintType::SixDof => {
            let c = typed::<SixDofConstraint>(world, id);
            values.extend(v(c.total_lambda_position()));
            values.extend(v(c.total_lambda_rotation()));
            values.extend(v(c.total_lambda_motor_translation()));
            values.extend(v(c.total_lambda_motor_rotation()));
        }
        ConstraintType::Gear => values.push(typed::<GearConstraint>(world, id).total_lambda()),
        ConstraintType::RackAndPinion => {
            values.push(typed::<RackAndPinionConstraint>(world, id).total_lambda());
        }
        ConstraintType::Pulley => {
            values.push(typed::<PulleyConstraint>(world, id).total_lambda_position());
        }
        ConstraintType::Path => {
            let c = typed::<PathConstraint>(world, id);
            values.extend(c.total_lambda_position());
            values.push(c.total_lambda_position_limits());
            values.push(c.total_lambda_motor());
            values.extend(c.total_lambda_rotation_hinge());
            values.extend(v(c.total_lambda_rotation()));
        }
        kind => panic!("no readouts listed for {kind:?}"),
    }
    values
}

/// The bits of [`readouts`], for exact comparisons.
pub fn readout_bits(world: &PhysicsWorld, id: AnyConstraintId) -> Vec<u32> {
    readouts(world, id).into_iter().map(f32::to_bits).collect()
}
