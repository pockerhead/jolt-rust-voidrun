//! The magnitudes the safe API accepts, and why.
//!
//! Jolt checks most magnitudes only with debug assertions (the `asserts` feature) and otherwise
//! computes with whatever it is given, so a finite but huge input can overflow Jolt's `f32`
//! arithmetic inside a step. joltphysics bounds the caller-given magnitudes the [audit](#audit)
//! table lists with the constants of this module; the same table names the inputs that are only
//! checked to be finite or ordered, and why. This module states which Jolt assertion paths the
//! bounds are derived for, which are only covered by tests, and which are not covered at all. It
//! does not claim that no accepted input can reach a Jolt assertion.
//!
//! # Frame and units
//! Positions are in metres in the world frame, which must keep every component of a caller-given
//! position within [`MAX_POSITION`]; [`PhysicsWorld::rebase`] moves the world into a new frame.
//! Shapes and local offsets are in metres around a shape's centre of mass, within
//! [`MAX_SHAPE_EXTENT`]. Velocities are in m/s and rad/s, accelerations in m/s² and rad/s², masses
//! in kg. [`MAX_LINEAR_VELOCITY`] and [`MAX_ANGULAR_VELOCITY`] are Jolt's own defaults; every
//! other constant is crate policy, chosen from Jolt's documented ranges ("Conventions and
//! Limits", "Big Worlds" in Jolt's `Docs/Architecture.md`) or from the arithmetic below.
//!
//! # Derived bounds
//! These hold with every input at its bound and a time step `dt <= 1` s
//! ([`PhysicsWorld::MAX_DELTA_TIME`]); numbers are rounded.
//! - **Velocities at creation.** Jolt asserts `Length() <= mMaxLinearVelocity` (and the angular
//!   counterpart) when it creates a body (`Body.cpp:424`, `MotionProperties.h:48`). The checks
//!   compare Jolt's own `Vec3::Length`, called through joltc, with Jolt's defaults; a test creates
//!   bodies on the bound in 64 directions, which the asserts leg runs.
//! - **Integration.** Each step Jolt adds gravity times the gravity factor and the accumulated
//!   force times the inverse mass to the velocity, then asserts that the squared speed is finite
//!   (`MotionProperties.inl:26-28`) before clamping it. Before the clamp
//!   `|v| <= 500 + (|gf|·|g| + |F|/m)·dt <= 500 + 1000·5e8 + 2·5e8`, about 5e11 m/s, where the
//!   factor 2 also covers a vehicle's gravity force on its chassis. Its square, about 2.5e23, is
//!   finite. The angular velocity is bounded the same way with [`MAX_ANGULAR_ACCELERATION`] and
//!   the largest principal inverse inertia.
//! - **Force and torque accumulation.** With `mass <= MAX_MASS` an accepted accumulated force is
//!   at most `MAX_ACCELERATION · MAX_MASS`, about 5e14 N, so Jolt's `f32` sums (`Body.h:183,191`)
//!   and `mInvMass * F` (`MotionProperties.inl:134`) cannot overflow. For a force at a point the
//!   lever must be finite in `f32` and every product `lever_i · force_j` at most 1e37, so Jolt's
//!   cross product (`Body.inl:127-131`) stays finite even when its two products cancel.
//! - **Soft body forces.** Jolt adds a soft body's accumulated force to every vertex as
//!   `F · w / N · dt` (`SoftBodyMotionProperties.cpp:334`), `w` the vertex's inverse mass and `N`
//!   the vertex count. The accumulated force may give the vertex of the largest inverse mass at
//!   most [`MAX_ACCELERATION`], and is at most `MAX_ACCELERATION · MAX_MASS` (5e14 N) whatever
//!   the inverse masses, as on a rigid body: a body whose vertices are all pinned (`w = 0`)
//!   cannot collect an unbounded force. Changing a vertex's inverse mass rechecks the force
//!   accumulated in the current step against the new inverse masses, so unpinning a vertex
//!   cannot release a force beyond the bound either.
//! - **Soft body pressure.** Before each solver sub-step of `dt` seconds Jolt computes the
//!   six-volume `V = Σ (x1 × x2) · x3` over the faces in `f32`, from the vertex positions about
//!   the body origin, and when `V > 0` adds `w · pressure · dt / V · ((x2 - x1) × (x3 - x1))` to
//!   the velocity of each vertex of every face (`SoftBodyMotionProperties.cpp:107-118,291-322`);
//!   nothing bounds `1 / V`. [`PhysicsWorld::create_soft_body`] accepts a pressure only when
//!   `pressure · A <= MAX_ACCELERATION / MAX_VERTEX_INVERSE_MASS · V_low`, where `V_low` is a lower bound of
//!   the six-volume Jolt computes before the first step and `A` the largest sum, over the faces
//!   of one vertex, of an upper bound of `|x2 - x1| · |x3 - x1|` in Jolt's arithmetic. So the
//!   faces must enclose a positive volume, wound counter-clockwise seen from outside, and no
//!   vertex, whatever its inverse mass (at most [`MAX_VERTEX_INVERSE_MASS`], also after
//!   [`SoftBodyMut::set_vertex_inverse_mass`](crate::SoftBodyMut::set_vertex_inverse_mass)),
//!   gains more than `MAX_ACCELERATION · dt` per sub-step from the pressure at the start
//!   geometry; the pressure coefficient and impulses stay finite, so a kinematic vertex gets
//!   `0 · finite`, not `0 · ∞`. `V_low` does not replay Jolt's `f32` operations, whose order
//!   and rounding depend on the build (SSE4.1 `dpps`, fused multiply-adds, the rotation Jolt
//!   bakes into the vertices); it is the six-volume in `f64` minus a bound of everything those
//!   operations can change, with `u = 2^-24` and `|p|` a vertex's distance from the body
//!   origin: a rotation within Jolt's normalization tolerance scales the six-volume by
//!   `1 ± 4e-5` and distances by at most `1 + 2e-5`, and its rounding moves a position by at
//!   most `15 u |p|`; one face's term `(x1 × x2) · x3` is off by at most
//!   `64 u |p1| |p2| |p3|`; each addition of the running sum rounds by at most `u` times the
//!   partial sum, which grows the error by at most `(1 - u)^-N` for `N` faces; underflow adds
//!   at most `1e-36` per face. An edge in `A` is bounded by
//!   `1.0001 · (|e| + 15 u (|pa| + |pb|))`. A tetrahedron with three 1 m edges at a right
//!   corner takes a pressure of up to about 9e4; the ball of radius 0.5 m in
//!   `pressure_at_the_bound_steps_finitely` takes [`MAX_SOFT_BODY_PRESSURE`].
//! - **Rigid body inertia.** Jolt decomposes the inertia tensor of every body that is not static
//!   with `EigenValueSymmetric` when it creates the body (`MotionProperties::SetMassProperties`,
//!   also for a ragdoll part in `Ragdoll::Stabilize`), before it checks whether the moments are
//!   near zero. A tensor that is not exactly diagonal comes from a compound child's rotation or
//!   position, or from an offset centre of mass. Its decomposition asserts as the soft body
//!   inertia's below does, and the same floor applies: [`PhysicsWorld::create_body`], the inner
//!   body of [`PhysicsWorld::create_character`](crate::PhysicsWorld::create_character), and the
//!   parts of [`RagdollSettings::new`](crate::RagdollSettings::new) and
//!   [`RagdollSettings::new_stabilized`](crate::RagdollSettings::new_stabilized) accept such a
//!   tensor only when `det I / (sum of the principal 2 × 2 minors)`, a lower bound of its smallest
//!   principal moment, is at least `1001 · 8 · u · |I|_F`, about 4.8e-4 of its Frobenius norm. The
//!   check reads the `f32` tensor Jolt computes itself (joltc's `JPH_Shape_GetMassProperties` and
//!   `JPH_MassProperties_ScaleToMass`), so it needs no error term for the tensor. `Stabilize`
//!   (`Ragdoll.cpp:133-183`) multiplies each part's tensor by its new mass over its old, one
//!   rounding per element, before it decomposes it, and rebuilds a parent's tensor from that
//!   decomposition with every moment raised to at least the smaller of twice its largest and
//!   its children's sum; neither moves the smallest moment down by more than a few `u |I|_F`,
//!   far inside the margin. An exactly diagonal tensor is decomposed exactly and only needs
//!   invertible moments or Jolt's near-zero fallback.
//!   In the asserts build, seeded thin boxes, capsules and cylinders in rotated compound children,
//!   pairs of such children, and offset centres of mass, with half lengths from 1 nm to 50 m and
//!   masses from 1 g to 1000 t, asserted for `λ_min / |I|_F` up to 8.2e-5 under the earlier rule
//!   (`det I / |I|_F² >= 1e-5 |I|_F`, skipped for a tensor below Jolt's near-zero limit), and none
//!   of the 5 271 accepted at or above 1e-4 did. Made thinner until this floor refuses them, 2 703
//!   such shapes were created and stepped without an assertion, and about 2 500 with the floor
//!   divided by 3; divided by 4 one of ten seeded runs asserted, divided by 6 all ten did, so the
//!   margin over Jolt's onset is 3 to 4, as for soft bodies. The floor refuses a square needle box
//!   in a rotated child when it is about 54 times longer than wide, a capsule or cylinder about 47
//!   times longer than its diameter, also where Jolt decomposed it without an assertion.
//!   In the asserts leg, `rigid_body_inertia_at_its_bound_decomposes` (bodies),
//!   `inner_body_inertia_at_its_bound_decomposes` (character inner bodies) and
//!   `stabilized_ragdoll_inertia_at_its_bound_decomposes` (a three-part chain whose masses
//!   `Stabilize` redistributes) create and step seeded rotated slender shapes at the thinnest
//!   thickness the floor accepts and check that one 1e-4 thinner is refused; each aborts with
//!   the floor divided by 6 and passes with it divided by 3.
//! - **Soft body inertia.** Jolt sums a soft body's inertia tensor about the body origin in
//!   `f32` from its vertices (`SoftBodyMotionProperties::CalculateMassAndInertia`) when the body
//!   is created and after every
//!   [`SoftBodyMut::set_vertex_inverse_mass`](crate::SoftBodyMut::set_vertex_inverse_mass), and
//!   decomposes it with `EigenValueSymmetric`, which asserts that every eigenvector `v` with
//!   eigenvalue `λ` satisfies `|M v − λ v|² <= 1e-6 · max(|M v|², λ²)` (`EigenValueSymmetric.h:88`).
//!   A body with a kinematic vertex skips all of this (infinite mass and inertia). Jacobi's
//!   residual `|M v − λ v|` is a multiple `k` of `u |M|_F`, `u = 2^-24`, whatever the
//!   eigenvalue, so the assertion holds for the smallest principal moment `λ_min` when
//!   `λ_min >= 1001 · k · u · |M|_F`. A scalar `f32` emulation of Jolt's decomposition
//!   measured `k` at most 4 over 42 000 tensors (random spectra and rotations, near-axis tilts,
//!   and tensors summed from random point clouds), so `k` is measured, not derived; the bound
//!   takes `k = 8`, a ratio `λ_min / |M|_F` of at least about 4.8e-4. In the asserts build,
//!   4 000 seeded clouds of point masses moved to the boundary of this rule with its floor
//!   divided by 3 (and 4 random rotations each) decomposed without an assertion, and with the
//!   floor divided by 4 Jolt asserted, so the margin over Jolt's measured onset is 3 to 4. The
//!   asserts leg creates bodies at the bound in `soft_body_inertia_at_its_bound_decomposes`
//!   (which aborts with the floor divided by 4). Jolt's second
//!   check, that the decomposition rebuilds each column of `M` to `1e-5` of its length
//!   (`MassProperties.cpp:56`), never fired in the emulation for tensors of point masses: it
//!   needs two moments much smaller than the third, which point masses cannot give (each
//!   moment is at most the sum of the other two).
//!   [`PhysicsWorld::create_soft_body`] and `set_vertex_inverse_mass` check
//!   it in `f64` with `λ_min` bounded from below by `det M / (sum of the principal 2 × 2
//!   minors)`, after subtracting a bound of how far Jolt's `f32` tensor can be from the `f64`
//!   one in Frobenius norm: `3 γ_{N+3} Σ m |p|²` for the `N`-term sums of terms of at most three
//!   roundings, and, when Jolt bakes a rotation into the vertices (computed with Jolt's
//!   rotation matrix in `f64`), `(1 + √3)(2 δ + δ²) Σ m |p|²` for its rounding of each position
//!   by at most `δ |p|`, `δ = 16 u` (the 15 u of the pressure bound with the rotation's slack).
//!   A tensor Jolt sums exactly diagonal (no rotation, every vertex on a coordinate axis) is
//!   decomposed exactly; it is accepted as a rigid body's is, when its moments are all above
//!   1e-30 or it is near zero. The rule refuses a body whose vertices lie far from its origin
//!   compared with their spread: an 11 × 11 cloth of 1 m with 1 kg vertices is accepted at
//!   11 m from its origin and refused from 12 m (Jolt asserted at 50 m), and a free straight
//!   line of vertices, whose smallest moment is zero. The rule does not know the shape, so it
//!   also refuses free bodies thinner than about 1/70 of their length (ribbons) or with a
//!   radius below about 1/130 of their length (tubes, ropes), which Jolt decomposes: centred
//!   on the origin, a 2.5 m ribbon 1, 2 or 3 cm wide and a 3 m tube of radius 1 or 2 cm were
//!   refused, and Jolt created and stepped them without an assert at baked rotations of 0,
//!   0.001 and 0.3 rad. A kinematic vertex skips the check.
//! - **Vehicle gravity.** Jolt adds `gravity / inverse_mass` to the chassis
//!   (`VehicleConstraint::OnStep`); the chassis is a dynamic body, so the force is at most
//!   `5e8 · 1e6`, about 5e14 N.
//! - **Character weight and push.** A character presses on what it stands on with the impulse
//!   `mass · |g| · dt` at the ground contact point (`CharacterVirtual.cpp:1474-1481`), so the
//!   impulse also turns the ground body. [`PhysicsWorld::update_character`] accepts at most
//!   [`MAX_WEIGHT_IMPULSE`], 1e9 N·s, which changes the linear velocity of a body of the
//!   smallest mass by at most 1e12 m/s and the angular velocity of a body whose principal
//!   inverse inertia is at most `√3 · 1e6` by at most about 6e18 rad/s; both squares are
//!   finite (see [`MAX_WEIGHT_IMPULSE`] for the derivation). Its push impulse is capped at
//!   `delta_velocity / inv_effective_mass` (`CharacterVirtual.cpp:795-811`), whose effective
//!   mass includes the body's rotation, so the velocity change at the contact is at most the
//!   relative normal speed whatever the strength.
//! - **Springs.** Jolt derives a stiffness `k` and damping `c` from every spring
//!   (`SpringPart.h:36-55,91-104`). Both stay at most [`MAX_SPRING_COEFFICIENT`]: in stiffness mode
//!   directly, in frequency mode through an upper bound of the effective mass. For a ragdoll joint
//!   that bound is computed at creation from the parts' masses and inertias, including Jolt's
//!   `Stabilize` (`Ragdoll.cpp:135-185`); see [`SpringSettings`](crate::SpringSettings). For a
//!   world constraint it is computed at creation from its two bodies: over the dynamic ones, the
//!   larger of the mass and the largest principal moment of inertia
//!   ([`PhysicsWorld::create_constraint`]); the constraint's spring setters use the same bound.
//!   For a wheel's suspension it is [`MAX_MASS`], since Jolt's suspension effective mass is at most the
//!   chassis mass (`VehicleConstraint.cpp:448-451`).
//! - **Anti-roll bars.** Jolt computes `stiffness · length difference · dt` for each bar
//!   (`VehicleConstraint.cpp:289-293`) and passes it as the bias `b` of the wheel's suspension
//!   constraint (`VehicleConstraint.cpp:508`), whose impulse is `-K⁻¹ (J v + b)`
//!   (`AxisConstraintPart.h:300-301`): a velocity term, scaled by an effective mass that is at
//!   most each body's own along the axis. With
//!   [`VehicleAntiRollBar::MAX_STIFFNESS`](crate::VehicleAntiRollBar::MAX_STIFFNESS) and wheel
//!   lengths at most [`MAX_SHAPE_EXTENT`], `b` is at most 5e14 m/s, so the velocity change along
//!   the suspension axis stays finite and squares finitely.
//! - **Restitution.** At most 1, so the restitution target speed is at most the approach speed.
//! - **Friction.** At most [`MAX_FRICTION`], so Jolt's combined friction `sqrt(f1 · f2)`
//!   (`ContactConstraintManager.h:554`) is finite and the friction impulse bound, combined friction
//!   times the normal impulse (`ContactConstraintManager.cpp:1714-1715`), is never `0 · ∞` = NaN.
//!   A cube sliding on a floor, both at `f32::MAX`, had NaN velocities within three 60 Hz steps.
//! - **Kinematic drive.** Jolt's `MoveKinematic` sets a velocity of `move / dt` without clamping
//!   it (`MotionProperties.inl:9-21`). [`RagdollMut::drive_to_pose_using_kinematics`](crate::RagdollMut::drive_to_pose_using_kinematics)
//!   computes every part's velocity with Jolt's own operations first and accepts the drive only
//!   when each stays within [`MAX_LINEAR_VELOCITY`] and [`MAX_ANGULAR_VELOCITY`], the bounds of
//!   every other velocity input.
//! - **Six-DOF translation limits.** Jolt corrects a violated limit by the distance beyond it
//!   times the effective mass (`SixDOFConstraint.cpp:380-410,780-790`); limits within
//!   [`MAX_SHAPE_EXTENT`] keep that finite. A limit of 1e30 m moved two parts to NaN positions in
//!   a few steps.
//! - **Contact constraint capacity.** [`WorldSettings::MAX_CONTACT_CONSTRAINTS`] stays below the
//!   count above which `ContactConstraintManager::Init` asserts; a native compile-time check pins
//!   it.
//!
//! # Covered by tests only
//! The asserts leg of CI runs every test with Jolt's assertions; these paths are exercised there
//! by scenes with inputs at their bounds, not derived:
//! - contact and constraint impulses the solver generates: spheres of [`MIN_MASS`] and
//!   [`MAX_MASS`] colliding head-on at the velocity bounds, restitution 1, friction at
//!   [`MAX_FRICTION`] including a speculative contact with zero normal impulse, motors at the
//!   spring bound, six-DOF translation limits at the extent bound between parts of both mass
//!   extremes, suspension springs and anti-roll bars at their bounds
//!   (`bodies_at_every_bound_step_finitely`, `friction_at_the_bound_keeps_contacts_finite`,
//!   `six_dof_translation_limits_at_the_bound_step_finitely`,
//!   `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely`,
//!   `motor_springs_at_the_coefficient_bound_drive_finitely`);
//! - impulses at a lever arm on the lightest bodies: the weight impulse of a character of
//!   [`MAX_MASS`] at [`MAX_WEIGHT_IMPULSE`] on the edge of a 6 cm cube of [`MIN_MASS`], the
//!   cube that turns fastest (`character_weight_impulse_at_a_lever_arm_is_bounded`), and a box
//!   of [`MIN_MASS`] pressed on its floor and pushed at the velocity bound by such a character
//!   (`character_at_its_bounds_pushes_the_lightest_body`);
//! - query arithmetic: rays, shape casts and collisions from frame corners
//!   (`query_inputs_are_bounded_by_the_frame`) and a sphere at the extent bound cast across
//!   the frame and collided with separations up to the extent bound
//!   (`queries_with_shapes_at_the_extent_bound_stay_finite`);
//! - world constraints: gears at both ends of `1..=`[`MAX_GEAR_RATIO`], racks and pinions and
//!   pulleys at both [`MAX_RATIO`] bounds
//!   between bodies of both mass extremes, one of them turning or sliding at the velocity bound
//!   (`ratios_at_the_bound_step_finitely`, `pulleys_at_the_ratio_bound_step_finitely`); motor and
//!   limit springs accepted at the effective-mass bound and rejected one representable frequency
//!   above it (`world_constraint_springs_are_bounded_by_the_bodies_effective_mass`,
//!   `slider_swing_twist_and_six_dof_motor_springs_are_bounded`,
//!   `path_motor_springs_and_friction_are_bounded`); targets at their bounds
//!   (`constraint_targets_are_bounded`, `slider_cone_swing_twist_and_six_dof_targets_are_bounded`,
//!   `path_inputs_are_bounded`); hinge, slider, swing-twist and path friction of `f32::MAX` on
//!   bodies moving at the velocity bounds (`constraint_friction_at_f32_max_steps_finitely`,
//!   `slider_and_swing_twist_friction_at_f32_max_steps_finitely`,
//!   `path_motor_springs_and_friction_are_bounded`); and a path segment at the margin of the
//!   segment check stepped end to end with every rotation constraint
//!   (`path_validation_accepts_its_boundary`); points that hold a dynamic body at
//!   [`MAX_LEVER_ARM_RATIO`]: a ball joint and a hinge on a static body, and a weld, a six-DOF
//!   joint with every axis fixed and a swing-twist joint with zero ranges on a static body or
//!   between two bodies, each holding a 1 g, 6 cm cube or a 1 kg, 1 m cube at the velocity
//!   bounds under gravity (`constraints_at_the_lever_arm_bound_step_finitely`).
//!
//! # Not covered
//! - State the simulation produces itself is not an input and is not checked again: a body Jolt
//!   carries out of the frame, the positions a rebase computes (only checked to be finite), or a
//!   character state restored with [`CharacterMut::restore_state`](crate::CharacterMut::restore_state),
//!   which can only come from [`CharacterRef::save_state`](crate::CharacterRef::save_state).
//! - Bodies with a principal inverse inertia above `√3 · 1e6`, such as a light needle-thin shape
//!   or a centre of mass far from the shape. [`PhysicsWorld::create_body`] bounds no inverse
//!   inertia from above: an exactly diagonal tensor (an unrotated shape, or a centre of mass
//!   moved along a principal axis) only needs finite inverse moments, and the rigid body
//!   inertia floor (see [Derived bounds](#derived-bounds)) bounds how badly conditioned any
//!   other tensor is, not how small it is. The boundary tests of that floor step slender
//!   bodies for two ticks without contacts or impulses, so impulses at a lever arm on such a
//!   body, including a character's weight impulse, are neither derived nor tested. The same holds
//!   for the suspension effective mass Jolt forms from a wheel's force point and the chassis's
//!   inverse inertia (`VehicleConstraint.cpp:448-451`).
//! - Constraints whose solver diverges for reasons the lever-arm ratio does not measure. In the
//!   `asserts` build Jolt then asserts that a squared velocity is finite
//!   (`MotionProperties.inl:28` or `:38`); in the release build the joint tears apart by metres
//!   and the state can become NaN: Jolt's velocity clamp scales a velocity whose squared length
//!   overflows to zero, turns an already infinite component into NaN (`inf * 0`) and lets NaN
//!   through. No impact is needed. Measured with one constraint to a static
//!   body under gravity unless stated otherwise:
//!   - a hinge whose pin lies off the least-inertia axis of a slender body, outside its cross
//!     section (an offset `d` of about twice the half thickness `t` or more). A 2 kg rod of 1 m
//!     by 2 cm hinged 5 cm off its axis (a lever-arm ratio of 43.5) asserted while swinging
//!     about the hinge at 10 rad/s; a 10 kg barrier arm of 3 m by 5 cm on a 5 cm bracket
//!     (ratio 12) asserted at 30 rad/s. In the release build both became NaN within a few
//!     hundred steps of such a swing (the step depends on the scene); a 1 m rod 1 cm thick with the pin 5 cm beside it
//!     along the hinge line asserted while falling from horizontal under gravity alone. On 1 m
//!     rods swung at up to 47 rad/s, `t` = 5 mm failed from `d` = 1 cm, `t` = 1 cm from 2 cm,
//!     `t` = 2 cm only at 10 cm, and `t` = 5 cm not up to 10 cm;
//!   - chains of hinges with non-parallel axes, also between cubes: of 100 seeded chains of 3 to
//!     10 cubes, 3 asserted at kicks of 15 m/s and 1.4 rad/s per body at lever-arm ratios of 10
//!     to 100, and 18 at ratios of 100 to 500; in the release build 2 of 100 chains became NaN
//!     at 15 m/s and 31 of 100 at 50 m/s;
//!   - a light body held by two to four constraints to static anchors, also without a hinge
//!     (point and fixed, cone, distance and six-DOF, ...): of 300 seeded scenes at 0.3 to 1 of
//!     the lever-arm bound, 24 asserted at the velocity bounds with every constraint kind and 8
//!     without hinges, 3 at a tenth of the bounds and none at 3 %.
//!
//!   The same probes found no assert and no NaN for a 2 m by 1 m by 5 cm door hinged at its
//!   edge, capsule limbs on hinges, a rod hinged on its axis, or the 12-capsule test ragdoll, at
//!   kicks up to 499 m/s and 47 rad/s, but their joints still opened by more than 0.5 m: of 24
//!   kicks at 150 m/s, 8 for the door, 10 for the rod and 9 for a forearm or shin capsule, and
//!   at 499 m/s 16, 24 and 19, up to 3.3 m for the door, 4.1 m for the rod and 7.3 m for the
//!   capsules; one of 24 exploded ragdolls opened by 1 m at 150 m/s. A six-DOF joint with the
//!   hinge's free axis did not assert on the slender bodies but let the joint drift apart by
//!   0.2 to 3.3 m while swinging and up to 6.9 m under kicks of 50 m/s, so it is no
//!   workaround. No
//!   input bound in this module excludes these cases.
//! - A slider adds the distance travelled along its axis to the lever of body 1
//!   (`SliderConstraint.cpp`), which [`MAX_LEVER_ARM_RATIO`] checks only at creation; with a
//!   dynamic body 1 and a long travel the lever grows beyond the bound.
//! - Inputs that are only checked to be finite or ordered, as the audit table says: ray
//!   directions, damping, motor force and torque limits, ragdoll joint friction (world
//!   constraint friction is probed as above), wheel friction curves
//!   and the wheel and drivetrain values that only have to give finite step coefficients.
//! - A soft body with pressure whose volume shrinks after creation (crushed, or its vertices
//!   moved or recentred by Jolt so that an open mesh encloses less): Jolt divides the pressure
//!   by the volume of every sub-step (`SoftBodyMotionProperties.cpp:300-307`), and the
//!   creation check holds only for the start geometry.
//! - Soft body constraint stability: [`MAX_COMPLIANCE`] keeps Jolt's compliance terms finite,
//!   not the solver convergent. Measured: a 2 m cube of 1 g vertices, three of them kinematic,
//!   with six tetrahedral volume constraints of compliances 0 to 1e20, per-vertex attributes
//!   with edge and shear compliances up to 1e20 and LRA multipliers up to 1e4, distance bends,
//!   100 iterations, no damping, restitution 1, a vertex speed limit of 1 m/s, one vertex given
//!   155 m/s and an accepted force of 3.7e6 N, had NaN vertices after its first step in the
//!   release build and asserted that a squared velocity is finite (`MotionProperties.inl:28`)
//!   in the asserts build. With a tenth of the force, 5 iterations, no volume constraints or
//!   uniform attributes it stayed finite. No input bound in this module excludes it. Two more
//!   seeded scenes of the same kind failed in the asserts build: a cube of 1 g vertices with
//!   six volume constraints, 37 iterations, a step of 0.1 s, friction 1000, a vertex radius of
//!   2000 m, a force of 3.7e6 N and a vertex unpinned to 1 kg asserted at
//!   `MotionProperties.inl:28`; a cube with pressure 1e5 under gravity of about 1000 m/s²,
//!   pushed by repeated forces of 3.8e9 N and resting against rigid bodies (without them it
//!   did not fail), grew bounds beyond `cLargeFloat` (`QuadTree.cpp:68`).
//! - `RagdollSettings::new_stabilized` reports Jolt's `Stabilize` failing to decompose an
//!   inertia tensor as an error, but Jolt asserts on that path first (`Ragdoll.cpp:158`).
//! - The assertion `errors == EPhysicsUpdateError::None` at the end of every step that drops
//!   contacts (`PhysicsSystem.cpp:679`) is intentional: [`PhysicsWorld::step`] returns the same
//!   errors in its [`StepReport`](crate::StepReport), and joltphysics' assertion handler lets the
//!   process continue for this assertion only.
//!
//! # Audit
//! Every public setter and constructor that takes a magnitude, the rule it applies and the test
//! that covers it at its boundary. "New" rows were added with this policy; "existing" rows name
//! the test that already covered them.
//!
//! | Input | Rule | Test |
//! |---|---|---|
//! | `WorldSettings::gravity`, `PhysicsWorld::set_gravity` | [`MAX_ACCELERATION`] | new: `world_gravity_is_bounded_by_max_acceleration` |
//! | `WorldSettings::max_contact_constraints` | `1..=`[`WorldSettings::MAX_CONTACT_CONSTRAINTS`] | new: `contact_constraint_capacity_is_bounded` |
//! | `WorldSettings::max_bodies`, `worker_threads` | Jolt's and joltphysics' counts | existing: `invalid_settings_are_rejected`, `worker_thread_bounds_are_validated` |
//! | `WorldSettings::job_system` (`JobSystem::max_concurrency`) | `1..=`[`WorldSettings::MAX_CONCURRENCY`](crate::WorldSettings::MAX_CONCURRENCY), read once in `PhysicsWorld::new` | new: `max_concurrency_is_bounded` |
//! | `WorldSettings::max_body_pairs`, `temp_allocator_size` | at least 1; Jolt asserts nothing on their size, and joltc's temp allocator falls back to `malloc` | existing: `invalid_settings_are_rejected` |
//! | `PhysicsWorld::step` delta time | `MIN_DELTA_TIME..=MAX_DELTA_TIME` | existing: `step_rejects_delta_time_above_the_bound`, `step_rejects_delta_time_below_the_bound` |
//! | `PhysicsWorld::rebase` translation | `2 *` [`MAX_POSITION`] per axis; results finite | new: `rebase_translation_is_bounded_by_twice_the_frame` |
//! | `BodySettings::position` | [`MAX_POSITION`] | new: `body_settings_are_bounded` |
//! | `BodySettings::linear_velocity`, `angular_velocity` | [`MAX_LINEAR_VELOCITY`], [`MAX_ANGULAR_VELOCITY`] | new: `body_settings_are_bounded`, `creation_velocities_agree_with_jolts_length_in_many_directions` |
//! | `BodySettings::restitution` | `0..=1` | new: `body_settings_are_bounded` |
//! | `BodySettings::gravity_factor` | [`MAX_GRAVITY_FACTOR`] | new: `body_settings_are_bounded` |
//! | `BodySettings::mass`, `PhysicsWorld::create_body` computed mass | [`MIN_MASS`]`..=`[`MAX_MASS`] for dynamic bodies | new: `body_settings_are_bounded`, `computed_dynamic_mass_is_bounded_and_kinematic_mass_is_not` |
//! | `PhysicsWorld::create_body` computed inertia of a dynamic or kinematic body (also a character's inner body and a part of `RagdollSettings::new`, `new_stabilized`) | an exactly diagonal tensor with invertible moments or near zero; otherwise smallest principal moment bounded from below at least `MIN_INERTIA_RATIO` (4.8e-4) of its Frobenius norm (see [Derived bounds](#derived-bounds)) | new: `rigid_body_inertia_floor_holds_at_its_boundary`, `rigid_body_inertia_at_its_bound_decomposes`, `inner_body_inertia_at_its_bound_decomposes`, `stabilized_ragdoll_inertia_at_its_bound_decomposes` |
//! | `BodySettings::friction` | `0..=`[`MAX_FRICTION`] | new: `body_settings_are_bounded`, `friction_at_the_bound_keeps_contacts_finite` |
//! | `BodySettings::linear_damping`, `angular_damping` | finite, at least 0: Jolt scales by `max(0, 1 - c·dt)` (`MotionProperties.inl:144-145`) | existing: `invalid_damping_is_rejected` |
//! | `BodySettings::rotation`, `BodyMut::set_rotation` | finite unit quaternion | existing: `invalid_body_settings_are_rejected` |
//! | `BodyMut::set_position`, `set_position_and_rotation` | [`MAX_POSITION`] | new: `body_setters_are_bounded_and_rejection_changes_nothing` |
//! | `BodyMut::set_linear_velocity`, `set_angular_velocity` | [`MAX_LINEAR_VELOCITY`], [`MAX_ANGULAR_VELOCITY`] | new: `body_setters_are_bounded_and_rejection_changes_nothing` |
//! | `BodyMut::add_force` | accumulated `|F| / m <=` [`MAX_ACCELERATION`] | new: `forces_are_bounded_by_the_acceleration_they_give` |
//! | `BodyMut::add_torque`, `add_force_at_point` | accumulated torque within [`MAX_ANGULAR_ACCELERATION`]; point within [`MAX_POSITION`]; `f32` torque products | new: `torques_are_bounded_by_the_angular_acceleration_they_give`, `point_torque_rejects_overflowing_products_even_when_they_cancel` |
//! | `BodyMut::reset_forces` | none | existing: `reset_forces_ignores_static_and_kinematic_bodies` |
//! | `CharacterSettings::mass` | `0..=`[`MAX_MASS`] | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterSettings::shape_offset` | [`MAX_SHAPE_EXTENT`] per axis | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterSettings::predictive_contact_distance`, `character_padding`, `collision_tolerance` | `0..=`[`MAX_SHAPE_EXTENT`] (tolerance positive) | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterSettings::max_strength` and the other settings | existing ranges; strength needs no bound (see above) | existing: `invalid_settings_and_poses_are_rejected_without_side_effects` |
//! | `PhysicsWorld::create_character` position, `CharacterMut::set_position` | [`MAX_POSITION`] | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterMut::set_linear_velocity` | [`MAX_LINEAR_VELOCITY`]; Jolt does not clamp a character | new: `character_settings_and_setters_are_bounded` |
//! | `CharacterMut::set_up`, `set_rotation` | unit vector, unit quaternion | existing: `invalid_settings_and_poses_are_rejected_without_side_effects` |
//! | `PhysicsWorld::update_character` gravity | [`MAX_ACCELERATION`] | new: `character_update_gravity_and_steps_are_bounded` |
//! | `PhysicsWorld::update_character` weight impulse | character mass times gravity times delta time at most [`MAX_WEIGHT_IMPULSE`] | new: `character_weight_impulse_at_a_lever_arm_is_bounded`, `weight_impulse_check_accepts_its_bound_and_rejects_beyond` |
//! | `ExtendedUpdateSettings` steps and forward distances | [`MAX_SHAPE_EXTENT`] | new: `character_update_gravity_and_steps_are_bounded` |
//! | `CharacterMut::restore_state` | only states from `save_state` exist | existing: `a_restored_state_saves_the_same_bytes` |
//! | `PhysicsWorld::restore_state` | only states from `save_state`/`save_state_of` of the same world at the same epoch exist | new: `tests/state.rs` |
//! | `VehicleMut::set_gravity` | [`MAX_ACCELERATION`] | new: `gravity_is_bounded_by_max_acceleration` |
//! | `WheelSettings::new` position, `suspension_force_point` | [`MAX_SHAPE_EXTENT`] per axis | new: `wheel_magnitudes_are_bounded_by_the_policy` |
//! | `WheelSettings` suspension min, max and preload lengths, radius, width | `0..=`[`MAX_SHAPE_EXTENT`] (radius positive, max length at least min length) | new: `wheel_magnitudes_are_bounded_by_the_policy` |
//! | `WheelSettings::suspension_spring` | Jolt's stiffness and damping at most [`MAX_SPRING_COEFFICIENT`] for a chassis of [`MAX_MASS`] | new: `suspension_springs_are_bounded_by_the_coefficient`, `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely` |
//! | `VehicleAntiRollBar::stiffness` | `0..=VehicleAntiRollBar::MAX_STIFFNESS` | new: `anti_roll_bars_are_validated`, `vehicle_springs_and_anti_roll_bar_at_their_bounds_step_finitely` |
//! | `WheelSettings` inertia, angular damping, brake torques; engine, transmission and differential settings | finite, in their ranges, and every step coefficient they form finite at both time-step extremes; not bounded one by one | existing: `wheel_values_are_validated`, `step_coefficients_of_wheels_must_be_finite`, `step_coefficients_of_the_drivetrain_must_be_finite`, `engine_values_are_validated`, `transmission_values_are_validated`, `differential_values_are_validated` |
//! | `WheelSettings` friction curves | finite points with increasing slip; the friction values are not bounded | existing: `wheel_values_are_validated` |
//! | `VehicleSettings` up, forward, max pitch roll angle; collision testers | unit vectors and angle ranges; tester radius below every wheel's reach | existing: `vehicle_values_are_validated`, `collision_testers_are_validated` |
//! | `VehicleMut::set_driver_input`, `set_max_pitch_roll_angle`, `set_collision_tester` | existing ranges | existing: `driver_input_drives_steers_and_brakes`, `invalid_vehicles_create_nothing` |
//! | `SpringSettings::StiffnessAndDamping` | [`MAX_SPRING_COEFFICIENT`] | new: `stiffness_springs_are_bounded_by_the_coefficient` |
//! | `SpringSettings::FrequencyAndDamping` in `RagdollSettings::new`, `new_stabilized` | `B·ω²` and `2·B·ζ·ω` at most [`MAX_SPRING_COEFFICIENT`] | new: `motor_springs_are_bounded_by_the_parts_effective_mass`, `motor_spring_of_1e20_hz_is_rejected` |
//! | `MotorSettings::force_limits`, `torque_limits`; angle limits | finite, `min <= max`; Jolt clamps the motor impulse to `dt · limit` | existing: `motors_and_springs_are_validated`, `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated` |
//! | constraint frame points | [`MAX_POSITION`] | new: `constraint_frame_points_are_bounded`, `constraint_targets_are_bounded` |
//! | the points where `PhysicsWorld::create_constraint` holds a dynamic body (frame points, automatic points, every point of a path); no setter moves them, and a rebase moves them with their bodies | lever-arm ratio at most [`MAX_LEVER_ARM_RATIO`] | new: `lever_arms_are_bounded_by_the_bodies_size`, `far_and_light_constraint_points_are_refused`, `constraints_at_the_lever_arm_bound_step_finitely` |
//! | `SpringSettings::FrequencyAndDamping` in `PhysicsWorld::create_constraint`, `ConstraintMut::<DistanceConstraint>::set_limits_spring`, `ConstraintMut::<HingeConstraint>::set_motor_settings`, `set_limits_spring` | `B·ω²` and `2·B·ζ·ω` at most [`MAX_SPRING_COEFFICIENT`], `B` from the constraint's two bodies | new: `world_constraint_springs_are_bounded_by_the_bodies_effective_mass` |
//! | `DistanceRange::Range`, `ConstraintMut::<DistanceConstraint>::set_distance` | `0 <= min <= max <=` [`MAX_SHAPE_EXTENT`] | new: `constraint_targets_are_bounded` |
//! | `ConstraintMut::<HingeConstraint>::set_target_angle` | `[-π, π]`; Jolt clamps it to the limits | new: `constraint_targets_are_bounded` |
//! | `ConstraintMut::<HingeConstraint>::set_target_angular_velocity` | [`MAX_ANGULAR_VELOCITY`] | new: `constraint_targets_are_bounded` |
//! | `ConstraintMut::<HingeConstraint>::set_limits` | Jolt's hinge ranges, `min == max` only with a soft spring | new: `hinge_setters_check_their_values` |
//! | `ConstraintMut::<HingeConstraint>::set_max_friction_torque` | finite, at least 0; Jolt clamps the friction impulse to `dt · limit` | new: `constraint_friction_at_f32_max_steps_finitely` |
//! | `SliderConstraintSettings::limits`, `ConstraintMut::<SliderConstraint>::set_limits` | `min` in `[-`[`MAX_SHAPE_EXTENT`]`, 0]`, `max` in `[0, `[`MAX_SHAPE_EXTENT`]`]`, `min == max` only with a soft spring | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SliderConstraint>::set_target_position` | [`MAX_SHAPE_EXTENT`]; Jolt clamps it to the limits | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SliderConstraint>::set_target_velocity` | [`MAX_LINEAR_VELOCITY`] | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `SliderConstraintSettings::max_friction_force`, `ConstraintMut::<SliderConstraint>::set_max_friction_force`, `ConstraintMut::<SwingTwistConstraint>::set_max_friction_torque` | finite, at least 0 | new: `slider_and_swing_twist_friction_at_f32_max_steps_finitely` |
//! | `SpringSettings::FrequencyAndDamping` in the slider, swing-twist and six-DOF motor and limit spring setters of `ConstraintMut` | as for `create_constraint` | new: `slider_swing_twist_and_six_dof_motor_springs_are_bounded` |
//! | `ConeConstraintSettings::new` half angle, `ConstraintMut::<ConeConstraint>::set_half_cone_angle` | `[0, π]`, as Jolt asserts | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SwingTwistConstraint>::set_target_angular_velocity_cs`, `ConstraintMut::<SixDofConstraint>::set_target_angular_velocity_cs` | [`MAX_ANGULAR_VELOCITY`] | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SwingTwistConstraint>::set_target_orientation_cs`, `ConstraintMut::<SixDofConstraint>::set_target_orientation_cs` | finite unit quaternion; Jolt clamps it to the limits | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SixDofConstraint>::set_target_velocity_cs` | [`MAX_LINEAR_VELOCITY`] | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `ConstraintMut::<SixDofConstraint>::set_target_position_cs` | [`MAX_SHAPE_EXTENT`] per axis | new: `slider_cone_swing_twist_and_six_dof_targets_are_bounded` |
//! | `GearConstraintSettings::new`, `teeth` ratio | `1..=`[`MAX_GEAR_RATIO`] (see `GearConstraintSettings` and [`MAX_GEAR_RATIO`] for both bounds) | new: `coupling_ratios_are_bounded`, `ratios_at_the_bound_step_finitely`; behaviour at the bound in `tests/constraints.rs` |
//! | `RackAndPinionConstraintSettings::new`, `teeth` ratio | magnitude within `1 / `[`MAX_RATIO`]`..=`[`MAX_RATIO`]; `teeth` length positive within [`MAX_SHAPE_EXTENT`] | new: `coupling_ratios_are_bounded`, `ratios_at_the_bound_step_finitely` |
//! | `PulleyConstraintSettings::ratio` | positive, within `1 / `[`MAX_RATIO`]`..=`[`MAX_RATIO`] | new: `pulley_ratio_and_lengths_are_bounded`, `pulleys_at_the_ratio_bound_step_finitely` |
//! | `PulleyLength::Range`, `ConstraintMut::<PulleyConstraint>::set_length` | `0 <= min <= max <= (1 + ratio) ·` [`MAX_SHAPE_EXTENT`] | new: `pulley_ratio_and_lengths_are_bounded` |
//! | `PulleyConstraintSettings::new` body and fixed points | [`MAX_POSITION`]; a rebase moves fixed points as re-expressed state, checked finite only | new: `pulley_ratio_and_lengths_are_bounded` |
//! | `HermitePath::new` points | 2 to `HermitePath::MAX_POINTS`; positions and tangents within [`MAX_SHAPE_EXTENT`] per axis; unit normal | new: `invalid_paths_are_rejected`, `path_inputs_are_bounded` |
//! | `HermitePath::new` segments | chord at least 1 mm; derivative along the chord at least twice its bound along the normal and above an `f32` margin, so Jolt's normal stays unit | new: `invalid_paths_are_rejected`, `path_validation_accepts_its_boundary` |
//! | `PathConstraintSettings::path_position`, `path_rotation`, `path_fraction` | [`MAX_SHAPE_EXTENT`] per axis; unit quaternion; `[0, max_fraction]` | new: `path_inputs_are_bounded` |
//! | `PathConstraintSettings::max_friction_force`, `ConstraintMut::<PathConstraint>::set_max_friction_force` | finite, at least 0 | new: `path_motor_springs_and_friction_are_bounded` |
//! | `PathConstraintSettings::position_motor`, `ConstraintMut::<PathConstraint>::set_position_motor_settings` springs | as for `create_constraint` | new: `path_motor_springs_and_friction_are_bounded` |
//! | `ConstraintMut::<PathConstraint>::set_target_velocity`, `set_target_path_fraction` | [`MAX_LINEAR_VELOCITY`]; `[0, max_fraction]` | new: `path_inputs_are_bounded` |
//! | `ConstraintRef::<PathConstraint>::closest_fraction` | point within [`MAX_SHAPE_EXTENT`] per axis, finite hint | new: `path_inputs_are_bounded` |
//! | `SwingTwistConstraintSettings::max_friction_torque`, `HingeConstraintSettings::max_friction_torque`, `SixDofConstraintSettings::max_friction` | finite, at least 0; Jolt clamps the friction impulse to `dt · limit` and applies no more than stops the relative motion | existing: `swing_twist_limits_are_validated`, `hinge_limits_are_validated`, `six_dof_limits_are_validated` |
//! | `SixDofAxis::Limited` on a translation axis | finite, `min < max`, within [`MAX_SHAPE_EXTENT`] | new: `six_dof_limits_are_validated`, `six_dof_translation_limits_at_the_bound_step_finitely` |
//! | `RagdollSettings::new`, `new_stabilized` part masses | [`MIN_MASS`]`..=`[`MAX_MASS`], also for kinematic parts (`RagdollMut::set_motion_type` can make them dynamic) | new: `part_masses_and_velocities_are_bounded` |
//! | `RagdollMut::set_pose`, `drive_to_pose_using_motors` | root offset and positions within [`MAX_POSITION`] | new: `poses_are_validated` |
//! | `RagdollMut::drive_to_pose_using_kinematics` | pose as above; every part's velocity, as Jolt computes it, within [`MAX_LINEAR_VELOCITY`] and [`MAX_ANGULAR_VELOCITY`], checked for all parts before any changes | new: `poses_are_validated`, `kinematic_drive_is_bounded_by_the_velocities_it_implies` |
//! | `RagdollMut::set_linear_and_angular_velocity` | [`MAX_LINEAR_VELOCITY`], [`MAX_ANGULAR_VELOCITY`] | new: `part_masses_and_velocities_are_bounded` |
//! | `Shape::new_box*`, `new_sphere`, `new_cylinder*`, `new_capsule` | dimensions within [`MAX_SHAPE_EXTENT`] | new: `primitive_extents_are_bounded` |
//! | `Shape::new_compound`, `new_offset_center_of_mass` | positions and offset within [`MAX_SHAPE_EXTENT`]; local bounds within it | new: `decorated_and_compound_extents_are_bounded` |
//! | `Shape::new_height_field` | local bounds within [`MAX_SHAPE_EXTENT`] | new: `height_field_extent_is_bounded` |
//! | `HeightFieldSettings`, `CompoundChild::rotation` | existing ranges | existing: `invalid_height_fields_are_rejected`, `empty_or_invalid_compounds_are_rejected` |
//! | `PhysicsWorld::cast_ray` origin | [`MAX_POSITION`] | new: `query_inputs_are_bounded_by_the_frame` |
//! | `RayCast` direction | finite, not zero | existing: `invalid_rays_are_rejected`; new: `a_ray_with_a_huge_finite_direction_is_cast` |
//! | `ShapeCast`, `CollideShape` position | [`MAX_POSITION`] | new: `query_inputs_are_bounded_by_the_frame` |
//! | `ShapeCast` direction | `2 *` [`MAX_POSITION`] per axis | new: `query_inputs_are_bounded_by_the_frame` |
//! | `ShapeCast::target_distance` | `ShapeCast::MAX_TARGET_DISTANCE` | existing: `target_distance_at_the_bound_gives_a_finite_depth` |
//! | `CollideShape::max_separation_distance` | `0..=`[`MAX_SHAPE_EXTENT`] | new: `query_inputs_are_bounded_by_the_frame` |
//! | `SoftBodySharedSettingsBuilder::build` vertices | at least one; position within [`MAX_SHAPE_EXTENT`] per axis; velocity within [`MAX_LINEAR_VELOCITY`]; inverse mass 0 or the inverse of a mass within [`MIN_MASS`]`..=`[`MAX_MASS`] | new: `soft_body_shared_settings_are_bounded`, `invalid_vertices_are_rejected` |
//! | `SoftBodySharedSettingsBuilder::build` total mass | masses of the movable vertices (`1 / w` in `f32`, as Jolt) add up to at most [`MAX_MASS`] | new: `soft_body_total_mass_is_bounded`, `total_movable_mass_is_bounded` |
//! | `SoftBodySharedSettingsBuilder::build` faces | indices name vertices, three different ones; every edge at least [`MIN_SOFT_BODY_EDGE_LENGTH`] in `f32`; area above 0; with distance bends the vertices opposite a shared edge as far apart | new: `soft_body_shared_settings_are_bounded`, `invalid_faces_are_rejected`, `edge_lengths_are_measured_in_f32_like_jolt`, `distance_bends_need_separate_opposite_vertices` |
//! | `SoftBodyVertexAttributes` compliances | `0..=`[`MAX_COMPLIANCE`] | new: `soft_body_shared_settings_are_bounded`, `attributes_are_validated` |
//! | `SoftBodyVertexAttributes::long_range_attachment` multiplier | `1..=`[`MAX_RATIO`] (see there for why Jolt's square of the distance stays finite) | new: `soft_body_shared_settings_are_bounded` |
//! | `SoftBodySharedSettingsBuilder::edge`, `dihedral_bend`, `volume` | indices name different vertices; compliance `0..=`[`MAX_COMPLIANCE`]; edge and shared bend edge at least [`MIN_SOFT_BODY_EDGE_LENGTH`]; tetrahedron six-volume finite and not 0 in `f32` | new: `soft_body_explicit_constraints_are_bounded`, `invalid_explicit_constraints_are_rejected` |
//! | `SoftBodySettings` position, rotation, object layer, friction, restitution, gravity factor | as for `BodySettings` | new: `soft_body_settings_are_bounded` |
//! | `SoftBodySettings::num_iterations` | `1..=SoftBodySettings::MAX_ITERATIONS`; Jolt divides the step by it | new: `soft_body_settings_are_bounded` |
//! | `SoftBodySettings::linear_damping`, `max_linear_velocity`, `vertex_radius` | finite, at least 0; `(0, `[`MAX_LINEAR_VELOCITY`]`]`; `0..=`[`MAX_SHAPE_EXTENT`] | new: `soft_body_settings_are_bounded` |
//! | `SoftBodySettings::pressure` | `0..=`[`MAX_SOFT_BODY_PRESSURE`]; above 0 only when the faces enclose a volume large enough for it (see [Derived bounds](#derived-bounds)) | new: `soft_body_settings_are_bounded`, `pressure_at_the_bound_steps_finitely`, `pressure_needs_a_volume_for_its_faces`, `a_sliver_at_the_pressure_bound_steps_finitely` |
//! | `PhysicsWorld::create_soft_body` vertices (positions about the origin, masses) | without a kinematic vertex: smallest principal moment of the inertia at least `MIN_INERTIA_RATIO` (4.8e-4) of its Frobenius norm after Jolt's `f32` error, or an exactly diagonal tensor that is near zero or has every moment above 1e-30 (see [Derived bounds](#derived-bounds)) | new: `soft_body_inertia_is_checked_at_creation`, `soft_body_inertia_at_its_bound_decomposes` |
//! | `SoftBodyMut::set_vertex_velocity` | [`MAX_LINEAR_VELOCITY`] | new: `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing` |
//! | `SoftBodyMut::set_vertex_inverse_mass` | 0 or the inverse of a mass within [`MIN_MASS`]`..=`[`MAX_MASS`]; total movable mass at most [`MAX_MASS`]; the force accumulated this step within the soft body force bound for the new inverse masses; the inertia rule of `create_soft_body` at the current vertex positions | new: `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing`, `unpinning_cannot_release_an_accumulated_force`, `unpinning_checks_the_inertia_at_the_current_positions` |
//! | `SoftBodyMut::move_kinematic_vertex` | target within [`MAX_POSITION`]; a time step `step` accepts; the implied velocity within [`MAX_LINEAR_VELOCITY`] | new: `soft_body_vertex_writes_are_bounded_and_rejection_changes_nothing` |
//! | `BodyMut::add_force` on a soft body | accumulated `|F| · w_max / N <=` [`MAX_ACCELERATION`], `N` the vertex count (Jolt's divisor), and `|F| <= MAX_ACCELERATION · MAX_MASS` | new: `soft_body_forces_are_bounded_by_the_acceleration_of_a_vertex`, `unpinning_cannot_release_an_accumulated_force` |
//! | `DebugLineSettings` (feature `debug-renderer`) | centre within [`MAX_POSITION`], radius at most twice it | new: `center_and_radius_are_bounded_by_the_frame` |
//!
//! [`WorldSettings::MAX_CONTACT_CONSTRAINTS`]: crate::WorldSettings::MAX_CONTACT_CONSTRAINTS

use crate::math::jolt_length;
use crate::{PhysicsWorld, Quat, RVec3, Real, Vec3};

/// Largest absolute value of each component of a caller-given world position, in metres:
/// 5 km with `f32` positions, 10 000 km with the `double-precision` feature.
///
/// Crate policy, not a Jolt assertion threshold. Jolt's "Big Worlds" documentation
/// (`Docs/Architecture.md`) says single-precision simulation is accurate within roughly 5 km of
/// the origin, and that double precision handles worlds of thousands of km; at 10 000 km Jolt's
/// `f32` broad phase still has a resolution of about 1 m.
pub const MAX_POSITION: Real = (if core::mem::size_of::<Real>() == 8 {
    1.0e7_f64
} else {
    5.0e3_f64
}) as Real;

/// Largest absolute value of each component of a shape's local bounds (around its centre of
/// mass), of a shape offset and of a local step distance, in metres.
///
/// Crate policy from Jolt's "Conventions and Limits" documentation, which recommends static
/// objects of 0.1 to 2000 m; the bound applies on each side of the centre of mass. It bounds a
/// shape's inertia to at most `6 * mass * MAX_SHAPE_EXTENT²`.
pub const MAX_SHAPE_EXTENT: f32 = 2000.0;

/// Largest linear velocity a caller may give a body or character, in m/s.
///
/// Jolt's default `BodyCreationSettings::mMaxLinearVelocity` (`BodyCreationSettings.h:111`). Jolt
/// asserts `Length() <= mMaxLinearVelocity` when it creates a body (`Body.cpp:424`,
/// `MotionProperties.h:48`) and clamps a body's velocity to it every step.
pub const MAX_LINEAR_VELOCITY: f32 = 500.0;

/// Largest angular velocity a caller may give a body, in rad/s.
///
/// Jolt's default `BodyCreationSettings::mMaxAngularVelocity` (`BodyCreationSettings.h:112`),
/// written as Jolt writes it so the `f32` bits match.
pub const MAX_ANGULAR_VELOCITY: f32 = 0.25 * core::f32::consts::PI * 60.0;

/// Largest length of a caller-given acceleration (world gravity, a character's or vehicle's
/// gravity, the acceleration a body's added forces give it), in m/s²: about 5e8.
///
/// Crate policy, not a Jolt limit. A larger acceleration already reaches Jolt's speed clamp
/// within every step [`PhysicsWorld::step`] accepts, so the bound removes no motion that the
/// clamp keeps.
pub const MAX_ACCELERATION: f32 = MAX_LINEAR_VELOCITY / PhysicsWorld::MIN_DELTA_TIME;

/// Largest angular acceleration a body's added torques may give it, in rad/s²: about 4.71e7.
///
/// Crate policy with the reasoning of [`MAX_ACCELERATION`], for Jolt's angular speed clamp.
pub const MAX_ANGULAR_ACCELERATION: f32 = MAX_ANGULAR_VELOCITY / PhysicsWorld::MIN_DELTA_TIME;

/// Largest absolute gravity factor of a body.
///
/// Crate policy (like [`WorldSettings::MAX_WORKER_THREADS`](crate::WorldSettings::MAX_WORKER_THREADS)).
pub const MAX_GRAVITY_FACTOR: f32 = 1000.0;

/// Largest friction coefficient of a body.
///
/// Crate policy. Jolt combines the friction of two bodies in contact as
/// `sqrt(friction1 * friction2)` (`ContactConstraintManager.h:554`) and multiplies the result by
/// the contact's normal impulse (`ContactConstraintManager.cpp:1714-1715`). A product that
/// overflows makes the combined friction infinite, and an infinite friction times a zero normal
/// impulse is NaN, which reaches the bodies' velocities; two bodies with friction `f32::MAX` do
/// that within a few steps. Any coefficient whose square is finite (below about 1.8e19) avoids
/// it; 1000 is far above the friction of real materials.
pub const MAX_FRICTION: f32 = 1000.0;

/// Smallest mass of a dynamic body or ragdoll part, in kg: an inverse mass of at most 1000 per kg,
/// a 1 cm cube of water.
///
/// Crate policy.
pub const MIN_MASS: f32 = 1.0e-3;

/// Largest inverse mass of a movable soft body vertex, in 1/kg: 1000, whose inverse Jolt
/// computes in `f32` as exactly [`MIN_MASS`]. `1.0 / MIN_MASS` itself rounds to 999.99994.
///
/// Derived from [`MIN_MASS`].
pub const MAX_VERTEX_INVERSE_MASS: f32 = 1000.0;

/// Largest mass of a dynamic body, ragdoll part or character, in kg: a 10 m cube of water, the
/// top of the dynamic object sizes Jolt documents.
///
/// Crate policy. It bounds the forces [`MAX_ACCELERATION`] accepts (at most 5e14 N), contact
/// effective masses, a vehicle's gravity force and a character's weight impulse.
pub const MAX_MASS: f32 = 1.0e6;

/// Largest spring stiffness `k` and damping `c` Jolt may derive from a constraint spring
/// (`SpringPart.h:36-55,91-104`).
///
/// Crate policy. `c + dt * k` stays finite (at most 2e30 for `dt <= 1`), so the softness, bias
/// and effective mass Jolt computes from them stay finite.
pub const MAX_SPRING_COEFFICIENT: f32 = 1.0e30;

/// Largest weight impulse a character may press on what it stands on during one update, its
/// mass times the length of the update's gravity times the update's delta time, in N·s.
///
/// Crate policy. Jolt applies the weight impulse at the ground contact point
/// (`CharacterVirtual.cpp:1474-1481`), so it also turns the ground body. Jolt keeps a body's
/// principal moments of inertia only while their vector is longer than 1e-6 (`Vec3::IsNearZero`
/// in `MotionProperties.cpp:46-56`) and otherwise uses the inertia of a sphere of radius 1, an
/// inverse of `2.5 / mass`, at most 2500 for [`MIN_MASS`]. So a body whose principal moments are
/// equal has an inverse inertia of at most `√3 · 1e6`, and the ground contact lies within
/// `√3 ·` [`MAX_SHAPE_EXTENT`] of its centre of mass. For such a body and every body with a
/// smaller principal inverse inertia, the angular velocity change is at most
/// `√3e6 · 3464 · 1e9`, about 6e18 rad/s, whose square is finite; the linear velocity change is
/// at most `1e9 · 1e3` m/s. Without the bound, a character of [`MAX_MASS`] at
/// [`MAX_ACCELERATION`] with a one-second update (5e14 N·s) on the edge of a 6 cm cube of
/// [`MIN_MASS`] overflows the cube's squared angular speed, which Jolt asserts on
/// (`MotionProperties.inl:38`). The bound allows a character of [`MAX_MASS`] at 1000 m/s² with
/// one-second updates, far above the characters of a game.
pub const MAX_WEIGHT_IMPULSE: f32 = 1.0e9;

/// Largest magnitude of a rack-and-pinion or pulley ratio; the smallest is its inverse. Gears
/// have their own, tighter range (see [`MAX_GEAR_RATIO`]).
///
/// Crate policy. Jolt multiplies the inverse mass or inertia of body 2 by the ratio's square in
/// the effective mass, and body 2's velocity (for racks and pulleys also its impulse) by the
/// ratio (`GearConstraintPart.h:81,122`, `RackAndPinionConstraintPart.h:82,123`,
/// `IndependentAxisConstraintPart.h:71,112` for pulleys). With a principal inverse inertia of at
/// most `√3 · 1e6` (see [`MAX_WEIGHT_IMPULSE`]), `ratio² · I⁻¹` is at most about 1.7e14, far from
/// `f32` overflow. A ratio of 1e4 already turns a pinion ten thousand radians per metre of its
/// rack; tests step both bounds on the lightest and heaviest bodies.
pub const MAX_RATIO: f32 = 1.0e4;

/// Largest gear ratio; the smallest is 1 (see
/// [`GearConstraintSettings`](crate::GearConstraintSettings)).
///
/// Crate policy, measured. Jolt 5.6 applies a gear's impulse to body 2 without the ratio
/// (`GearConstraintPart::ApplyVelocityStep`), so each solver iteration keeps up to `1 − 1/ratio`
/// of the velocity error `ω1 + ratio · ω2`, the worst case being a body 1 much heavier than
/// body 2. With Jolt's 10 velocity iterations per step the gear then needs more steps to restore
/// the relation the larger the ratio. The bound is the largest round ratio that, after a
/// disturbance, brings the error back to within 2 % of its initial value within 10 steps for any
/// mass distribution: measured worst 1.6 % at ratio 10, 6.5 % at 20, 60 % at 100, and at 1e4
/// 91 % still after 60 steps. The first step after a disturbance leaves up to
/// `(1 − 1/ratio)^10`, 35 % at ratio 10. Tested at the bound by
/// `gear_keeps_its_velocity_relation_at_the_largest_ratio`.
pub const MAX_GEAR_RATIO: f32 = 10.0;

/// Largest lever-arm ratio a world constraint may give a dynamic body: how far the point where
/// the constraint holds the body lies from its centre of mass, measured against the body's own
/// size.
///
/// The lever-arm ratio of a body at a point `r` from its centre of mass is
/// `mass · trace([r]× I⁻¹ [r]×ᵀ)`, with `I⁻¹` the body's inverse inertia: summed over the body's
/// principal axes, the squared distance of the point from each axis divided by the squared
/// radius of gyration about it. A sphere or cube with radius of gyration `k` has
/// `2 · (|r| / k)²`, so the bound allows `|r|` up to about `22 · k`; a rod held at its end has a
/// ratio of 6 at any length.
/// [`PhysicsWorld::create_constraint`] checks every point a constraint holds a dynamic body by
/// (for a path, every point of the path; for an automatic point, the point Jolt picks
/// between the centres of mass, weighted by inverse mass towards the lighter body).
///
/// Crate policy, measured, not derived. Jolt solves each constraint part with its effective mass
/// `K = Σ (m⁻¹ · 1 + [r]× I⁻¹ [r]×ᵀ)` (`PointConstraintPart.h`, `AxisConstraintPart.h`) in `f32`.
/// A ratio of at most `B` per body bounds the lever terms by `B · m⁻¹`, so `K`'s condition number
/// stays below `1 + B`. Two failures were measured with a body at the velocity bounds, under
/// gravity, for 120 steps, in the `asserts` build:
/// - a body held far from its centre of mass: a 1 g, 6 cm cube on a hinge 116 m away (a ratio
///   of 4.5e7) went to NaN, two 1 kg, 1 m cubes joined by a point 3000 m away (1.1e8) moved
///   erratically and at 4000 m Jolt asserted that a squared velocity is finite
///   (`MotionProperties.inl:28`), and a cube on a static body took angular velocities rounded to
///   powers of two from a ratio of about 1e8;
/// - two light bodies held rigidly (a fixed constraint, a six-DOF constraint with every axis
///   fixed, a swing-twist constraint with zero ranges): their accumulated impulse grows step
///   after step until Jolt asserts that the squared angular velocity is finite
///   (`MotionProperties.inl:38`), from `|r| / k` of about 37 (a ratio of 2700) for 1 g and 1 kg
///   cubes of 6 cm and 20 cm, for the 6 cm cube also at a tenth of the velocity bounds; none of
///   24 seeded cases failed at `|r| / k` of 34 or below. Point, hinge, cone, slider and six-DOF
///   constraints with limited rotations did not fail at `|r| / k` of 650.
///
/// A derivation in the style of [`MAX_WEIGHT_IMPULSE`] (products of the largest accepted
/// inverse inertia, lever and impulse kept finite in `f32`) allows levers of hundreds of metres
/// and does not exclude the second failure, so the bound is the measured onset divided by 2.7.
/// It allows a door on a hinge at its edge, a weld at the surface of a part, and a pendulum bob
/// of radius `a` on a point or hinge constraint up to about `14 · a` from the pivot; a longer
/// pendulum is a [`DistanceConstraintSettings`](crate::DistanceConstraintSettings) whose points
/// lie on the bodies.
pub const MAX_LEVER_ARM_RATIO: f32 = 1000.0;

/// Shortest distance between two soft body vertices that a face edge or an explicit edge
/// joins, in metres: 1 mm, measured as Jolt measures a rest length (an `f32` difference and an
/// `f32` length).
///
/// Crate policy. Jolt only asserts that a rest length is above zero
/// (`SoftBodySharedSettings.cpp:226,377`) and divides by edge lengths while it solves; the
/// bound keeps a degenerate edge out of the solver with a margin.
pub const MIN_SOFT_BODY_EDGE_LENGTH: f32 = 1.0e-3;

/// Largest compliance (inverse stiffness) of a soft body constraint, in the units of the
/// constraint's own equation; 0 is rigid.
///
/// Crate policy, derived. Jolt divides each compliance by the squared sub-step
/// (`SoftBodyMotionProperties.cpp:371,445,496,577,594`). [`PhysicsWorld::step`] always runs one
/// collision step, so a sub-step is at least [`PhysicsWorld::MIN_DELTA_TIME`] divided by
/// [`SoftBodySettings::MAX_ITERATIONS`](crate::SoftBodySettings::MAX_ITERATIONS), 1e-8 s, and
/// `compliance / dt²` is at most `1e20 · 1e16 = 1e36`, below `f32::MAX`; Jolt's average of two
/// compliances, `0.5 · (c1 + c2)`, stays finite as well. This proves that the product is finite,
/// not that the solver is stable at every compliance.
pub const MAX_COMPLIANCE: f32 = 1.0e20;

/// Largest pressure coefficient of a soft body (`n · R · T` in Jolt's terms, N·m).
///
/// Crate policy, measured. Jolt applies `pressure · dt / (6 · volume)` times each face's area
/// as an impulse (`SoftBodyMotionProperties.cpp:290-312`). A closed ball of 1 m with vertex
/// masses at [`MIN_MASS`] and at the total-mass bound, at this pressure, stepped 600 times on a
/// floor in the `asserts` build, stays finite (`pressure_at_the_bound_steps_finitely`).
///
/// A new body with pressure must also enclose enough volume for its faces (see
/// [Derived bounds](self#derived-bounds)); a body crushed to a tiny volume later is not covered.
pub const MAX_SOFT_BODY_PRESSURE: f32 = 1.0e6;

/// Largest force in newtons that the pressure of a new soft body may give a vertex in Jolt's
/// formula: [`MAX_ACCELERATION`] for a vertex of [`MAX_VERTEX_INVERSE_MASS`], 5e5 N (see
/// [Derived bounds](self#derived-bounds)).
const MAX_PRESSURE_VERTEX_FORCE: f64 = MAX_ACCELERATION as f64 / MAX_VERTEX_INVERSE_MASS as f64;

/// Unit roundoff of `f32`, `2^-24`.
const F32_UNIT_ROUNDOFF: f64 = f32::EPSILON as f64 / 2.0;

/// What [`is_soft_body_pressure`] needs to know of a soft body's vertices and faces, computed
/// once when its shared settings are built; the rule and its error bounds are derived in
/// [Derived bounds](self#derived-bounds).
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SoftBodyPressureGeometry {
    /// Lower bound of the six-volume Jolt computes in `f32` before the first step, in m³.
    six_volume_low: f64,
    /// Largest sum, over the faces of one vertex, of the bound of `|x2 - x1| · |x3 - x1|` in
    /// Jolt's arithmetic, in m².
    largest_vertex_face_area: f64,
}

impl SoftBodyPressureGeometry {
    /// The geometry of `faces` (vertex indices, each valid) between vertices at `positions`
    /// about the body origin.
    pub(crate) fn new(positions: &[Vec3], faces: &[[u32; 3]]) -> Self {
        // The error terms of the derivation, in units of `u`, and the rotation's slack.
        const ROTATION_VOLUME: f64 = 4.0e-5;
        const ROTATION_LENGTH: f64 = 1.0001;
        const POSITION_ERROR: f64 = 15.0;
        const TERM_ERROR: f64 = 64.0;
        const UNDERFLOW_PER_FACE: f64 = 1.0e-36;
        let u = F32_UNIT_ROUNDOFF;
        let point = |index: u32| {
            let p = positions[index as usize];
            [p.x, p.y, p.z].map(f64::from)
        };
        let edge = |from: [f64; 3], to: [f64; 3]| {
            let e = [to[0] - from[0], to[1] - from[1], to[2] - from[2]];
            ROTATION_LENGTH * (norm(e) + POSITION_ERROR * u * (norm(from) + norm(to)))
        };
        let mut six_volume = 0.0;
        let mut partial_sums = 0.0;
        let mut length_products = 0.0;
        let mut vertex_face_area = vec![0.0_f64; positions.len()];
        for &face in faces {
            let [x1, x2, x3] = face.map(point);
            let c = [
                x1[1] * x2[2] - x1[2] * x2[1],
                x1[2] * x2[0] - x1[0] * x2[2],
                x1[0] * x2[1] - x1[1] * x2[0],
            ];
            six_volume += c[0] * x3[0] + c[1] * x3[1] + c[2] * x3[2];
            partial_sums += six_volume.abs();
            length_products += norm(x1) * norm(x2) * norm(x3);
            let area = edge(x1, x2) * edge(x1, x3);
            for index in face {
                vertex_face_area[index as usize] += area;
            }
        }
        let face_count = faces.len() as f64;
        let growth = (1.0 - u).powf(-face_count);
        let error = growth
            * (u * (TERM_ERROR * length_products + (1.0 + ROTATION_VOLUME) * partial_sums)
                + UNDERFLOW_PER_FACE * face_count);
        Self {
            six_volume_low: (1.0 - ROTATION_VOLUME) * six_volume - error,
            largest_vertex_face_area: vertex_face_area.into_iter().fold(0.0, f64::max),
        }
    }
}

/// The length of `v`.
fn norm(v: [f64; 3]) -> f64 {
    (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt()
}

/// Whether a soft body of `geometry` may be created with `pressure`, already checked to be
/// within `0..=MAX_SOFT_BODY_PRESSURE`: always without pressure, otherwise when the faces
/// enclose a positive volume large enough for the pressure (see
/// [Derived bounds](self#derived-bounds)).
pub(crate) fn is_soft_body_pressure(pressure: f32, geometry: &SoftBodyPressureGeometry) -> bool {
    pressure == 0.0
        || (geometry.six_volume_low > 0.0
            && f64::from(pressure) * geometry.largest_vertex_face_area
                <= MAX_PRESSURE_VERTEX_FORCE * geometry.six_volume_low)
}

/// Whether a soft body with `vertex_count` vertices, the largest inverse mass among them
/// `largest_inverse_mass`, may hold the accumulated force `force` in newtons, computed in
/// `f64` (see [Derived bounds](self#derived-bounds)).
pub(crate) fn is_soft_body_force(
    force: [f64; 3],
    largest_inverse_mass: f32,
    vertex_count: u32,
) -> bool {
    let length = norm(force);
    let per_vertex = f64::from(largest_inverse_mass) / f64::from(vertex_count.max(1));
    length <= f64::from(MAX_ACCELERATION) * f64::from(MAX_MASS)
        && length * per_vertex <= f64::from(MAX_ACCELERATION)
}

/// Smallest principal moment of an inertia tensor Jolt decomposes, relative to the tensor's
/// Frobenius norm, that [`is_rigid_body_inertia`] and [`is_soft_body_inertia`] accept:
/// `1001 · 8 · u`, about 4.8e-4 (see [Derived bounds](self#derived-bounds)).
const MIN_INERTIA_RATIO: f64 = 1001.0 * 8.0 * F32_UNIT_ROUNDOFF;

/// Whether Jolt decomposes the non-diagonal inertia `tensor` of a rigid body (Jolt's own `f32`
/// tensor, widened to `f64`) without an assertion and into positive principal moments (see
/// [Derived bounds](self#derived-bounds)).
///
/// The rule is scale-free: a tensor small enough for Jolt's unit-sphere fallback is decomposed
/// first all the same, so it is checked like any other.
pub(crate) fn is_rigid_body_inertia(tensor: [[f64; 3]; 3]) -> bool {
    let [[a, b, c], [d, e, f], [g, h, i]] = tensor;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
    let minors = (a * e - b * d) + (a * i - c * g) + (e * i - f * h);
    let frobenius = tensor.iter().flatten().map(|v| v * v).sum::<f64>().sqrt();
    // `det / minors` is at most the smallest principal moment (see `is_soft_body_inertia`).
    det > 0.0 && minors > 0.0 && det / minors >= MIN_INERTIA_RATIO * frobenius
}

/// The vertices of a soft body as Jolt's inertia computation sees them, reduced to what
/// [`is_soft_body_inertia`] needs.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct SoftBodyMassDistribution {
    /// `Σ m p pᵀ` over the vertices in `f64`, `p` the position about the body origin and `m` the
    /// mass as Jolt computes it (`1.0f / w` in `f32`).
    second_moment: [[f64; 3]; 3],
    vertex_count: usize,
    /// Whether a vertex is kinematic: Jolt then gives the body infinite mass and inertia
    /// without decomposing anything.
    has_kinematic_vertex: bool,
    /// Whether every position has at most one non-zero component, so that every off-diagonal
    /// term Jolt adds is exactly zero.
    on_axes: bool,
}

impl SoftBodyMassDistribution {
    /// The distribution of vertices given as (position about the body origin, inverse mass).
    pub(crate) fn new(vertices: impl IntoIterator<Item = (Vec3, f32)>) -> Self {
        let mut distribution = Self {
            second_moment: [[0.0; 3]; 3],
            vertex_count: 0,
            has_kinematic_vertex: false,
            on_axes: true,
        };
        for (position, inverse_mass) in vertices {
            distribution.vertex_count += 1;
            if inverse_mass == 0.0 {
                distribution.has_kinematic_vertex = true;
                continue;
            }
            let mass = f64::from(1.0 / inverse_mass);
            let p = [position.x, position.y, position.z].map(f64::from);
            for (i, row) in distribution.second_moment.iter_mut().enumerate() {
                for (j, entry) in row.iter_mut().enumerate() {
                    *entry += mass * p[i] * p[j];
                }
            }
            distribution.on_axes &= p.iter().filter(|&&c| c != 0.0).count() <= 1;
        }
        distribution
    }
}

/// Whether Jolt can decompose the inertia of a soft body whose vertices have `distribution`
/// about its origin, rotated by `rotation` when Jolt bakes a rotation into the vertices,
/// without an assertion and into a finite inverse inertia (see
/// [Derived bounds](self#derived-bounds)).
pub(crate) fn is_soft_body_inertia(
    distribution: &SoftBodyMassDistribution,
    rotation: Option<Quat>,
) -> bool {
    if distribution.has_kinematic_vertex {
        return true;
    }
    let u = F32_UNIT_ROUNDOFF;
    let summed_terms = (distribution.vertex_count + 3) as f64 * u;
    if summed_terms >= 0.5 {
        return false;
    }
    // The identity quaternion gives Jolt the exact identity matrix.
    let rotation = rotation.filter(|q| !(q.x == 0.0 && q.y == 0.0 && q.z == 0.0));
    let c = match rotation {
        Some(q) => {
            let r = jolt_rotation_matrix(q);
            let rc = multiply(r, distribution.second_moment);
            multiply(rc, transpose(r))
        }
        None => distribution.second_moment,
    };
    let trace = c[0][0] + c[1][1] + c[2][2];
    let inertia: [[f64; 3]; 3] = std::array::from_fn(|i| {
        std::array::from_fn(|j| if i == j { trace - c[i][j] } else { -c[i][j] })
    });
    if rotation.is_none() && distribution.on_axes {
        // Jolt's tensor is exactly diagonal and its decomposition exact; as for rigid bodies,
        // a near-zero tensor gets the unit-sphere inertia and otherwise every moment is inverted.
        let diagonal = [inertia[0][0], inertia[1][1], inertia[2][2]];
        let length_sq: f64 = diagonal.iter().map(|d| d * d).sum();
        return length_sq <= 0.5e-12 || diagonal.iter().all(|&d| d >= 1.0e-30);
    }
    let position_error = if rotation.is_some() { 16.0 * u } else { 0.0 };
    let error = trace
        * ((1.0 + 3.0_f64.sqrt()) * (2.0 * position_error + position_error * position_error)
            + 3.0 * summed_terms / (1.0 - summed_terms) * (1.0 + position_error).powi(2));
    let frobenius = inertia.iter().flatten().map(|v| v * v).sum::<f64>().sqrt();
    let [[a, b, cc], [d, e, f], [g, h, i]] = inertia;
    let det = a * (e * i - f * h) - b * (d * i - f * g) + cc * (d * h - e * g);
    let minors = (a * e - b * d) + (a * i - cc * g) + (e * i - f * h);
    if !(det > 0.0 && minors > 0.0 && frobenius >= 1.0e-30) {
        return false;
    }
    // `det / minors` is at most the smallest principal moment: the sum of the principal
    // minors, the products of two moments, exceeds the product of the two larger ones.
    det / minors - error >= MIN_INERTIA_RATIO * (frobenius + error)
}

/// The matrix of Jolt's `Mat44::sRotation(q)`, in `f64`.
fn jolt_rotation_matrix(q: Quat) -> [[f64; 3]; 3] {
    let [x, y, z, w] = [q.x, q.y, q.z, q.w].map(f64::from);
    [
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ]
}

fn multiply(a: [[f64; 3]; 3], b: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| (0..3).map(|k| a[i][k] * b[k][j]).sum()))
}

fn transpose(a: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    std::array::from_fn(|i| std::array::from_fn(|j| a[j][i]))
}

/// Whether every component of `position` is at most [`MAX_POSITION`] in absolute value.
pub(crate) fn is_in_frame(position: RVec3) -> bool {
    [position.x, position.y, position.z]
        .iter()
        .all(|c| c.abs() <= MAX_POSITION)
}

/// Whether every component of `displacement` is at most `2 * MAX_POSITION` in absolute value:
/// the largest move between two positions in the frame.
pub(crate) fn is_frame_displacement(displacement: RVec3) -> bool {
    [displacement.x, displacement.y, displacement.z]
        .iter()
        .all(|c| c.abs() <= 2.0 * MAX_POSITION)
}

/// [`is_frame_displacement`] for an `f32` vector.
pub(crate) fn is_frame_span(span: Vec3) -> bool {
    [span.x, span.y, span.z]
        .iter()
        .all(|&c| Real::from(c).abs() <= 2.0 * MAX_POSITION)
}

/// Whether every component of `offset` is at most [`MAX_SHAPE_EXTENT`] in absolute value.
pub(crate) fn is_local_offset(offset: Vec3) -> bool {
    [offset.x, offset.y, offset.z]
        .iter()
        .all(|c| c.abs() <= MAX_SHAPE_EXTENT)
}

/// Whether `distance` is finite and within `0..=MAX_SHAPE_EXTENT`.
pub(crate) fn is_local_distance(distance: f32) -> bool {
    (0.0..=MAX_SHAPE_EXTENT).contains(&distance)
}

/// Whether `velocity` is finite and Jolt's own length of it at most [`MAX_LINEAR_VELOCITY`].
pub(crate) fn is_linear_velocity(velocity: Vec3) -> bool {
    velocity.is_finite() && jolt_length(velocity) <= MAX_LINEAR_VELOCITY
}

/// Whether `velocity` is finite and Jolt's own length of it at most [`MAX_ANGULAR_VELOCITY`].
pub(crate) fn is_angular_velocity(velocity: Vec3) -> bool {
    velocity.is_finite() && jolt_length(velocity) <= MAX_ANGULAR_VELOCITY
}

/// Whether `acceleration` is finite and its length, computed in `f64`, at most
/// [`MAX_ACCELERATION`].
pub(crate) fn is_acceleration(acceleration: Vec3) -> bool {
    acceleration.is_finite() && f64_length(acceleration) <= f64::from(MAX_ACCELERATION)
}

/// Whether `factor` is finite and at most [`MAX_GRAVITY_FACTOR`] in absolute value.
pub(crate) fn is_gravity_factor(factor: f32) -> bool {
    factor.abs() <= MAX_GRAVITY_FACTOR
}

/// Whether `friction` is finite and within `0..=MAX_FRICTION`.
pub(crate) fn is_friction(friction: f32) -> bool {
    (0.0..=MAX_FRICTION).contains(&friction)
}

/// Whether a character of `mass` updated with `gravity` for `delta_time` seconds presses with
/// a weight impulse of at most [`MAX_WEIGHT_IMPULSE`], computed in `f64` so that it cannot
/// overflow. The inputs are already checked to be finite and within their own bounds.
pub(crate) fn is_weight_impulse(mass: f32, gravity: Vec3, delta_time: f32) -> bool {
    f64::from(mass) * f64_length(gravity) * f64::from(delta_time) <= f64::from(MAX_WEIGHT_IMPULSE)
}

/// Whether `mass` is finite and within `MIN_MASS..=MAX_MASS`.
pub(crate) fn is_mass(mass: f32) -> bool {
    (MIN_MASS..=MAX_MASS).contains(&mass)
}

/// Whether `inverse_mass` is the inverse of a mass within `MIN_MASS..=MAX_MASS`, the range of
/// a movable soft body vertex.
pub(crate) fn is_vertex_inverse_mass(inverse_mass: f32) -> bool {
    (1.0 / MAX_MASS..=MAX_VERTEX_INVERSE_MASS).contains(&inverse_mass)
}

/// Whether `compliance` is finite and within `0..=MAX_COMPLIANCE`.
pub(crate) fn is_compliance(compliance: f32) -> bool {
    (0.0..=MAX_COMPLIANCE).contains(&compliance)
}

/// Whether `ratio` is finite and its magnitude within `1 / MAX_RATIO..=MAX_RATIO`.
pub(crate) fn is_ratio(ratio: f32) -> bool {
    (1.0 / MAX_RATIO..=MAX_RATIO).contains(&ratio.abs())
}

/// The lever-arm ratio (see [`MAX_LEVER_ARM_RATIO`]) of a dynamic body with `inverse_mass` and
/// principal inverse inertia `inverse_inertia` at `lever`, given in the body's principal frame;
/// computed in `f64`.
pub(crate) fn lever_arm_ratio(inverse_mass: f32, inverse_inertia: Vec3, lever: Vec3) -> f64 {
    let d = [inverse_inertia.x, inverse_inertia.y, inverse_inertia.z].map(f64::from);
    let r = [lever.x, lever.y, lever.z].map(f64::from);
    let squared = r.iter().map(|c| c * c).sum::<f64>();
    let trace: f64 = (0..3).map(|k| d[k] * (squared - r[k] * r[k])).sum();
    trace / f64::from(inverse_mass)
}

/// The largest lever-arm ratio of the same body at any point within `distance` of its centre of
/// mass: the two largest principal inverse inertias times `distance²`, over the inverse mass.
pub(crate) fn lever_arm_ratio_within(
    inverse_mass: f32,
    inverse_inertia: Vec3,
    distance: f64,
) -> f64 {
    let mut d = [inverse_inertia.x, inverse_inertia.y, inverse_inertia.z].map(f64::from);
    d.sort_by(f64::total_cmp);
    (d[1] + d[2]) * distance * distance / f64::from(inverse_mass)
}

/// Whether `ratio` is at most [`MAX_LEVER_ARM_RATIO`]; false for NaN.
pub(crate) fn is_lever_arm_ratio(ratio: f64) -> bool {
    ratio <= f64::from(MAX_LEVER_ARM_RATIO)
}

/// The length of `v`, computed in `f64` so that it cannot overflow.
pub(crate) fn f64_length(v: Vec3) -> f64 {
    let [x, y, z] = [v.x, v.y, v.z].map(f64::from);
    (x * x + y * y + z * z).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    const NON_FINITE: [f32; 3] = [f32::NAN, f32::INFINITY, f32::NEG_INFINITY];

    /// The axis-aligned vectors with `value` on one axis, in both directions.
    fn on_axes(value: f32) -> Vec<Vec3> {
        let mut vectors = Vec::new();
        for axis in 0..3 {
            for sign in [1.0, -1.0] {
                let mut v = [0.0; 3];
                v[axis] = sign * value;
                vectors.push(Vec3::from(v));
            }
        }
        vectors
    }

    fn real_on_axes(value: Real) -> Vec<RVec3> {
        let mut vectors = Vec::new();
        for axis in 0..3 {
            for sign in [1.0, -1.0] {
                let mut v = [0.0; 3];
                v[axis] = sign * value;
                vectors.push(RVec3::from(v));
            }
        }
        vectors
    }

    #[test]
    fn max_position_follows_the_precision_of_real() {
        let expected = if core::mem::size_of::<Real>() == 8 {
            1.0e7
        } else {
            5.0e3
        };
        assert_eq!(MAX_POSITION, expected);
    }

    #[test]
    fn position_checks_accept_their_bound_and_reject_beyond() {
        type Check = fn(RVec3) -> bool;
        let checks: [(Check, Real); 2] = [
            (is_in_frame, MAX_POSITION),
            (is_frame_displacement, 2.0 * MAX_POSITION),
        ];
        for (check, bound) in checks {
            for v in real_on_axes(bound) {
                assert!(check(v), "{v:?}");
            }
            for v in real_on_axes(bound.next_up()) {
                assert!(!check(v), "{v:?}");
            }
            for value in NON_FINITE {
                for v in real_on_axes(Real::from(value)) {
                    assert!(!check(v), "{v:?}");
                }
            }
        }
    }

    #[test]
    fn vector_checks_accept_their_bound_and_reject_beyond() {
        type Check = fn(Vec3) -> bool;
        // `Real` is `f32` without the `double-precision` feature, so the cast is a no-op there.
        #[allow(clippy::unnecessary_cast)]
        let span = (2.0 * MAX_POSITION) as f32;
        let checks: [(Check, f32); 5] = [
            (is_frame_span, span),
            (is_local_offset, MAX_SHAPE_EXTENT),
            (is_linear_velocity, MAX_LINEAR_VELOCITY),
            (is_angular_velocity, MAX_ANGULAR_VELOCITY),
            (is_acceleration, MAX_ACCELERATION),
        ];
        for (check, bound) in checks {
            for v in on_axes(bound) {
                assert!(check(v), "{v:?}");
            }
            for v in on_axes(bound.next_up()) {
                assert!(!check(v), "{v:?}");
            }
            for value in NON_FINITE {
                for v in on_axes(value) {
                    assert!(!check(v), "{v:?}");
                }
            }
        }
    }

    #[test]
    fn the_largest_vertex_inverse_mass_is_the_smallest_mass_for_jolt() {
        assert_eq!(1.0 / MAX_VERTEX_INVERSE_MASS, MIN_MASS);
        assert!(1.0 / MAX_VERTEX_INVERSE_MASS.next_up() < MIN_MASS);
    }

    #[test]
    fn scalar_checks_accept_their_range_and_reject_beyond() {
        type Check = fn(f32) -> bool;
        let checks: [(Check, &[f32], &[f32]); 6] = [
            (
                is_friction,
                &[0.0, MAX_FRICTION],
                &[-f32::MIN_POSITIVE, MAX_FRICTION.next_up()],
            ),
            (
                is_local_distance,
                &[0.0, MAX_SHAPE_EXTENT],
                &[-f32::MIN_POSITIVE, MAX_SHAPE_EXTENT.next_up()],
            ),
            (
                is_gravity_factor,
                &[-MAX_GRAVITY_FACTOR, 0.0, MAX_GRAVITY_FACTOR],
                &[
                    (-MAX_GRAVITY_FACTOR).next_down(),
                    MAX_GRAVITY_FACTOR.next_up(),
                ],
            ),
            (
                is_mass,
                &[MIN_MASS, MAX_MASS],
                &[0.0, MIN_MASS.next_down(), MAX_MASS.next_up()],
            ),
            (
                is_vertex_inverse_mass,
                &[1.0 / MAX_MASS, 1.0 / MIN_MASS, MAX_VERTEX_INVERSE_MASS],
                &[
                    0.0,
                    (1.0 / MAX_MASS).next_down(),
                    MAX_VERTEX_INVERSE_MASS.next_up(),
                ],
            ),
            (
                is_compliance,
                &[0.0, MAX_COMPLIANCE],
                &[-f32::MIN_POSITIVE, MAX_COMPLIANCE.next_up()],
            ),
        ];
        for (check, accepted, rejected) in checks {
            for &value in accepted {
                assert!(check(value), "{value}");
            }
            for &value in rejected.iter().chain(&NON_FINITE) {
                assert!(!check(value), "{value}");
            }
        }
    }

    /// The vector along `direction` (components 1 or 0) whose Jolt length is the largest at
    /// most `bound`, found by stepping one component.
    fn on_the_jolt_bound(direction: Vec3, bound: f32) -> Vec3 {
        let mut v = direction.scale(bound / jolt_length(direction));
        while jolt_length(v) > bound {
            v.x = v.x.next_down();
        }
        while jolt_length(Vec3::new(v.x.next_up(), v.y, v.z)) <= bound {
            v.x = v.x.next_up();
        }
        v
    }

    #[test]
    fn velocity_checks_use_jolts_length_on_diagonals() {
        for (check, bound) in [
            (is_linear_velocity as fn(Vec3) -> bool, MAX_LINEAR_VELOCITY),
            (is_angular_velocity, MAX_ANGULAR_VELOCITY),
        ] {
            let v = on_the_jolt_bound(Vec3::new(1.0, 1.0, 1.0), bound);
            assert!(check(v), "{v:?}");
            assert!(!check(Vec3::new(v.x.next_up(), v.y, v.z)), "{v:?}");
        }
    }

    #[test]
    fn weight_impulse_keeps_the_angular_speed_of_the_ground_body_finite() {
        let largest_kept_inverse_inertia = 3.0_f64.sqrt() * 1.0e6;
        let largest_lever = 3.0_f64.sqrt() * f64::from(MAX_SHAPE_EXTENT);
        let angular_speed =
            largest_kept_inverse_inertia * largest_lever * f64::from(MAX_WEIGHT_IMPULSE)
                + f64::from(MAX_ANGULAR_VELOCITY);
        assert!(angular_speed * angular_speed < f64::from(f32::MAX) / 4.0);
        let sphere_inverse_inertia = 2.5 / f64::from(MIN_MASS);
        assert!(sphere_inverse_inertia < largest_kept_inverse_inertia);
    }

    #[test]
    fn weight_impulse_check_accepts_its_bound_and_rejects_beyond() {
        let gravity = MAX_WEIGHT_IMPULSE / MAX_MASS;
        for down in on_axes(gravity) {
            assert!(is_weight_impulse(MAX_MASS, down, 1.0), "{down:?}");
            assert!(is_weight_impulse(0.0, down, 1.0), "{down:?}");
        }
        for down in on_axes(gravity.next_up()) {
            assert!(!is_weight_impulse(MAX_MASS, down, 1.0), "{down:?}");
        }
        assert!(!is_weight_impulse(
            MAX_MASS,
            Vec3::new(0.0, -gravity, 0.0),
            1.0f32.next_up()
        ));
    }

    #[test]
    fn lever_arm_ratios_measure_the_lever_against_the_radius_of_gyration() {
        // A 2 kg cube of side 1 m: inertia 2 / 6 about every axis, radius of gyration² 1 / 6.
        let inverse_inertia = Vec3::new(3.0, 3.0, 3.0);
        let ratio = lever_arm_ratio(0.5, inverse_inertia, Vec3::new(0.0, 2.0, 0.0));
        assert!((ratio - 2.0 * 4.0 * 6.0).abs() < 1e-9, "{ratio}");
        // A thin rod of 1 kg and 2 m along y held at its end: 6 whatever its thickness.
        let rod = Vec3::new(3.0, 1.0e4, 3.0);
        let ratio = lever_arm_ratio(1.0, rod, Vec3::new(0.0, 1.0, 0.0));
        assert!((ratio - 6.0).abs() < 1e-9, "{ratio}");
        // Anywhere within a distance: the two largest inverse inertias.
        let within = lever_arm_ratio_within(1.0, rod, 1.0);
        assert!((within - (1.0e4 + 3.0)).abs() < 1e-6, "{within}");
        let bound = f64::from(MAX_LEVER_ARM_RATIO);
        assert!(is_lever_arm_ratio(bound));
        assert!(!is_lever_arm_ratio(bound.next_up()));
        assert!(!is_lever_arm_ratio(f64::NAN));
        assert!(!is_lever_arm_ratio(f64::INFINITY));
    }

    #[test]
    fn accelerations_are_bounded_by_the_speed_clamp_per_step() {
        let per_step = MAX_ACCELERATION * PhysicsWorld::MIN_DELTA_TIME;
        assert!((per_step - MAX_LINEAR_VELOCITY).abs() <= MAX_LINEAR_VELOCITY * 1.0e-6);
        let per_step = MAX_ANGULAR_ACCELERATION * PhysicsWorld::MIN_DELTA_TIME;
        assert!((per_step - MAX_ANGULAR_VELOCITY).abs() <= MAX_ANGULAR_VELOCITY * 1.0e-6);
    }

    /// The tetrahedron with a right corner at `origin`, unit edges along x and z and the fourth
    /// vertex `height` above `(1, 0, 1)`, faces wound counter-clockwise seen from outside.
    fn tetrahedron(origin: Vec3, height: f32) -> (Vec<Vec3>, Vec<[u32; 3]>) {
        let positions = [
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, height, 1.0],
        ]
        .map(|[x, y, z]| Vec3::new(origin.x + x, origin.y + y, origin.z + z))
        .to_vec();
        (positions, vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]])
    }

    /// Six-volumes as Jolt might compute them in `f32` after rotating the positions by
    /// `rotation` with Jolt's quaternion rotation: once with separate multiplications and once
    /// with fused multiply-adds in the cross and dot products.
    fn f32_six_volumes(positions: &[Vec3], faces: &[[u32; 3]], rotation: crate::Quat) -> [f32; 2] {
        let rotated: Vec<Vec3> = positions
            .iter()
            .map(|&p| crate::math::jolt_rotate(rotation, p))
            .collect();
        let mut plain = 0.0_f32;
        let mut fused = 0.0_f32;
        for &[a, b, c] in faces {
            let [x1, x2, x3] = [a, b, c].map(|i| rotated[i as usize]);
            let cross = Vec3::new(
                x1.y * x2.z - x1.z * x2.y,
                x1.z * x2.x - x1.x * x2.z,
                x1.x * x2.y - x1.y * x2.x,
            );
            plain += (cross.x * x3.x + cross.y * x3.y) + cross.z * x3.z;
            let cross = Vec3::new(
                x1.y.mul_add(x2.z, -(x1.z * x2.y)),
                x1.z.mul_add(x2.x, -(x1.x * x2.z)),
                x1.x.mul_add(x2.y, -(x1.y * x2.x)),
            );
            fused += cross.z.mul_add(x3.z, cross.x.mul_add(x3.x, cross.y * x3.y));
        }
        [plain, fused]
    }

    #[test]
    fn pressure_volume_bound_is_below_f32_six_volumes() {
        // Seeded rotations, normalized within Jolt's tolerance.
        let mut seed = 0x2545_f491_u32;
        let mut next = || {
            seed ^= seed << 13;
            seed ^= seed >> 17;
            seed ^= seed << 5;
            seed as f32 / u32::MAX as f32 * 2.0 - 1.0
        };
        let mut checked = 0;
        // Rounding grows with the distance from the origin, so heights from 1e-12 to 1 cover
        // the region where the bound becomes positive for each origin.
        for origin in [
            Vec3::ZERO,
            Vec3::new(3.0, -2.0, 4.0),
            Vec3::new(10.0, -7.0, 15.0),
        ] {
            for step in 0..=48 {
                let height = 10.0_f32.powf(-12.0 + step as f32 / 4.0);
                let (positions, faces) = tetrahedron(origin, height);
                let geometry = SoftBodyPressureGeometry::new(&positions, &faces);
                if geometry.six_volume_low <= 0.0 {
                    continue;
                }
                for _ in 0..64 {
                    let rotation =
                        crate::Quat::from_xyzw(next(), next(), next(), next()).normalized();
                    for volume in f32_six_volumes(&positions, &faces, rotation) {
                        assert!(
                            f64::from(volume) >= geometry.six_volume_low,
                            "origin {origin:?}, height {height}: {volume} < {}",
                            geometry.six_volume_low
                        );
                    }
                    checked += 1;
                }
            }
        }
        assert!(checked >= 64 * 30, "{checked}");
    }

    #[test]
    fn pressure_needs_a_positive_volume_and_bounds_the_vertex_force() {
        // The thin tetrahedron of a review: 1 m edges, faces with area, six-volume 1e-36.
        let (positions, faces) = tetrahedron(Vec3::ZERO, 1.0e-36);
        let sliver = SoftBodyPressureGeometry::new(&positions, &faces);
        assert!(sliver.six_volume_low < 0.0, "{sliver:?}");
        assert!(is_soft_body_pressure(0.0, &sliver));
        assert!(!is_soft_body_pressure(f32::MIN_POSITIVE, &sliver));
        // No faces enclose nothing.
        let empty = SoftBodyPressureGeometry::new(&positions, &[]);
        assert!(!is_soft_body_pressure(f32::MIN_POSITIVE, &empty));
        // A unit tetrahedron: the vertex of the largest face sum is (1, 1, 1), whose faces have
        // the edge products sqrt(3) * 1, 1 * sqrt(3) and sqrt(2) * sqrt(2).
        let (positions, faces) = tetrahedron(Vec3::ZERO, 1.0);
        let unit = SoftBodyPressureGeometry::new(&positions, &faces);
        let area = 2.0 * 3.0_f64.sqrt() + 2.0;
        assert!(
            (unit.largest_vertex_face_area / area - 1.0).abs() < 1.0e-3,
            "{unit:?}"
        );
        assert!((unit.six_volume_low - 1.0).abs() < 1.0e-4, "{unit:?}");
        let bound = MAX_PRESSURE_VERTEX_FORCE * unit.six_volume_low / unit.largest_vertex_face_area;
        let bound = bound as f32;
        assert!(is_soft_body_pressure(bound.next_down(), &unit));
        assert!(!is_soft_body_pressure(bound.next_up(), &unit));
    }

    /// An 11 × 11 cloth of 1 kg vertices, 0.1 m apart, centred `distance` from the origin
    /// along (1, 0, 1).
    fn offset_cloth(distance: f32) -> SoftBodyMassDistribution {
        let offset = distance / 2.0_f32.sqrt();
        SoftBodyMassDistribution::new((0..121).map(|i| {
            let x = (i % 11) as f32 * 0.1 - 0.5 + offset;
            let z = (i / 11) as f32 * 0.1 - 0.5 + offset;
            (Vec3::new(x, 0.0, z), 1.0)
        }))
    }

    /// `scale · R diag(1, 1, moment) Rᵀ` for a fixed general rotation `R`.
    fn rotated_needle(moment: f64, scale: f64) -> [[f64; 3]; 3] {
        let r = jolt_rotation_matrix(crate::Quat::from_xyzw(0.3, -0.2, 0.5, 0.78).normalized());
        let d = [1.0, 1.0, moment];
        std::array::from_fn(|i| {
            std::array::from_fn(|j| scale * (0..3).map(|k| r[i][k] * d[k] * r[j][k]).sum::<f64>())
        })
    }

    #[test]
    fn rigid_body_inertia_floor_holds_at_its_boundary() {
        // For diag(1, 1, m), det / minors = m / (1 + 2m) and |I|_F = sqrt(2 + m²).
        let (mut low, mut high) = (0.0_f64, 1.0_f64);
        for _ in 0..200 {
            let m = 0.5 * (low + high);
            if m / (1.0 + 2.0 * m) >= MIN_INERTIA_RATIO * (2.0 + m * m).sqrt() {
                high = m;
            } else {
                low = m;
            }
        }
        assert!((6.7e-4..6.9e-4).contains(&high), "{high}");
        // The rule does not depend on the scale, also below Jolt's near-zero limit.
        for scale in [1.0e-30, 1.0e-9, 1.0, 1.0e9] {
            assert!(is_rigid_body_inertia(rotated_needle(
                high * (1.0 + 1e-6),
                scale
            )));
            assert!(!is_rigid_body_inertia(rotated_needle(
                high * (1.0 - 1e-6),
                scale
            )));
        }
        // Not positive definite.
        assert!(!is_rigid_body_inertia(rotated_needle(0.0, 1.0)));
        assert!(!is_rigid_body_inertia(rotated_needle(-0.1, 1.0)));
    }

    #[test]
    fn soft_body_inertia_needs_vertices_around_the_origin() {
        let rotation = crate::Quat::from_xyzw(0.3, -0.2, 0.5, 0.78).normalized();
        for rotation in [None, Some(rotation)] {
            for distance in [0.0, 1.0, 5.0, 10.0] {
                assert!(is_soft_body_inertia(&offset_cloth(distance), rotation));
            }
            for distance in [15.0, 50.0, 1000.0] {
                assert!(!is_soft_body_inertia(&offset_cloth(distance), rotation));
            }
        }
        // The identity rotation is no rotation for Jolt.
        assert!(is_soft_body_inertia(
            &offset_cloth(10.0),
            Some(crate::Quat::IDENTITY)
        ));
        // A kinematic vertex skips the decomposition.
        let pinned = SoftBodyMassDistribution::new([
            (Vec3::new(1000.0, 0.0, 1000.0), 0.0),
            (Vec3::new(1000.1, 0.0, 1000.0), 1.0),
        ]);
        assert!(is_soft_body_inertia(&pinned, None));
        // Exactly diagonal tensors: near zero, or every moment invertible.
        let at = |positions: &[[f32; 3]]| {
            SoftBodyMassDistribution::new(positions.iter().map(|&p| (Vec3::from(p), 1.0)))
        };
        assert!(is_soft_body_inertia(&at(&[[0.0; 3]]), None));
        assert!(is_soft_body_inertia(
            &at(&[[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, -3.0]]),
            None
        ));
        assert!(!is_soft_body_inertia(&at(&[[1.0, 0.0, 0.0]]), None));
        assert!(!is_soft_body_inertia(
            &at(&[[1.0, 0.0, 0.0], [-2.0, 0.0, 0.0]]),
            None
        ));
        // A line off the axes has the same zero moment.
        assert!(!is_soft_body_inertia(
            &at(&[[0.6, 0.8, 0.0], [1.2, 1.6, 0.0]]),
            None
        ));
        // A rotation makes the axis-aligned set general; its moments still pass.
        assert!(is_soft_body_inertia(
            &at(&[[1.0, 0.0, 0.0], [0.0, 2.0, 0.0], [0.0, 0.0, -3.0]]),
            Some(rotation)
        ));
    }

    #[test]
    fn soft_body_force_is_bounded_whatever_the_inverse_masses() {
        let bound = f64::from(MAX_ACCELERATION) * f64::from(MAX_MASS);
        for w in [0.0, 1.0 / MAX_MASS] {
            assert!(is_soft_body_force([bound, 0.0, 0.0], w, 4));
            assert!(!is_soft_body_force([bound * (1.0 + 1e-12), 0.0, 0.0], w, 4));
        }
        let per_vertex = 4.0 * f64::from(MAX_ACCELERATION);
        assert!(is_soft_body_force([0.0, per_vertex, 0.0], 1.0, 4));
        assert!(!is_soft_body_force(
            [0.0, per_vertex, 0.0],
            1.0f32.next_up(),
            4
        ));
        assert!(!is_soft_body_force([f64::NAN, 0.0, 0.0], 0.0, 4));
    }
}
