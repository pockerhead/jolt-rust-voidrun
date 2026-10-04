# Limits

Why the magnitudes `oxijolt::limits` accepts have the values they have: for each bound, the Jolt
arithmetic it keeps finite. [coverage.md](coverage.md) lists every input with its rule and
boundary test, the paths only tests cover, and what no bound excludes. Line numbers refer to the
vendored Jolt 5.6 sources (`crates/oxijolt-sys/vendor/JoltPhysics/Jolt/Physics/`).

The derivations hold with every input at its bound and a time step `dt <= 1` s
(`PhysicsWorld::MAX_DELTA_TIME`); numbers are rounded.

## Frame and extent

`MAX_POSITION` follows Jolt's "Big Worlds" documentation (`Docs/Architecture.md`):
single-precision simulation is accurate within roughly 5 km of the origin, and double precision
handles worlds of thousands of km; at 10 000 km Jolt's `f32` broad phase still has a resolution
of about 1 m. It is not a Jolt assertion threshold.

`MAX_SHAPE_EXTENT` follows Jolt's "Conventions and Limits" documentation, which recommends static
objects of 0.1 to 2000 m; the bound applies on each side of the centre of mass. It bounds a
shape's inertia to at most `6 · mass · MAX_SHAPE_EXTENT²`.

## Accelerations

`MAX_ACCELERATION` is `MAX_LINEAR_VELOCITY / PhysicsWorld::MIN_DELTA_TIME`, about 5e8 m/s². A
larger acceleration already reaches Jolt's speed clamp within every step `PhysicsWorld::step`
accepts, so the bound removes no motion that the clamp keeps. `MAX_ANGULAR_ACCELERATION` is the
same for Jolt's angular speed clamp.

## Velocities at creation

Jolt asserts `Length() <= mMaxLinearVelocity` (and the angular
counterpart) when it creates a body (`Body.cpp:424`, `MotionProperties.h:48`). The checks
compare Jolt's own `Vec3::Length`, called through joltc, with Jolt's defaults; a test creates
bodies on the bound in 64 directions, which the asserts leg runs.

`MAX_LINEAR_VELOCITY` is Jolt's default `BodyCreationSettings::mMaxLinearVelocity`
(`BodyCreationSettings.h:111`), `MAX_ANGULAR_VELOCITY` its default `mMaxAngularVelocity`
(`BodyCreationSettings.h:112`).

## Integration

Each step Jolt adds gravity times the gravity factor and the accumulated
force times the inverse mass to the velocity, then asserts that the squared speed is finite
(`MotionProperties.inl:26-28`) before clamping it. Before the clamp
`|v| <= 500 + (|gf|·|g| + |F|/m)·dt <= 500 + 1000·5e8 + 2·5e8`, about 5e11 m/s, where the
factor 2 also covers a vehicle's gravity force on its chassis. Its square, about 2.5e23, is
finite. The angular velocity is bounded the same way with `MAX_ANGULAR_ACCELERATION` and
the largest principal inverse inertia.

## Force and torque accumulation

With `mass <= MAX_MASS` an accepted accumulated force is
at most `MAX_ACCELERATION · MAX_MASS`, about 5e14 N, so Jolt's `f32` sums (`Body.h:183,191`)
and `mInvMass * F` (`MotionProperties.inl:134`) cannot overflow. For a force at a point the
lever must be finite in `f32` and every product `lever_i · force_j` at most 1e37, so Jolt's
cross product (`Body.inl:127-131`) stays finite even when its two products cancel.

## Soft body forces

Jolt adds a soft body's accumulated force to every vertex as
`F · w / N · dt` (`SoftBodyMotionProperties.cpp:334`), `w` the vertex's inverse mass and `N`
the vertex count. The accumulated force may give the vertex of the largest inverse mass at
most `MAX_ACCELERATION`, and is at most `MAX_ACCELERATION · MAX_MASS` (5e14 N) whatever
the inverse masses, as on a rigid body: a body whose vertices are all pinned (`w = 0`)
cannot collect an unbounded force. Changing a vertex's inverse mass rechecks the force
accumulated in the current step against the new inverse masses, so unpinning a vertex
cannot release a force beyond the bound either.

## Soft body pressure

Before each solver sub-step of `dt` seconds Jolt computes the
six-volume `V = Σ (x1 × x2) · x3` over the faces in `f32`, from the vertex positions about
the body origin, and when `V > 0` adds `w · pressure · dt / V · ((x2 - x1) × (x3 - x1))` to
the velocity of each vertex of every face (`SoftBodyMotionProperties.cpp:107-118,291-322`);
nothing bounds `1 / V`. `PhysicsWorld::create_soft_body` accepts a pressure only when
`pressure · A <= MAX_ACCELERATION / MAX_VERTEX_INVERSE_MASS · V_low`, where `V_low` is a lower bound of
the six-volume Jolt computes before the first step and `A` the largest sum, over the faces
of one vertex, of an upper bound of `|x2 - x1| · |x3 - x1|` in Jolt's arithmetic. So the
faces must enclose a positive volume, wound counter-clockwise seen from outside, and no
vertex, whatever its inverse mass (at most `MAX_VERTEX_INVERSE_MASS`, also after
`SoftBodyMut::set_vertex_inverse_mass`),
gains more than `MAX_ACCELERATION · dt` per sub-step from the pressure at the start
geometry; the pressure coefficient and impulses stay finite, so a kinematic vertex gets
`0 · finite`, not `0 · ∞`. `V_low` does not replay Jolt's `f32` operations, whose order
and rounding depend on the build (SSE4.1 `dpps`, fused multiply-adds, the rotation Jolt
bakes into the vertices); it is the six-volume in `f64` minus a bound of everything those
operations can change, with `u = 2^-24` and `|p|` a vertex's distance from the body
origin: a rotation within Jolt's normalization tolerance scales the six-volume by
`1 ± 4e-5` and distances by at most `1 + 2e-5`, and its rounding moves a position by at
most `15 u |p|`; one face's term `(x1 × x2) · x3` is off by at most
`64 u |p1| |p2| |p3|`; each addition of the running sum rounds by at most `u` times the
partial sum, which grows the error by at most `(1 - u)^-N` for `N` faces; underflow adds
at most `1e-36` per face. An edge in `A` is bounded by
`1.0001 · (|e| + 15 u (|pa| + |pb|))`. A tetrahedron with three 1 m edges at a right
corner takes a pressure of up to about 9e4; the ball of radius 0.5 m in
`pressure_at_the_bound_steps_finitely` takes `MAX_SOFT_BODY_PRESSURE`.

`MAX_SOFT_BODY_PRESSURE` is measured: Jolt applies `pressure · dt / (6 · volume)` times each
face's area as an impulse (`SoftBodyMotionProperties.cpp:290-312`), and a closed ball of 1 m with
vertex masses at `MIN_MASS` and at the total-mass bound, at this pressure, stepped 600 times on a
floor in the `asserts` build, stays finite (`pressure_at_the_bound_steps_finitely`).

## Rigid body inertia

Jolt decomposes the inertia tensor of every body that is not static
with `EigenValueSymmetric` when it creates the body (`MotionProperties::SetMassProperties`,
also for a ragdoll part in `Ragdoll::Stabilize`), before it checks whether the moments are
near zero. A tensor that is not exactly diagonal comes from a compound child's rotation or
position, or from an offset centre of mass. Its decomposition asserts as the soft body
inertia's below does, and the same floor applies: `PhysicsWorld::create_body`, the inner
body of `PhysicsWorld::create_character`, and the
parts of `RagdollSettings::new` and
`RagdollSettings::new_stabilized` accept such a
tensor only when `det I / (sum of the principal 2 × 2 minors)`, a lower bound of its smallest
principal moment, is at least `1001 · 8 · u · |I|_F`, about 4.8e-4 of its Frobenius norm. The
check reads the `f32` tensor Jolt computes itself (joltc's `JPH_Shape_GetMassProperties` and
`JPH_MassProperties_ScaleToMass`), so it needs no error term for the tensor. `Stabilize`
(`Ragdoll.cpp:133-183`) multiplies each part's tensor by its new mass over its old, one
rounding per element, before it decomposes it, and rebuilds a parent's tensor from that
decomposition with every moment raised to at least the smaller of twice its largest and
its children's sum; neither moves the smallest moment down by more than a few `u |I|_F`,
far inside the margin. An exactly diagonal tensor is decomposed exactly and only needs
invertible moments or Jolt's near-zero fallback.
In the asserts build, seeded thin boxes, capsules and cylinders in rotated compound children,
pairs of such children, and offset centres of mass, with half lengths from 1 nm to 50 m and
masses from 1 g to 1000 t, asserted for `λ_min / |I|_F` up to 8.2e-5 under the earlier rule
(`det I / |I|_F² >= 1e-5 |I|_F`, skipped for a tensor below Jolt's near-zero limit), and none
of the 5 271 accepted at or above 1e-4 did. Made thinner until this floor refuses them, 2 703
such shapes were created and stepped without an assertion, and about 2 500 with the floor
divided by 3; divided by 4 one of ten seeded runs asserted, divided by 6 all ten did, so the
margin over Jolt's onset is 3 to 4, as for soft bodies. The floor refuses a square needle box
in a rotated child when it is about 54 times longer than wide, a capsule or cylinder about 47
times longer than its diameter, also where Jolt decomposed it without an assertion.
In the asserts leg, `rigid_body_inertia_at_its_bound_decomposes` (bodies),
`inner_body_inertia_at_its_bound_decomposes` (character inner bodies) and
`stabilized_ragdoll_inertia_at_its_bound_decomposes` (a three-part chain whose masses
`Stabilize` redistributes) create and step seeded rotated slender shapes at the thinnest
thickness the floor accepts and check that one 1e-4 thinner is refused; each aborts with
the floor divided by 6 and passes with it divided by 3.

## Soft body inertia

Jolt sums a soft body's inertia tensor about the body origin in
`f32` from its vertices (`SoftBodyMotionProperties::CalculateMassAndInertia`) when the body
is created and after every
`SoftBodyMut::set_vertex_inverse_mass`, and
decomposes it with `EigenValueSymmetric`, which asserts that every eigenvector `v` with
eigenvalue `λ` satisfies `|M v − λ v|² <= 1e-6 · max(|M v|², λ²)` (`EigenValueSymmetric.h:88`).
A body with a kinematic vertex skips all of this (infinite mass and inertia). Jacobi's
residual `|M v − λ v|` is a multiple `k` of `u |M|_F`, `u = 2^-24`, whatever the
eigenvalue, so the assertion holds for the smallest principal moment `λ_min` when
`λ_min >= 1001 · k · u · |M|_F`. A scalar `f32` emulation of Jolt's decomposition
measured `k` at most 4 over 42 000 tensors (random spectra and rotations, near-axis tilts,
and tensors summed from random point clouds), so `k` is measured, not derived; the bound
takes `k = 8`, a ratio `λ_min / |M|_F` of at least about 4.8e-4. In the asserts build,
4 000 seeded clouds of point masses moved to the boundary of this rule with its floor
divided by 3 (and 4 random rotations each) decomposed without an assertion, and with the
floor divided by 4 Jolt asserted, so the margin over Jolt's measured onset is 3 to 4. The
asserts leg creates bodies at the bound in `soft_body_inertia_at_its_bound_decomposes`
(which aborts with the floor divided by 4). Jolt's second
check, that the decomposition rebuilds each column of `M` to `1e-5` of its length
(`MassProperties.cpp:56`), never fired in the emulation for tensors of point masses: it
needs two moments much smaller than the third, which point masses cannot give (each
moment is at most the sum of the other two).
`PhysicsWorld::create_soft_body` and `set_vertex_inverse_mass` check
it in `f64` with `λ_min` bounded from below by `det M / (sum of the principal 2 × 2
minors)`, after subtracting a bound of how far Jolt's `f32` tensor can be from the `f64`
one in Frobenius norm: `3 γ_{N+3} Σ m |p|²` for the `N`-term sums of terms of at most three
roundings, and, when Jolt bakes a rotation into the vertices (computed with Jolt's
rotation matrix in `f64`), `(1 + √3)(2 δ + δ²) Σ m |p|²` for its rounding of each position
by at most `δ |p|`, `δ = 16 u` (the 15 u of the pressure bound with the rotation's slack).
A tensor Jolt sums exactly diagonal (no rotation, every vertex on a coordinate axis) is
decomposed exactly; it is accepted as a rigid body's is, when its moments are all above
1e-30 or it is near zero. The rule refuses a body whose vertices lie far from its origin
compared with their spread: an 11 × 11 cloth of 1 m with 1 kg vertices is accepted at
11 m from its origin and refused from 12 m (Jolt asserted at 50 m), and a free straight
line of vertices, whose smallest moment is zero. The rule does not know the shape, so it
also refuses free bodies thinner than about 1/70 of their length (ribbons) or with a
radius below about 1/130 of their length (tubes, ropes), which Jolt decomposes: centred
on the origin, a 2.5 m ribbon 1, 2 or 3 cm wide and a 3 m tube of radius 1 or 2 cm were
refused, and Jolt created and stepped them without an assert at baked rotations of 0,
0.001 and 0.3 rad. A kinematic vertex skips the check.

## Soft body edge length

Jolt only asserts that a rest length is above zero (`SoftBodySharedSettings.cpp:226,377`) and
divides by edge lengths while it solves; `MIN_SOFT_BODY_EDGE_LENGTH` keeps a degenerate edge out
of the solver with a margin.

## Soft body long-range attachments

An LRA rest distance is at most the sum of all edge lengths, below
`2³² · 2√3 · MAX_SHAPE_EXTENT` (about 3e13 m). With the multiplier of
`SoftBodyVertexAttributes::long_range_attachment` at most `MAX_RATIO` it is at most 3e17 m, and
Jolt's square of it (`SoftBodyMotionProperties.cpp:695`) stays below 1e35.

## Soft body compliance

Jolt divides each compliance by the squared sub-step
(`SoftBodyMotionProperties.cpp:371,445,496,577,594`). `PhysicsWorld::step` always runs one
collision step, so a sub-step is at least `PhysicsWorld::MIN_DELTA_TIME` divided by
`SoftBodySettings::MAX_ITERATIONS`, 1e-8 s, and `compliance / dt²` is at most
`1e20 · 1e16 = 1e36`, below `f32::MAX`; Jolt's average of two compliances, `0.5 · (c1 + c2)`, stays
finite as well. This proves that the product is finite, not that the solver is stable at every
compliance.

## Vehicle gravity

Jolt adds `gravity / inverse_mass` to the chassis
(`VehicleConstraint::OnStep`); the chassis is a dynamic body, so the force is at most
`5e8 · 1e6`, about 5e14 N.

## Character weight and push

A character presses on what it stands on with the impulse
`mass · |g| · dt` at the ground contact point (`CharacterVirtual.cpp:1474-1481`), so the
impulse also turns the ground body. `PhysicsWorld::update_character` accepts at most
`MAX_WEIGHT_IMPULSE`, 1e9 N·s, which changes the linear velocity of a body of the
smallest mass by at most 1e12 m/s and the angular velocity of a body whose principal
inverse inertia is at most `√3 · 1e6` by at most about 6e18 rad/s; both squares are
finite (derivation below). Its push impulse is capped at
`delta_velocity / inv_effective_mass` (`CharacterVirtual.cpp:795-811`), whose effective
mass includes the body's rotation, so the velocity change at the contact is at most the
relative normal speed whatever the strength.

`MAX_WEIGHT_IMPULSE` in detail. Jolt keeps a body's principal moments of inertia only while their
vector is longer than 1e-6 (`Vec3::IsNearZero` in `MotionProperties.cpp:46-56`) and otherwise
uses the inertia of a sphere of radius 1, an inverse of `2.5 / mass`, at most 2500 for
`MIN_MASS`. So a body whose principal moments are equal has an inverse inertia of at most
`√3 · 1e6`, and the ground contact lies within `√3 · MAX_SHAPE_EXTENT` of its centre of mass. For
such a body and every body with a smaller principal inverse inertia, the angular velocity change
is at most `√3e6 · 3464 · 1e9`, about 6e18 rad/s, whose square is finite; the linear velocity
change is at most `1e9 · 1e3` m/s. Without the bound, a character of `MAX_MASS` at
`MAX_ACCELERATION` with a one-second update (5e14 N·s) on the edge of a 6 cm cube of `MIN_MASS`
overflows the cube's squared angular speed, which Jolt asserts on (`MotionProperties.inl:38`).

## Springs

Jolt derives a stiffness `k` and damping `c` from every spring
(`SpringPart.h:36-55,91-104`). Both stay at most `MAX_SPRING_COEFFICIENT`: in stiffness mode
directly, in frequency mode through an upper bound of the effective mass. For a ragdoll joint
that bound is computed at creation from the parts' masses and inertias, including Jolt's
`Stabilize` (`Ragdoll.cpp:135-185`); see `SpringSettings`. For a
world constraint it is computed at creation from its two bodies: over the dynamic ones, the
larger of the mass and the largest principal moment of inertia
(`PhysicsWorld::create_constraint`); the constraint's spring setters use the same bound.
For a wheel's suspension it is `MAX_MASS`, since Jolt's suspension effective mass is at most the
chassis mass (`VehicleConstraint.cpp:448-451`).

`MAX_SPRING_COEFFICIENT` keeps `c + dt · k` finite (at most 2e30 for `dt <= 1`), so the softness,
bias and effective mass Jolt computes from them stay finite.

## Anti-roll bars

Jolt computes `stiffness · length difference · dt` for each bar
(`VehicleConstraint.cpp:289-293`) and passes it as the bias `b` of the wheel's suspension
constraint (`VehicleConstraint.cpp:508`), whose impulse is `-K⁻¹ (J v + b)`
(`AxisConstraintPart.h:300-301`): a velocity term, scaled by an effective mass that is at
most each body's own along the axis. With
`VehicleAntiRollBar::MAX_STIFFNESS` and wheel
lengths at most `MAX_SHAPE_EXTENT`, `b` is at most 5e14 m/s, so the velocity change along
the suspension axis stays finite and squares finitely.

## Restitution

At most 1, so the restitution target speed is at most the approach speed.

## Friction

At most `MAX_FRICTION`, so Jolt's combined friction `sqrt(f1 · f2)`
(`ContactConstraintManager.h:554`) is finite and the friction impulse bound, combined friction
times the normal impulse (`ContactConstraintManager.cpp:1714-1715`), is never `0 · ∞` = NaN.
A cube sliding on a floor, both at `f32::MAX`, had NaN velocities within three 60 Hz steps.

`MAX_FRICTION` keeps that product finite: any coefficient whose square is finite (below about
1.8e19) does, and 1000 is far above the friction of real materials.

## Coupling ratios

Jolt multiplies the inverse mass or inertia of body 2 by the ratio's square in the effective mass,
and body 2's velocity (for racks and pulleys also its impulse) by the ratio
(`GearConstraintPart.h:81,122`, `RackAndPinionConstraintPart.h:82,123`,
`IndependentAxisConstraintPart.h:71,112` for pulleys). With a principal inverse inertia of at most
`√3 · 1e6` (see "Character weight and push"), `MAX_RATIO² · I⁻¹` is at most about 1.7e14, far
from `f32` overflow. A ratio of 1e4 already turns a pinion ten thousand radians per metre of its
rack; tests step both bounds on the lightest and heaviest bodies.

Gears get a tighter range, `1..=MAX_GEAR_RATIO`, because of a defect in Jolt's gear solver
(Jolt 5.6, unchanged on Jolt's master as of 2026-09-28): `GearConstraintPart::ApplyVelocityStep`
and `SolvePositionConstraint` apply the impulse to body 2 as `λ · I2⁻¹ · b`, without the ratio of
the Jacobian `[a, r·b]`, while the effective mass `1 / (A + r²·B)` includes it squared (`A`, `B`
the inverse inertias about the two axes). Each solver iteration therefore keeps
`1 − (A + r·B) / (A + r²·B)` of the velocity error `ω1 + r · ω2`:
- below 1 it approaches `1 − 1/r` for a light body 2, which is negative: below 1/2, and for every
  negative ratio, its magnitude exceeds 1 and the error grows until the bodies' velocities are not
  finite;
- from 1 up it is in `[0, 1)` but approaches `1 − 1/r` for a heavy body 1, so with Jolt's 10
  velocity iterations per step the gear needs more steps to restore the relation the larger the
  ratio.

The upper bound is measured. `MAX_GEAR_RATIO` is the largest
round ratio that, after a disturbance, brings the error back to within 2 % of its initial value
within 10 steps for the mass distributions measured: worst 1.6 % at ratio 10, 6.5 % at 20, 60 % at
100, and at 1e4 91 % still after 60 steps. The first step after a disturbance leaves up to
`(1 − 1/ratio)^10`, 35 % at ratio 10. `gear_keeps_its_velocity_relation_at_the_largest_ratio`
tests it at the bound.

## Lever-arm ratio

`MAX_LEVER_ARM_RATIO` is measured, not derived. Jolt solves each constraint part with its
effective mass `K = Σ (m⁻¹ · 1 + [r]× I⁻¹ [r]×ᵀ)` (`PointConstraintPart.h`,
`AxisConstraintPart.h`) in `f32`. A ratio of at most `B` per body bounds the lever terms by
`B · m⁻¹`, so `K`'s condition number stays below `1 + B`. Two failures were measured with a body
at the velocity bounds, under gravity, for 120 steps, in the `asserts` build:
- a body held far from its centre of mass: a 1 g, 6 cm cube on a hinge 116 m away (a ratio of
  4.5e7) went to NaN, two 1 kg, 1 m cubes joined by a point 3000 m away (1.1e8) moved
  erratically and at 4000 m Jolt asserted that a squared velocity is finite
  (`MotionProperties.inl:28`), and a cube on a static body took angular velocities rounded to
  powers of two from a ratio of about 1e8;
- two light bodies held rigidly (a fixed constraint, a six-DOF constraint with every axis fixed,
  a swing-twist constraint with zero ranges): their accumulated impulse grows step after step
  until Jolt asserts that the squared angular velocity is finite (`MotionProperties.inl:38`),
  from `|r| / k` of about 37 (a ratio of 2700) for 1 g and 1 kg cubes of 6 cm and 20 cm, for the
  6 cm cube also at a tenth of the velocity bounds; none of 24 seeded cases failed at `|r| / k`
  of 34 or below. Point, hinge, cone, slider and six-DOF constraints with limited rotations did
  not fail at `|r| / k` of 650.

A derivation in the style of `MAX_WEIGHT_IMPULSE` (products of the largest accepted inverse
inertia, lever and impulse kept finite in `f32`) allows levers of hundreds of metres and does not
exclude the second failure, so the bound is the measured onset divided by 2.7.

## Kinematic drive

Jolt's `MoveKinematic` sets a velocity of `move / dt` without clamping
it (`MotionProperties.inl:9-21`). `RagdollMut::drive_to_pose_using_kinematics`
computes every part's velocity with Jolt's own operations first and accepts the drive only
when each stays within `MAX_LINEAR_VELOCITY` and `MAX_ANGULAR_VELOCITY`, the bounds of
every other velocity input.

## Six-DOF translation limits

Jolt corrects a violated limit by the distance beyond it
times the effective mass (`SixDOFConstraint.cpp:380-410,780-790`); limits within
`MAX_SHAPE_EXTENT` keep that finite. A limit of 1e30 m moved two parts to NaN positions in
a few steps.

## Contact constraint capacity

`WorldSettings::MAX_CONTACT_CONSTRAINTS` stays below the
count above which `ContactConstraintManager::Init` asserts; a native compile-time check pins
it.

## Contact settings

A `ContactListener` may change the settings Jolt resolves a contact with. Every setter of
`ContactSettings` and `SoftBodyContactSettings` checks its value and refuses one outside its
range with a `ContactSettingsError`, leaving the settings unchanged. The rules below that depend
on the contact (sensor bodies, the lever `R`) are checked again on the value a listener returns,
against the contact it was called for, because a listener can assign a value kept from another
contact; `docs/events.md` ("Rejected contact settings") says what happens to a value that fails.

### Friction and restitution

Combined friction follows the body rule, `0..=limits::MAX_FRICTION`. Jolt's default combined
friction of two bodies at the body bound is `sqrt(MAX_FRICTION²) = MAX_FRICTION`, so a listener
cannot give a contact more friction than two bodies already can, and the body bound's probe
(`friction_at_the_bound_keeps_contacts_finite`) covers it. Combined restitution follows the
body rule `0..=1`.

### Inverse mass and inertia scales

Jolt documents the scales as "0 = infinite mass, 1 = use original mass, 2 = body has half the
mass" and multiplies them into the inverse mass and the inverse inertia of each body for the
contact (`ContactConstraintManager.cpp:930-949`); the soft body solver does the same for the
vertices and the other body (`SoftBodyMotionProperties.cpp:205-215`). Every bound this crate
derives for masses and inertia (`limits::MIN_MASS`, the inertia conditioning floor) assumes the
inverse mass and inertia a body has; a scale above 1 would raise them past what those bounds
were derived for. The API therefore accepts at most 1: a contact can make a body heavier or
immovable, never lighter. This is a policy of this API, not a Jolt limit.

Jolt treats only an exact 0 as immovable (`ContactConstraintManager.cpp:917-918`,
`SoftBodyMotionProperties.cpp:167-169`). When no dynamic body of a rigid contact keeps a
factor above 0 (a dynamic body with 0 against a static or kinematic one, or two dynamic bodies
with 0 each), Jolt creates no contact constraint and the bodies pass through each other. Any other factor goes into the inverse effective mass,
so a tiny one makes the body enormously heavy: with a 1 kg cube landing at 8 m/s, a factor of
1e-36 on its inverse mass and inertia overflows the contact impulse, the cube's position is NaN
three steps later, and an asserts build stops on `MotionProperties.h:233`. A cloth landing on a
block with the vertex factor 0 and the block's factor 1e-35 goes NaN the same way, through
`p = dv / (w1 + w2)` (`SoftBodyMotionProperties.cpp:763-788`).

A positive factor is therefore at least `limits::MIN_CONTACT_SCALE = MIN_MASS / MAX_MASS`
(1e-9). It turns a body of `MIN_MASS` into one of `MAX_MASS`, the spread of masses two bodies
can already have, and makes any accepted body at most `MAX_MASS / MIN_CONTACT_SCALE = 1e15` kg
heavy for the contact. At the velocity bounds such a contact needs an impulse of about
`1e15 kg * 1000 m/s = 1e18` N s, twenty orders of magnitude below `f32::MAX`; the onsets measured
above lie 26 to 27 orders below the floor. `contact_scales_at_their_floor_step_finitely` throws
cubes and spheres of `MIN_MASS` and `MAX_MASS` at a floor at the velocity bounds, discrete and
with `MotionQuality::LinearCast`, and a light cube at a heavy one, with the mass and inertia
factors at the floor or 0; `soft_body_contact_scales_at_their_floor_step_finitely` throws a
cloth at blocks of both masses with the vertex and block factors at the floor or 0. Both stay
finite and assert-free in the asserts build.

`MIN_CONTACT_SCALE` is written as the literal `1e-9`: the `f32` division `MIN_MASS / MAX_MASS`
rounds one step above it, which would refuse the round number callers type.

### Sensor contacts

Jolt starts a contact's `mIsSensor` as `body1.IsSensor() || body2.IsSensor()` and asserts that a
callback does not turn a contact with a sensor body into an ordinary one
(`ContactConstraintManager.cpp:915`, `:1194`, `:1466`, "Sensors cannot be converted into regular
bodies by a contact callback!"). `ContactSettings::set_is_sensor(false)` is refused for such a
contact; making an ordinary contact a sensor contact is allowed. The soft body path has no such
assertion, so `SoftBodyContactSettings::set_is_sensor` takes either value.

### Surface velocity

Jolt applies the relative surface velocity at the friction point as `v + ω × r1`, `r1` the
friction point relative to body 1's centre of mass (`ContactConstraintManager.cpp:164-168`), and
feeds the result into the friction constraint as a target velocity. Bounding `v` by
`limits::MAX_LINEAR_VELOCITY` and `ω` by `limits::MAX_ANGULAR_VELOCITY` alone would still allow
`|ω| · |r1|` of about 47 rad/s times a lever of up to 2 km (`limits::MAX_SHAPE_EXTENT`), some
9e4 m/s. The setters therefore also require, in `f64`,

    |v| + |ω| · R <= limits::MAX_LINEAR_VELOCITY

with `R` the largest distance from body 1's centre of mass to a contact point of the manifold,
on either body, computed once per callback from the manifold the listener sees. The friction
point is an average of those points, so `|r1| <= R`, and `|v + ω × r1| <= |v| + |ω| · |r1|` keeps
the target speed within the linear velocity bound bodies have. Setting one of the two velocities
checks it together with the current value of the other.

The bound is checked by `surface_velocities_are_bounded_alone_and_together` (a lever of 2000 m:
`|ω|` of 0.25 rad/s fills the bound alone, and the next `f32` above is refused) and exercised by
`a_conveyor_moves_a_resting_cube` (2 m/s moves a resting cube along the floor).
