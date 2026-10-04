# Events

How `PhysicsWorld` records contact, body activation and soft body contact events, and which
Jolt code paths produce them. Line numbers refer to the vendored Jolt 5.6 sources
(`crates/oxijolt-sys/vendor/JoltPhysics/Jolt/Physics/`) and joltc
(`crates/oxijolt-sys/vendor/joltc/src/joltc.cpp`).

## Listeners

A world installs nothing until `set_event_settings` asks for events or `set_contact_listener`
sets a listener. It then creates the native listeners these need and attaches them to its Jolt
system:

| Listener | Created when | Jolt interface |
|---|---|---|
| joltc's `ManagedContactListener` (`joltc.cpp:7923-8037`) | `contacts`, or a contact listener is set | `ContactListener` |
| joltc's `ManagedBodyActivationListener` (`joltc.cpp:8040-8095`) | `body_activation` | `BodyActivationListener` |
| the extension's `ManagedSoftBodyContactListener` (`native/joltc_ext/joltc_ext_listener.cpp`) | `soft_body_contacts` or `soft_body_validations`, or a contact listener is set | `SoftBodyContactListener` |

Each listener type calls one process-global proc table, installed once; per-world state goes
through the listener's `userData`, which points at a context the world shares with the native
listeners through an `Arc`. The context's settings never change. Changing the settings detaches
and destroys the old listeners, moves the old context's unqueued events and panic to the world,
and builds new ones. Dropping a world detaches its listeners first, before it removes
constraints, vehicles, ragdolls and character inner bodies, which deactivate bodies.

## Where each event comes from

| Event | Jolt path | Thread |
|---|---|---|
| `ContactEvent::Added` / `Persisted` | `ContactConstraintManager::TemplatedAddContactConstraint` (`ContactConstraintManager.cpp:1166-1183`), `GetContactsFromCache` (`:912`), `OnCCDContactAdded` (`:1387-1454`) | Jolt's workers and the stepping thread |
| `ContactEvent::Removed` | `FinalizeContactCacheAndCallContactPointRemovedCallbacks` (`:446-448`), called at the end of a step (`PhysicsSystem.cpp:2461`) | one job |
| `ActivationEvent` | `BodyManager::ActivateBodies` (`BodyManager.cpp:495-520`), `DeactivateBodies` (`:532-565`) under `mActiveBodiesMutex` | workers during a step; the calling thread when a body is created, woken or removed |
| `SoftBodyValidation` | `SoftBodyMotionProperties` collision collector (`SoftBodyMotionProperties.cpp:163`), and continuous collision detection of a rigid body against a soft body (`PhysicsSystem.cpp:1895`) | workers |
| `SoftBodyContacts` | `SoftBodyMotionProperties::UpdateSoftBodyState` (`SoftBodyMotionProperties.cpp:842`) | one worker per soft body |

`PhysicsSystem::RestoreState` changes which bodies are awake through
`BodyManager::RestoreState` (`BodyManager.cpp:810-823`), which does not call the activation
listener: a restore reports no activation changes.

## What the callbacks read

The callbacks copy the data Jolt hands them and nothing else. None of them locks a body,
changes the world or keeps a pointer.

- Added and Persisted read both bodies' ids (`JPH_Body_GetID`), the manifold through joltc's
  getters (`joltc.cpp:8099-8132`), and each side's material: the body's shape
  (`JPH_Body_GetShape`, which calls Jolt's const `Body::GetShape`) and
  `Shape::GetMaterial(sub-shape id)`. Jolt does not change a body's shape during a step. For
  the checks of the contact settings they also read whether either body is a sensor
  (`JPH_Body_IsSensor`) and body 1's centre of mass (`JPH_Body_GetCenterOfMassPosition`).
- Removed reads the four ids of the pair through the extension's `JPH_SubShapeIDPair_*`
  getters, which use Jolt's accessors.
- Activation records the id Jolt passes; Jolt calls it under its active-bodies mutex.
- Soft body validation reads the two ids and the settings only: on the continuous collision
  path another thread may be updating the soft body.
- Soft body contacts read the soft body's centre of mass and rotation and the manifold. Jolt
  stores the manifold's points relative to this step's centre of mass, which it moves right
  after the callback (`SoftBodyMotionProperties.cpp:882-893`), so the callback converts them to
  world space at once: `position = com + R * local`, `normal = R * n`.

## Canonical order

The world sorts the events of each step before it queues them; events from between steps keep
the order they happened in.

- Contacts sort by `(body1, sub_shape1, body2, sub_shape2)` raw ids, then Added before
  Persisted before Removed, then by every payload field bit for bit: the settings, the
  materials, the normal, the penetration depth, the point count and each point's coordinates.
  Only identical events compare equal, and each copy is kept. One key can appear twice in a
  step: a `MotionQuality::LinearCast` body that touches in the discrete stage and is then hit
  by its own continuous cast gets Added from the first and Persisted from the second
  (`ContactConstraintManager.cpp:1421-1454`); Jolt casts each pair at most once per step
  (`PhysicsSystem.cpp:1983-1987`). A cube rarely does this, because the discrete contact
  already slows it below the cast threshold; a tilted rod thrown at the floor does, because
  the discrete contact stops its lower end but not its centre of mass (the event determinism
  scene uses one).
- Activations sort stably by body id. Jolt reports one body's changes under its mutex in causal
  order, which the stable sort keeps.
- Soft body validations sort by soft body id, other body id, result and settings bits; one
  step can validate a pair twice (the collision pass and a continuous cast).
- Soft body contacts sort by soft body id and then payload. Within one snapshot the vertices
  are in vertex order, and the sensors are sorted by id because Jolt removes sensors from its
  list by swapping in the last one (`SoftBodyMotionProperties.cpp:835-839`).

The event determinism test (`crates/oxijolt/tests/event_determinism.rs`) compares the
serialized events of every step, with every bit, for 1 and 4 worker threads and for caller job
systems, and checks that its scene stepped with every event on and a no-op contact listener
simulates bit for bit like the same scene with nothing installed. It requires every step to be
complete (`StepReport::is_complete`); steps in which Jolt dropped contacts because a buffer was
full are not covered.

## Lifetimes and the queue

- Events wait in the world until `take_events`; nothing bounds the queue, so a caller that
  records events takes them every step. The queue has no step boundaries.
- `restore_state` neither clears nor rewinds the queue: take the events before a rollback and
  drop the ones of the abandoned steps.
- A Removed pair or Deactivated id can name a body that was removed; it is not looked up.
- Enabling contacts while bodies touch reports their next contacts as Persisted or Removed
  without an Added, because Jolt reports against its contact cache.

## Panics

Every callback runs its body under `catch_unwind`. The first panic is kept and resumed by
`step` after the update returned (after a caller job system's `queue_job` panic, if any), or by
`take_events` when it happened between steps; later panics of the same update are dropped, also
when their payload's `Drop` panics. Recording continues after a panic, and the next step works
normally.

Once a panic is kept, callbacks that start skip the user's `ContactListener` until the panic is
resumed. A worker that already passed that check, or is inside the listener, finishes its call;
its settings are still applied when they are valid.

## Rejected contact settings

A `ContactListener` can assign a whole `ContactSettings` value it kept from another contact. The
setters checked that value against the contact it was read for, so the callback checks the
returned value again, field by field, against the contact it runs for: its sensor bodies and
the lever of its surface velocity (`docs/limits.md`, "Contact settings"). Without this check a
kept ordinary value turns a sensor contact into a solid one, which Jolt asserts against
(`ContactConstraintManager.cpp:915`).

A value that fails is not written to Jolt, which keeps its own settings for that contact. The
callback records a `ContactSettingsRejection` (the pair and the first rule broken) in
`WorldEvents::rejected_contact_settings`, whatever the event settings, and `StepReport` counts
the step's rejections. A rejection is not a panic: the listener is still called for every other
contact of the step, so which contacts get its settings does not depend on the thread count.
Rejections sort by pair and error like the other events of a step.

## Leaks

`crates/oxijolt/tests/listener_leaks.rs` measures the process's private bytes over two
phases: replacing listeners and materials in one world, and creating, stepping and dropping a
world with listeners, material shapes, bodies and a cloth every round (dropped with its
listeners attached and its bodies alive). A new world per round grows private bytes by about
1 to 2 MB over its first few thousand rounds and then stops. Measured locally (one test per
process): with 50 warm-up rounds, four consecutive blocks of 3 000 rounds grew by 1.8 MB,
0.19 MB, 0 and 0; this holds with Jolt's thread pool (1 or 4 workers) or a caller job system,
for empty, populated and stepped worlds, with or without listeners, and spawning and joining a
thread alone does not grow it. After the plateau, blocks of 6 000 rounds moved between -1.8 MB
and +0.2 MB. It is the heap settling, not an object per world left behind.

The settling also comes late: a single window of 6 000 rounds after 4 000 warm-up rounds grew by
1.8 MB and 2.0 MB in two CI runs on `windows-latest`. Locally it grew by 1.7 to 1.8 MB in two of
nine runs, each time inside one block of 1 000 rounds while every other block stayed within
±210 kB, and another run released 1.8 MB in one block. So each phase measures seven consecutive
blocks of 1 000 rounds (the world phase after 4 000 warm-up rounds, the replacement phase after
500) and allows the median block 100 bytes per round (100 kB). A leak grows every block, a heap
step only one. The median also passes a leak that grows only three or fewer of the seven blocks:
bursts of 400 kB every 2 000 rounds or 1.5 MB every 3 000 rounds pass. Against this gate, materials that were never released gave a median of 370 kB per block,
native contact listeners that were never destroyed 780 kB, and a dropped world that forgot its
listeners 1.26 MB; each failed it. A leak of one small native object per world (a forgotten
`JPH_ContactListener` alone: about 33 bytes per round) stays below what the block noise lets this
gate resolve.
