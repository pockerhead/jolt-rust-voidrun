//! Object layers, broad-phase layers and the table that says which object layers collide.
//!
//! Every body lives in one object layer. Each object layer maps to one broad-phase layer, the
//! coarse tree the broad phase keeps its bodies in. Two bodies can only collide when their
//! object layers are enabled as a pair. See Jolt's documentation on collision detection:
//! <https://jrouwe.github.io/JoltPhysicsDocs/5.3.0/index.html#collision-detection>.

use std::ptr::NonNull;

use joltc_sys::*;

use crate::WorldError;

/// The object layer of a body: which other bodies it can collide with.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ObjectLayer(u32);

impl ObjectLayer {
    /// Layer 0 in [`CollisionLayers::default`]: static geometry.
    pub const NON_MOVING: Self = Self(0);
    /// Layer 1 in [`CollisionLayers::default`]: everything that moves.
    pub const MOVING: Self = Self(1);

    /// Wraps a raw layer index. It is only valid in a world whose [`CollisionLayers`] has more
    /// object layers than `value`.
    pub const fn new(value: u32) -> Self {
        Self(value)
    }

    /// The raw layer index.
    pub const fn get(self) -> u32 {
        self.0
    }
}

/// A broad-phase layer: one of the trees the broad phase sorts bodies into.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct BroadPhaseLayer(u8);

impl BroadPhaseLayer {
    /// Broad-phase layer 0 in [`CollisionLayers::default`]: static geometry.
    pub const NON_MOVING: Self = Self(0);
    /// Broad-phase layer 1 in [`CollisionLayers::default`]: everything that moves.
    pub const MOVING: Self = Self(1);

    /// Wraps a raw broad-phase layer index.
    pub const fn new(value: u8) -> Self {
        Self(value)
    }

    /// The raw broad-phase layer index.
    pub const fn get(self) -> u8 {
        self.0
    }
}

/// The collision layer setup of a world: object layers, the broad-phase layer of each, and the
/// pairs of object layers that collide.
///
/// Object layers are numbered in the order [`add_object_layer`](Self::add_object_layer) creates
/// them, and each one is mapped to a broad-phase layer when it is created.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CollisionLayers {
    broad_phase_layer_count: u8,
    broad_phase_of: Vec<BroadPhaseLayer>,
    pairs: Vec<(ObjectLayer, ObjectLayer)>,
}

impl CollisionLayers {
    /// Largest number of object layers a world accepts.
    const MAX_OBJECT_LAYERS: usize = u16::MAX as usize;

    /// Creates a setup with `broad_phase_layer_count` broad-phase layers and no object layers.
    pub fn new(broad_phase_layer_count: u8) -> Self {
        Self {
            broad_phase_layer_count,
            broad_phase_of: Vec::new(),
            pairs: Vec::new(),
        }
    }

    /// Adds the next object layer, kept in `broad_phase`, and returns it.
    pub fn add_object_layer(&mut self, broad_phase: BroadPhaseLayer) -> ObjectLayer {
        let layer = ObjectLayer(self.broad_phase_of.len() as u32);
        self.broad_phase_of.push(broad_phase);
        layer
    }

    /// Lets bodies in layer `a` collide with bodies in layer `b`. The relation is symmetric.
    pub fn enable_collision(&mut self, a: ObjectLayer, b: ObjectLayer) -> &mut Self {
        self.pairs.push((a, b));
        self
    }

    /// Number of object layers added so far.
    pub fn object_layer_count(&self) -> u32 {
        self.broad_phase_of.len() as u32
    }

    /// Number of broad-phase layers.
    pub fn broad_phase_layer_count(&self) -> u8 {
        self.broad_phase_layer_count
    }

    /// Checks everything Jolt's layer tables index with: an unmapped object layer or an
    /// out-of-range index reads past the tables in a release build.
    pub(crate) fn validate(&self) -> Result<(), WorldError> {
        if self.broad_phase_of.is_empty() {
            return Err(WorldError::InvalidLayers("there are no object layers"));
        }
        if self.broad_phase_of.len() > Self::MAX_OBJECT_LAYERS {
            return Err(WorldError::InvalidLayers("more than 65535 object layers"));
        }
        // 0xff is Jolt's invalid broad-phase layer, which a u8 count of at most 255 excludes.
        if self.broad_phase_layer_count == 0 {
            return Err(WorldError::InvalidLayers("there are no broad-phase layers"));
        }
        if self
            .broad_phase_of
            .iter()
            .any(|layer| layer.0 >= self.broad_phase_layer_count)
        {
            return Err(WorldError::InvalidLayers(
                "an object layer maps to a broad-phase layer that does not exist",
            ));
        }
        let count = self.object_layer_count();
        if self.pairs.iter().any(|(a, b)| a.0 >= count || b.0 >= count) {
            return Err(WorldError::InvalidLayers(
                "a collision pair names an object layer that does not exist",
            ));
        }
        Ok(())
    }

    /// Builds Jolt's table-based layer interface and filters from this setup.
    ///
    /// # Safety
    /// `JPH_Init` has returned true and [`validate`](Self::validate) returned `Ok`.
    pub(crate) unsafe fn create_tables(&self) -> Result<LayerTables, WorldError> {
        let object_layers = self.object_layer_count();
        let broad_phase_layers = u32::from(self.broad_phase_layer_count);

        // SAFETY: Jolt is initialised (caller contract); the table takes only a count.
        let pair_filter =
            PairFilter::new(unsafe { JPH_ObjectLayerPairFilterTable_Create(object_layers) })?;
        for &(a, b) in &self.pairs {
            // SAFETY: `pair_filter` is live, and `validate` checked both layers are below the
            // count the table was created with.
            unsafe {
                JPH_ObjectLayerPairFilterTable_EnableCollision(pair_filter.0.as_ptr(), a.0, b.0)
            };
        }

        // SAFETY: Jolt is initialised (caller contract); the table takes only counts.
        let broad_phase = BroadPhaseInterface::new(unsafe {
            JPH_BroadPhaseLayerInterfaceTable_Create(object_layers, broad_phase_layers)
        })?;
        for (object_layer, broad_phase_layer) in (0..).zip(&self.broad_phase_of) {
            // SAFETY: `broad_phase` is live; both indices are within the counts it was created
            // with (`validate`).
            unsafe {
                JPH_BroadPhaseLayerInterfaceTable_MapObjectToBroadPhaseLayer(
                    broad_phase.0.as_ptr(),
                    object_layer,
                    broad_phase_layer.0,
                )
            };
        }

        // SAFETY: both tables are live and fully mapped with the counts passed here. Jolt reads
        // them in the constructor and keeps no reference to either
        // (`ObjectVsBroadPhaseLayerFilterTable.h`).
        let object_vs_broad_phase = ObjectVsBroadPhaseFilter::new(unsafe {
            JPH_ObjectVsBroadPhaseLayerFilterTable_Create(
                broad_phase.0.as_ptr(),
                broad_phase_layers,
                pair_filter.0.as_ptr(),
                object_layers,
            )
        })?;

        Ok(LayerTables {
            broad_phase,
            pair_filter,
            object_vs_broad_phase,
        })
    }
}

impl Default for CollisionLayers {
    /// The setup of Jolt's HelloWorld sample: [`ObjectLayer::NON_MOVING`] in
    /// [`BroadPhaseLayer::NON_MOVING`] and [`ObjectLayer::MOVING`] in
    /// [`BroadPhaseLayer::MOVING`]; moving bodies collide with both layers, static ones only
    /// with moving ones.
    fn default() -> Self {
        let mut layers = Self::new(2);
        let non_moving = layers.add_object_layer(BroadPhaseLayer::NON_MOVING);
        let moving = layers.add_object_layer(BroadPhaseLayer::MOVING);
        layers
            .enable_collision(moving, non_moving)
            .enable_collision(moving, moving);
        layers
    }
}

/// The three layer objects a physics system needs. Owned here until
/// `JPH_PhysicsSystem_Create` succeeds with them; from then on the system owns them and
/// `JPH_PhysicsSystem_Destroy` deletes them, so the caller calls
/// [`forget`](Self::forget).
pub(crate) struct LayerTables {
    broad_phase: BroadPhaseInterface,
    pair_filter: PairFilter,
    object_vs_broad_phase: ObjectVsBroadPhaseFilter,
}

impl LayerTables {
    /// The pointers for `JPH_PhysicsSystemSettings`, still owned by `self`, as
    /// `(broad_phase, pair_filter, object_vs_broad_phase)`.
    pub(crate) fn as_raw(
        &self,
    ) -> (
        *mut JPH_BroadPhaseLayerInterface,
        *mut JPH_ObjectLayerPairFilter,
        *mut JPH_ObjectVsBroadPhaseLayerFilter,
    ) {
        (
            self.broad_phase.0.as_ptr(),
            self.pair_filter.0.as_ptr(),
            self.object_vs_broad_phase.0.as_ptr(),
        )
    }

    /// Gives up ownership without destroying anything, once a physics system owns the tables.
    pub(crate) fn forget(self) {
        std::mem::forget(self);
    }
}

/// Owns a `JPH_ObjectLayerPairFilter`.
struct PairFilter(NonNull<JPH_ObjectLayerPairFilter>);

impl PairFilter {
    fn new(ptr: *mut JPH_ObjectLayerPairFilter) -> Result<Self, WorldError> {
        NonNull::new(ptr)
            .map(Self)
            .ok_or(WorldError::AllocationFailed("object layer pair filter"))
    }
}

impl Drop for PairFilter {
    fn drop(&mut self) {
        // SAFETY: this value owns the filter, which no physics system references.
        unsafe { JPH_ObjectLayerPairFilter_Destroy(self.0.as_ptr()) };
    }
}

/// Owns a `JPH_BroadPhaseLayerInterface`.
struct BroadPhaseInterface(NonNull<JPH_BroadPhaseLayerInterface>);

impl BroadPhaseInterface {
    fn new(ptr: *mut JPH_BroadPhaseLayerInterface) -> Result<Self, WorldError> {
        NonNull::new(ptr)
            .map(Self)
            .ok_or(WorldError::AllocationFailed("broad-phase layer interface"))
    }
}

impl Drop for BroadPhaseInterface {
    fn drop(&mut self) {
        // SAFETY: this value owns the interface, which no physics system references.
        unsafe { JPH_BroadPhaseLayerInterface_Destroy(self.0.as_ptr()) };
    }
}

/// Owns a `JPH_ObjectVsBroadPhaseLayerFilter`.
struct ObjectVsBroadPhaseFilter(NonNull<JPH_ObjectVsBroadPhaseLayerFilter>);

impl ObjectVsBroadPhaseFilter {
    fn new(ptr: *mut JPH_ObjectVsBroadPhaseLayerFilter) -> Result<Self, WorldError> {
        NonNull::new(ptr)
            .map(Self)
            .ok_or(WorldError::AllocationFailed(
                "object vs broad-phase layer filter",
            ))
    }
}

impl Drop for ObjectVsBroadPhaseFilter {
    fn drop(&mut self) {
        // SAFETY: this value owns the filter, which no physics system references.
        unsafe { JPH_ObjectVsBroadPhaseLayerFilter_Destroy(self.0.as_ptr()) };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_maps_and_pairs_like_jolts_hello_world() {
        let layers = CollisionLayers::default();
        assert_eq!(layers.object_layer_count(), 2);
        assert_eq!(layers.broad_phase_layer_count(), 2);
        assert_eq!(
            layers.broad_phase_of,
            [BroadPhaseLayer::NON_MOVING, BroadPhaseLayer::MOVING]
        );
        assert_eq!(
            layers.pairs,
            [
                (ObjectLayer::MOVING, ObjectLayer::NON_MOVING),
                (ObjectLayer::MOVING, ObjectLayer::MOVING)
            ]
        );
        assert_eq!(layers.validate(), Ok(()));
    }

    #[test]
    fn validate_rejects_inconsistent_tables() {
        let invalid = |layers: &CollisionLayers| {
            matches!(layers.validate(), Err(WorldError::InvalidLayers(_)))
        };

        assert!(invalid(&CollisionLayers::new(1)));

        let mut no_broad_phase = CollisionLayers::new(0);
        no_broad_phase.add_object_layer(BroadPhaseLayer::new(0));
        assert!(invalid(&no_broad_phase));

        let mut unknown_broad_phase = CollisionLayers::new(2);
        unknown_broad_phase.add_object_layer(BroadPhaseLayer::new(2));
        assert!(invalid(&unknown_broad_phase));

        let mut unknown_pair = CollisionLayers::new(1);
        let only = unknown_pair.add_object_layer(BroadPhaseLayer::new(0));
        unknown_pair.enable_collision(only, ObjectLayer::new(1));
        assert!(invalid(&unknown_pair));
    }

    /// Structures, terrain, items, actors and features: items collide with the first four,
    /// actors with structures and terrain.
    fn five_layers() -> CollisionLayers {
        let mut layers = CollisionLayers::new(2);
        let structures = layers.add_object_layer(BroadPhaseLayer::NON_MOVING);
        let terrain = layers.add_object_layer(BroadPhaseLayer::NON_MOVING);
        let items = layers.add_object_layer(BroadPhaseLayer::MOVING);
        let actors = layers.add_object_layer(BroadPhaseLayer::MOVING);
        let _features = layers.add_object_layer(BroadPhaseLayer::NON_MOVING);
        for other in [structures, terrain, items, actors] {
            layers.enable_collision(items, other);
        }
        layers
            .enable_collision(actors, structures)
            .enable_collision(actors, terrain);
        layers
    }

    #[test]
    fn five_layer_tables_are_built_and_released() {
        let layers = five_layers();
        assert_eq!(layers.object_layer_count(), 5);
        assert_eq!(layers.validate(), Ok(()));
        assert!(crate::world::ensure_initialized());
        for _ in 0..100 {
            // SAFETY: Jolt is initialised and `validate` passed just above.
            let tables = unsafe { layers.create_tables() }.unwrap();
            drop(tables);
        }
    }
}
