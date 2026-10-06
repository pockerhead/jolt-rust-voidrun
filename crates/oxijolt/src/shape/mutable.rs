//! [`MutableCompound`]: a compound edited at run time that publishes immutable shapes.

use std::collections::BTreeMap;
use std::fmt;
use std::ptr::{null, null_mut};

use oxijolt_sys::*;

use super::compound::{
    build_compound, check_child_pose, check_expanded, compound_ids, fits_jolt_ids, sub_shape_ids,
    RawCompoundChild, SubShapeIds, EMPTY_COMPOUND_RULE, SUB_SHAPE_ID_RULE,
};
use super::{initialize, CompoundChild, Shape, ShapeSettings};
use crate::owned::Owned;
use crate::{Quat, ShapeError, Vec3};

/// Most children a [`MutableCompound`] holds: Jolt's mutable compound counts its blocks of four
/// children as `(count + 3) >> 2` in 32 bits (`MutableCompoundShape.h`).
const MAX_CHILDREN: u32 = u32::MAX - 3;

/// The error of an edit that would leave more than [`MAX_CHILDREN`] children.
const CHILD_COUNT_RULE: &str = "a mutable compound has at most u32::MAX - 3 children";

/// Histogram slots: sub-shape id bits `0..=32`.
const ID_SLOTS: usize = 33;

/// A compound shape edited at run time: children are added, removed, moved and replaced, and
/// [`to_shape`](Self::to_shape) publishes the current children as a new, immutable [`Shape`].
///
/// [`Shape`]s never change, so an edit reaches no body by itself. Install a published shape
/// with [`BodyMut::set_shape`](crate::BodyMut::set_shape), which applies the body rules (static
/// only parts, mass, inertia, owners), updates mass and centre of mass, wakes the bodies around
/// and marks the world changed for [`PhysicsWorld::restore_state`]. Each `set_shape` changes one
/// body: when one shape goes to several bodies and a later one refuses it, the earlier ones keep
/// it. Bodies, worlds and scaled shapes that hold an earlier publication keep it unchanged.
///
/// Children are numbered in order, from 0; positions are metres relative to the compound's
/// origin. A published shape equals [`Shape::new_compound`] of the current children: Jolt moves
/// its centre of mass to the children's mass-weighted centre, and the extent rule of
/// `new_compound` applies after that move, so a light child far from heavy ones can end up
/// beyond [`limits::MAX_SHAPE_EXTENT`]. Removing a child moves the later ones down by one, and a
/// count that crosses a power of two changes the width of every child index, so a sub-shape id
/// Jolt reported for one publication (a hit, a contact, a character's ground) may name another
/// child, or none, of the next.
///
/// The editor is the caller's state: [`WorldState`](crate::WorldState) does not hold it, and a
/// state saved before a publication was installed no longer restores (`set_shape` changed the
/// world). A caller that edits during a rollback detour restores its own children before it
/// publishes again.
///
/// Edits cost O(n) in the number of children (Jolt recomputes the bounds of its blocks), and
/// [`to_shape`](Self::to_shape) builds a static compound, O(n log n), plus the children's mass
/// properties: suited to edits at event rate, such as destruction and building, not to animation
/// every step.
///
/// # Example
/// ```
/// use oxijolt::*;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let mut world = PhysicsWorld::new(WorldSettings::default())?;
/// let brick = Shape::new_box(Vec3::new(0.5, 0.5, 0.5))?;
/// let mut wall = MutableCompound::new()?;
/// for i in 0..4 {
///     let position = Vec3::new(i as f32, 0.0, 0.0);
///     wall.add_shape(&CompoundChild { shape: &brick, position, rotation: Quat::IDENTITY, user_data: i })?;
/// }
/// let body = world.create_body(&wall.to_shape()?, &BodySettings::new_static())?;
///
/// // A brick is destroyed: the later ones move down by one.
/// wall.remove_shape(1)?;
/// assert_eq!(wall.sub_shape_user_data(1), Some(2));
/// world.body_mut(body)?.set_shape(&wall.to_shape()?, None, Activation::Activate)?;
/// # Ok(())
/// # }
/// ```
///
/// [`limits::MAX_SHAPE_EXTENT`]: crate::limits::MAX_SHAPE_EXTENT
/// [`PhysicsWorld::restore_state`]: crate::PhysicsWorld::restore_state
pub struct MutableCompound {
    /// A Jolt `MutableCompoundShape` holding one reference to each child, every child at the
    /// identity pose with user data 0. It is never attached to a body or handed out; only this
    /// value reads and changes it.
    holder: Owned<JPH_Shape>,
    /// Pose, user data and ids of each child, index-aligned with `holder`.
    children: Vec<ChildEntry>,
    /// The ids of `children`, counted.
    ids: IdHistogram,
    /// The sum of [`SubShapeIds::expanded`] over `children`; with the compound itself, at most
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`].
    ///
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`]: crate::limits::MAX_EXPANDED_SUB_SHAPES
    expanded: u32,
}

/// What the editor keeps of a child besides the shape the holder keeps.
#[derive(Clone, Copy)]
struct ChildEntry {
    position: Vec3,
    rotation: Quat,
    user_data: u32,
    ids: SubShapeIds,
}

// SAFETY: the holder is referenced by this value only and changes only through `&mut self`;
// `&self` only reads it (`GetSubShape`) and the Rust fields. The children are immutable Jolt
// shapes whose reference counts are atomic
// (https://jrouwe.github.io/JoltPhysicsDocs/5.6.0/index.html#memory-management).
unsafe impl Send for MutableCompound {}
// SAFETY: as for `Send`; no method taking `&self` writes to the holder.
unsafe impl Sync for MutableCompound {}

impl fmt::Debug for MutableCompound {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("MutableCompound")
            .field("sub_shape_count", &self.sub_shape_count())
            .finish_non_exhaustive()
    }
}

/// Whether a compound may hold `count` children.
fn fits_child_count(count: usize) -> bool {
    count <= MAX_CHILDREN as usize
}

/// Refuses a compound whose children's expanded trees hold `children` shapes in all.
fn check_children_expanded(children: u32) -> Result<(), ShapeError> {
    check_expanded(children.saturating_add(1))
}

/// The ids of a child shape.
fn child_ids(shape: &Shape) -> SubShapeIds {
    // SAFETY: `shape` is borrowed for the call.
    unsafe { sub_shape_ids(shape.as_ptr(), &mut BTreeMap::new()) }
}

impl MutableCompound {
    /// An editor without children.
    pub fn new() -> Result<Self, ShapeError> {
        Self::from_children(&[])
    }

    /// An editor holding `children`, in order.
    ///
    /// Fails with [`ShapeError::InvalidSettings`] when a position or rotation breaks the rules
    /// of [`Shape::new_compound`], when there are more than `u32::MAX - 3` children or when the
    /// children's sub-shape ids would not fit Jolt's 32 bits, and with
    /// [`ShapeError::TooManySubShapes`] above [`limits::MAX_EXPANDED_SUB_SHAPES`]. Nothing is
    /// built then.
    ///
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`]: crate::limits::MAX_EXPANDED_SUB_SHAPES
    pub fn from_children(children: &[CompoundChild<'_>]) -> Result<Self, ShapeError> {
        if !fits_child_count(children.len()) {
            return Err(ShapeError::InvalidSettings(CHILD_COUNT_RULE));
        }
        for child in children {
            check_child_pose(child.position, child.rotation)?;
        }
        let mut memo = BTreeMap::new();
        let mut ids = IdHistogram::new();
        let mut expanded = 0_u32;
        let entries: Vec<_> = children
            .iter()
            .map(|child| {
                // SAFETY: `children` borrows every shape for the call.
                let child_ids = unsafe { sub_shape_ids(child.shape.as_ptr(), &mut memo) };
                ids.count(child_ids, 1);
                expanded = expanded.saturating_add(child_ids.expanded);
                ChildEntry {
                    position: child.position,
                    rotation: child.rotation,
                    user_data: child.user_data,
                    ids: child_ids,
                }
            })
            .collect();
        if !ids.fits(entries.len() as u32, None) {
            return Err(ShapeError::InvalidSettings(SUB_SHAPE_ID_RULE));
        }
        // Building the holder walks the expanded tree.
        check_children_expanded(expanded)?;
        Ok(Self {
            holder: holder_of(children)?,
            children: entries,
            ids,
            expanded,
        })
    }

    /// Number of children.
    pub fn sub_shape_count(&self) -> u32 {
        self.children.len() as u32
    }

    /// The user data of child `index`; `None` when there is no such child.
    pub fn sub_shape_user_data(&self, index: u32) -> Option<u32> {
        self.children
            .get(index as usize)
            .map(|entry| entry.user_data)
    }

    /// Adds `child` after the last child and returns its index.
    ///
    /// Fails with [`ShapeError::InvalidSettings`] when the pose breaks the rules of
    /// [`Shape::new_compound`], when the compound already holds `u32::MAX - 3` children or when
    /// the ids of the compound with the new child would not fit Jolt's 32 bits, and with
    /// [`ShapeError::TooManySubShapes`] when the compound would hold more than
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`] shapes; the compound is unchanged then.
    ///
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`]: crate::limits::MAX_EXPANDED_SUB_SHAPES
    pub fn add_shape(&mut self, child: &CompoundChild<'_>) -> Result<u32, ShapeError> {
        let index = self.sub_shape_count();
        if !fits_child_count(self.children.len() + 1) {
            return Err(ShapeError::InvalidSettings(CHILD_COUNT_RULE));
        }
        check_child_pose(child.position, child.rotation)?;
        let ids = child_ids(child.shape);
        if !self.ids.fits(index + 1, Some(ids)) {
            return Err(ShapeError::InvalidSettings(SUB_SHAPE_ID_RULE));
        }
        let expanded = self.expanded.saturating_add(ids.expanded);
        check_children_expanded(expanded)?;
        let (position, rotation) = (Vec3::ZERO.to_jph(), Quat::IDENTITY.to_jph());
        // SAFETY: the holder is a live mutable compound that only `self`, borrowed mutably,
        // reads or changes. The child is live for the call and the holder takes its own
        // reference; the pose is a valid identity. `u32::MAX` appends.
        unsafe {
            JPH_MutableCompoundShape_AddShape(
                self.holder.as_ptr().cast(),
                &position,
                &rotation,
                child.shape.as_ptr(),
                0,
                u32::MAX,
            );
        }
        self.ids.count(ids, 1);
        self.expanded = expanded;
        self.children.push(ChildEntry {
            position: child.position,
            rotation: child.rotation,
            user_data: child.user_data,
            ids,
        });
        Ok(index)
    }

    /// Removes child `index`; the children after it move down by one.
    ///
    /// Fails with [`ShapeError::NoSubShape`] when there is no such child.
    pub fn remove_shape(&mut self, index: u32) -> Result<(), ShapeError> {
        self.check_index(index)?;
        // SAFETY: the holder is a live mutable compound that only `self`, borrowed mutably,
        // reads or changes, and `index` names one of its children (index-aligned with
        // `children`, checked above). The holder releases its reference to the child.
        unsafe { JPH_MutableCompoundShape_RemoveShape(self.holder.as_ptr().cast(), index) };
        let entry = self.children.remove(index as usize);
        self.ids.count(entry.ids, -1);
        self.expanded -= entry.ids.expanded;
        Ok(())
    }

    /// Moves child `index` to `position` and `rotation` and, with `shape`, replaces its shape.
    /// The child keeps its user data.
    ///
    /// Fails with [`ShapeError::NoSubShape`] when there is no such child, with
    /// [`ShapeError::InvalidSettings`] when the pose breaks the rules of
    /// [`Shape::new_compound`] or when the new shape's ids would not fit Jolt's 32 bits, and with
    /// [`ShapeError::TooManySubShapes`] when the compound would hold more than
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`] shapes; the compound is unchanged then.
    ///
    /// [`limits::MAX_EXPANDED_SUB_SHAPES`]: crate::limits::MAX_EXPANDED_SUB_SHAPES
    pub fn modify_shape(
        &mut self,
        index: u32,
        position: Vec3,
        rotation: Quat,
        shape: Option<&Shape>,
    ) -> Result<(), ShapeError> {
        self.check_index(index)?;
        check_child_pose(position, rotation)?;
        if let Some(shape) = shape {
            let ids = child_ids(shape);
            // The current children fit, and replacing one can only lower their maxima: the
            // compound fits with the new child exactly when it fits next to all current ones.
            if !self.ids.fits(self.sub_shape_count(), Some(ids)) {
                return Err(ShapeError::InvalidSettings(SUB_SHAPE_ID_RULE));
            }
            let old = self.children[index as usize].ids;
            let expanded = (self.expanded - old.expanded).saturating_add(ids.expanded);
            check_children_expanded(expanded)?;
            let (zero, identity) = (Vec3::ZERO.to_jph(), Quat::IDENTITY.to_jph());
            // SAFETY: the holder is a live mutable compound that only `self`, borrowed
            // mutably, reads or changes, and `index` names one of its children (checked
            // above). The new shape is live for the call; the holder takes its own reference
            // and releases the old child's. The pose is a valid identity.
            unsafe {
                JPH_MutableCompoundShape_ModifyShape2(
                    self.holder.as_ptr().cast(),
                    index,
                    &zero,
                    &identity,
                    shape.as_ptr(),
                );
            }
            self.ids.count(old, -1);
            self.ids.count(ids, 1);
            self.expanded = expanded;
            self.children[index as usize].ids = ids;
        }
        let entry = &mut self.children[index as usize];
        entry.position = position;
        entry.rotation = rotation;
        Ok(())
    }

    /// The current children as a new shape, equal to [`Shape::new_compound`] of them; see the
    /// type's documentation. The editor and every body stay as they are.
    ///
    /// Fails with [`ShapeError::InvalidSettings`] when there is no child, and with the errors
    /// of [`Shape::new_compound`] otherwise, such as [`ShapeError::InvalidDimensions`] when the
    /// recentred compound leaves [`limits::MAX_SHAPE_EXTENT`].
    ///
    /// [`limits::MAX_SHAPE_EXTENT`]: crate::limits::MAX_SHAPE_EXTENT
    pub fn to_shape(&self) -> Result<Shape, ShapeError> {
        if self.children.is_empty() {
            return Err(ShapeError::InvalidSettings(EMPTY_COMPOUND_RULE));
        }
        let raw: Vec<_> = (0..self.sub_shape_count())
            .zip(&self.children)
            .map(|(index, entry)| RawCompoundChild {
                shape: self.child_shape(index),
                position: entry.position,
                rotation: entry.rotation,
                user_data: entry.user_data,
                ids: entry.ids,
            })
            .collect();
        // SAFETY: the holder keeps every child shape alive while `self` is borrowed; every pose
        // passed `check_child_pose` and the count is at most `MAX_CHILDREN`.
        unsafe { build_compound(&raw) }
    }

    /// The shape of child `index`, kept alive by the holder.
    fn child_shape(&self, index: u32) -> *const JPH_Shape {
        debug_assert!(index < self.sub_shape_count());
        let mut shape = null();
        // SAFETY: the holder is live and only read here; `index` names one of its children
        // (index-aligned with `children`), and joltc writes only the outputs that are not null.
        unsafe {
            JPH_CompoundShape_GetSubShape(
                self.holder.as_ptr().cast(),
                index,
                &mut shape,
                null_mut(),
                null_mut(),
                null_mut(),
            );
        }
        shape
    }

    fn check_index(&self, index: u32) -> Result<(), ShapeError> {
        let count = self.sub_shape_count();
        if index < count {
            Ok(())
        } else {
            Err(ShapeError::NoSubShape { index, count })
        }
    }
}

/// How many children use each id width and each 0-bit push position, so an edit checks the
/// compound's ids without visiting every child.
struct IdHistogram {
    /// Children per [`SubShapeIds::width`].
    widths: [u32; ID_SLOTS],
    /// Children per [`SubShapeIds::zero_width_at`]; children without a 0-bit push are not
    /// counted.
    zero_widths: [u32; ID_SLOTS],
}

impl IdHistogram {
    fn new() -> Self {
        Self {
            widths: [0; ID_SLOTS],
            zero_widths: [0; ID_SLOTS],
        }
    }

    /// Adds `by` (1 or -1) to the slots of `ids`.
    fn count(&mut self, ids: SubShapeIds, by: i32) {
        let slot = |bits: u32| (bits as usize).min(ID_SLOTS - 1);
        let width = &mut self.widths[slot(ids.width)];
        *width = width.wrapping_add_signed(by);
        if let Some(at) = ids.zero_width_at {
            let zero = &mut self.zero_widths[slot(at)];
            *zero = zero.wrapping_add_signed(by);
        }
    }

    /// Whether a compound of `count` children fits Jolt's ids: the counted children and, when
    /// given, one more child with `extra` ids.
    fn fits(&self, count: u32, extra: Option<SubShapeIds>) -> bool {
        let highest = |slots: &[u32; ID_SLOTS]| slots.iter().rposition(|&n| n > 0);
        let counted = SubShapeIds {
            width: highest(&self.widths).map_or(0, |slot| slot as u32),
            zero_width_at: highest(&self.zero_widths).map(|slot| slot as u32),
            // The editor counts expanded shapes itself.
            expanded: 0,
        };
        let ids = compound_ids([Some(counted), extra].into_iter().flatten(), count);
        ids.width <= 32 && fits_jolt_ids(ids)
    }
}

/// The Jolt mutable compound of an editor of `children`: a reference to each child at the
/// identity pose with user data 0, built in one pass.
fn holder_of(children: &[CompoundChild<'_>]) -> Result<Owned<JPH_Shape>, ShapeError> {
    initialize()?;
    // SAFETY: Jolt is initialised. The returned settings hold one reference, which the guard
    // takes over.
    let settings =
        unsafe { ShapeSettings::from_raw(JPH_MutableCompoundShapeSettings_Create().cast()) }?;
    let (position, rotation) = (Vec3::ZERO.to_jph(), Quat::IDENTITY.to_jph());
    for child in children {
        // SAFETY: the settings are live, owned by the guard and mutable compound settings,
        // which derive from `CompoundShapeSettings` with single inheritance. The child shape is
        // live and the settings store their own reference; the pose is a valid identity.
        unsafe {
            JPH_CompoundShapeSettings_AddShape2(
                settings.as_ptr(),
                &position,
                &rotation,
                child.shape.as_ptr(),
                0,
            );
        }
    }
    let Shape(holder) = settings.create()?;
    Ok(holder)
}

#[cfg(test)]
mod tests;
