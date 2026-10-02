//! Ownership of joltc objects: one owner type, and one destroy call per joltc type.
//!
//! [`Owned<T>`] owns either a whole joltc object or exactly one reference to a ref-counted Jolt
//! object. joltc's `*_Create` functions return such objects already holding one reference, and
//! their `*_Destroy` functions call `Release`. Each joltc type says how it is destroyed by
//! implementing [`JoltObject`] next to the code that uses it.

use std::mem::ManuallyDrop;
use std::ptr::NonNull;

/// A joltc object type that Rust owns through a pointer returned by a joltc `Create` call.
pub(crate) trait JoltObject {
    /// Destroys the object, or releases the one reference its owner holds.
    ///
    /// # Safety
    /// `ptr` points to a live object of this type, and the caller owns the object or one
    /// reference to it, which this call ends. Nothing uses that ownership afterwards.
    unsafe fn destroy(ptr: *mut Self);
}

/// Owns one joltc object, or one reference to a ref-counted one, and destroys or releases it
/// on drop.
pub(crate) struct Owned<T: JoltObject> {
    ptr: NonNull<T>,
}

impl<T: JoltObject> Owned<T> {
    /// Takes over `ptr`; `None` when it is null.
    ///
    /// # Safety
    /// If not null, `ptr` points to a live `T`, and the caller hands over either sole ownership
    /// of the object or exactly one reference it owns. No other owner destroys that same unit.
    pub(crate) unsafe fn from_raw(ptr: *mut T) -> Option<Self> {
        NonNull::new(ptr).map(|ptr| Self { ptr })
    }

    /// The owned object, still owned by `self`.
    pub(crate) fn as_ptr(&self) -> *mut T {
        self.ptr.as_ptr()
    }

    /// The owned object as a `NonNull`, still owned by `self`.
    pub(crate) fn as_non_null(&self) -> NonNull<T> {
        self.ptr
    }

    /// Gives up ownership without destroying, for when joltc takes the object over.
    pub(crate) fn into_raw(self) -> *mut T {
        ManuallyDrop::new(self).ptr.as_ptr()
    }
}

impl<T: JoltObject> Drop for Owned<T> {
    fn drop(&mut self) {
        // SAFETY: `from_raw` handed one ownership unit to this handle; `into_raw` consumes the
        // handle without running this destructor, so the unit is ended exactly once.
        unsafe { T::destroy(self.ptr.as_ptr()) }
    }
}

#[cfg(test)]
mod tests {
    use std::cell::Cell;
    use std::rc::Rc;

    use super::*;

    /// A heap object that counts how often it was destroyed.
    struct Probe {
        destroyed: Rc<Cell<u32>>,
    }

    impl JoltObject for Probe {
        unsafe fn destroy(ptr: *mut Self) {
            // SAFETY: the tests create every probe with `Box::into_raw` and hand its one
            // ownership unit to the caller, which ends it here (trait contract).
            let probe = unsafe { Box::from_raw(ptr) };
            probe.destroyed.set(probe.destroyed.get() + 1);
        }
    }

    fn probe(counter: &Rc<Cell<u32>>) -> *mut Probe {
        Box::into_raw(Box::new(Probe {
            destroyed: Rc::clone(counter),
        }))
    }

    #[test]
    fn null_is_none() {
        // SAFETY: a null pointer hands over nothing.
        assert!(unsafe { Owned::<Probe>::from_raw(std::ptr::null_mut()) }.is_none());
    }

    #[test]
    fn drop_destroys_once() {
        let counter = Rc::new(Cell::new(0));
        // SAFETY: the probe is live and this test owns it; the handle takes it over.
        let owned = unsafe { Owned::from_raw(probe(&counter)) }.unwrap();
        assert_eq!(counter.get(), 0);
        drop(owned);
        assert_eq!(counter.get(), 1);
    }

    #[test]
    fn into_raw_does_not_destroy() {
        let counter = Rc::new(Cell::new(0));
        let raw = probe(&counter);
        // SAFETY: the probe is live and this test owns it; the handle takes it over.
        let owned = unsafe { Owned::from_raw(raw) }.unwrap();
        assert_eq!(owned.into_raw(), raw);
        assert_eq!(counter.get(), 0);
        // SAFETY: `into_raw` gave the ownership back to this test, which hands it over again.
        drop(unsafe { Owned::from_raw(raw) }.unwrap());
        assert_eq!(counter.get(), 1);
    }
}
