use core::cell::{Cell, UnsafeCell};

/// Sync wrapper for shared mutable data
pub struct SyncUnsafeCell<T>(UnsafeCell<T>);
unsafe impl<T> Sync for SyncUnsafeCell<T> {}

impl<T> SyncUnsafeCell<T> {
    #[inline]
    pub const fn new(inner: T) -> Self {
        Self(UnsafeCell::new(inner))
    }
    #[inline]
    pub const fn get(&self) -> &T {
        unsafe { &*self.0.get() }
    }
    #[allow(clippy::mut_from_ref)]
    #[inline]
    pub const fn get_mut(&self) -> &mut T {
        unsafe { &mut *self.0.get() }
    }
}

/// Sync wrapper for shared mutable primitive types
pub struct SyncCell<T>(pub Cell<T>);
unsafe impl<T> Sync for SyncCell<T> {}

impl<T> SyncCell<T> {
    #[inline]
    pub const fn new(inner: T) -> Self {
        Self(Cell::new(inner))
    }
}
