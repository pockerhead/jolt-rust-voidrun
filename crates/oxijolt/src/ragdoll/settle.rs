//! Settle detection: whether a ragdoll has come to rest.

use crate::{RagdollError, RagdollRef};

/// Counts consecutive calm updates of a ragdoll and reports when it has settled.
///
/// An update is calm when every part moves slower than both speed limits
/// ([`RagdollRef::is_calm`]); any other update starts the count again. The ragdoll has settled
/// once [`calm_ticks`](Self::calm_ticks) calm updates have followed each other. Call
/// [`update`](Self::update) once per tick, after the step. A timeout, if any, is the caller's.
///
/// The default is the game's rest rule: below 0.05 m/s and 0.1 rad/s for 30 ticks.
#[derive(Clone, Debug, PartialEq)]
pub struct SettleDetector {
    max_linear_speed: f32,
    max_angular_speed: f32,
    calm_ticks: u32,
    calm_so_far: u32,
}

impl Default for SettleDetector {
    fn default() -> Self {
        Self {
            max_linear_speed: 0.05,
            max_angular_speed: 0.1,
            calm_ticks: 30,
            calm_so_far: 0,
        }
    }
}

impl SettleDetector {
    /// A detector with speed limits in m/s and rad/s, both finite and positive, and the number of
    /// calm updates in a row that settle a ragdoll, at least 1.
    pub fn new(
        max_linear_speed: f32,
        max_angular_speed: f32,
        calm_ticks: u32,
    ) -> Result<Self, RagdollError> {
        let positive = |value: f32| value.is_finite() && value > 0.0;
        if !(positive(max_linear_speed) && positive(max_angular_speed)) {
            return Err(RagdollError::InvalidValue(
                "settle speed limits must be finite and positive",
            ));
        }
        if calm_ticks == 0 {
            return Err(RagdollError::InvalidValue(
                "settling needs at least one calm tick",
            ));
        }
        Ok(Self {
            max_linear_speed,
            max_angular_speed,
            calm_ticks,
            calm_so_far: 0,
        })
    }

    /// Records one tick of `ragdoll` and returns whether it has settled.
    pub fn update(&mut self, ragdoll: &RagdollRef<'_>) -> bool {
        self.record(ragdoll.is_calm(self.max_linear_speed, self.max_angular_speed))
    }

    /// Forgets every calm update recorded so far.
    pub fn reset(&mut self) {
        self.calm_so_far = 0;
    }

    /// Calm updates in a row that settle a ragdoll.
    pub fn calm_ticks(&self) -> u32 {
        self.calm_ticks
    }

    fn record(&mut self, calm: bool) -> bool {
        self.calm_so_far = if calm {
            self.calm_so_far.saturating_add(1)
        } else {
            0
        };
        self.calm_so_far >= self.calm_ticks
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn calm_updates_in_a_row_settle() {
        let mut detector = SettleDetector::new(0.05, 0.1, 3).unwrap();
        assert_eq!(detector.calm_ticks(), 3);
        assert!(!detector.record(true));
        assert!(!detector.record(true));
        assert!(!detector.record(false));
        assert!(!detector.record(true));
        assert!(!detector.record(true));
        assert!(detector.record(true));
        assert!(detector.record(true));
        detector.reset();
        assert!(!detector.record(true));
    }

    #[test]
    fn default_is_the_rest_rule() {
        let detector = SettleDetector::default();
        assert_eq!(detector, SettleDetector::new(0.05, 0.1, 30).unwrap());
    }

    #[test]
    fn invalid_limits_are_rejected() {
        for (linear, angular, ticks) in [
            (0.0, 0.1, 30),
            (0.05, f32::NAN, 30),
            (f32::INFINITY, 0.1, 30),
            (0.05, 0.1, 0),
        ] {
            assert!(matches!(
                SettleDetector::new(linear, angular, ticks),
                Err(RagdollError::InvalidValue(_))
            ));
        }
    }
}
