// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Composition and sampling cadence for the retained firmware ceiling.
//!
//! Temperature decoding and regulation belong to the PMP provider and the
//! shared C cooling policy. This module defines no temperature thresholds.

pub(crate) const PSTATE_MAX: u32 = 9;
pub(crate) const COOLING_MAX: u32 = 8;
pub(crate) const SAMPLE_NS: u64 = 5_000_000;

/// Completion polling must not turn into a PMP MMIO read on every pass.
pub(crate) fn read_due(last: Option<u64>, now: u64) -> bool {
    last.is_none_or(|last| now.wrapping_sub(last) >= SAMPLE_NS)
}

/// Zero is not an unlimited selector in the firmware. Every published state
/// is 1..=9, bounded by the initialized Main table and the user ceiling.
pub(crate) fn limit(configured: u32, ceiling: u32, user: u32) -> u32 {
    ceiling.min(PSTATE_MAX).min(configured).min(user).max(1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn provider_sampling_is_independent_of_completion_polling() {
        assert!(read_due(None, 0));
        assert!(!read_due(Some(0), 0));
        assert!(!read_due(Some(100_000_000), 104_999_999));
        assert!(read_due(Some(100_000_000), 105_000_000));
        assert!(read_due(Some(u64::MAX - SAMPLE_NS + 1), 0));
    }

    #[test]
    fn initialized_selectors_and_user_caps_bound_every_request() {
        assert_eq!(COOLING_MAX, PSTATE_MAX - 1);
        for configured in [0, 1, 7, 9, 10, u32::MAX] {
            for ceiling in [0, 1, 5, 9, 10, u32::MAX] {
                for user in [0, 1, 3, 9, 10, u32::MAX] {
                    let value = limit(configured, ceiling, user);
                    assert!((1..=PSTATE_MAX).contains(&value));
                    assert!(value <= configured.max(1));
                    assert!(value <= ceiling.max(1));
                    assert!(value <= user.max(1));
                }
            }
        }
    }
}
