// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Bounded isolation for a compute watchdog blocked behind another VM's older kick.
//! Publication order is evidence of a possible blocker, not proof of a firmware
//! dependency. Renewals are bounded; the oldest unfinished queue is never excused.

pub(crate) const MAX_RENEWALS: u8 = 8;

/// Keep the scheduler poll, control-ring waits and ordinary jobs at two seconds.
/// Native ray traversal can legitimately exceed one poll (CTS stress). Four
/// extra polls give it ten seconds, still one tenth of the legacy scheduler's
/// fallback. This is an execution allowance, not evidence of GPU progress, and
/// is consumed once per scheduler job, independently of cross-VM/replay grace.
pub(crate) const POLL_MS: u32 = 2000;
const RAY_QUERY_RENEWALS: u8 = 4;

pub(crate) fn renew_ray_query(published: bool, enabled: bool, renewals: &mut u8) -> bool {
    if !published || !enabled || *renewals >= RAY_QUERY_RENEWALS {
        return false;
    }
    *renewals += 1;
    true
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Head {
    pub(crate) vm: usize,
    pub(crate) published: u64,
    pub(crate) finished: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Decision {
    Own,
    Defer,
    Spare,
}

pub(crate) fn classify(own: Head, renewals: u8, others: impl Iterator<Item = Head>) -> Decision {
    if own.finished || !others.into_iter().any(|other| {
        other.vm != own.vm && !other.finished && other.published < own.published
    }) {
        Decision::Own
    } else if renewals < MAX_RENEWALS {
        Decision::Defer
    } else {
        Decision::Spare
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn head(vm: usize, published: u64, finished: bool) -> Head {
        Head { vm, published, finished }
    }
    #[test]
    fn ray_query_execution_is_bounded_at_ten_seconds() {
        let mut renewals = 0;
        for poll in 1..=4 {
            assert!(renew_ray_query(true, true, &mut renewals));
            assert_eq!(renewals, poll);
        }
        for _ in 0..256 {
            assert!(!renew_ray_query(true, true, &mut renewals));
        }
        assert_eq!(POLL_MS * (u32::from(renewals) + 1), 10_000);
    }
    #[test]
    fn ordinary_and_unpublished_jobs_keep_the_first_deadline() {
        for (published, enabled) in [(false, false), (false, true), (true, false)] {
            let mut renewals = 0;
            assert!(!renew_ray_query(published, enabled, &mut renewals));
            assert_eq!(renewals, 0);
        }
    }
    #[test]
    fn ray_query_and_cross_vm_credits_are_independent_and_finite() {
        let mut ray = 0;
        let own = head(5, 101, false);
        for _ in 0..4 {
            assert!(renew_ray_query(true, true, &mut ray));
        }
        for victim in 0..MAX_RENEWALS {
            assert!(!renew_ray_query(true, true, &mut ray));
            assert_eq!(classify(own, victim, [head(7, 100, false)].into_iter()), Decision::Defer);
        }
        assert_eq!(classify(own, MAX_RENEWALS, [head(7, 100, false)].into_iter()), Decision::Spare);
        assert_eq!(POLL_MS * (1 + u32::from(ray) + u32::from(MAX_RENEWALS)), 26_000);
    }
    #[test]
    fn exhausted_or_invalid_credit_never_wraps() {
        for value in 4..=u8::MAX {
            let mut renewals = value;
            assert!(!renew_ray_query(true, true, &mut renewals));
            assert_eq!(renewals, value);
        }
    }
    #[test]
    fn earlier_other_vm_gets_bounded_grace() {
        let own = head(5, 101, false);
        for count in 0..MAX_RENEWALS {
            assert_eq!(classify(own, count, [head(7, 100, false)].into_iter()), Decision::Defer);
        }
        assert_eq!(classify(own, MAX_RENEWALS, [head(7, 100, false)].into_iter()), Decision::Spare);
        assert_eq!(classify(own, 2, [].into_iter()), Decision::Own);
    }
    #[test]
    fn oldest_unfinished_queue_is_never_excused() {
        assert_eq!(classify(head(7, 100, false), 0, [head(5, 101, false)].into_iter()), Decision::Own);
    }
    #[test]
    fn same_vm_finished_and_later_work_do_not_block() {
        let own = head(5, 101, false);
        assert_eq!(classify(own, 0, [head(5, 100, false), head(7, 100, true), head(7, 101, false), head(7, 102, false)].into_iter()), Decision::Own);
        assert_eq!(classify(head(5, 101, true), 0, [head(7, 100, false)].into_iter()), Decision::Own);
    }
}
