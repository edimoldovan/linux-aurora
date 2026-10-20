// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Qualified J713/25G83 selection of the original ADT's sixteen performance states.
//!
//! Four common low states are followed by six equal-voltage pairs. These are indices into
//! the machine's original ADT, not frequencies or voltages copied from a reference machine.

pub(crate) const SOURCE_STATES: usize = 16;
pub(crate) const MAIN: [usize; 10] = [0, 1, 2, 3, 5, 7, 9, 11, 13, 15];
pub(crate) const AUXILIARY: [usize; 10] = [0, 1, 2, 3, 4, 6, 8, 10, 12, 14];

use super::super::fw::initdata::{Performance, PERF_STATES};

/// Allocation-free selection, also exercised directly by the host-side source-data tests.
pub(crate) fn select(
    source: &super::super::adt::perf::Source,
) -> core::result::Result<Performance, super::super::adt::perf::Error> {
    use super::super::adt::perf::Error;
    if source.count != SOURCE_STATES {
        return Err(Error::Shape);
    }
    let mut performance = Performance {
        frequency: [0; PERF_STATES],
        auxiliary: [0; PERF_STATES],
        voltage: [0; PERF_STATES],
        sram: [0; PERF_STATES],
    };
    for i in 0..PERF_STATES {
        let (main, aux) = (MAIN[i], AUXILIARY[i]);
        if source.frequency_mhz[main] != source.sram_frequency_mhz[main]
            || source.frequency_mhz[aux] != source.sram_frequency_mhz[aux]
            || source.core_mv[main] != source.core_mv[aux]
            || source.sram_mv[main] != source.sram_mv[aux]
        {
            return Err(Error::Shape);
        }
        performance.frequency[i] = source.frequency_mhz[main];
        performance.auxiliary[i] = source.frequency_mhz[aux];
        performance.voltage[i] = source.core_mv[main];
        performance.sram[i] = source.sram_mv[main];
    }
    if !performance.valid() {
        return Err(Error::Range);
    }
    Ok(performance)
}
