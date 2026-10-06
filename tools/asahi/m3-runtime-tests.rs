// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Host tests for the production M3 admission, IO ownership and thermal policy.
//! Run: rustc --edition=2021 --test tools/asahi/m3-runtime-tests.rs -o /tmp/m3-runtime-tests
#![allow(dead_code)]

#[path = "../../drivers/gpu/drm/asahi/m3/t8122_admission.rs"]
mod t8122_admission;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_layout.rs"]
mod m3_init_layout;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_init_storage.rs"]
mod m3_init_storage;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_thermal_policy.rs"]
mod m3_thermal_policy;
