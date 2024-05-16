// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2024.

//! Components for the epaper_v2 screen.
//!
//! Usage
//! -----
//! ```rust
//!
//! ```

use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil;

// Setup static space for the objects.
#[macro_export]
macro_rules! epaper_v2_component_static {
    ($S: ty $(,)?) => {{
        let buffer = kernel::static_buf!([u8; capsules_extra::epaper_v2::BUFFER_SIZE]);
        let epaper_v2 = kernel::static_buf!(
            capsules_extra::epaper_v2::EPaper<
                'static,
                capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, $S>,
            >
        );

        (buffer, epaper_v2)
    };};
}

pub type EPaperV2ComponentType<S> = capsules_extra::epaper_v2::EPaper<
    'static,
    capsules_core::virtualizers::virtual_spi::MuxSpiMaster<'static, S>,
>;

pub struct EPaperV2Component<S: hil::spi::SpiMaster<'static> + 'static> {
    spi_device:
        &'static capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, S>,
    use_charge_pump: bool,
}

impl<S: hil::spi::SpiMaster<'static> + 'static> EPaperV2Component<S> {
    pub fn new(
        spi_device: &'static capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<
            'static,
            S,
        >,
        use_charge_pump: bool,
    ) -> EPaperV2Component<S> {
        EPaperV2Component {
            spi_device,
            use_charge_pump,
        }
    }
}

impl<S: hil::spi::SpiMaster<'static> + 'static> Component for EPaperV2Component<S> {
    type StaticInput = (
        &'static mut MaybeUninit<[u8; capsules_extra::epaper_v2::BUFFER_SIZE]>,
        &'static mut MaybeUninit<
            capsules_extra::epaper_v2::EPaper<
                'static,
                capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, S>,
            >,
        >,
    );
    type Output = &'static capsules_extra::epaper_v2::EPaper<
        'static,
        capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, S>,
    >;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let buffer = static_buffer
            .0
            .write([0; capsules_extra::epaper_v2::BUFFER_SIZE]);

        let epaper_v2 = static_buffer
            .1
            .write(capsules_extra::epaper_v2::EPaper::new(
                &self.spi_device,
                buffer,
                self.use_charge_pump,
            ));

        self.spi_device.setup();
        epaper_v2
    }
}
