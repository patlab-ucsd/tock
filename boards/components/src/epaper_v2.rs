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

use capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice;
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil::spi::{SpiMaster, SpiMasterDevice};
use kernel::hil::{self, gpio};

// Setup static space for the objects.
#[macro_export]
macro_rules! epaper_v2_component_static {
    ($S: ty, $G:ty $(,)?) => {{
        let spi_device = kernel::static_buf!(
            capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, $S>
        );
        let buffer = kernel::static_buf!([u8; capsules_extra::epaper_v2::BUFFER_SIZE]);
        let epaper_v2 = kernel::static_buf!(
            capsules_extra::epaper_v2::EPaper<
                'static,
                capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, $S>,
                $G,
            >
        );

        (spi_device, buffer, epaper_v2)
    };};
}

pub type EPaperV2ComponentType<S, G> = capsules_extra::epaper_v2::EPaper<
    'static,
    capsules_core::virtualizers::virtual_spi::MuxSpiMaster<'static, S>,
    G,
>;

pub struct EPaperV2Component<S: 'static + hil::spi::SpiMaster<'static>, G: 'static + hil::gpio::Pin>
{
    spi_device: &'static capsules_core::virtualizers::virtual_spi::MuxSpiMaster<'static, S>,
    gpio: &'static G,
    chip_select: S::ChipSelect,
    use_charge_pump: bool,
}

impl<S: 'static + hil::spi::SpiMaster<'static>, G: 'static + hil::gpio::Pin>
    EPaperV2Component<S, G>
{
    pub fn new(
        spi_device: &'static capsules_core::virtualizers::virtual_spi::MuxSpiMaster<'static, S>,
        gpio: &'static G,
        chip_select: S::ChipSelect,
        use_charge_pump: bool,
    ) -> EPaperV2Component<S, G> {
        EPaperV2Component {
            spi_device,
            gpio,
            chip_select,
            use_charge_pump,
        }
    }
}

impl<S: 'static + SpiMaster<'static>, G: 'static + gpio::Pin> Component
    for EPaperV2Component<S, G>
{
    type StaticInput = (
        &'static mut MaybeUninit<VirtualSpiMasterDevice<'static, S>>,
        &'static mut MaybeUninit<[u8; capsules_extra::epaper_v2::BUFFER_SIZE]>,
        &'static mut MaybeUninit<
            capsules_extra::epaper_v2::EPaper<'static, VirtualSpiMasterDevice<'static, S>, G>,
        >,
    );
    type Output =
        &'static capsules_extra::epaper_v2::EPaper<'static, VirtualSpiMasterDevice<'static, S>, G>;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let eeink_spi = static_buffer.0.write(VirtualSpiMasterDevice::new(
            self.spi_device,
            self.chip_select,
        ));

        let buffer = static_buffer
            .1
            .write([0; capsules_extra::epaper_v2::BUFFER_SIZE]);

        let epaper_v2 = static_buffer
            .2
            .write(capsules_extra::epaper_v2::EPaper::new(
                eeink_spi,
                self.gpio,
                buffer,
                self.use_charge_pump,
            ));

        eeink_spi.setup();
        eeink_spi.set_client(epaper_v2);

        epaper_v2
    }
}
