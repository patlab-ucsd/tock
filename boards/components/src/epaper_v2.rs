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

use capsules_core::virtualizers::virtual_alarm::{MuxAlarm, VirtualMuxAlarm};
use capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice;
use core::mem::MaybeUninit;
use kernel::component::Component;
use kernel::hil::spi::{SpiMaster, SpiMasterDevice};
use kernel::hil::time::Alarm;
use kernel::hil::{self, gpio};

// Setup static space for the objects.
#[macro_export]
macro_rules! epaper_v2_component_static {
    ($S: ty, $G:ty, $GI:ty, $A:ty $(,)?) => {{
        let spi_device = kernel::static_buf!(
            capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, $S>
        );
        let buffer = kernel::static_buf!([u8; capsules_extra::epaper_v2::BUFFER_SIZE]);
        let epaper_v2 = kernel::static_buf!(
            capsules_extra::epaper_v2::EPaper<
                'static,
                capsules_core::virtualizers::virtual_spi::VirtualSpiMasterDevice<'static, $S>,
                $G,
                $GI,
                capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, $A>,
            >
        );
        let alarm = kernel::static_buf!(
            capsules_core::virtualizers::virtual_alarm::VirtualMuxAlarm<'static, $A>
        );
        (spi_device, buffer, epaper_v2, alarm)
    };};
}

pub type EPaperV2ComponentType<S, G, GI, A> = capsules_extra::epaper_v2::EPaper<
    'static,
    capsules_core::virtualizers::virtual_spi::MuxSpiMaster<'static, S>,
    G,
    GI,
    A,
>;

pub struct EPaperV2Component<
    S: 'static + hil::spi::SpiMaster<'static>,
    G: 'static + hil::gpio::Output,
    GI: 'static + hil::gpio::Input,
    A: 'static + hil::time::Alarm<'static>,
> {
    spi_device: &'static capsules_core::virtualizers::virtual_spi::MuxSpiMaster<'static, S>,
    gpio: &'static G,
    chip_select: S::ChipSelect,
    gpio_busy: &'static GI,
    gpio_reset: &'static G,
    mux_alarm: &'static MuxAlarm<'static, A>,
}

impl<
        S: 'static + hil::spi::SpiMaster<'static>,
        G: 'static + hil::gpio::Output,
        GI: 'static + hil::gpio::Input,
        A: 'static + hil::time::Alarm<'static>,
    > EPaperV2Component<S, G, GI, A>
{
    pub fn new(
        spi_device: &'static capsules_core::virtualizers::virtual_spi::MuxSpiMaster<'static, S>,
        gpio: &'static G,
        chip_select: S::ChipSelect,
        gpio_busy: &'static GI,
        gpio_reset: &'static G,
        mux_alarm: &'static MuxAlarm<'static, A>,
    ) -> EPaperV2Component<S, G, GI, A> {
        EPaperV2Component {
            spi_device,
            gpio,
            chip_select,
            gpio_busy,
            gpio_reset,
            mux_alarm,
        }
    }
}

impl<
        S: 'static + SpiMaster<'static>,
        G: 'static + gpio::Output,
        GI: 'static + gpio::Input + gpio::Interrupt<'static>,
        A: 'static + hil::time::Alarm<'static>,
    > Component for EPaperV2Component<S, G, GI, A>
{
    type StaticInput = (
        &'static mut MaybeUninit<VirtualSpiMasterDevice<'static, S>>,
        &'static mut MaybeUninit<[u8; capsules_extra::epaper_v2::BUFFER_SIZE]>,
        &'static mut MaybeUninit<
            capsules_extra::epaper_v2::EPaper<
                'static,
                VirtualSpiMasterDevice<'static, S>,
                G,
                GI,
                VirtualMuxAlarm<'static, A>,
            >,
        >,
        &'static mut MaybeUninit<VirtualMuxAlarm<'static, A>>,
    );
    type Output = &'static capsules_extra::epaper_v2::EPaper<
        'static,
        VirtualSpiMasterDevice<'static, S>,
        G,
        GI,
        VirtualMuxAlarm<'static, A>,
    >;

    fn finalize(self, static_buffer: Self::StaticInput) -> Self::Output {
        let eeink_spi = static_buffer.0.write(VirtualSpiMasterDevice::new(
            self.spi_device,
            self.chip_select,
        ));

        let epaper_virtual_alarm = static_buffer.3.write(VirtualMuxAlarm::new(self.mux_alarm));
        epaper_virtual_alarm.setup();

        let buffer = static_buffer
            .1
            .write([0; capsules_extra::epaper_v2::BUFFER_SIZE]);

        self.gpio_reset.set();
        let epaper_v2 = static_buffer
            .2
            .write(capsules_extra::epaper_v2::EPaper::new(
                eeink_spi,
                self.gpio,
                buffer,
                self.gpio_busy,
                self.gpio_reset,
                epaper_virtual_alarm,
            ));

        epaper_virtual_alarm.set_alarm_client(epaper_v2);

        gpio::Interrupt::set_client(self.gpio_busy, epaper_v2);

        eeink_spi.setup();
        eeink_spi.set_client(epaper_v2);

        epaper_v2
    }
}
