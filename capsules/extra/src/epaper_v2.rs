// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2023.

//! 7.5inch e-PaperV2

use core::cell::Cell;
use core::fmt::Error;
use kernel::hil;
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::ErrorCode;

pub const BUFFER_SIZE: usize = 1032;

// TODO - check if these are correct.
const WIDTH: usize = 800;
const HEIGHT: usize = 480;

// all commands that are on command table (pg 22.)
#[derive(Copy, Clone, PartialEq)]
pub enum Command {}

// in match statement add binary that corresponds to each
// command type
impl Command {
    fn encode(self, buffer: &mut SubSliceMut<'static, u8>) {
        let take = match self {};

        // Move the available region of the buffer to what is remaining after
        // this command was encoded.
        // buffer.slice(take..);
    }
}

// #[derive(Copy, Clone, PartialEq)]
#[derive(Clone, Copy, PartialEq)]
enum State {
    Idle,
    Init,
    SimpleCommand,
    Write,
}

pub struct EPaper<'a, S: hil::spi::SpiMasterDevice<'a>> {
    spi: &'a S,
    state: Cell<State>,
    client: OptionalCell<&'a dyn hil::screen::ScreenClient>,
    setup_client: OptionalCell<&'a dyn hil::screen::ScreenSetupClient>,
    buffer: TakeCell<'static, [u8]>,
    write_buffer: MapCell<SubSliceMut<'static, u8>>,
}

impl<'a, S: hil::spi::SpiMasterDevice<'a>> EPaper<'a, S> {
    pub fn new(spi: &'a S, buffer: &'static mut [u8], enable_charge_pump: bool) -> EPaper<'a, S> {
        EPaper {
            spi,
            state: Cell::new(State::Idle),
            client: OptionalCell::empty(),
            setup_client: OptionalCell::empty(),
            buffer: TakeCell::new(buffer),
            write_buffer: MapCell::empty(),
        }
    }

    pub fn init_screen(&self) {
        // @EDWARD: This will be called on the main loop so you can use this
        // as a quick way for now to debug if everything is "hooked up" correctly
        // and if the screen is working.
    }

    fn send_sequence(&self, sequence: &[Command]) -> Result<(), ErrorCode> {
        Ok(())
    }
}

impl<'a, S: hil::spi::SpiMasterDevice<'a>> hil::screen::ScreenSetup<'a> for EPaper<'a, S> {
    fn set_client(&self, client: &'a dyn hil::screen::ScreenSetupClient) {
        self.setup_client.set(client);
    }

    fn set_resolution(&self, _resolution: (usize, usize)) -> Result<(), ErrorCode> {
        // todo
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_pixel_format(&self, _depth: hil::screen::ScreenPixelFormat) -> Result<(), ErrorCode> {
        // todo
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_rotation(&self, _rotation: hil::screen::ScreenRotation) -> Result<(), ErrorCode> {
        // todo
        Err(ErrorCode::NOSUPPORT)
    }

    fn get_num_supported_resolutions(&self) -> usize {
        // todo
        1
    }

    fn get_supported_resolution(&self, index: usize) -> Option<(usize, usize)> {
        // todo
        match index {
            0 => Some((WIDTH, HEIGHT)),
            _ => None,
        }
    }

    fn get_num_supported_pixel_formats(&self) -> usize {
        // todo
        1
    }

    fn get_supported_pixel_format(&self, index: usize) -> Option<hil::screen::ScreenPixelFormat> {
        // todo
        match index {
            0 => Some(hil::screen::ScreenPixelFormat::Mono),
            _ => None,
        }
    }
}

// TODO
impl<'a, S: hil::spi::SpiMasterDevice<'a>> hil::screen::Screen<'a> for EPaper<'a, S> {
    fn set_client(&self, client: &'a dyn hil::screen::ScreenClient) {
        self.client.set(client);
    }

    fn get_resolution(&self) -> (usize, usize) {
        (WIDTH, HEIGHT)
    }

    fn get_pixel_format(&self) -> hil::screen::ScreenPixelFormat {
        hil::screen::ScreenPixelFormat::Mono
    }

    fn get_rotation(&self) -> hil::screen::ScreenRotation {
        hil::screen::ScreenRotation::Normal
    }

    fn set_write_frame(
        &self,
        x: usize,
        y: usize,
        width: usize,
        height: usize,
    ) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn write(&self, data: SubSliceMut<'static, u8>, _continue: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_brightness(&self, brightness: u16) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_power(&self, enabled: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_invert(&self, enabled: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

impl<'a, S: hil::spi::SpiMasterDevice<'a>> hil::i2c::I2CClient for EPaper<'a, S> {
    fn command_complete(&self, buffer: &'static mut [u8], _status: Result<(), hil::i2c::Error>) {}
}
