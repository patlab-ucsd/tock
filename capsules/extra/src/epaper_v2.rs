// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2023.

//! 7.5inch e-PaperV2

use core::cell::Cell;
use kernel::hil;
use kernel::hil::time::ConvertTicks;
use kernel::utilities::cells::{OptionalCell, TakeCell};
use kernel::utilities::leasable_buffer::SubSliceMut;
use kernel::ErrorCode;

pub const BUFFER_SIZE: usize = 1032;

// TODO - check if these are correct.
const WIDTH: usize = 800;
const HEIGHT: usize = 480;

// Black pixel is 0, white pixel is 1
const OLD_IMAGE: [u8; BUFFER_SIZE] = [1; BUFFER_SIZE];
const NEW_IMAGE: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];

#[derive(Clone, Copy, PartialEq)]
pub enum Command {
    PanelSetting(u8),
    PowerSetting(u8, u8, u8, u8),
    PowerOff,
    PowerOn,
    BoosterSoftStart(u8, u8, u8, u8),
    DeepSleep(u8),
    DataStartTransmission1,
    DataStop,
    DisplayRefresh,
    DisplayStartTransmission2,
    DualSPI(u8),
    PLLControl(u8),
    VCOMDataIntervalSetting(u8, Option<u8>),
    VCOMDCSetting(u8),
    TCONSetting(u8),
    ResolutionSetting(u8, u8, u8, u8),
}

// in match statement add binary that corresponds to each
// command type
impl Command {
    fn encode(self, buffer: &mut SubSliceMut<'static, u8>) -> Result<(), ErrorCode> {
        let encoded_buf: &[u8] = match self {
            Command::PanelSetting(data0) => &[0, data0],
            Command::PowerSetting(data0, data1, data2, data3) => &[1, data0, data1, data2, data3],
            Command::PowerOff => &[2],
            Command::PowerOn => &[4],
            Command::BoosterSoftStart(data0, data1, data2, data3) => {
                &[5, data0, data1, data2, data3]
            }
            Command::DeepSleep(data0) => &[7, data0],
            Command::DataStartTransmission1 => &[0x10],
            Command::DataStop => &[0x11],
            Command::DisplayRefresh => &[0x12],
            Command::DisplayStartTransmission2 => &[0x13],
            Command::DualSPI(data0) => &[0x15, data0],
            Command::PLLControl(data0) => &[0x30, data0],
            Command::VCOMDataIntervalSetting(data0, Some(data1)) => &[0x50, data0, data1],
            Command::VCOMDataIntervalSetting(data0, None) => &[0x50, data0],
            Command::VCOMDCSetting(data0) => &[0x58, data0],
            Command::TCONSetting(data0) => &[0x60, data0],
            Command::ResolutionSetting(data0, data1, data2, data3) => {
                &[0x61, data0, data1, data2, data3]
            }
        };

        // Move the available region of the buffer to what is remaining after
        // this command was encoded.
        if encoded_buf.len() > buffer.len() {
            return Err(ErrorCode::NOMEM);
        }

        buffer.slice(..encoded_buf.len());
        buffer.as_slice().copy_from_slice(encoded_buf);

        Ok(())
    }
}

enum EInkTask {
    SendCommand(Command),
    SendImage(&'static [u8]),
    Reset,
}
pub struct EPaper<
    'a,
    S: hil::spi::SpiMasterDevice<'a>,
    G: hil::gpio::Output,
    GI: hil::gpio::Input,
    A: hil::time::Alarm<'a>,
> {
    spi: &'a S,
    cd_gpio: &'a G,
    client: OptionalCell<&'a dyn hil::screen::ScreenClient>,
    setup_client: OptionalCell<&'a dyn hil::screen::ScreenSetupClient>,
    buffer: TakeCell<'static, [u8]>,
    pending_send: OptionalCell<usize>,
    busy_gpio: &'a GI,
    gpio_reset: &'a G,
    alarm: &'a A,
    task_queue: Cell<[Option<EInkTask>; 10]>,
}

const TASK_QUEUE_REPEAT_VALUE: Option<EInkTask> = None;

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > EPaper<'a, S, G, GI, A>
{
    pub fn new(
        spi: &'a S,
        cd_gpio: &'a G,
        buffer: &'static mut [u8],
        busy_gpio: &'a GI,
        gpio_reset: &'a G,
        alarm: &'a A,
    ) -> EPaper<'a, S, G, GI, A> {
        EPaper {
            spi,
            cd_gpio,
            client: OptionalCell::empty(),
            setup_client: OptionalCell::empty(),
            buffer: TakeCell::new(buffer),
            pending_send: OptionalCell::empty(),
            busy_gpio,
            gpio_reset,
            alarm,
            task_queue: Cell::new([TASK_QUEUE_REPEAT_VALUE; 10]),
        }
    }

    fn enqueue_task(&self, task: EInkTask) -> Result<(), ErrorCode> {
        let mut task_queue = self.task_queue.take();
        for i in 0..task_queue.len() {
            if task_queue[i].is_none() {
                task_queue[i] = Some(task);
                self.task_queue.set(task_queue);
                return Ok(());
            }
        }

        self.task_queue.set(task_queue);
        Err(ErrorCode::NOMEM)
    }

    fn check_busy(&self) -> bool {
        // check if busy; busy is active low
        if !self.busy_gpio.read() {
            let now = self.alarm.now();
            let dt = self.alarm.ticks_from_ms(200);
            self.alarm.set_alarm(now, dt);
            return true;
        }

        false
    }

    fn do_next_task(&self) {
        if self.check_busy() {
            return;
        }

        let mut task_queue = self.task_queue.take();
        let task = task_queue[0].take();
        for i in 1..task_queue.len() {
            task_queue[i - 1] = task_queue[i].take();
        }

        task_queue[task_queue.len() - 1] = None;

        self.task_queue.set(task_queue);

        if let Some(task) = task {
            match task {
                EInkTask::SendCommand(command) => {
                    self.send_command(command).unwrap();
                }
                EInkTask::SendImage(image) => {
                    self.buffer
                        .map(|buf| buf[0..image.len()].copy_from_slice(image));

                    self.spi
                        .read_write_bytes(self.buffer.take().unwrap(), None, image.len())
                        .unwrap();
                }
                EInkTask::Reset => {
                    self.gpio_reset.clear();
                    self.gpio_reset.set();
                }
            }
        }
    }

    pub fn send_sequence(&self) -> Result<(), ErrorCode> {
        self.enqueue_task(EInkTask::Reset)?;

        self.enqueue_task(EInkTask::SendCommand(Command::BoosterSoftStart(
            0x17, 0x17, 0x27, 0x17,
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::PowerSetting(
            0x07, 0x17, 0x3f, 0x3f,
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::PanelSetting(0x3F)))?;

        self.enqueue_task(EInkTask::SendCommand(Command::PLLControl(0x6)))?;

        self.enqueue_task(EInkTask::SendCommand(Command::ResolutionSetting(
            0x03, 0x20, 0x01, 0xE0,
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::DualSPI(0x00)))?;

        self.enqueue_task(EInkTask::SendCommand(Command::TCONSetting(0x22)))?;

        self.enqueue_task(EInkTask::SendCommand(Command::VCOMDCSetting(0x26)))?;

        self.enqueue_task(EInkTask::SendCommand(Command::VCOMDataIntervalSetting(
            0x31,
            Some(0x07),
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::DataStartTransmission1))?;

        self.enqueue_task(EInkTask::SendImage(&OLD_IMAGE))?;

        self.enqueue_task(EInkTask::SendCommand(Command::DisplayStartTransmission2))?;
        self.enqueue_task(EInkTask::SendImage(&NEW_IMAGE))?;

        self.enqueue_task(EInkTask::SendCommand(Command::DisplayRefresh))?;

        self.enqueue_task(EInkTask::SendCommand(Command::VCOMDataIntervalSetting(
            0x50, None,
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::PowerOff))?;

        self.enqueue_task(EInkTask::SendCommand(Command::DeepSleep(0xa5)))?;

        Ok(())
    }

    pub fn send_command(&self, command: Command) -> Result<(), ErrorCode> {
        let buffer = self.buffer.take().unwrap();
        let mut send_slice: SubSliceMut<'static, u8> = SubSliceMut::new(buffer);
        let result = command.encode(&mut send_slice);

        if result.is_err() {
            return result;
        }

        let send_len = send_slice.len();
        if send_len > 1 {
            self.pending_send.set(send_len - 1);
        }

        // Set CD high for data, low for command
        self.cd_gpio.clear();
        self.spi
            .read_write_bytes(send_slice.take(), None, 1)
            .map_err(|(err, buf, _)| {
                self.buffer.replace(buf);
                err
            })
    }

    pub fn test_init(&self) {
        self.send_sequence().unwrap();
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > hil::screen::ScreenSetup<'a> for EPaper<'a, S, G, GI, A>
{
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
impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > hil::screen::Screen<'a> for EPaper<'a, S, G, GI, A>
{
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
        _x: usize,
        _y: usize,
        _width: usize,
        _height: usize,
    ) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn write(&self, _data: SubSliceMut<'static, u8>, _continue: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_brightness(&self, _brightness: u16) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_power(&self, _enabled: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }

    fn set_invert(&self, _enabled: bool) -> Result<(), ErrorCode> {
        Err(ErrorCode::NOSUPPORT)
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > hil::spi::SpiMasterClient for EPaper<'a, S, G, GI, A>
{
    fn read_write_done(
        &self,
        write: &'static mut [u8],
        _read: Option<&'static mut [u8]>,
        _len: usize,
        _spi_status: Result<(), ErrorCode>,
    ) {
        if self.pending_send.is_some() {
            let remaining_len = self.pending_send.take().unwrap();
            write.copy_within(1..remaining_len, 0);

            // Set CD high for data, low for command
            self.cd_gpio.set();
            self.spi
                .read_write_bytes(write, None, remaining_len)
                .unwrap();
        } else {
            self.buffer.replace(write);
            self.do_next_task();
        }
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > kernel::hil::gpio::Client for EPaper<'a, S, G, GI, A>
{
    fn fired(&self) {
        kernel::debug!("gpio fired");
    }
}

impl<
        'a,
        S: hil::spi::SpiMasterDevice<'a>,
        G: hil::gpio::Output,
        GI: hil::gpio::Input,
        A: hil::time::Alarm<'a>,
    > kernel::hil::time::AlarmClient for EPaper<'a, S, G, GI, A>
{
    fn alarm(&self) {
        self.do_next_task();
    }
}
