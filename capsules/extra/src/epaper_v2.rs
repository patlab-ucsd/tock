// Licensed under the Apache License, Version 2.0 or the MIT License.
// SPDX-License-Identifier: Apache-2.0 OR MIT
// Copyright Tock Contributors 2023.

//! 7.5inch e-PaperV2

use core::cell::Cell;
use kernel::hil;
use kernel::hil::time::ConvertTicks;
use kernel::utilities::cells::{MapCell, OptionalCell, TakeCell};
use kernel::utilities::leasable_buffer::{SubSlice, SubSliceMut};
use kernel::ErrorCode;

// TODO - check if these are correct.
const WIDTH: usize = 800;
const HEIGHT: usize = 480;

pub const BUFFER_SIZE: usize = WIDTH / 8;

type X = usize;
type Y = usize;
type WIDTH = usize;
type HEIGHT = usize;

#[derive(PartialEq)]
enum EInkState {
    InitInProgress,
    Idle,
    ReadyWrite(X, Y, WIDTH, HEIGHT),
    WriteInProgress(SubSliceMut<'static, u8>, X, Y, WIDTH, HEIGHT, usize),
    Off,
}

#[derive(Clone, Copy, PartialEq, Debug)]
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
                &[6, data0, data1, data2, data3]
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
            Command::TCONSetting(data0) => &[0x60, data0],
            Command::ResolutionSetting(data0, data1, data2, data3) => {
                &[0x61, data0, data1, data2, data3]
            }
            Command::VCOMDCSetting(data0) => &[0x82, data0],
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

#[derive(Clone, Copy, PartialEq, Debug)]
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
    gpio_power: &'a G,
    alarm: &'a A,
    reset_inprogress: Cell<bool>,
    task_queue: Cell<[Option<EInkTask>; 10]>,
    state: MapCell<EInkState>,
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
        gpio_power: &'a G,
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
            gpio_power,
            alarm,
            reset_inprogress: Cell::new(false),
            task_queue: Cell::new([TASK_QUEUE_REPEAT_VALUE; 10]),
            state: MapCell::new(EInkState::Off),
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
            kernel::debug!("[EInk] Busy -- setting alarm for 200ms.");
            let now = self.alarm.now();
            let dt = self.alarm.ticks_from_ms(200);
            self.alarm.set_alarm(now, dt);
            return true;
        }

        false
    }

    fn tasks_complete(&self) -> bool {
        let task_queue = self.task_queue.take();
        let mut complete = false;
        //   kernel::debug!("Task queue: {:?}", task_queue);
        if task_queue[0].is_none() {
            complete = true;
        }

        self.task_queue.set(task_queue);
        return complete;
    }

    fn do_next_task(&self) {
        // TODO - I don't like the logic flow here... what we need to do
        // is to only mark as busy if the next task is not going to reset.
        if self.check_busy() {
            let task_queue = self.task_queue.take();
            self.task_queue.set(task_queue);
            if let Some(EInkTask::Reset) = task_queue[0] {
            } else {
                return;
            }
        }

        // Pop the first task from the queue.
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
                    kernel::debug!("[EInk] Sending image.");
                    self.cd_gpio.set();
                    self.buffer
                        .map(|buf| buf[0..image.len()].copy_from_slice(image));

                    self.spi
                        .read_write_bytes(SubSliceMut::new(self.buffer.take().unwrap()), None)
                        .unwrap();
                }
                EInkTask::Reset => {
                    kernel::debug!("[EInk] Resetting.");
                    self.reset_inprogress.set(true);
                    self.gpio_reset.clear();
                    let now = self.alarm.now();
                    let dt = self.alarm.ticks_from_ms(200);
                    self.alarm.set_alarm(now, dt);
                }
            }
        } else {
            if let EInkState::WriteInProgress(
                buf_slice,
                frame_x,
                frame_y,
                frame_width,
                frame_height,
                row_number,
            ) = self.state.take().unwrap()
            {}
        }
    }

    fn consecutive_send(
        &self,
        mut buf_slice: SubSliceMut<'static, u8>,
        frame_x: usize,
        frame_y: usize,
        frame_width: usize,
        frame_height: usize,
        row_number: usize,
    ) {
        for x in 0..frame_width {
            // outside the frame
            if row_number < frame_y
                || row_number >= frame_y + frame_height
                || x < frame_x
                || x >= frame_x + frame_width
            {
                buf_slice[x] = 0;
            }
        }

        let mut send_data: &'static mut [u8] = buf_slice.take();

        self.buffer.map(|buf| {
            buf[0..frame_width].copy_from_slice(&mut send_data);
        });

        let remaining_data = SubSliceMut::new(&mut send_data[frame_width..]);
        let _ = self.enqueue_task(EInkTask::SendImage(self.buffer.take().unwrap()));

        self.state.replace(EInkState::WriteInProgress(
            remaining_data,
            frame_x,
            frame_y,
            frame_width,
            frame_height,
            row_number + 1,
        ));
    }

    pub fn init_sequence(&self) -> Result<(), ErrorCode> {
        self.state.replace(EInkState::InitInProgress);

        self.enqueue_task(EInkTask::Reset)?;

        self.enqueue_task(EInkTask::SendCommand(Command::PowerSetting(
            0x07, 0x07, 0x3f, 0x3f,
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::BoosterSoftStart(
            0x17, 0x17, 0x28, 0x17,
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::PowerOn))?;

        self.enqueue_task(EInkTask::SendCommand(Command::PanelSetting(0x1F)))?;

        self.enqueue_task(EInkTask::SendCommand(Command::ResolutionSetting(
            0x03, 0x20, 0x01, 0xE0,
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::DualSPI(0x00)))?;

        self.enqueue_task(EInkTask::SendCommand(Command::TCONSetting(0x22)))?;

        self.enqueue_task(EInkTask::SendCommand(Command::VCOMDataIntervalSetting(
            0x10,
            Some(0x07),
        )))?;

        Ok(())
    }

    fn complete_screen_write(&self) -> Result<(), ErrorCode> {
        self.enqueue_task(EInkTask::SendCommand(Command::DisplayRefresh))?;

        self.enqueue_task(EInkTask::SendCommand(Command::VCOMDataIntervalSetting(
            0x31,
            Some(0x07),
        )))?;

        self.enqueue_task(EInkTask::SendCommand(Command::PowerOff))?;

        self.enqueue_task(EInkTask::SendCommand(Command::DeepSleep(0xa5)))?;
        Ok(())
    }

    pub fn send_sequence(&self, buf: &'static [u8]) -> Result<(), ErrorCode> {
        self.enqueue_task(EInkTask::SendCommand(Command::DisplayStartTransmission2))?;
        self.enqueue_task(EInkTask::SendImage(buf))?;
        Ok(())
    }

    pub fn send_command(&self, command: Command) -> Result<(), ErrorCode> {
        kernel::debug!("[EInk] Sending command {:?}", command);
        let buffer = self.buffer.take().unwrap();
        let mut send_slice: SubSliceMut<'static, u8> = SubSliceMut::new(buffer);
        let result = command.encode(&mut send_slice);

        if result.is_err() {
            return result;
        }

        let send_len = send_slice.len();
        if send_len > 1 {
            self.pending_send.set(send_len);
        }

        // Set CD high for data, low for command
        self.cd_gpio.clear();
        self.spi
            .read_write_bytes(send_slice, None)
            .map_err(|(err, buf, _)| {
                self.buffer.replace(buf.take());
                err
            })
    }

    pub fn test_init(&self) {
        // self.send_sequence().unwrap();
        kernel::debug!("[EInk] Complete enqueuing tasks.");
        self.do_next_task();
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
        1
    }

    fn get_supported_pixel_format(&self, index: usize) -> Option<hil::screen::ScreenPixelFormat> {
        match index {
            0 => Some(hil::screen::ScreenPixelFormat::Mono),
            _ => None,
        }
    }
}

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
        x: usize,
        y: usize,
        width: usize,
        height: usize,
    ) -> Result<(), ErrorCode> {
        let state = self.state.take().unwrap();
        if state != EInkState::Idle {
            if state != EInkState::InitInProgress {
                self.init_sequence().unwrap();
            }
            self.state.replace(state);
            return Err(ErrorCode::BUSY);
        }

        if x + width > WIDTH || y + height > HEIGHT {
            self.state.replace(state);
            return Err(ErrorCode::INVAL);
        }

        self.state
            .replace(EInkState::ReadyWrite(x, y, width, height));

        Ok(())
    }

    fn write(&self, data: SubSliceMut<'static, u8>, _continue: bool) -> Result<(), ErrorCode> {
        if let EInkState::ReadyWrite(frame_x, frame_y, frame_width, frame_height) =
            self.state.take().unwrap()
        {
            if data.len() > frame_width * frame_height {
                return Err(ErrorCode::SIZE);
            }

            self.enqueue_task(EInkTask::SendCommand(Command::DisplayStartTransmission2))?;

            self.consecutive_send(data, frame_x, frame_y, frame_width, frame_height, 0)
        }
        Ok(())
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
        write: SubSliceMut<u8>,
        _read: Option<SubSliceMut<u8>>,
        _spi_status: Result<usize, ErrorCode>,
    ) {
        // THIS NEEDS TO BE IMPLEMENTED. THE RECENT
        // SPI CHANGES CREATE ISSUES HERE WITH THE
        // BORROW CHECKER. WE NEED TO KICK OFF A NEW
        // TRANSACTION HERE TO COMPLETE THE EEINK SEND.
        /*
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
        */
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
        if self.reset_inprogress.get() {
            self.reset_inprogress.set(false);
            self.gpio_reset.set();
            self.do_next_task();
        } else if !self.tasks_complete() {
            self.do_next_task();
        } else {
            self.gpio_power.clear();
        }
    }
}
