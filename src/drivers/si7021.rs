use core::cell::Cell;
use common::take_cell::TakeCell;
use main::{AppId, Callback, Driver};
use hil::i2c;
use hil::alarm;

// Buffer to use for I2C messages
pub static mut BUFFER : [u8; 14] = [0; 14];

/// Register values
// const REGISTER_AUTO_INCREMENT: u8 = 0x80;

// const CTRL_REG1_POWER_ON: u8 = 0x80;
// const CTRL_REG1_BLOCK_DATA_ENABLE: u8 = 0x04;
// const CTRL_REG2_ONE_SHOT: u8 = 0x01;
// const CTRL_REG3_INTERRUPT1_DATAREADY: u8 = 0x04;

#[allow(dead_code)]
enum Registers {
    MeasRelativeHumidityHoldMode = 0xe5,
    MeasRelativeHumidityNoHoldMode = 0xf5,
    MeasTemperatureHoldMode = 0xe3,
    MeasTemperatureNoHoldMode = 0xf3,
    ReadTemperaturePreviousRHMeasurement = 0xe0,
    Reset = 0xfe,
    WriteRHTUserRegister1 = 0xe6,
    ReadRHTUserRegister1 = 0xe7,
    WriteHeaterControlRegister = 0x51,
    ReadHeaterControlRegister = 0x11,
    ReadElectronicIdByteOneA = 0xfa,
    ReadElectronicIdByteOneB = 0x0f,
    ReadElectronicIdByteTwoA = 0xfc,
    ReadElectronicIdByteTwoB = 0xc9,
    ReadFirmwareVersionA = 0x84,
    ReadFirmwareVersionB = 0xb8,
}








/// States of the I2C protocol with the LPS331AP.
#[derive(Clone,Copy,PartialEq)]
enum State {
    Idle,

    // /// Read the WHO_AM_I register. This should return 0xBB.
    // SelectWhoAmI,
    // ReadingWhoAmI,

    // /// Process of taking a pressure measurement.
    // /// Start with chip powered off
    // TakeMeasurementInit,
    // /// Then clear the current reading (just in case it exists)
    // /// to reset the interrupt line.
    // TakeMeasurementClear,
    // /// Enable a single shot measurement with interrupt when data is ready.
    // TakeMeasurementConfigure,

    // /// Read the 3 pressure registers.
    // ReadMeasurement,
    // /// Calculate pressure and call the callback with the value.
    // GotMeasurement,



SelectElectronicId1,
ReadElectronicId1,
SelectElectronicId2,
ReadElectronicId2,
TakeMeasurementInit,
ReadRhMeasurement,
ReadTempMeasurement,
GotMeasurement,








    /// Disable I2C and release buffer
    Done,
}

pub struct SI7021<'a, A: alarm::Alarm + 'a> {
    i2c: &'a i2c::I2CDevice,
    alarm: &'a A,
    callback: Cell<Option<Callback>>,
    state: Cell<State>,
    buffer: TakeCell<&'static mut [u8]>
}

impl<'a, A: alarm::Alarm + 'a> SI7021<'a, A> {
    pub fn new(i2c: &'a i2c::I2CDevice, alarm: &'a A, buffer: &'static mut [u8]) -> SI7021<'a, A> {
    // pub fn new(i2c: &'a i2c::I2CDevice, alarm: &'a A, buffer: &'static mut [u8]) -> SI7021<'a> {
        // setup and return struct
        SI7021{
            i2c: i2c,
            alarm: alarm,
            callback: Cell::new(None),
            state: Cell::new(State::Idle),
            buffer: TakeCell::new(buffer)
        }
    }

    pub fn read_id(&self) {
        self.buffer.take().map(|buffer| {
            // turn on i2c to send commands
            self.i2c.enable();

            buffer[0] = Registers::ReadElectronicIdByteOneA as u8;
            buffer[1] = Registers::ReadElectronicIdByteOneB as u8;
            self.i2c.write(buffer, 2);
            self.state.set(State::SelectElectronicId1);
        });
    }

    pub fn take_measurement(&self) {

        // self.interrupt_pin.enable_input(gpio::InputMode::PullNone);
        // self.interrupt_pin.enable_interrupt(0, gpio::InterruptMode::RisingEdge);

        self.buffer.take().map(|buffer| {
            // turn on i2c to send commands
            self.i2c.enable();

            buffer[0] = Registers::MeasRelativeHumidityNoHoldMode as u8;
            self.i2c.write(buffer, 1);
            self.state.set(State::TakeMeasurementInit);
        });
    }
}

impl<'a, A: alarm::Alarm + 'a> i2c::I2CClient for SI7021<'a, A> {
    fn command_complete(&self, buffer: &'static mut [u8], _error: i2c::Error) {
        match self.state.get() {
            State::SelectElectronicId1 => {
                // buffer[0] = Registers::ReadElectronicIdByteOneA as u8;
                // buffer[1] = Registers::ReadElectronicIdByteOneB as u8;
                // self.i2c.write(buffer, 2);
                // self.state.set(State::SelectElectronicId1);

                self.i2c.read(buffer, 8);
                self.state.set(State::ReadElectronicId1);
            },
            State::ReadElectronicId1 => {
                buffer[6] = buffer[0];
                buffer[7] = buffer[1];
                buffer[8] = buffer[2];
                buffer[9] = buffer[3];
                buffer[10] = buffer[4];
                buffer[11] = buffer[5];
                buffer[12] = buffer[6];
                buffer[13] = buffer[7];
                buffer[0] = Registers::ReadElectronicIdByteTwoA as u8;
                buffer[1] = Registers::ReadElectronicIdByteTwoB as u8;
                self.i2c.write(buffer, 2);
                self.state.set(State::SelectElectronicId2);
                // self.buffer.replace(buffer);
                // self.i2c.disable();
                // self.state.set(State::Idle);
            },
            State::SelectElectronicId2 => {
                // buffer[0] = Registers::ReadElectronicIdByteOneA as u8;
                // buffer[1] = Registers::ReadElectronicIdByteOneB as u8;
                // self.i2c.write(buffer, 2);
                // self.state.set(State::SelectElectronicId1);

                self.i2c.read(buffer, 6);
                self.state.set(State::ReadElectronicId2);
            },
            State::ReadElectronicId2 => {
                self.buffer.replace(buffer);
                self.i2c.disable();
                self.state.set(State::Idle);
            },
            State::TakeMeasurementInit => {
                let now = self.alarm.now();
                let tics = self.alarm.now().wrapping_add(1000);
                self.alarm.set_alarm(tics);
                // self.state.set(State::TakeMeasurementWait);
                self.buffer.replace(buffer);
                self.i2c.disable();
                self.state.set(State::Idle);


                // let now = self.alarm.now();
                // let mut next_alarm = u32::max_value();
                // let mut next_dist = u32::max_value();
                // for timer in self.app_timer.iter() {
                //     timer.enter(|timer, _| {
                //         if timer.interval > 0 {
                //             let t_alarm = timer.t0.wrapping_add(timer.interval);
                //             let t_dist = t_alarm.wrapping_sub(now);
                //             if next_dist > t_dist {
                //                 next_alarm = t_alarm;
                //                 next_dist = t_dist;
                //             }
                //         }
                //     });
                // }
                // if next_alarm != u32::max_value() {
                //     self.alarm.set_alarm(next_alarm);
                // }



                // buffer[0] = Registers::PressOutXl as u8 | REGISTER_AUTO_INCREMENT;
                // self.i2c.write(buffer, 1);
                // self.state.set(State::TakeMeasurementClear);
            },
            // State::TakeMeasurementClear => {
            //     self.i2c.read(buffer, 3);
            //     self.state.set(State::TakeMeasurementConfigure);
            // },
            State::ReadRhMeasurement => {
                buffer[2] = buffer[0];
                buffer[3] = buffer[1];
                buffer[0] = Registers::ReadTemperaturePreviousRHMeasurement as u8;
                self.i2c.write(buffer, 1);
                self.state.set(State::ReadTempMeasurement);
            },
            State::ReadTempMeasurement => {
                self.i2c.read(buffer, 2);
                self.state.set(State::GotMeasurement);
            },
            State::GotMeasurement => {
                let pressure = (((buffer[2] as u32) << 16) | ((buffer[1] as u32) << 8) | (buffer[0] as u32)) as u32;

                // Returned as microbars
                let pressure_ubar = (pressure * 1000) / 4096;

                self.callback.get().map(|mut cb|
                    cb.schedule(pressure_ubar as usize, 0, 0)
                );

                // buffer[0] = Registers::CtrlReg1 as u8;
                // buffer[1] = 0;
                // self.i2c.write(buffer, 2);
                // self.state.set(State::Done);
                self.buffer.replace(buffer);
                self.i2c.disable();
                self.state.set(State::Idle);
            },
            State::Done => {
                self.buffer.replace(buffer);
                self.i2c.disable();
                self.state.set(State::Idle);
            },
            _ => {}
        }
    }
}

impl<'a, A: alarm::Alarm + 'a> alarm::AlarmClient for SI7021<'a, A> {
    fn fired(&self) {
        self.buffer.take().map(|buffer| {
            // turn on i2c to send commands
            self.i2c.enable();

            self.i2c.read(buffer, 2);
            self.state.set(State::ReadRhMeasurement);
        });
    }
}

impl<'a, A: alarm::Alarm + 'a> Driver for SI7021<'a, A> {
    fn subscribe(&self, subscribe_num: usize, callback: Callback) -> isize {
        match subscribe_num {
            // Set a callback
            0 => {
                // Set callback function
                self.callback.set(Some(callback));
                0
            },
            // default
            _ => -1
        }
    }

    fn command(&self, command_num: usize, _: usize, _: AppId) -> isize {
        match command_num {
            // Take a pressure measurement
            0 => {
                self.take_measurement();
                0
            },
            // default
            _ => -1
        }

    }
}
