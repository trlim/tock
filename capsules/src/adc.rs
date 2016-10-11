use core::cell::Cell;
use kernel::{AppId, Callback, Driver};
use kernel::hil::adc::{Client, AdcSingle};

pub struct ADC<'a> {
    adc: &'a AdcSingle,
    channel: Cell<u8>,
    callback: Cell<Option<Callback>>,
}

impl<'a> ADC<'a> {
    pub fn new(adc: &'a AdcSingle) -> ADC<'a> {
        ADC {
            adc: adc,
            channel: Cell::new(0),
            callback: Cell::new(None),
        }
    }

    fn initialize(&self) {
        self.adc.initialize();
    }

    fn sample(&self, channel: u8) {
        self.channel.set(channel);
        self.adc.sample(channel);
    }
}

impl<'a> Client for ADC<'a> {
    fn sample_done(&self, sample: u16) {
        if self.callback.get().is_some() {
            self.callback.get().unwrap().schedule(0, self.channel.get() as usize, sample as usize);
        }
    }
}

impl<'a> Driver for ADC<'a> {
    fn subscribe(&self, subscribe_num: usize, callback: Callback) -> isize {
        match subscribe_num {
            // subscribe to ADC sample done
            0 => {
                self.callback.set(Some(callback));
                0
            }

            // default
            _ => -1,
        }
    }

    fn command(&self, command_num: usize, data: usize, _: AppId) -> isize {
        match command_num {
            // Initialize ADC
            0 => {
                self.initialize();
                0
            }
            // Sample on channel
            1 => {
                self.sample(data as u8);
                0
            }

            //default
            _ => -1
        }
    }
}
