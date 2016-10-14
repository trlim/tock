pub trait Client {
    fn sample_done(&self, sample: u16);
}

pub trait AdcSingle {
    fn initialize(&self) -> bool;
    fn sample(&self, channel: u8) -> bool;
    // fn cancel_sample(&self) -> bool;
}

// pub trait Frequency {
//    fn frequency() -> u32;
// }
//
// pub trait AdcContinuous {
//    type Frequency: Frequency;
//    fn compute_interval(&self, interval:u32) -> u32;
//    fn sample_continuous(&self, channel: u8, interval:u32);
//    fn cancel_sampling(&self);
// }
