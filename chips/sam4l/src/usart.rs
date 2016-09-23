/// USART SAM4L implementation

// rust language modules
use core::mem;
// local modules
use nvic;
use dma;
use pm;
// other modules
use kernel::hil;

// Register map for SAM4L USART
#[repr(C, packed)]
struct Registers {
    cr: u32,
    mr: u32,
    ier: u32,
    idr: u32,
    imr: u32,
    csr: u32,
    rhr: u32,
    thr: u32,
    brgr: u32, // 0x20
    rtor: u32,
    ttgr: u32,
    reserved0: [u32; 5],
    fidi: u32, // 0x40
    ner: u32,
    reserved1: u32,
    ifr: u32,
    man: u32,
    linmr: u32,
    linir: u32,
    linbrr: u32,
    wpmr: u32,
    wpsr: u32,
    version: u32,
}
const SIZE: usize = 0x4000;
const BASE_ADDRESS: usize = 0x40024000;

#[derive(Copy,Clone)]
pub enum Location {
    USART0,
    USART1,
    USART2,
    USART3,
}

pub struct USART {
    regs: *mut Registers,
    client: Option<&'static hil::uart::Client>,
    clock: pm::Clock,
    nvic: nvic::NvicIdx,
    rx_dma: Option<&'static mut dma::DMAChannel>,
    rx_dma_peripheral: dma::DMAPeripheral,
    tx_dma: Option<&'static mut dma::DMAChannel>,
    tx_dma_peripheral: dma::DMAPeripheral,
}

// USART hardware peripherals on SAM4L
pub static mut USART0: USART =
    USART::new(Location::USART0, pm::PBAClock::USART0, nvic::NvicIdx::USART0);
pub static mut USART1: USART =
    USART::new(Location::USART1, pm::PBAClock::USART1, nvic::NvicIdx::USART1);
pub static mut USART2: USART =
    USART::new(Location::USART2, pm::PBAClock::USART2, nvic::NvicIdx::USART2);
pub static mut USART3: USART =
    USART::new(Location::USART3, pm::PBAClock::USART3, nvic::NvicIdx::USART3);

/*
impl USART {
    const fn new(location: Location, clock: pm::PBAClock, nvic: nvic::NvicIdx) -> USART {
        USART {
            regs: (BASE_ADDRESS + (location as usize) * SIZE) as *mut Registers,
            clock: pm::Clock::PBA(clock),
            nvic: nvic,
            dma: None,
            dma_peripheral: dma::DMAPeripheral::USART0_RX, // Set to some default.
            // This is updated when a
            // real DMA is configured.
            client: None,
        }
    }

    pub fn set_client<C: hil::uart::Client>(&mut self, client: &'static C) {
        self.client = Some(client);
    }

    pub fn set_dma(&mut self, dma: &'static mut dma::DMAChannel, dma_peripheral: dma::DMAPeripheral) {
        self.dma = Some(dma);
        self.dma_peripheral = dma_peripheral;
    }

    fn set_baud_rate(&self, baud_rate: u32) {
        let cd = 48000000 / (8 * baud_rate);
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.brgr, cd);
    }

    fn set_mode(&self, mode: u32) {
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.mr, mode);
    }

    fn enable_clock(&self) {
        unsafe {
            pm::enable_clock(self.clock);
        }
    }

    fn enable_nvic(&self) {
        unsafe {
            nvic::enable(self.nvic);
        }
    }

    fn disable_nvic(&self) {
        unsafe {
            nvic::disable(self.nvic);
        }
    }

    pub fn enable_rx_interrupts(&self) {
        self.enable_nvic();
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.ier, 1 as u32);
    }

    pub fn enable_tx_interrupts(&mut self) {
        self.enable_nvic();
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.ier, 2 as u32);
    }

    pub fn disable_rx_interrupts(&mut self) {
        self.disable_nvic();
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.idr, 1 as u32);
    }

    pub fn handle_interrupt(&mut self) {
        //XXX: fuck this
        use kernel::hil::uart::UART;
        if self.rx_ready() {
            let regs: &Registers = unsafe { mem::transmute(self.regs) };
            let c = read_volatile(&regs.rhr) as u8;
            match self.client {
                Some(ref client) => client.read_done(c),
                None => {}
            }
        }
    }

    pub fn reset_rx(&mut self) {
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.cr, 1 << 2);
    }
}
*/

//XXX: Moved. Fix me
/*
pub struct USARTParams {
    // pub client: &'static Shared<hil::uart::Client>,
    pub baud_rate: u32,
    pub data_bits: u8,
    pub parity: Parity,
    pub mode: Mode,
}

impl Controller for USART {
    type Config = USARTParams;

    fn configure(&self, params: USARTParams) {
        //   self.client = Some(params.client.borrow_mut());
        let chrl = ((params.data_bits - 1) & 0x3) as u32;
        let mode =
            (params.mode as u32) /* mode */
            | 0 << 4 /*USCLKS*/
            | chrl << 6 /* Character Length */
            | (params.parity as u32) << 9 /* Parity */
            | 0 << 12 /* Number of stop bits = 1 */
            | 1 << 19 /* Oversample at 8 times baud rate */;

        self.enable_clock();
        self.set_baud_rate(params.baud_rate);
        self.set_mode(mode);
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.ttgr, 4);
        self.enable_rx_interrupts();
    }
}
*/

impl dma::DMAClient for USART {
    fn xfer_done(&mut self, pid: dma::DMAPeripheral) {
        // determine if it was an RX or TX transfer
        if pid == self.dma_rx_peripheral {

            // RX transfer was completed

            // disable RX and RX interrupts
            self.disable_rx_interrupts();
            self.disable_rx();

            // get buffer
            let buffer = match self.rx_dma.as_mut() {
                Some(rx_dma) => {
                    let buf = rx_dma.abort_xfer();
                    rx_dma.disable();
                    buf
                }
                None => None,
            };

            // alert client
            self.client.as_ref().map(move |c| {
                //XXX: how do I get the length of a DMA transaction?
                //  should this just be the length of the buffer?
                buffer.map(|buf| c.receive_complete(buf, ?LENOFBUFFER?, hil::uart::Error::CommandComplete));
            });

        } else if pid == self.dma_tx_peripheral {

            // TX transfer was completed

            // disable TX and TX interrupts
            self.disable_tx_interrupts();
            self.disable_tx();

            // get buffer
            let buffer = match self.tx_dma.as_mut() {
                Some(tx_dma) => {
                    let buf = tx_dma.abort_xfer();
                    tx_dma.disable();
                    buf
                }
                None => None,
            };

            // alert client
            self.client.as_ref().map(move |c| {
                buffer.map(|buf| c.transmit_complete(buf, hil::uart::Error::CommandComplete));
            });
        }
    }
}

/// Implementation of kernel::hil::UART
impl hil::uart::UART for USART {
    fn init (&mut self, params: hil::uart::UARTParams) {
        // set USART mode register
        let mode = 0;
        mode |= 0x1 << 19;  //OVER: oversample at 8 times baud rate
        mode |= 0x3 <<  6;  //CHRL: 8-bit characters
        mode |= 0x0 <<  4;  //USCLKS: select CLK_USART

        match params.stop_bits {
            hil::uart::StopBits::StopBits_1 => mode |= 0x0 << 12,   //NBSTOP: 1 stop bit
            hil::uart::StopBits::StopBits_2 => mode |= 0x2 << 12,   //NBSTOP: 2 stop bits
        };

        match params.parity {
            hil::uart::Parity::None => mode |= 0x4 << 9,    //PAR: no parity
            hil::uart::Parity::Odd  => mode |= 0x1 << 9,    //PAR: odd parity
            hil::uart::Parity::Even => mode |= 0x0 << 9,    //PAR: even parity
        };

        if (params.hw_flow_control) {
            mode |= 0x2 << 0;   //MODE: hardware handshaking
        } else {
            mode |= 0x0 << 0;   //MODE: normal
        }

        self.set_mode(mode);

        // set baud rate
        // NOTE: dependent on oversampling rate
        //XXX: how do you determine the current clock frequency?
        let clock_divider = CLK / (8 * params.parity);
        self.set_baud_rate_divider(clock_divider);

        // set transmitter timeguard
        //XXX: is this necessary
        self.set_tx_timeguard(4);

        // stop any TX and RX and clear status
        self.reset();

        // disable interrupts
        self.disable_interrupts();

        // enable USART clock
        self.enable_clock();
    }

    fn transmit (&self, tx_data: &'static [u8], tx_len: usize) {

        // enable TX and TX interrupts
        self.enable_tx();
        self.enable_tx_interrupts();

        // set up dma transfer and start transmission
        self.tx_dma.as_ref().map(move |dma| {
            dma.enable();
            dma.do_xfer(self.tx_dma_peripheral, tx_data, tx_len);
        });
    }

    fn receive_buffer (&self, rx_buffer: &'static mut [u8]) {

        // enable RX and RX interrupts
        self.enable_rx();
        self.enable_rx_interrupts();

        // set up dma transfer and start reception
        self.rx_dma.as_ref().map(move |dma| {
            dma.enable();
            dma.do_xfer(self.rx_dma_peripheral, rx_buffer, rx_buffer.len());
        });
    }

    fn receive_until_finished (&self, rx_buffer: &'static mut [u8], timeout: u8) {

        // enable receive timeout
        self.enable_rx_timeout(timeout);

        // enable RX and RX interrupts
        self.enable_rx();
        self.enable_rx_interrupts();

        // set up dma transfer and start reception
        self.rx_dma.as_ref().map(move |dma| {
            dma.enable();
            dma.do_xfer(self.rx_dma_peripheral, rx_buffer, rx_buffer.len());
        });
    }

    fn receive_until_terminator (&self, rx_buffer: &'static mut [u8], terminator: u8) {

        // enable receive terminator
        self.enable_receive_terminator(terminator);

        // enable RX and RX interrupts
        self.enable_rx();
        self.enable_rx_interrupts();

        // set up dma transfer and start reception
        self.rx_dma.as_ref().map(move |dma| {
            dma.enable();
            dma.do_xfer(self.rx_dma_peripheral, rx_buffer, rx_buffer.len());
        });
    }
}

/*
impl hil::uart::UART for USART {
    fn init(&mut self, params: hil::uart::UARTParams) {
        let chrl = ((params.data_bits - 1) & 0x3) as u32;
        let mode =
            (params.mode as u32) /* mode */
            | 0 << 4 /*USCLKS*/
            | chrl << 6 /* Character Length */
            | (params.parity as u32) << 9 /* Parity */
            | 0 << 12 /* Number of stop bits = 1 */
            | 1 << 19 /* Oversample at 8 times baud rate */;

        self.enable_clock();
        self.set_baud_rate(params.baud_rate);
        self.set_mode(mode);
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.ttgr, 4);
    }

    fn send_byte(&self, byte: u8) {
        while !self.tx_ready() {}
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.thr, byte as u32);
    }

    fn send_bytes(&self, bytes: &'static mut [u8], len: usize) {
        self.dma.as_ref().map(move |dma| {
            dma.enable();
            dma.do_xfer(self.dma_peripheral, bytes, len);
        });
    }

    fn rx_ready(&self) -> bool {
        let regs: &Registers = unsafe { mem::transmute(self.regs) };
        read_volatile(&regs.csr) & 0b1 != 0
    }

    fn tx_ready(&self) -> bool {
        let regs: &Registers = unsafe { mem::transmute(self.regs) };
        read_volatile(&regs.csr) & 0b10 != 0
    }


    fn read_byte(&self) -> u8 {
        while !self.rx_ready() {}
        let regs: &Registers = unsafe { mem::transmute(self.regs) };
        read_volatile(&regs.rhr) as u8
    }

    fn enable_rx(&self) {
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.cr, 1 << 4);
    }

    fn disable_rx(&mut self) {
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.cr, 1 << 5);
    }

    fn enable_tx(&self) {
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.cr, 1 << 6);
    }

    fn disable_tx(&mut self) {
        let regs: &mut Registers = unsafe { mem::transmute(self.regs) };
        write_volatile(&mut regs.cr, 1 << 7);
    }
}
*/

// Register interrupt handlers
interrupt_handler!(usart0_handler, USART0);
interrupt_handler!(usart1_handler, USART1);
interrupt_handler!(usart2_handler, USART2);
interrupt_handler!(usart3_handler, USART3);
