/// USART SAM4L implementation

// rust language modules
use core::mem;
// local modules
use nvic;
use dma;
use pm;
// other modules
use kernel::hil;
use kernel::common::take_cell::TakeCell;
use kernel::common::volatile_cell::VolatileCell;

// Register map for SAM4L USART
#[repr(C, packed)]
struct USARTRegisters {
    cr:         VolatileCell<u32>, // 0x00
    mr:         VolatileCell<u32>,
    ier:        VolatileCell<u32>,
    idr:        VolatileCell<u32>,
    imr:        VolatileCell<u32>,
    csr:        VolatileCell<u32>,
    rhr:        VolatileCell<u32>,
    thr:        VolatileCell<u32>,
    brgr:       VolatileCell<u32>,
    rtor:       VolatileCell<u32>,
    ttgr:       VolatileCell<u32>, // 0x28
    _reserved0: [VolatileCell<u32>; 5],
    fidi:       VolatileCell<u32>, // 0x40
    ner:        VolatileCell<u32>,
    _reserved1: VolatileCell<u32>,
    ifr:        VolatileCell<u32>,
    man:        VolatileCell<u32>,
    linmr:      VolatileCell<u32>,
    linir:      VolatileCell<u32>,
    linbrr:     VolatileCell<u32>, // 0x5C
    _reserved2: [VolatileCell<u32>; 33],
    wpmr:       VolatileCell<u32>, // 0xE4
    wpsr:       VolatileCell<u32>,
    _reserved3: [VolatileCell<u32>; 4],
    version:    VolatileCell<u32>, // 0xFC
}

const USART_BASE_ADDRS: [*mut USARTRegisters; 4] = [
    0x40024000 as *mut USARTRegisters,
    0x40028000 as *mut USARTRegisters,
    0x4002C000 as *mut USARTRegisters,
    0x40030000 as *mut USARTRegisters,
];

pub struct USART {
    registers: *mut USARTRegisters,
    clock: pm::Clock,
    nvic: nvic::NvicIdx,
    rx_dma: TakeCell<&'static dma::DMAChannel>,
    rx_dma_peripheral: dma::DMAPeripheral,
    tx_dma: TakeCell<&'static dma::DMAChannel>,
    tx_dma_peripheral: dma::DMAPeripheral,
    client: TakeCell<&'static hil::uart::Client>,
}

// USART hardware peripherals on SAM4L
pub static mut USART0: USART =
    USART::new(USART_BASE_ADDRS[0], pm::PBAClock::USART0, nvic::NvicIdx::USART0, dma::DMAPeripheral::USART0_RX, dma::DMAPeripheral::USART0_TX);
pub static mut USART1: USART =
    USART::new(USART_BASE_ADDRS[1], pm::PBAClock::USART1, nvic::NvicIdx::USART1, dma::DMAPeripheral::USART1_RX, dma::DMAPeripheral::USART1_TX);
pub static mut USART2: USART =
    USART::new(USART_BASE_ADDRS[2], pm::PBAClock::USART2, nvic::NvicIdx::USART2, dma::DMAPeripheral::USART2_RX, dma::DMAPeripheral::USART2_TX);
pub static mut USART3: USART =
    USART::new(USART_BASE_ADDRS[3], pm::PBAClock::USART3, nvic::NvicIdx::USART3, dma::DMAPeripheral::USART3_RX, dma::DMAPeripheral::USART3_TX);

impl USART {
    const fn new (base_addr: *mut USARTRegisters, clock: pm::PBAClock, nvic: nvic::NvicIdx,
                  rx_dma_peripheral: dma::DMAPeripheral, tx_dma_peripheral: dma::DMAPeripheral) -> USART {
        USART {
            registers: base_addr,
            clock: pm::Clock::PBA(clock),
            nvic: nvic,

            // these get defined later by `chip.rs`
            rx_dma: TakeCell::empty(),
            rx_dma_peripheral: rx_dma_peripheral,
            tx_dma: TakeCell::empty(),
            tx_dma_peripheral: tx_dma_peripheral,

            // this gets defined later by `main.rs`
            client: TakeCell::empty(),
        }
    }

    pub fn set_client (&self, client: &'static hil::uart::Client) {
        self.client.replace(client);
    }

    pub fn set_dma (&self, rx_dma: &'static dma::DMAChannel, tx_dma: &'static dma::DMAChannel) {
        self.rx_dma.replace(rx_dma);
        self.tx_dma.replace(tx_dma);
    }

    pub fn enable_rx (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let cr_val = 0x0000 |
            (1 << 4); // RXEN
        regs.cr.set(cr_val);

        //XXX: enable/disable clock when necessary
    }

    pub fn enable_tx (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let cr_val = 0x0000 |
            (1 << 6); // TXEN
        regs.cr.set(cr_val);

        //XXX: enable/disable clock when necessary
    }

    pub fn disable_rx (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let cr_val = 0x0000 |
            (1 << 5); // RXDIS
        regs.cr.set(cr_val);

        //XXX: enable/disable clock when necessary
    }

    pub fn disable_tx (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let cr_val = 0x0000 |
            (1 << 7); // TXDIS
        regs.cr.set(cr_val);

        //XXX: enable/disable clock when necessary
    }

    pub fn abort_rx (&self) {
        //XXX: What's the best way to do this? Don't want a ton of waste
    }

    pub fn abort_tx (&self) {
        //XXX: What's the best way to do this? Don't want a ton of waste
    }

    pub fn enable_rx_interrupts (&self) {
        self.enable_nvic();
    }

    pub fn enable_tx_interrupts (&self) {
        self.enable_nvic();
    }

    pub fn disable_rx_interrupts (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let idr_val = 0x0000 |
            (1 << 12) | // RXBUFF
            (1 <<  8) | // TIMEOUT
            (1 <<  7) | // PARE
            (1 <<  6) | // FRAME
            (1 <<  5) | // OVRE
            (1 << 1);   // RXRDY
        regs.idr.set(idr_val);

        //XXX: disable nvic if no interrupts are enabled
    }

    pub fn disable_tx_interrupts (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let idr_val = 0x0000 |
            (1 << 9) | // TXEMPTY
            (1 << 1);  // TXREADY
        regs.idr.set(idr_val);

        // disable nvic if no interrupts are enabled
    }

    pub fn disable_interrupts (&self) {
        self.disable_nvic();
        self.disable_rx_interrupts();
        self.disable_tx_interrupts();
    }

    pub fn reset (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};

        // reset status bits, transmitter, and receiver
        let cr_val = 0x0000 |
            (1 << 8) | // RSTSTA
            (1 << 3) | // RSTTX
            (1 <<2);   // RSTRX
        regs.cr.set(cr_val);
    }

    pub fn handle_interrupt (&self) {
    }

    fn enable_clock (&self) {
        unsafe {
            pm::enable_clock(self.clock);
        }
    }

    fn enable_nvic (&self) {
        unsafe {
            nvic::enable(self.nvic);
        }
    }

    fn disable_nvic (&self) {
        unsafe {
            nvic::disable(self.nvic);
        }
    }

    fn set_mode (&self, mode: u32) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        regs.mr.set(mode);
    }

    fn set_baud_rate_divider (&self, clock_divider: u16) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let brgr_val: u32 = 0x0000 | clock_divider as u32;
        regs.brgr.set(brgr_val);
    }

    fn set_tx_timeguard (&self, timeguard: u8) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let ttgr_val: u32 = 0x0000 | timeguard as u32;
        regs.ttgr.set(ttgr_val);
    }

    fn enable_rx_timeout (&self, timeout: u8) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        let rtor_val: u32 = 0x0000 | timeout as u32;
        regs.rtor.set(rtor_val);
    }

    fn disable_rx_timeout (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        regs.rtor.set(0);
    }

    fn enable_rx_terminator (&self, terminator: u8) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        //XXX: what to do here
        panic!("didn't write terminator stuff yet");
    }

    fn disable_rx_terminator (&self) {
        let regs: &mut USARTRegisters = unsafe {mem::transmute(self.registers)};
        //XXX: what to do here
        panic!("didn't write terminator stuff yet");
    }
}

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

//XXX: Moved. Delete me altogether
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
    fn xfer_done (&self, pid: dma::DMAPeripheral) {
        // determine if it was an RX or TX transfer
        if pid == self.rx_dma_peripheral {

            // RX transfer was completed

            // disable RX and RX interrupts
            self.disable_rx_interrupts();
            self.disable_rx();

            // get buffer
            let buffer = self.rx_dma.map_or(None, |rx_dma| {
                let buf = rx_dma.abort_xfer();
                rx_dma.disable();
                buf
            });

            // alert client
            self.client.map(|c| {
                //XXX: how do I get the length of a DMA transaction?
                //  should this just be the length of the buffer?
                buffer.map(|buf| {
                    let length = buf.len();
                    c.receive_complete(buf, length, hil::uart::Error::CommandComplete);
                });
            });

        } else if pid == self.tx_dma_peripheral {

            // TX transfer was completed

            // disable TX and TX interrupts
            self.disable_tx_interrupts();
            self.disable_tx();

            // get buffer
            let buffer = self.tx_dma.map_or(None, |tx_dma| {
                let buf = tx_dma.abort_xfer();
                tx_dma.disable();
                buf
            });

            // alert client
            self.client.map(|c| {
                buffer.map(|buf| c.transmit_complete(buf, hil::uart::Error::CommandComplete));
            });
        }
    }
}

/// Implementation of kernel::hil::UART
impl hil::uart::UART for USART {
    fn init (&self, params: hil::uart::UARTParams) {
        // set USART mode register
        let mut mode = 0;
        mode |= 0x1 << 19;  //OVER: oversample at 8 times baud rate
        mode |= 0x3 <<  6;  //CHRL: 8-bit characters
        mode |= 0x0 <<  4;  //USCLKS: select CLK_USART

        match params.stop_bits {
            hil::uart::StopBits::StopBits1 => mode |= 0x0 << 12,   //NBSTOP: 1 stop bit
            hil::uart::StopBits::StopBits2 => mode |= 0x2 << 12,   //NBSTOP: 2 stop bits
        };

        match params.parity {
            hil::uart::Parity::None => mode |= 0x4 << 9,    //PAR: no parity
            hil::uart::Parity::Odd  => mode |= 0x1 << 9,    //PAR: odd parity
            hil::uart::Parity::Even => mode |= 0x0 << 9,    //PAR: even parity
        };

        if params.hw_flow_control {
            mode |= 0x2 << 0;   //MODE: hardware handshaking
        } else {
            mode |= 0x0 << 0;   //MODE: normal
        }

        self.set_mode(mode);

        // set baud rate
        // NOTE: dependent on oversampling rate
        //XXX: how do you determine the current clock frequency?
        let clock_divider = 48000000 / (8 * params.baud_rate);
        self.set_baud_rate_divider(clock_divider as u16);

        // set transmitter timeguard
        //XXX: is this necessary
        self.set_tx_timeguard(4);

        // stop any TX and RX and clear status
        self.reset();

        // disable interrupts
        self.disable_interrupts();

        // enable USART clock
        //XXX: should we instead only enable clock when in use?
        self.enable_clock();
    }

    fn transmit (&self, tx_data: &'static mut [u8], tx_len: usize) {

        // quit current transmission if any
        self.abort_tx();

        // enable TX
        self.enable_tx();

        // set up dma transfer and start transmission
        self.tx_dma.map(move |dma| {
            dma.enable();
            dma.do_xfer(self.tx_dma_peripheral, tx_data, tx_len);
        });
    }

    fn receive_buffer (&self, rx_buffer: &'static mut [u8]) {

        // quit current reception if any
        self.abort_rx();

        // enable RX
        self.enable_rx();

        // set up dma transfer and start reception
        self.rx_dma.map(move |dma| {
            dma.enable();
            let length = rx_buffer.len();
            dma.do_xfer(self.rx_dma_peripheral, rx_buffer, length);
        });
    }

    fn receive_until_finished (&self, rx_buffer: &'static mut [u8], timeout: u8) {

        // quit current reception if any
        self.abort_rx();

        // enable receive timeout
        self.enable_rx_timeout(timeout);

        // enable RX
        self.enable_rx();

        // set up dma transfer and start reception
        self.rx_dma.map(move |dma| {
            dma.enable();
            let length = rx_buffer.len();
            dma.do_xfer(self.rx_dma_peripheral, rx_buffer, length);
        });
    }

    fn receive_until_terminator (&self, rx_buffer: &'static mut [u8], terminator: u8) {

        // quit current reception if any
        self.abort_rx();

        // enable receive terminator
        self.enable_rx_terminator(terminator);

        // enable RX
        self.enable_rx();

        // set up dma transfer and start reception
        self.rx_dma.map(move |dma| {
            dma.enable();
            let length = rx_buffer.len();
            dma.do_xfer(self.rx_dma_peripheral, rx_buffer, length);
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
