use crate::atomic_mutex::AtomicMutex;
use bare_metal::CriticalSection;
use core::mem;
use core::slice;
use cortex_m::interrupt;
use stm32f3xx_hal::stm32::USB_FS;
use usb_device::endpoint::EndpointType;
use usb_device::{Result, UsbError};
use vcell::VolatileCell;

type EndpointBuffer = &'static mut [VolatileCell<u32>];

pub const NUM_ENDPOINTS: usize = 8;

#[repr(C)]
struct BufferDescriptor {
    pub addr_tx: VolatileCell<usize>,
    pub count_tx: VolatileCell<usize>,
    pub addr_rx: VolatileCell<usize>,
    pub count_rx: VolatileCell<usize>,
}

pub fn calculate_count_rx(mut size: usize) -> Result<(usize, u16)> {
    if size <= 62 {
        // Buffer size is in units of 2 bytes, 0 = 0 bytes
        size = (size + 1) & !0x01;

        let size_bits = size >> 1;

        Ok((size, (size_bits << 10) as u16))
    } else if size <= 1024 {
        // Buffer size is in units of 32 bytes, 0 = 32 bytes
        size = (size + 31) & !0x1f;

        let size_bits = (size >> 5) - 1;

        Ok((size, (0x8000 | (size_bits << 10)) as u16))
    } else {
        Err(UsbError::EndpointMemoryOverflow)
    }
}

pub trait GenericEndpoint {
    fn ep_type(&self) -> Option<EndpointType>;
    fn set_ep_type(&mut self, ep_type: EndpointType);
    fn is_out_buf_set(&self) -> bool;
    fn set_out_buf(&mut self, addr: usize, size_and_bits: (usize, u16));
    fn is_in_buf_set(&self) -> bool;
    fn set_in_buf(&mut self, addr: usize, max_packet_size: usize);
    fn configure(&self, cs: &CriticalSection);
    fn write(&self, buf: &[u8]) -> Result<usize>;
    fn read(&self, buf: &mut [u8]) -> Result<usize>;
    fn clear_ctr_rx(&self, _cs: &CriticalSection);
    fn clear_ctr_tx(&self, _cs: &CriticalSection);
    fn set_stat_rx(&self, _cs: &CriticalSection, status: EndpointStatus);
    fn set_stat_tx(&self, _cs: &CriticalSection, status: EndpointStatus);
    fn stat_tx_bits(&self) -> u8;
    fn stat_rx_bits(&self) -> u8;
    fn ctr_tx_bit_is_set(&self) -> bool;
    fn ctr_rx_bit_is_set(&self) -> bool;
    fn setup_bit_is_set(&self) -> bool;
}

macro_rules! endpoints {
    [ $( [ $Endpoint:ident, $USB_EPR:ident, $usb_epr:ident ] ),* ] => {
        $(
            use stm32f3xx_hal::stm32::usb_fs::{$USB_EPR, $usb_epr};

            /// Arbitrates access to the endpoint-specific registers and packet buffer memory.
            #[derive(Default)]
            pub struct $Endpoint {
                out_buf: Option<AtomicMutex<EndpointBuffer>>,
                in_buf: Option<AtomicMutex<EndpointBuffer>>,
                ep_type: Option<EndpointType>,
                index: u8,
            }

            impl $Endpoint {
                const MEM_ADDR: *mut VolatileCell<u32> = 0x4000_6000 as *mut VolatileCell<u32>;

                pub fn new(index: u8) -> $Endpoint {
                    $Endpoint {
                        out_buf: None,
                        in_buf: None,
                        ep_type: None,
                        index,
                    }
                }

                fn make_buf(addr: usize, size: usize)
                    -> Option<AtomicMutex<&'static mut [VolatileCell<u32>]>>
                {
                    Some(AtomicMutex::new(
                        unsafe {
                            slice::from_raw_parts_mut(
                                Self::MEM_ADDR.offset((addr >> 1) as isize),
                                size >> 1)
                        }
                    ))
                }

                fn descr(&self) -> &'static BufferDescriptor {
                    unsafe { &*(Self::MEM_ADDR as *const BufferDescriptor).offset(self.index as isize) }
                }

                fn reg(&self) -> &'static $USB_EPR {
                    unsafe { &(*USB_FS::ptr()).$usb_epr }
                }

                fn write_mem(&self, mem: &[VolatileCell<u32>], mut buf: &[u8]) {
                    let mut addr = 0;

                    while buf.len() >= 2 {
                        mem[addr].set((buf[0] as u16 | ((buf[1] as u16) << 8)) as u32);
                        addr += 1;

                        buf = &buf[2..];
                    }

                    if buf.len() > 0 {
                        mem[addr].set(buf[0] as u32);
                    }
                }

                fn read_mem(&self, mem: &[VolatileCell<u32>], mut buf: &mut [u8]) {
                    let mut addr = 0;

                    while buf.len() >= 2 {
                        let word = mem[addr].get();

                        buf[0] = word as u8;
                        buf[1] = (word >> 8) as u8;

                        addr += 1;

                        buf = &mut {buf}[2..];
                    }

                    if buf.len() > 0 {
                        buf[0] = mem[addr].get() as u8;
                    }
                }

                fn clear_toggle_bits(w: &mut $usb_epr::W) -> &mut $usb_epr::W {
                    unsafe {
                        w
                            .dtog_rx().clear_bit()
                            .dtog_tx().clear_bit()
                            .stat_rx().bits(0)
                            .stat_tx().bits(0)
                    }
                }
            }

            impl GenericEndpoint for $Endpoint {
                fn ep_type(&self) -> Option<EndpointType> {
                    self.ep_type
                }

                fn set_ep_type(&mut self, ep_type: EndpointType) {
                    self.ep_type = Some(ep_type);
                }

                fn is_out_buf_set(&self) -> bool {
                    self.out_buf.is_some()
                }

                fn set_out_buf(&mut self, addr: usize, size_and_bits: (usize, u16)) {
                    self.out_buf = Self::make_buf(addr, size_and_bits.0);

                    let descr = self.descr();
                    descr.addr_rx.set(addr);
                    descr.count_rx.set(size_and_bits.1 as usize);
                }

                fn is_in_buf_set(&self) -> bool {
                    self.in_buf.is_some()
                }

                fn set_in_buf(&mut self, addr: usize, max_packet_size: usize) {
                    self.in_buf = Self::make_buf(addr, max_packet_size);

                    let descr = self.descr();
                    descr.addr_tx.set(addr);
                    descr.count_tx.set(0);
                }

                fn configure(&self, cs: &CriticalSection) {
                    let ep_type = match self.ep_type {
                        Some(t) => t,
                        None => { return },
                    };

                    self.reg().modify(|_, w| unsafe  {
                            Self::clear_toggle_bits(w)
                                .ctr_rx().clear_bit()
                                // dtog_rx
                                // stat_rx
                                .ep_type().bits(ep_type.bits())
                                .ep_kind().clear_bit()
                                .ctr_tx().clear_bit()
                                // dtog_rx
                                // stat_tx
                                .ea().bits(self.index)
                    });

                    if self.out_buf.is_some() {
                        self.set_stat_rx(cs, EndpointStatus::Valid);
                    }

                    if self.in_buf.is_some() {
                        self.set_stat_tx(cs, EndpointStatus::Nak);
                    }
                }

                fn write(&self, buf: &[u8]) -> Result<usize> {
                    let guard = self.in_buf.as_ref().unwrap().try_lock();

                    let in_buf = match guard {
                        Some(ref b) => b,
                        None => { return Err(UsbError::WouldBlock); }
                    };

                    if buf.len() > in_buf.len() << 1 {
                        return Err(UsbError::BufferOverflow);
                    }

                    let reg = self.reg();

                    match reg.read().stat_tx().bits().into() {
                        EndpointStatus::Valid | EndpointStatus::Disabled => return Err(UsbError::WouldBlock),
                        _ => {},
                    };

                    self.write_mem(in_buf, buf);
                    self.descr().count_tx.set(buf.len());

                    interrupt::free(|cs| {
                        self.set_stat_tx(cs, EndpointStatus::Valid);
                    });

                    Ok(buf.len())
                }

                fn read(&self, buf: &mut [u8]) -> Result<usize> {
                    let guard = self.out_buf.as_ref().unwrap().try_lock();

                    let out_buf = match guard {
                        Some(ref b) => b,
                        None => { return Err(UsbError::WouldBlock); }
                    };

                    let reg = self.reg();
                    let reg_v = reg.read();

                    let status: EndpointStatus = reg_v.stat_rx().bits().into();

                    if status == EndpointStatus::Disabled || !reg_v.ctr_rx().bit_is_set() {
                        return Err(UsbError::WouldBlock);
                    }

                    let count = self.descr().count_rx.get() & 0x3ff;
                    if count > buf.len() {
                        return Err(UsbError::BufferOverflow);
                    }

                    self.read_mem(out_buf, &mut buf[0..count]);

                    interrupt::free(|cs| {
                        self.clear_ctr_rx(cs);
                        self.set_stat_rx(cs, EndpointStatus::Valid);
                    });

                    Ok(count)
                }

                fn clear_ctr_rx(&self, _cs: &CriticalSection) {
                    self.reg().modify(|_, w| Self::clear_toggle_bits(w).ctr_rx().clear_bit());
                }

                fn clear_ctr_tx(&self, _cs: &CriticalSection) {
                    self.reg().modify(|_, w| Self::clear_toggle_bits(w).ctr_tx().clear_bit());
                }

                fn set_stat_rx(&self, _cs: &CriticalSection, status: EndpointStatus) {
                    self.reg().modify(|r, w| unsafe {
                        Self::clear_toggle_bits(w)
                            .stat_rx().bits(r.stat_rx().bits() ^ (status as u8))
                    });
                }

                fn set_stat_tx(&self, _cs: &CriticalSection, status: EndpointStatus) {
                    self.reg().modify(|r, w| unsafe {
                        Self::clear_toggle_bits(w)
                            .stat_tx().bits(r.stat_tx().bits() ^ (status as u8))
                    });
                }

                fn stat_tx_bits(&self) -> u8 {
                    self.reg().read().stat_tx().bits()
                }

                fn stat_rx_bits(&self) -> u8 {
                    self.reg().read().stat_rx().bits()
                }

                fn ctr_tx_bit_is_set(&self) -> bool {
                    self.reg().read().ctr_tx().bit_is_set()
                }

                fn ctr_rx_bit_is_set(&self) -> bool {
                    self.reg().read().ctr_rx().bit_is_set()
                }

                fn setup_bit_is_set(&self) -> bool {
                    self.reg().read().setup().bit_is_set()
                }
            }
        )*
    };
}

endpoints![
    [Endpoint0, USB_EP0R, usb_ep0r],
    [Endpoint1, USB_EP1R, usb_ep1r],
    [Endpoint2, USB_EP2R, usb_ep2r],
    [Endpoint3, USB_EP3R, usb_ep3r],
    [Endpoint4, USB_EP4R, usb_ep4r],
    [Endpoint5, USB_EP5R, usb_ep5r],
    [Endpoint6, USB_EP6R, usb_ep6r],
    [Endpoint7, USB_EP7R, usb_ep7r]
];

pub enum Endpoint {
    Endpoint0(Endpoint0),
    Endpoint1(Endpoint1),
    Endpoint2(Endpoint2),
    Endpoint3(Endpoint3),
    Endpoint4(Endpoint4),
    Endpoint5(Endpoint5),
    Endpoint6(Endpoint6),
    Endpoint7(Endpoint7),
}

macro_rules! inner {
    ( $self:expr, $e:ident, $opr:expr ) => {{
        match $self {
            Endpoint::Endpoint0($e) => $opr,
            Endpoint::Endpoint1($e) => $opr,
            Endpoint::Endpoint2($e) => $opr,
            Endpoint::Endpoint3($e) => $opr,
            Endpoint::Endpoint4($e) => $opr,
            Endpoint::Endpoint5($e) => $opr,
            Endpoint::Endpoint6($e) => $opr,
            Endpoint::Endpoint7($e) => $opr,
        }
    }};
}

impl GenericEndpoint for Endpoint {
    fn ep_type(&self) -> Option<EndpointType> {
        inner!(self, e, e.ep_type())
    }
    fn set_ep_type(&mut self, ep_type: EndpointType) {
        inner!(self, e, e.set_ep_type(ep_type))
    }
    fn is_out_buf_set(&self) -> bool {
        inner!(self, e, e.is_out_buf_set())
    }
    fn set_out_buf(&mut self, addr: usize, size_and_bits: (usize, u16)) {
        inner!(self, e, e.set_out_buf(addr, size_and_bits))
    }
    fn is_in_buf_set(&self) -> bool {
        inner!(self, e, e.is_in_buf_set())
    }
    fn set_in_buf(&mut self, addr: usize, max_packet_size: usize) {
        inner!(self, e, e.set_in_buf(addr, max_packet_size))
    }
    fn configure(&self, cs: &CriticalSection) {
        inner!(self, e, e.configure(cs))
    }
    fn write(&self, buf: &[u8]) -> Result<usize> {
        inner!(self, e, e.write(buf))
    }
    fn read(&self, buf: &mut [u8]) -> Result<usize> {
        inner!(self, e, e.read(buf))
    }
    fn clear_ctr_rx(&self, _cs: &CriticalSection) {
        inner!(self, e, e.clear_ctr_rx(_cs))
    }
    fn clear_ctr_tx(&self, _cs: &CriticalSection) {
        inner!(self, e, e.clear_ctr_tx(_cs))
    }
    fn set_stat_rx(&self, _cs: &CriticalSection, status: EndpointStatus) {
        inner!(self, e, e.set_stat_rx(_cs, status))
    }
    fn set_stat_tx(&self, _cs: &CriticalSection, status: EndpointStatus) {
        inner!(self, e, e.set_stat_tx(_cs, status))
    }
    fn stat_tx_bits(&self) -> u8 {
        inner!(self, e, e.stat_tx_bits())
    }
    fn stat_rx_bits(&self) -> u8 {
        inner!(self, e, e.stat_rx_bits())
    }
    fn ctr_tx_bit_is_set(&self) -> bool {
        inner!(self, e, e.ctr_tx_bit_is_set())
    }
    fn ctr_rx_bit_is_set(&self) -> bool {
        inner!(self, e, e.ctr_rx_bit_is_set())
    }
    fn setup_bit_is_set(&self) -> bool {
        inner!(self, e, e.setup_bit_is_set())
    }
}

impl Endpoint {
    pub const MEM_START: usize = mem::size_of::<BufferDescriptor>() * NUM_ENDPOINTS;
    pub const MEM_SIZE: usize = 512;

    pub fn new(index: u8) -> Endpoint {
        match index as usize {
            0 => Endpoint::Endpoint0(Endpoint0::new(index)),
            1 => Endpoint::Endpoint1(Endpoint1::new(index)),
            2 => Endpoint::Endpoint2(Endpoint2::new(index)),
            3 => Endpoint::Endpoint3(Endpoint3::new(index)),
            4 => Endpoint::Endpoint4(Endpoint4::new(index)),
            5 => Endpoint::Endpoint5(Endpoint5::new(index)),
            6 => Endpoint::Endpoint6(Endpoint6::new(index)),
            7 => Endpoint::Endpoint7(Endpoint7::new(index)),
            _ => unreachable!("non-existing endpoint"),
        }
    }
}

trait EndpointTypeExt {
    fn bits(self) -> u8;
}

impl EndpointTypeExt for EndpointType {
    fn bits(self) -> u8 {
        const BITS: [u8; 4] = [0b01, 0b10, 0b00, 0b11];
        return BITS[self as usize];
    }
}

#[repr(u8)]
#[derive(PartialEq, Eq, Debug)]
#[allow(unused)]
pub enum EndpointStatus {
    Disabled = 0b00,
    Stall = 0b01,
    Nak = 0b10,
    Valid = 0b11,
}

impl From<u8> for EndpointStatus {
    fn from(v: u8) -> EndpointStatus {
        if v <= 0b11 {
            unsafe { mem::transmute(v) }
        } else {
            EndpointStatus::Disabled
        }
    }
}
