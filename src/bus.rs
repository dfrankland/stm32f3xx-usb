use crate::{
    atomic_mutex::AtomicMutex,
    endpoint::{calculate_count_rx, Endpoint, EndpointStatus, GenericEndpoint, NUM_ENDPOINTS},
};
use bare_metal::Mutex;
use core::{cell::RefCell, mem};
use cortex_m::{asm::delay, interrupt};
use stm32f3xx_hal::{
    gpio::{gpioa, Output, PushPull},
    prelude::*,
    rcc,
    stm32::{RCC, USB_FS},
};
use usb_device::{
    bus::{PollResult, UsbBusAllocator},
    endpoint::{EndpointAddress, EndpointType},
    Result, UsbDirection, UsbError,
};

struct Reset {
    delay: u32,
    pin: Mutex<RefCell<gpioa::PA12<Output<PushPull>>>>,
}

/// USB peripheral driver for STM32F3xx microcontrollers.
pub struct UsbBus {
    regs: AtomicMutex<USB_FS>,
    endpoints: [Endpoint; NUM_ENDPOINTS],
    next_ep_mem: usize,
    max_endpoint: usize,
    reset: Option<Reset>,
}

impl UsbBus {
    fn new(regs: USB_FS, apb1: &mut rcc::APB1, reset: Option<Reset>) -> UsbBusAllocator<Self> {
        // TODO: apb1.enr is not public, figure out how this should really interact with the HAL
        // crate

        let _ = apb1;
        interrupt::free(|_| {
            unsafe { (&*RCC::ptr()) }
                .apb1enr
                .modify(|_, w| w.usben().set_bit());
        });

        let bus = UsbBus {
            regs: AtomicMutex::new(regs),
            next_ep_mem: Endpoint::MEM_START,
            max_endpoint: 0,
            endpoints: unsafe {
                let mut endpoints: [Endpoint; NUM_ENDPOINTS] = mem::uninitialized();

                for i in 0..NUM_ENDPOINTS {
                    endpoints[i] = Endpoint::new(i as u8);
                }

                endpoints
            },
            reset,
        };

        UsbBusAllocator::new(bus)
    }

    /// Constructs a new USB peripheral driver.
    pub fn usb(regs: USB_FS, apb1: &mut rcc::APB1) -> UsbBusAllocator<Self> {
        UsbBus::new(regs, apb1, None)
    }

    /// Constructs a new USB peripheral driver with the "reset" method enabled.
    pub fn usb_with_reset<M>(
        regs: USB_FS,
        apb1: &mut rcc::APB1,
        clocks: &rcc::Clocks,
        moder: &mut gpioa::MODER,
        otyper: &mut gpioa::OTYPER,
        pa12: gpioa::PA12<M>,
    ) -> UsbBusAllocator<Self> {
        UsbBus::new(
            regs,
            apb1,
            Some(Reset {
                delay: clocks.sysclk().0,
                pin: Mutex::new(RefCell::new(pa12.into_push_pull_output(moder, otyper))),
            }),
        )
    }

    fn alloc_ep_mem(next_ep_mem: &mut usize, size: usize) -> Result<usize> {
        assert!(size & 1 == 0);

        let addr = *next_ep_mem;
        if addr + size > Endpoint::MEM_SIZE {
            return Err(UsbError::EndpointMemoryOverflow);
        }

        *next_ep_mem += size;

        Ok(addr)
    }
}

impl usb_device::bus::UsbBus for UsbBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        ep_type: EndpointType,
        max_packet_size: u16,
        _interval: u8,
    ) -> Result<EndpointAddress> {
        for index in ep_addr
            .map(|a| a.index()..a.index() + 1)
            .unwrap_or(1..NUM_ENDPOINTS)
        {
            let ep = &mut self.endpoints[index];

            match ep.ep_type() {
                None => {
                    ep.set_ep_type(ep_type);
                }
                Some(t) if t != ep_type => {
                    continue;
                }
                _ => {}
            };

            match ep_dir {
                UsbDirection::Out if !ep.is_out_buf_set() => {
                    let (out_size, bits) = calculate_count_rx(max_packet_size as usize)?;

                    let addr = Self::alloc_ep_mem(&mut self.next_ep_mem, out_size)?;

                    ep.set_out_buf(addr, (out_size, bits));

                    return Ok(EndpointAddress::from_parts(index, ep_dir));
                }
                UsbDirection::In if !ep.is_in_buf_set() => {
                    let size = (max_packet_size as usize + 1) & !0x01;

                    let addr = Self::alloc_ep_mem(&mut self.next_ep_mem, size as usize)?;

                    ep.set_in_buf(addr, size as usize);

                    return Ok(EndpointAddress::from_parts(index, ep_dir));
                }
                _ => {}
            }
        }

        Err(match ep_addr {
            Some(_) => UsbError::InvalidEndpoint,
            None => UsbError::EndpointOverflow,
        })
    }

    fn enable(&mut self) {
        let mut max = 0;
        for (index, ep) in self.endpoints.iter().enumerate() {
            if ep.is_out_buf_set() || ep.is_in_buf_set() {
                max = index;
            }
        }

        self.max_endpoint = max;

        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            regs.usb_cntr.modify(|_, w| w.pdwn().clear_bit());

            regs.btable.modify(|_, w| unsafe { w.btable().bits(0) });
            regs.usb_cntr.modify(|_, w| {
                w.fres()
                    .clear_bit()
                    .resetm()
                    .set_bit()
                    .suspm()
                    .set_bit()
                    .wkupm()
                    .set_bit()
                    .errm()
                    .set_bit()
                    .pmaovrm()
                    .set_bit()
                    .ctrm()
                    .set_bit()
            });
            regs.istr.modify(|_, w| unsafe { w.bits(0) });
        });
    }

    fn reset(&self) {
        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            regs.istr.modify(|_, w| unsafe { w.bits(0) });
            regs.daddr.modify(|_, w| w.ef().set_bit().add().bit(false));

            for ep in self.endpoints.iter() {
                ep.configure(cs);
            }
        });
    }

    fn set_device_address(&self, addr: u8) {
        interrupt::free(|cs| {
            self.regs
                .lock(cs)
                .daddr
                .modify(|_, w| w.add().bit(addr != 0));
        });
    }

    fn poll(&self) -> PollResult {
        let mut guard = self.regs.try_lock();

        let regs = match guard {
            Some(ref mut r) => r,
            // re-entrant call, any interrupts will be handled by the already-running call or the
            // next call
            None => {
                return PollResult::None;
            }
        };

        let istr = regs.istr.read();

        if istr.wkup().bit_is_set() {
            regs.istr.modify(|_, w| w.wkup().clear_bit());

            let fnr = regs.fnr.read();
            //let bits = (fnr.rxdp().bit_is_set() as u8) << 1 | (fnr.rxdm().bit_is_set() as u8);

            match (fnr.rxdp().bit_is_set(), fnr.rxdm().bit_is_set()) {
                (false, false) | (false, true) => PollResult::Resume,
                _ => {
                    // Spurious wakeup event caused by noise
                    PollResult::Suspend
                }
            }
        } else if istr.reset().bit_is_set() {
            regs.istr.modify(|_, w| w.reset().clear_bit());

            PollResult::Reset
        } else if istr.susp().bit_is_set() {
            regs.istr.modify(|_, w| w.susp().clear_bit());

            PollResult::Suspend
        } else if istr.ctr().bit_is_set() {
            let mut ep_out = 0;
            let mut ep_in_complete = 0;
            let mut ep_setup = 0;
            let mut bit = 1;

            for ep in &self.endpoints[0..=self.max_endpoint] {
                if ep.ctr_rx_bit_is_set() {
                    ep_out |= bit;

                    if ep.setup_bit_is_set() {
                        ep_setup |= bit;
                    }
                }

                if ep.ctr_tx_bit_is_set() {
                    ep_in_complete |= bit;

                    interrupt::free(|cs| {
                        ep.clear_ctr_tx(cs);
                    });
                }

                bit <<= 1;
            }

            PollResult::Data {
                ep_out,
                ep_in_complete,
                ep_setup,
            }
        } else {
            PollResult::None
        }
    }

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> Result<usize> {
        if !ep_addr.is_in() {
            return Err(UsbError::InvalidEndpoint);
        }

        self.endpoints[ep_addr.index()].write(buf)
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> Result<usize> {
        if !ep_addr.is_out() {
            return Err(UsbError::InvalidEndpoint);
        }

        self.endpoints[ep_addr.index()].read(buf)
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        interrupt::free(|cs| {
            if self.is_stalled(ep_addr) == stalled {
                return;
            }

            let ep = &self.endpoints[ep_addr.index()];

            match (stalled, ep_addr.direction()) {
                (true, UsbDirection::In) => ep.set_stat_tx(cs, EndpointStatus::Stall),
                (true, UsbDirection::Out) => ep.set_stat_rx(cs, EndpointStatus::Stall),
                (false, UsbDirection::In) => ep.set_stat_tx(cs, EndpointStatus::Nak),
                (false, UsbDirection::Out) => ep.set_stat_rx(cs, EndpointStatus::Valid),
            };
        });
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        let ep = &self.endpoints[ep_addr.index()];

        let status = match ep_addr.direction() {
            UsbDirection::In => ep.stat_tx_bits(),
            UsbDirection::Out => ep.stat_rx_bits(),
        };

        status == (EndpointStatus::Stall as u8)
    }

    fn suspend(&self) {
        interrupt::free(|cs| {
            self.regs
                .lock(cs)
                .usb_cntr
                .modify(|_, w| w.fsusp().set_bit().lpmode().set_bit());
        });
    }

    fn resume(&self) {
        interrupt::free(|cs| {
            self.regs
                .lock(cs)
                .usb_cntr
                .modify(|_, w| w.fsusp().clear_bit().lpmode().clear_bit());
        });
    }

    fn force_reset(&self) -> Result<()> {
        interrupt::free(|cs| {
            let regs = self.regs.lock(cs);

            match self.reset {
                Some(ref reset) => {
                    let pdwn = regs.usb_cntr.read().pdwn().bit_is_set();
                    regs.usb_cntr.modify(|_, w| w.pdwn().set_bit());

                    reset.pin.borrow(cs).borrow_mut().set_low();
                    delay(reset.delay);

                    regs.usb_cntr.modify(|_, w| w.pdwn().bit(pdwn));

                    Ok(())
                }
                None => Err(UsbError::Unsupported),
            }
        })
    }
}
