//! USB peripheral driver for STM32F3xx microcontrollers.

#![no_std]

mod endpoint;

mod atomic_mutex;

/// USB peripheral driver.
pub mod bus;

pub use crate::bus::UsbBus;
