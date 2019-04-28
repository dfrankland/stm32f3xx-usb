//! USB peripheral driver for STM32F3xx microcontrollers.

#![no_std]

mod atomic_mutex;
mod endpoint;

/// USB peripheral driver.
pub mod bus;
pub use bus::UsbBus;
