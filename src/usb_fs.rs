use stm32f3xx_hal::stm32::{USB_FS, usb_fs};

macro_rules! impl_usb_epr_for {
    [ $( [ $USB_EPR:ident, $usb_epr:ident ] ),* ] => {
        $(
            use stm32f3xx_hal::stm32::usb_fs::{$USB_EPR, $usb_epr};

            impl UsbEpr {
                pub fn modify<F>(&self, f: F)
                where
                    for<'w> F: FnOnce(&$usb_epr::R, &'w mut $usb_epr::W) -> &'w mut $usb_epr::W,
                {
                    self.inner_usb_epr.modify(f)
                }

                pub fn read(&self) -> $usb_epr::R {
                    self.inner_usb_epr.read()
                }

                pub fn write<F>(&self, f: F)
                where
                    F: FnOnce(&mut $usb_epr::W) -> &mut $usb_epr::W,
                {
                    self.inner_usb_epr.write(f)
                }

                pub fn reset(&self) {
                    self.inner_usb_epr.reset()
                }
            }
        )*
    };
}

pub struct UsbEpr<T> {
    inner_usb_epr: T,
}

impl_usb_epr_for![
    [USB_EP0R, usb_ep0r],
    [USB_EP1R, usb_ep1r],
    [USB_EP2R, usb_ep2r],
    [USB_EP3R, usb_ep3r],
    [USB_EP4R, usb_ep4r],
    [USB_EP5R, usb_ep5r],
    [USB_EP6R, usb_ep6r],
    [USB_EP7R, usb_ep7r]
];
