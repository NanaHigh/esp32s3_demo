use esp_idf_svc::hal;

use embedded_hal::spi::MODE_3;
use hal::delay::Ets;
use hal::gpio::{AnyIOPin, Output, PinDriver};
use hal::peripheral::Peripheral;
use hal::spi::{self, SpiAnyPins, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use hal::units::Hertz;

use display_interface_spi::SPIInterfaceNoCS;

use thiserror::Error;

use mipidsi::{
    models::{ST7735s, ST7789},
    Builder, ColorOrder, Display, Orientation,
};

/// Lcd error.
#[derive(Debug, Error)]
pub enum LcdError {
    #[error("Lcd error")]
    LcdError,
}

/// Lcd driver type.
#[derive(PartialEq)]
pub enum LcdDriverType {
    /// ST7735S.
    St7735s,
    St7789,
}

pub struct LcdConfig {
    pub drv_type: LcdDriverType,
    pub width: u16,
    pub height: u16,
    pub orientation: Orientation,
    pub color_order: ColorOrder,
}

impl LcdConfig {
    /// Create a new LCD configuration.
    pub fn new(drv_type: LcdDriverType, width: u16, height: u16) -> Self {
        Self {
            drv_type,
            width,
            height,
            orientation: Orientation::Portrait(false),
            color_order: ColorOrder::Rgb,
        }
    }

    /// Set orientation.
    pub fn set_orientation(&mut self, orientation: Orientation) -> Result<(), LcdError> {
        self.orientation = orientation;
        Ok(())
    }

    /// Set color order.
    pub fn set_color_order(&mut self, color_order: ColorOrder) -> Result<(), LcdError> {
        self.color_order = color_order;
        Ok(())
    }
}

/// To adapt the different LCD display.
pub enum LcdDisplay<'a> {
    St7735s(
        Display<
            SPIInterfaceNoCS<SpiDeviceDriver<'a, SpiDriver<'a>>, PinDriver<'a, AnyIOPin, Output>>,
            ST7735s,
            PinDriver<'a, AnyIOPin, Output>,
        >,
    ),
    St7789(
        Display<
            SPIInterfaceNoCS<SpiDeviceDriver<'a, SpiDriver<'a>>, PinDriver<'a, AnyIOPin, Output>>,
            ST7789,
            PinDriver<'a, AnyIOPin, Output>,
        >,
    ),
}

/// Lcd driver.
pub struct LcdDriver<'a> {
    pub lcd: LcdDisplay<'a>,
}

impl<'a> LcdDriver<'a> {
    /// Create a new LCD driver.
    pub fn new(
        spi: impl Peripheral<P = impl SpiAnyPins> + 'a,
        lcd_cfg: LcdConfig,
        sclk: AnyIOPin,
        sdo: AnyIOPin,
        sdi: AnyIOPin,
        rst: AnyIOPin,
        dc: AnyIOPin,
        cs: AnyIOPin,
        speed: Hertz,
    ) -> Result<Self, LcdError> {
        let rst_drv = PinDriver::output(rst).map_err(|_| LcdError::LcdError)?;
        let dc_drv = PinDriver::output(dc).map_err(|_| LcdError::LcdError)?;

        let mut delay = Ets;
        let spi_cfg = spi::config::Config::new().baudrate(speed).data_mode(MODE_3);

        let spi_device = SpiDeviceDriver::new_single(
            spi,
            sclk,
            sdo,
            Some(sdi),
            Some(cs),
            &SpiDriverConfig::new(),
            &spi_cfg,
        )
        .map_err(|_| LcdError::LcdError)?;

        // Display interface abstraction from SPI and DC.
        let di = SPIInterfaceNoCS::new(spi_device, dc_drv);

        match lcd_cfg.drv_type {
            LcdDriverType::St7735s => Ok(Self {
                lcd: LcdDisplay::St7735s(
                    Builder::st7735s(di)
                        .with_display_size(lcd_cfg.width, lcd_cfg.height)
                        .with_orientation(lcd_cfg.orientation)
                        .with_color_order(lcd_cfg.color_order)
                        .init(&mut delay, Some(rst_drv))
                        .map_err(|_| LcdError::LcdError)?,
                ),
            }),
            LcdDriverType::St7789 => Ok(Self {
                lcd: LcdDisplay::St7789(
                    Builder::st7789(di)
                        .with_display_size(lcd_cfg.width, lcd_cfg.height)
                        .with_orientation(lcd_cfg.orientation)
                        .with_color_order(lcd_cfg.color_order)
                        .init(&mut delay, Some(rst_drv))
                        .map_err(|_| LcdError::LcdError)?,
                ),
            }),
        }
    }

    /// Get ST735S display instance.
    pub fn as_st7735s(
        self,
    ) -> Result<
        Display<
            SPIInterfaceNoCS<SpiDeviceDriver<'a, SpiDriver<'a>>, PinDriver<'a, AnyIOPin, Output>>,
            ST7735s,
            PinDriver<'a, AnyIOPin, Output>,
        >,
        LcdError,
    > {
        match self.lcd {
            LcdDisplay::St7735s(display) => Ok(display),
            _ => Err(LcdError::LcdError),
        }
    }

    /// Get ST7789 display instance.
    pub fn as_st7789(
        self,
    ) -> Result<
        Display<
            SPIInterfaceNoCS<SpiDeviceDriver<'a, SpiDriver<'a>>, PinDriver<'a, AnyIOPin, Output>>,
            ST7789,
            PinDriver<'a, AnyIOPin, Output>,
        >,
        LcdError,
    > {
        match self.lcd {
            LcdDisplay::St7789(display) => Ok(display),
            _ => Err(LcdError::LcdError),
        }
    }
}
