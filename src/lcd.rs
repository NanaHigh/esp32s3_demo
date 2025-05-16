// TODO: improve code abstraction.
#![allow(clippy::type_complexity)]
#![allow(clippy::too_many_arguments)]
use esp_idf_svc::hal;

use embedded_hal::spi::MODE_3;
use hal::delay::Ets;
use hal::gpio::{AnyIOPin, Output, PinDriver};
use hal::peripheral::Peripheral;
use hal::spi::{self, Dma, SpiAnyPins, SpiDeviceDriver, SpiDriver, SpiDriverConfig};
use hal::units::Hertz;

use display_interface_spi::SPIInterfaceNoCS;

use thiserror::Error;

use mipidsi::{
    models::{ST7735s, ST7789},
    Builder, ColorOrder, Display, Orientation,
};

type SpiInterface<'a> =
    SPIInterfaceNoCS<SpiDeviceDriver<'a, SpiDriver<'a>>, PinDriver<'a, AnyIOPin, Output>>;
type LcdPinDriver<'a> = PinDriver<'a, AnyIOPin, Output>;
type St7735sDisplay<'a> = Display<SpiInterface<'a>, ST7735s, LcdPinDriver<'a>>;
type St7789Display<'a> = Display<SpiInterface<'a>, ST7789, LcdPinDriver<'a>>;

/// Lcd error.
#[derive(Debug, Error)]
pub enum LcdError {
    #[error("Lcd error")]
    LcdError,
    #[error("Lcd type mismatch error")]
    LcdMismatchError,
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
    pub dma: Dma,
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
            dma: Dma::Disabled,
        }
    }

    /// Set orientation.
    pub fn orientation(self, orientation: Orientation) -> Result<Self, LcdError> {
        Ok(Self {
            drv_type: self.drv_type,
            width: self.width,
            height: self.height,
            orientation,
            color_order: self.color_order,
            dma: self.dma,
        })
    }

    /// Set color order.
    pub fn color_order(self, color_order: ColorOrder) -> Result<Self, LcdError> {
        Ok(Self {
            drv_type: self.drv_type,
            width: self.width,
            height: self.height,
            orientation: self.orientation,
            color_order,
            dma: self.dma,
        })
    }

    /// Set DMA.
    pub fn dma(self, dma: Dma) -> Result<Self, LcdError> {
        Ok(Self {
            drv_type: self.drv_type,
            width: self.width,
            height: self.height,
            orientation: self.orientation,
            color_order: self.color_order,
            dma,
        })
    }
}

/// To adapt the different LCD display.
pub enum LcdDisplay<'a> {
    St7735s(St7735sDisplay<'a>),
    St7789(St7789Display<'a>),
}

/// LCD pin configuration
pub struct LcdPins {
    pub sclk: AnyIOPin,
    pub sdo: AnyIOPin,
    pub sdi: AnyIOPin,
    pub rst: AnyIOPin,
    pub dc: AnyIOPin,
    pub cs: AnyIOPin,
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
        pins: LcdPins,
        speed: Hertz,
    ) -> Result<Self, LcdError> {
        let rst_drv = PinDriver::output(pins.rst).map_err(|_| LcdError::LcdError)?;
        let dc_drv = PinDriver::output(pins.dc).map_err(|_| LcdError::LcdError)?;

        let mut delay = Ets;
        let spi_cfg = spi::config::Config::new().baudrate(speed).data_mode(MODE_3);

        let spi_device = SpiDeviceDriver::new_single(
            spi,
            pins.sclk,
            pins.sdo,
            Some(pins.sdi),
            Some(pins.cs),
            &SpiDriverConfig::new().dma(lcd_cfg.dma),
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
    pub fn as_st7735s(self) -> Result<St7735sDisplay<'a>, LcdError> {
        match self.lcd {
            LcdDisplay::St7735s(display) => Ok(display),
            _ => Err(LcdError::LcdMismatchError),
        }
    }

    /// Get ST7789 display instance.
    pub fn as_st7789(self) -> Result<St7789Display<'a>, LcdError> {
        match self.lcd {
            LcdDisplay::St7789(display) => Ok(display),
            _ => Err(LcdError::LcdMismatchError),
        }
    }
}
