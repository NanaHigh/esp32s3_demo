use core::borrow::BorrowMut;
use esp_idf_svc::hal;
use hal::delay::{FreeRtos, BLOCK};
use hal::i2c::I2cDriver;
use hal::io::Read;
use hal::sys::EspError;
use hal::uart::{AsyncUartDriver, UartDriver};
use thiserror::Error;

/// I2c sensor error.
#[derive(Debug, Error)]
pub enum I2cSensorError {
    #[error("Imu read error")]
    ImuReadError,
    #[error("Imu write error: {0}")]
    ImuWriteError(EspError),
    #[error("Gray sensor read error")]
    GrayReadError,
}

/// Imu mode.
#[derive(Debug, PartialEq)]
pub enum ImuMode {
    Axis9,
    Axis6,
}

/// I2c sensor driver.
pub struct I2cSensorDriver<'a> {
    i2c: I2cDriver<'a>,
}

impl<'a> I2cSensorDriver<'a> {
    const MAX_RETRIES: u8 = 3;
    const RETRY_DELAY_MS: u32 = 100;

    // If IMU read return ESP_FAIL, toggle this flag.
    const IMU_ERROR: bool = false;
    const IMU_ERR_ADDR: u8 = 0x00;
    const IMU_ADDR: u8 = 0x50;
    const IMU_ANGLE_ADDR: u8 = 0x3D;

    const IR_ADDR: u8 = 0x12;
    const IR_LINE_ADDR: u8 = 0x30;

    /// Create a new i2c sensor driver.
    pub fn new(i2c_drv: I2cDriver<'a>) -> Result<Self, I2cSensorError> {
        Ok(Self { i2c: i2c_drv })
    }

    /// Set imu mode.
    pub fn set_imu_mode(&mut self, mode: ImuMode) -> Result<(), I2cSensorError> {
        let mode = match mode {
            ImuMode::Axis9 => 0x00,
            ImuMode::Axis6 => 0x01,
        };
        // Unlock the IMU.
        self.i2c
            .write(
                if Self::IMU_ERROR {
                    Self::IMU_ERR_ADDR
                } else {
                    Self::IMU_ADDR
                },
                &[0x69, 0x88, 0xB5],
                BLOCK,
            )
            .map_err(I2cSensorError::ImuWriteError)?;
        FreeRtos::delay_ms(200);
        // Set the IMU mode.
        self.i2c
            .write(
                if Self::IMU_ERROR {
                    Self::IMU_ERR_ADDR
                } else {
                    Self::IMU_ADDR
                },
                &[0x24, mode, 0x00],
                BLOCK,
            )
            .map_err(I2cSensorError::ImuWriteError)?;
        FreeRtos::delay_ms(200);
        // Lock the IMU.
        self.i2c
            .write(
                if Self::IMU_ERROR {
                    Self::IMU_ERR_ADDR
                } else {
                    Self::IMU_ADDR
                },
                &[0x00, 0x00, 0x00],
                BLOCK,
            )
            .map_err(I2cSensorError::ImuWriteError)?;
        FreeRtos::delay_ms(200);
        Ok(())
    }

    /// Reset the yaw angle.
    /// Only available in axis 6 mode.
    pub fn reset_yaw_angle(&mut self) -> Result<(), I2cSensorError> {
        // Unlock the IMU.
        self.i2c
            .write(
                if Self::IMU_ERROR {
                    Self::IMU_ERR_ADDR
                } else {
                    Self::IMU_ADDR
                },
                &[0x69, 0x88, 0xB5],
                BLOCK,
            )
            .map_err(I2cSensorError::ImuWriteError)?;
        FreeRtos::delay_ms(200);
        FreeRtos::delay_ms(50);
        // Reset the yaw angle.
        self.i2c
            .write(
                if Self::IMU_ERROR {
                    Self::IMU_ERR_ADDR
                } else {
                    Self::IMU_ADDR
                },
                &[0x01, 0x04, 0x00],
                BLOCK,
            )
            .map_err(I2cSensorError::ImuWriteError)?;
        FreeRtos::delay_ms(200);
        // Lock the IMU.
        self.i2c
            .write(
                if Self::IMU_ERROR {
                    Self::IMU_ERR_ADDR
                } else {
                    Self::IMU_ADDR
                },
                &[0x00, 0x00, 0x00],
                BLOCK,
            )
            .map_err(I2cSensorError::ImuWriteError)?;
        Ok(())
    }

    /// Read the angle from the IMU (Blocking).
    /// Roll = angle_buf[0];
    /// Pitch = angle_buf[1];
    /// Yaw = angle_buf[2];
    pub fn read_angle(&mut self) -> Result<[f32; 3], I2cSensorError> {
        let angle_raw = &mut [0u8; 6];
        let mut attempts = 0;

        while attempts < Self::MAX_RETRIES {
            match self.i2c.write_read(
                if Self::IMU_ERROR {
                    Self::IMU_ERR_ADDR
                } else {
                    Self::IMU_ADDR
                },
                &[Self::IMU_ANGLE_ADDR],
                angle_raw,
                BLOCK,
            ) {
                Ok(_) => {
                    return Ok([
                        i16::from_le_bytes([angle_raw[0], angle_raw[1]]) as f32 / 32768.0 * 180.0,
                        i16::from_le_bytes([angle_raw[2], angle_raw[3]]) as f32 / 32768.0 * 180.0,
                        i16::from_le_bytes([angle_raw[4], angle_raw[5]]) as f32 / 32768.0 * 180.0,
                    ])
                }
                Err(e) => {
                    log::error!("IMU read error (attempt {}): {:?}", attempts + 1, e);
                    if attempts < Self::MAX_RETRIES - 1 {
                        FreeRtos::delay_ms(Self::RETRY_DELAY_MS);
                    }
                    attempts += 1;
                }
            }
        }
        Err(I2cSensorError::ImuReadError)
    }

    /// Read the line from the gray sensor (Blocking).
    /// Line = [led1, led2, led3, led4, led5, led6, led7, led8].
    /// True = black, false = white.
    pub fn read_line(&mut self) -> Result<[bool; 8], I2cSensorError> {
        let line = &mut [0u8];
        let mut attempts = 0;

        while attempts < Self::MAX_RETRIES {
            match self
                .i2c
                .write_read(Self::IR_ADDR, &[Self::IR_LINE_ADDR], line, BLOCK)
            {
                Ok(_) => {
                    return Ok([
                        line[0] & 0x80 == 0,
                        line[0] & 0x40 == 0,
                        line[0] & 0x20 == 0,
                        line[0] & 0x10 == 0,
                        line[0] & 0x08 == 0,
                        line[0] & 0x04 == 0,
                        line[0] & 0x02 == 0,
                        line[0] & 0x01 == 0,
                    ])
                }
                Err(e) => {
                    log::error!("IMU read error (attempt {}): {:?}", attempts + 1, e);
                    if attempts < Self::MAX_RETRIES - 1 {
                        FreeRtos::delay_ms(Self::RETRY_DELAY_MS);
                    }
                    attempts += 1;
                }
            }
        }
        Err(I2cSensorError::GrayReadError)
    }
}

/// Uart sensor error.
#[derive(Debug, Error)]
pub enum UartSensorError {
    #[error("Infrared sensor write error")]
    IrWriteError,
    #[error("Infrared sensor read error")]
    IrReadError,
    #[error("Camera communication error")]
    CamError,
    #[error("Camera val invalid")]
    CamInvalid,
}

/// Uart sensor driver.
pub struct UartSensorDriver<'a> {
    uart: UartDriver<'a>,
}

impl<'a> UartSensorDriver<'a> {
    const IR_ADDR: u8 = 0x51;
    const IR_DISTANCE_ADDR: u8 = 0x34;
    const IR_READ_CMD: u8 = 0x03;

    /// Create a new uart sensor driver.
    pub fn new(uart_drv: UartDriver<'a>) -> Result<Self, UartSensorError> {
        Ok(Self { uart: uart_drv })
    }

    /// Read the distance from the IR sensor (Blocking).
    pub fn read_distance(&mut self) -> Result<u16, UartSensorError> {
        // Send the command to read the distance using modbus.
        // Arguments 0: device address.
        // Arguments 1: command.
        // Arguments 2: high byte of the address.
        // Arguments 3: low byte of the address.
        // Arguments 4: high byte of the number of bytes to read.
        // Arguments 5: low byte of the number of bytes to read.
        // Arguments 6: CRC high byte (Calculated by the modbus protocol, change according to different commands).
        // Arguments 7: CRC low byte (Calculated by the modbus protocol, change according to different commands).
        let distance_cmd = &[
            Self::IR_ADDR,
            Self::IR_READ_CMD,
            0x00,
            Self::IR_DISTANCE_ADDR,
            0x00,
            0x01,
            0xC9,
            0x94,
        ];
        let distance_raw = &mut [0u8; 7];
        self.uart
            .write(distance_cmd)
            .map_err(|_| UartSensorError::IrWriteError)?;
        // Blocking wait.
        // Get the data to calculate the distance using modbus.
        // Arguments 0: device address.
        // Arguments 1: command.
        // Arguments 2: data byte 1.
        // Arguments 3: data byte 2.
        // ------------------------
        // Arguments n: data byte n-1.
        // The second-to-last arguments: CRC high byte (Calculated by the modbus protocol, change according to different datas).
        // The last arguments: CRC low byte (Calculated by the modbus protocol, change according to different datas).
        // CRC is not currently in use.
        self.uart
            .read_exact(distance_raw)
            .map_err(|_| UartSensorError::IrReadError)?;

        Ok(u16::from_be_bytes([distance_raw[3], distance_raw[4]]))
    }

    /// Read the camera label (Blocking).
    pub fn read_cam_label(&mut self) -> Result<u8, UartSensorError> {
        let raw = &mut [0u8; 1];
        self.uart
            .read(raw, BLOCK)
            .map_err(|_| UartSensorError::CamError)?;
        if raw[0] > 0 && raw[0] < 9 {
            Ok(raw[0])
        } else {
            Err(UartSensorError::CamInvalid)
        }
    }
}

/// Async art sensor driver.
pub struct AsyncUartSensorDriver<'a, T>
where
    T: BorrowMut<UartDriver<'a>>,
{
    uart: AsyncUartDriver<'a, T>,
}

impl<'a, T> AsyncUartSensorDriver<'a, T>
where
    T: BorrowMut<UartDriver<'a>>,
{
    const IR_ADDR: u8 = 0x51;
    const IR_DISTANCE_ADDR: u8 = 0x34;
    const IR_READ_CMD: u8 = 0x03;

    /// Create a new async uart sensor driver.
    pub fn new(uart_drv: AsyncUartDriver<'a, T>) -> Result<Self, UartSensorError> {
        Ok(Self { uart: uart_drv })
    }

    /// Read the distance from the IR sensor (Non blocking).
    pub async fn read_distance(&mut self) -> Result<u16, UartSensorError> {
        // Send the command to read the distance using modbus.
        // Arguments 0: device address.
        // Arguments 1: command.
        // Arguments 2: high byte of the address.
        // Arguments 3: low byte of the address.
        // Arguments 4: high byte of the number of bytes to read.
        // Arguments 5: low byte of the number of bytes to read.
        // Arguments 6: CRC high byte (Calculated by the modbus protocol, change according to different commands).
        // Arguments 7: CRC low byte (Calculated by the modbus protocol, change according to different commands).
        let distance_cmd = &[
            Self::IR_ADDR,
            Self::IR_READ_CMD,
            0x00,
            Self::IR_DISTANCE_ADDR,
            0x00,
            0x01,
            0xC9,
            0x94,
        ];
        let distance_raw = &mut [0u8; 7];
        self.uart
            .write(distance_cmd)
            .await
            .map_err(|_| UartSensorError::IrWriteError)?;
        // Get the data to calculate the distance using modbus.
        // Arguments 0: device address.
        // Arguments 1: command.
        // Arguments 2: data byte 1.
        // Arguments 3: data byte 2.
        // ------------------------
        // Arguments n: data byte n-1.
        // The second-to-last arguments: CRC high byte (Calculated by the modbus protocol, change according to different datas).
        // The last arguments: CRC low byte (Calculated by the modbus protocol, change according to different datas).
        // CRC is not currently in use.
        self.uart
            .read(distance_raw)
            .await
            .map_err(|_| UartSensorError::IrReadError)?;

        Ok(u16::from_be_bytes([distance_raw[3], distance_raw[4]]))
    }
}
