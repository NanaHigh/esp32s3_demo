use core::borrow::BorrowMut;
use esp_idf_svc::hal;
use hal::delay::BLOCK;
use hal::i2c::I2cDriver;
use hal::io::Read;
use hal::uart::{AsyncUartDriver, UartDriver};
use thiserror::Error;

/// I2c sensor error.
#[derive(Debug, Error)]
pub enum I2cSensorError {
    #[error("Imu communication error")]
    ImuError,
}

/// I2c sensor driver.
pub struct I2cSensorDriver<'a> {
    i2c: I2cDriver<'a>,
}

impl<'a> I2cSensorDriver<'a> {
    const IMU_ADDR: u8 = 0x50;
    const IMU_ANGLE_ADDR: u8 = 0x3D;

    /// Create a new i2c sensor driver.
    pub fn new(i2c_drv: I2cDriver<'a>) -> Result<Self, I2cSensorError> {
        Ok(Self { i2c: i2c_drv })
    }

    /// Read the angle from the IMU (Blocking).
    /// Roll = angle_buf[0];
    /// Pitch = angle_buf[1];
    /// Yaw = angle_buf[2];
    pub fn read_angle(&mut self) -> Result<[f32; 3], I2cSensorError> {
        let angle_raw = &mut [0u8; 6];

        self.i2c
            .write_read(Self::IMU_ADDR, &[Self::IMU_ANGLE_ADDR], angle_raw, BLOCK)
            .map_err(|_| I2cSensorError::ImuError)?;

        Ok([
            i16::from_le_bytes([angle_raw[0], angle_raw[1]]) as f32 / 32768.0 * 180.0,
            i16::from_le_bytes([angle_raw[2], angle_raw[3]]) as f32 / 32768.0 * 180.0,
            i16::from_le_bytes([angle_raw[4], angle_raw[5]]) as f32 / 32768.0 * 180.0,
        ])
    }
}

/// Uart sensor error.
#[derive(Debug, Error)]
pub enum UartSensorError {
    #[error("Infrared sensor communication error")]
    IrError,
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
            .map_err(|_| UartSensorError::IrError)?;
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
            .map_err(|_| UartSensorError::IrError)?;

        Ok(u16::from_be_bytes([distance_raw[3], distance_raw[4]]))
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
            .map_err(|_| UartSensorError::IrError)?;
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
            .map_err(|_| UartSensorError::IrError)?;

        Ok(u16::from_be_bytes([distance_raw[3], distance_raw[4]]))
    }
}
