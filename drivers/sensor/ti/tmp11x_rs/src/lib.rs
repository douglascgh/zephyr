#![no_std]
#![no_main]

use core::ffi::c_int;
use core::panic::PanicInfo;
use log::info;

use embedded_sensors_hal::sensor;
use embedded_sensors_hal::temperature::{DegreesCelsius, TemperatureSensor};

// Keep this in sync with the C side. Avoid pulling Zephyr headers.
#[repr(C)]
pub struct Tmp11xRsCtx {
    i2c: *const core::ffi::c_void, // Opaque pointer to Zephyr I2C device
    i2c_addr: u16,
}

// If you need logging from Rust, either FFI to printk or buffer and return errors.
// Minimal example:

#[unsafe(no_mangle)]
pub extern "C" fn tmp11x_rs_init(ctx: *mut Tmp11xRsCtx) -> c_int {
    if ctx.is_null() {
        return -22; // -EINVAL
    }
    // Perform any internal init. In a real driver, you might probe the sensor ID.
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn tmp11x_rs_sample_fetch(_ctx: *mut Tmp11xRsCtx) -> c_int {
    // For a real device, perform I2C read(s) here.
    // This skeleton keeps I2C on the C side for simplicity; or you can FFI back to C helpers.
    0
}

// Zephyr sensor_value { int32_t val1; int32_t val2; }
#[repr(C)]
pub struct SensorValue {
    val1: i32,
    val2: i32,
}

unsafe extern "C" {
    pub fn tmp11x_reg_read_wrapper(
        ptr: *const core::ffi::c_void,
        reg: u8,
        val: *mut u16,
    ) -> core::ffi::c_int;
}

/// Rust
/// Read a register from TMP11x via I2C.
/// # Safety
/// - `ptr` must be a valid pointer to the Zephyr I2C device context expected by the C side.
pub unsafe fn tmp11x_reg_read(ptr: *const core::ffi::c_void, reg: u8) -> Result<u16, i32> {
    let mut raw: u16 = 0;

    if ptr.is_null() {
        return Err(-22); // -EINVAL
    }

    // SAFETY: ptr is checked for null above
    let rc = unsafe { tmp11x_reg_read_wrapper(ptr, reg, &mut raw) };
    if rc < 0 {
        return Err(-5); // -EIO
    }

    Ok(raw)
}

/// Rust
/// Get the channel value for TMP11x and write into `out`.
///
/// # Safety
/// - `out` must be a valid, non-null, writable pointer to a `SensorValue`.
/// - The memory at `out` must be properly aligned for `SensorValue`.
/// - `ptr` must be a valid pointer to the Zephyr I2C device context expected by the C side.
/// - Caller must ensure concurrent access does not race with other writers to `out`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn tmp11x_rs_channel_get(
    ptr: *const core::ffi::c_void,
    _chan: i32,
    out: *mut SensorValue,
) -> c_int {
    if ptr.is_null() {
        return -22; // -EINVAL
    }

    if out.is_null() {
        return -22; // -EINVAL
    }

    unsafe {
        (*out).val1 = 1;
    }

    unsafe {
        (*out).val2 = 2;
    }

    let mut sensor = match TempSensorTmp11x::try_new(ptr) {
        Ok(sensor) => sensor,
        Err(_) => {
            info!("Error creating TempSensorTmp11x");
            return -22; // -EINVAL
        }
    };

    let current_temperature = match sensor.temperature() {
        Ok(temperature) => temperature,
        Err(e) => {
            info!("Error reading temperature: {:?}", e);
            return -5; // -EIO
        }
    };

    let current_temperature = current_temperature as i32;
    let whole = current_temperature / 100;
    let fractional = current_temperature - (whole * 100);

    // SAFETY: out is not null checked above
    unsafe {
        (*out).val1 = whole as i32;
    }

    // SAFETY: out is not null checked above
    unsafe {
        (*out).val2 = fractional as i32;
    }

    0
}

// TODO: Remove custom panic handler by adding zephyr crate.
#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    info!("panic: {}", info);
    loop {}
}

pub struct TempSensorTmp11x {
    ptr: *const core::ffi::c_void,
}

impl TempSensorTmp11x {
    const fn try_new(ptr: *const core::ffi::c_void) -> Result<Self, TempSensorTmp11xError> {
        if ptr.is_null() {
            return Err(TempSensorTmp11xError::Invalid);
        }

        Ok(Self { ptr })
    }
}

#[derive(Clone, Copy, Debug)]
pub enum TempSensorTmp11xError {
    Bus,
    Invalid,
    Unknown,
}

impl sensor::Error for TempSensorTmp11xError {
    fn kind(&self) -> sensor::ErrorKind {
        embedded_sensors_hal::sensor::ErrorKind::Other
    }
}

impl sensor::ErrorType for TempSensorTmp11x {
    type Error = TempSensorTmp11xError;
}

impl TemperatureSensor for TempSensorTmp11x {
    fn temperature(&mut self) -> Result<DegreesCelsius, Self::Error> {
        // Read temperature register (0x00) TMP11X_REG_TEMP
        let temp_raw =
            unsafe { tmp11x_reg_read(self.ptr, 0x00).map_err(|_| TempSensorTmp11xError::Bus)? };

        Ok(temp_raw.into())
    }
}
