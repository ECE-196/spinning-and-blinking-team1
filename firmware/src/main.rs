#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    clock::Clocks,
    gpio::{Io, Level, Output},
    mcpwm::PeripheralClockConfig,
    timer::timg::TimerGroup,
};
use esp_println::println;
use fugit::RateExtU32;
use esp_hal::mcpwm::McPwm;
use esp_hal::mcpwm::timer::PwmWorkingMode;
use esp_hal::mcpwm::operator::PwmPinConfig;

#[esp_hal_embassy::main]
async fn main(_spawner: Spawner) {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_hal_embassy::init(timg0.timer0);

    let io = Io::new(peripherals.GPIO, peripherals.IO_MUX);

    let mut led = Output::new(io.pins.gpio17, Level::Low);

    let motor_hi_pin = io.pins.gpio13;
    let motor_lo_pin = io.pins.gpio14;

    let clock_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);

    let (mut motor_hi, mut motor_lo) = mcpwm.operator0.with_pins(
        motor_hi_pin,
        PwmPinConfig::UP_ACTIVE_HIGH,
        motor_lo_pin,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    let timer_clock_cfg = clock_cfg
        .timer_clock_with_frequency(u8::MAX as u16, PwmWorkingMode::Increase, 20.kHz())
        .unwrap();

    mcpwm.timer0.start(timer_clock_cfg);

    let mut duty_cycle = 60;
    let mut increasing = true;
    let step = 5; 
    let mut l_r = true;
    loop {
        // Toggle LED
        led.toggle();

        if increasing && l_r {
            duty_cycle -= step;
            println!("fast!");

            if duty_cycle <= 0 {
                increasing = false;
            }
            motor_hi.set_timestamp(255 - duty_cycle);
            motor_lo.set_timestamp(duty_cycle);
        } else if !increasing && l_r {
            println!("slow");
            duty_cycle += step;
            if duty_cycle >= 60 {
                increasing = true;
                l_r = false
            }
            motor_lo.set_timestamp(duty_cycle);
            motor_hi.set_timestamp(255 - duty_cycle);
        } else if increasing && !l_r {
            println!("away fast!");
            duty_cycle -= step;
            if duty_cycle <= 0 {
                increasing = false;
            }
            motor_hi.set_timestamp(duty_cycle);
            motor_lo.set_timestamp(255 - duty_cycle);

        }

        else if !increasing && !l_r {
            println!("away slow!");
            duty_cycle += step;
            if duty_cycle >= 60 {
                increasing = true;
                l_r = true;
            }
            motor_hi.set_timestamp(duty_cycle);
            motor_lo.set_timestamp(255 - duty_cycle);

        }

        Timer::after_millis(500).await;
    }
}
