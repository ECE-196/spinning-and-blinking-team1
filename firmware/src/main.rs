#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_time::Timer;
use esp_backtrace as _;
use esp_hal::{
    clock::Clocks, gpio::{Io, Level, Output}, mcpwm::PeripheralClockConfig, timer::timg::TimerGroup
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

    let mut led = Output::new(io.pins.gpio18, Level::Low);
    // add these
    let motor_hi_pin = io.pins.gpio13;
    let motor_lo_pin = io.pins.gpio14;

    let clocks = Clocks::get();
    //println!("src clock: {}", clocks.crypto_pwm_clock);
    let clock_cfg = PeripheralClockConfig::with_frequency(32.MHz()).unwrap();    //let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    let mut mcpwm = McPwm::new(peripherals.MCPWM0, clock_cfg);
    mcpwm.operator0.set_timer(&mcpwm.timer0);
    let timer_clock_cfg = clock_cfg.timer_clock_with_frequency(u8::MAX as u16, PwmWorkingMode::Increase, 20.kHz()).unwrap();
    mcpwm.timer0.start(timer_clock_cfg);

    let (mut motor_hi, mut motor_lo) = mcpwm.operator0.with_pins(
        motor_hi_pin,
        PwmPinConfig::UP_ACTIVE_HIGH,
        motor_lo_pin,
        PwmPinConfig::UP_ACTIVE_HIGH,
    );

    motor_hi.set_timestamp(180);
    motor_lo.set_timestamp(0);

    let mut status = 0;//speed up 1 and slowing down 0
    let mut direction = 0;//spin sirection.
    let mut pin13= 255;
    let mut pin14 = 0;
    let maximum =255;
    let minimum = 180;

    loop {
        //println!("Hello, World!");
        //led.toggle();
        //Timer::after_millis(1_000).await;


        if status == 0 && direction == 0 {
            pin13 -= 1;
            if pin13 <= minimum {
                direction = 1;
                status = 1;
                pin13 = 0;
                pin14 = 180;
                Timer::after_millis(50).await;
            }
        }

        if status == 1 && direction == 1 {
            pin14 += 1;
            if pin14 >= maximum {
                direction = 1;
                status = 0;
            }
        }

        if status == 0 && direction == 1 {
            pin14 -= 1;
            if pin14 <= minimum {
                direction = 0;
                status = 1;
                pin14 = 0;
                pin13 = 180;
                Timer::after_millis(50).await;
            }
        }

        if status == 1 && direction == 0 {
            pin13 += 1;
            if pin13 >= maximum {
                direction = 0;
                status = 0;
            }
        }
        motor_hi.set_timestamp(pin13);
        motor_lo.set_timestamp(pin14);

        


        Timer::after_millis(50).await;
        

    }
}
