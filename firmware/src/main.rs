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

    clock::Clocks, gpio::{Io, Level, Output}, mcpwm::PeripheralClockConfig, timer::timg::TimerGroup
};
use esp_println::println;
use esp_wifi::current_millis;


    let mut led = Output::new(io.pins.gpio17, Level::Low);
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

    let cur_motor_value = 255;
    motor_hi.set_timestamp(cur_motor_value);
    motor_lo.set_timestamp(0);


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

        // println!("Hello, World!");
        // led.toggle();
        // Timer::after_millis(1_000).await;

        for cur_motor_value in (0..=255).rev() {
            motor_hi.set_timestamp(cur_motor_value);
            motor_lo.set_timestamp(0);
            Timer::after_millis(10).await;
        }

        Timer::after_millis(100).await; 
        
        for cur_motor_value in (0..=255).rev() {
            motor_hi.set_timestamp(0);
            motor_lo.set_timestamp(cur_motor_value);
            Timer::after_millis(10).await; 
        }
    
        Timer::after_millis(100).await;

    }
}
