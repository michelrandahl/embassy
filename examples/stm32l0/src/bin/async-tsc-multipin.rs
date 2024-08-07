// TODO: update docs text
//
//
// Example of async TSC (Touch Sensing Controller) that lights an LED when touch is detected.
//
// Suggested physical setup on STM32L073RZ Nucleo board:
// - Connect a 1000pF capacitor between pin A0 and GND. This is your sampling capacitor.
// - Connect one end of a 1K resistor to pin A1 and leave the other end loose.
//   The loose end will act as touch sensor which will register your touch.
//
// Troubleshooting the setup:
// - If no touch seems to be registered, then try to disconnect the sampling capacitor from GND momentarily,
//   now the led should light up. Next try using a different value for the sampling capacitor.
//   Also experiment with increasing the values for `ct_pulse_high_length`, `ct_pulse_low_length`, `pulse_generator_prescaler`, `max_count_value` and `discharge_delay`.
//
// All configuration values and sampling capacitor value have been determined experimentally.
// Suitable configuration and discharge delay values are highly dependent on the value of the sample capacitor. For example, a shorter discharge delay can be used with smaller capacitor values.
//
#![no_std]
#![no_main]

use defmt::*;
use embassy_stm32::{bind_interrupts, mode, peripherals};
use embassy_stm32::gpio::{Level, Output, Speed};
use embassy_stm32::tsc::{self, *};
use embassy_time::Timer;
use {defmt_rtt as _, panic_probe as _};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

bind_interrupts!(struct Irqs {
    TSC => InterruptHandler<embassy_stm32::peripherals::TSC>;
});

#[cortex_m_rt::exception]
unsafe fn HardFault(_: &cortex_m_rt::ExceptionFrame) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

/// This example is written for the nucleo-stm32l073rz, with a stm32l073rz chip.
///
/// Make sure you check/update the following (whether you use the L073RZ or another board):
///
/// * [ ] Update .cargo/config.toml with the correct `probe-rs run --chip STM32L073RZTx`chip name.
/// * [ ] Update Cargo.toml to have the correct `embassy-stm32` feature, for L073RZ it should be `stm32l073rz`.
/// * [ ] If your board has a special clock or power configuration, make sure that it is
///       set up appropriately.
/// * [ ] If your board has different pin mapping, update any pin numbers or peripherals
///       to match your schematic
///
/// If you are unsure, please drop by the Embassy Matrix chat for support, and let us know:
///
/// * Which example you are trying to run
/// * Which chip and board you are using
///
/// Embassy Chat: https://matrix.to/#/#embassy-rs:matrix.org

static TOUCH_CHANNEL : Channel<CriticalSectionRawMutex, (bool, bool), 3> = Channel::new();

const SENSOR_TRESHOLD : u16 = 40;

#[embassy_executor::task]
async fn tsc_task(
    mut touch_controller: Tsc<'static, peripherals::TSC, mode::Async>,
    tsc_sensor1: TscIOPin,
    tsc_sensor2: TscIOPin,
) {
    info!("TSC task started");
    let discharge_delay = 5; // ms

    async fn read_sensor(
        touch_controller: &mut Tsc<'static, peripherals::TSC, mode::Async>,
        sensor: TscIOPin,
        discharge_delay: u64,
    ) -> bool {
        touch_controller.set_active_channels(&[sensor]).unwrap();
        touch_controller.start();
        touch_controller.pend_for_acquisition().await;
        touch_controller.discharge_io(true);
        Timer::after_millis(discharge_delay).await;

        touch_controller.group_get_value(sensor.group()) < SENSOR_TRESHOLD
    }

    loop {
        let sensor1_touched = read_sensor(&mut touch_controller, tsc_sensor1, discharge_delay).await;
        let sensor2_touched = read_sensor(&mut touch_controller, tsc_sensor2, discharge_delay).await;

        TOUCH_CHANNEL.send((sensor1_touched, sensor2_touched)).await;
    }
}

#[embassy_executor::task]
async fn led_task(mut led : Output<'static>) {
    let mut led_state = false;
    loop {
        let (sensor1, sensor2) = TOUCH_CHANNEL.receive().await;
        
        match (sensor1, sensor2) {
            (false, false) => {
                led.set_low();
                led_state = false;
            },
            (true, false) => {
                led.set_high();
                led_state = true;
            },
            (false, true) => {
                led_state = !led_state;
                if led_state {
                    led.set_high();
                } else {
                    led.set_low();
                }
                Timer::after_millis(100).await;
            },
            (true, true) => {
                led_state = !led_state;
                if led_state {
                    led.set_high();
                } else {
                    led.set_low();
                }
                Timer::after_millis(50).await;
            },
        }
    }
}

// TODOs:
// - update this module docs
// - test the code on the board
// - rebase with latest master
//  * test the code on the board
// - update other examples for this family
//  * test the code on the board
// - update other examples for other families
//  * test the code on the board
//
// - use the code in our TSC tuning application to see if ergonomics and everything is okay

#[embassy_executor::main]
async fn main(spawner: embassy_executor::Spawner) {
    let device_config = embassy_stm32::Config::default();
    let context = embassy_stm32::init(device_config);

    let mut pin_group : PinGroup<peripherals::TSC, G4> =
        PinGroup::new();
    pin_group.set_io1(context.PA9, PinType::Sample);
    pin_group.set_io2(context.PA10, PinType::Channel);
    pin_group.set_io3(context.PA11, PinType::Channel);

    let (sensor1, _) = pin_group.get_io2().unwrap();
    let (sensor2, _) = pin_group.get_io3().unwrap();

    let mut config = tsc::Config {
        ct_pulse_high_length: ChargeTransferPulseCycle::_16,
        ct_pulse_low_length: ChargeTransferPulseCycle::_16,
        spread_spectrum: false,
        spread_spectrum_deviation: SSDeviation::new(2).unwrap(),
        spread_spectrum_prescaler: false,
        pulse_generator_prescaler: PGPrescalerDivider::_16,
        max_count_value: MaxCount::_255,
        io_default_mode: false,
        synchro_pin_polarity: false,
        acquisition_mode: false,
        max_count_interrupt: false,
        ..Default::default()
    };
    config.configure_io_masks(
        None,
        None,
        None,
        Some(&pin_group),
        None,
        None,
        None,
        None
    );

    let mut touch_controller = tsc::Tsc::new_async(
        context.TSC,
        None,
        None,
        None,
        Some(pin_group),
        None,
        None,
        None,
        None,
        config,
        Irqs,
    ).unwrap();

    // Check if TSC is ready
    if touch_controller.get_state() != State::Ready {
        crate::panic!("TSC not ready!");
    }

    spawner.spawn(tsc_task(touch_controller, sensor1, sensor2)).unwrap();

    info!("TSC initialized successfully");

    // LED2 on the STM32L073RZ nucleo-board (PA5)
    let led = Output::new(context.PA5, Level::High, Speed::Low);
    spawner.spawn(led_task(led)).unwrap();
}
