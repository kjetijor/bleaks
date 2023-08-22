//#![cfg_attr(not(test), no_std)]
#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![macro_use]

use defmt_rtt as _; // global logger
use embassy_nrf as _;
// time driver
use panic_probe as _;
use embassy_executor::Spawner;
use embassy_nrf::interrupt::InterruptExt;
use embassy_nrf::peripherals::SAADC;
use embassy_nrf::saadc::{AnyInput, Input, Saadc};
use embassy_nrf::{bind_interrupts, interrupt, saadc};
use defmt::{info, *};
// use embassy_time::{Timer,Duration};
use nrf_softdevice::ble::peripheral;
use nrf_softdevice::{raw, Softdevice, ble::peripheral::AdvertiseError};
use bitflags::bitflags;

bitflags! {
    pub struct Flags: u8 {
        const IS_LEAKING = 0b0000_0001;
    }
}

const DEVNAME: Option<&'static str> = option_env!("DEVNAME");

bind_interrupts!(struct Irqs {
    SAADC => saadc::InterruptHandler;
});
 
#[embassy_executor::task]
async fn softdevice_task(sd: &'static Softdevice) -> ! {
    info!("Starting SD task");
    sd.run().await
}

// SDA = 4, SCL = 5 - (seeed numbering, NRF52 P0.04, P0.05 (?)
// page19 of - https://files.seeedstudio.com/wiki/XIAO/Seeed-Studio-XIAO-Series-SOM-Datasheet.pdf
// See: https://github.com/embassy-rs/embassy/blob/main/examples/nrf52840/src/bin/twim.rs

fn init_adc(adc_pin: AnyInput, adc: SAADC) -> Saadc<'static, 1> {
    let config = saadc::Config::default();
    let channel_cfg = saadc::ChannelConfig::single_ended(adc_pin.degrade_saadc());
    interrupt::SAADC.set_priority(interrupt::Priority::P3);
    let saadc = saadc::Saadc::new(adc, Irqs, config, [channel_cfg]);
    saadc
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut nrf_config = embassy_nrf::config::Config::default();
    nrf_config.gpiote_interrupt_priority = interrupt::Priority::P2;
    nrf_config.time_interrupt_priority = interrupt::Priority::P2;
    let peripherals = embassy_nrf::init(nrf_config);
    let adc_pin = peripherals.P0_02.degrade_saadc();
    let mut saadc = init_adc(adc_pin, peripherals.SAADC);
    saadc.calibrate().await;

    let devname = DEVNAME.unwrap_or("TEST-SENSOR").as_bytes();
    let config = nrf_softdevice::Config {
        clock: Some(raw::nrf_clock_lf_cfg_t {
            source: raw::NRF_CLOCK_LF_SRC_RC as u8,
            rc_ctiv: 16,
            rc_temp_ctiv: 2,
            accuracy: raw::NRF_CLOCK_LF_ACCURACY_500_PPM as u8,
        }),
        conn_gap: Some(raw::ble_gap_conn_cfg_t {
            conn_count: 6,
            event_length: 24,
        }),
        conn_gatt: Some(raw::ble_gatt_conn_cfg_t { att_mtu: 256 }),
        gatts_attr_tab_size: Some(raw::ble_gatts_cfg_attr_tab_size_t { attr_tab_size: 32768 }),
        gap_role_count: Some(raw::ble_gap_cfg_role_count_t {
            adv_set_count: 1,
            periph_role_count: 3,
            central_role_count: 3,
            central_sec_count: 0,
            _bitfield_1: raw::ble_gap_cfg_role_count_t::new_bitfield_1(0),
        }),
        gap_device_name: Some(raw::ble_gap_cfg_device_name_t {
            p_value: devname.as_ptr() as _,
            current_len: devname.len() as u16,
            max_len: devname.len() as u16,
            write_perm: raw::ble_gap_conn_sec_mode_t {
                _bitfield_1: raw::ble_gap_conn_sec_mode_t::new_bitfield_1(0, 0),
            },
            _bitfield_1: raw::ble_gap_cfg_device_name_t::new_bitfield_1(raw::BLE_GATTS_VLOC_STACK as u8),
        }),
        ..Default::default()
    };


    info!("Enabling softdevice");
    let sd = Softdevice::enable(&config);
    unwrap!(spawner.spawn(softdevice_task(sd)));

    // https://docs.silabs.com/bluetooth/4.0/general/adv-and-scanning/bluetooth-adv-data-basics#advertising-data-format
    // For 5.x LE (?) - Extended Advertising - we can do 1650 bytes of advertising data (or rather 255) - silabs says big num
    // nordic says 254 - https://blog.nordicsemi.com/getconnected/bluetooth-5-advertising-extensions

    // For Legacy Advertising we can do 31    
    let mut ct: u8 = 0;
    loop {
        let mut flags = Flags::empty();
        
        let mut adc_input:[i16; 1] = [0; 1];
        info!("Sampling ADC");
        saadc.sample(&mut adc_input).await;

        if adc_input[0] > 500 {
            flags |= Flags::IS_LEAKING;
        }
        let sample_bytes = adc_input[0].to_be_bytes();
        
        info!("Advertising for {} {} with ADC input {} as bytes {:x}{:x}", devname, ct, adc_input[0], sample_bytes[0], sample_bytes[1]);
        // 0x181A
        #[rustfmt::skip]
        let adv_data = &[
            0x02, raw::BLE_GAP_AD_TYPE_FLAGS as u8, raw::BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE as u8,
            0x03, raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8, 0x18, 0x1a,
            0x06, raw::BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME as u8, b'L', b'E', b'A', b'K', b'S',
            0x09, raw::BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA as u8, 0xff, 0xff, 0x4c, 0x4b, flags.bits(), ct, sample_bytes[0], sample_bytes[1],
        ];
        #[rustfmt::skip]
        let scan_data = &[
            0x03, raw::BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE as u8, 0x18, 0x1a,
        ];

        let mut config = peripheral::Config::default();
        // 0.625ms * 1600 = 1s
        config.interval = 1600 * 2; // why does changing this make reception - not work ?
        config.max_events = Some(2);
        let adv = peripheral::NonconnectableAdvertisement::ScannableUndirected { adv_data, scan_data };
        if let Err(e) = peripheral::advertise(sd, adv, &config).await {
            if e != AdvertiseError::Timeout {
                crate::panic!("advertise error: {:?}", e);
            }
        }
        ct = ct + 1;
    }
}

const UHEX: &[u8; 16] = b"0123456789ABCDEF";

pub fn u8_to_hex(val: u8) -> (u8, u8) {
    (UHEX[(val >> 4) as usize], UHEX[(val & 0xF) as usize])
}
