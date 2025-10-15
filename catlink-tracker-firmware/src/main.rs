#![no_std]
#![no_main]
#![allow(static_mut_refs)]

mod extract_gga;
mod sx126x;
mod sync_cell;

use core::{arch::asm, mem::MaybeUninit, panic::PanicInfo};
use cortex_m::Peripherals as CorePeripherals;
use cortex_m_rt::{entry, pre_init};
use extract_gga::{calculate_sentence_length, is_gga};
use static_fifo_queue::Queue;
use stm32l4::stm32l412::{Interrupt, Peripherals as DevicePeripherals, interrupt};
use sx126x::SyncSpiDescriptor;
use sync_cell::{SyncCell, SyncUnsafeCell};

// ==== Device configuration ====
/// Device address
const ADDRESS: u32 = 0x1C40C;
/// Whitelisted gateway address
const GATEWAY_ADDRESS: u32 = 0x4D467;
/// Wake-up interval in seconds
const WAKEUP_INTERVAL: u16 = 20;
/// Shutdown if divided battery voltage is lower than this.
/// Divider ratio = 0.5, Vref = 3V
const LOW_BATTERY_THRESHOLD: u8 = 0x81; // 0x81/0xFF * 3 * 2 = 3.04V
/// Maximum number of packet retries
const MAX_TX_RETRIES: u8 = 3;
/// Sample battery voltage every 12 hours
const SAMPLE_BAT_INTERVAL: u16 = 2160; // 86400 / 2 / 20

/// GNSS configuration - disable all sentences except GGA,
/// set GGA output to once every 9 update cycles
const GNSS_SET_SENTENCE_OUTPUT: [u8; 43] = *b"$PCAS03,9,0,0,0,0,0,0,0,0,0,,,0,0,,,,0*3B\r\n";

/// SLEEPDEEP 1, SLEEPONEXIT 1
const SLEEPDEEP_ON: u32 = 0b110;
/// SLEEPDEEP 0, SLEEPONEXIT 1
const SLEEPDEEP_OFF: u32 = 0b010;

// Peripheral register addresses for DMA
const SPI1_DR: u32 = 0x4001_300C;
const USART1_RDR: u32 = 0x4001_3824;
const USART1_TDR: u32 = 0x4001_3828;

/// Application state
static STATE: SyncCell<State> = SyncCell::new(State::Init);
/// Queue of SPI commands to run
static COMMANDS: SyncUnsafeCell<Queue<&SyncSpiDescriptor, 16>> = SyncUnsafeCell::new(Queue::new());
/// Circular buffer for GNSS NMEA data
static NMEA_BUFFER: SyncUnsafeCell<[u8; 1024]> = SyncUnsafeCell::new([0; 1024]);
/// The beginning of the current NMEA sentence in [`NMEA_BUFFER`]
static SENTENCE_BEGIN: SyncCell<usize> = SyncCell::new(0);
/// Last sampled divided battery voltage
static BAT_DIV_VOL: SyncCell<u8> = SyncCell::new(LOW_BATTERY_THRESHOLD);
/// TX power setting
static TX_POWER: SyncCell<TxPower> = SyncCell::new(TxPower::Dbm22);
/// Counter for doing things at intervals
static INTERVAL_COUNTER: SyncCell<u16> = SyncCell::new(0);

// Peripheral structs
static mut CORE_PERIPHERALS: MaybeUninit<CorePeripherals> = MaybeUninit::uninit();
static mut DEVICE_PERIPHERALS: MaybeUninit<DevicePeripherals> = MaybeUninit::uninit();

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Init,
    Sleep,
    ScanAdCad(bool), // Do 2 CADs
    ScanAdCadGetIrq(bool),
    ScanAdWaitingSynRx,
    ScanAdProcessSynRx,
    ScanAdSynAckTx(bool), // Send 2 SynAcks
    GnssContinue,
    GnssTx(u8), // u8: nth retry
    GnssWaitingAck(u8),
    GnssProcessAck(u8),
    SleepBegin,
    SleepContinue,
}

#[repr(u8)]
#[derive(Copy, Clone, PartialEq, Eq)]
enum TxPower {
    Dbm10 = 0,
    Dbm14 = 1,
    Dbm17 = 2,
    Dbm20 = 3,
    Dbm22 = 4,
}
impl TxPower {
    #[inline]
    const fn from_bits(bits: u8) -> Self {
        match bits {
            0 => Self::Dbm10,
            1 => Self::Dbm14,
            2 => Self::Dbm17,
            3 => Self::Dbm20,
            4 => Self::Dbm22,
            _ => Self::Dbm22,
        }
    }
    #[inline]
    const fn set_power(self) {
        let commands = COMMANDS.get_mut();
        match self {
            Self::Dbm10 => {
                commands.enqueue(&sx126x::SET_PA_CONFIG_10);
                commands.enqueue(&sx126x::SET_TX_PARAMS_15);
            }
            Self::Dbm14 => {
                commands.enqueue(&sx126x::SET_PA_CONFIG_14);
                commands.enqueue(&sx126x::SET_TX_PARAMS_15);
            }
            Self::Dbm17 => {
                commands.enqueue(&sx126x::SET_PA_CONFIG_17);
                commands.enqueue(&sx126x::SET_TX_PARAMS_22);
            }
            Self::Dbm20 => {
                commands.enqueue(&sx126x::SET_PA_CONFIG_20);
                commands.enqueue(&sx126x::SET_TX_PARAMS_22);
            }
            Self::Dbm22 => {
                commands.enqueue(&sx126x::SET_PA_CONFIG_22);
                commands.enqueue(&sx126x::SET_TX_PARAMS_22);
            }
        }
    }
    #[inline]
    fn adjust(self, tx_power_control_bits: u8) -> Self {
        match tx_power_control_bits {
            0 => self, // No change
            1 => {
                // Decrease
                if self == Self::Dbm10 {
                    // Already at lowest power, do nothing
                    self
                } else {
                    let new_tx_power = Self::from_bits(self as u8 - 1);
                    Self::set_power(new_tx_power);
                    new_tx_power
                }
            }
            2 => {
                // Increase
                if self == Self::Dbm22 {
                    // Already at highest power, do nothing
                    self
                } else {
                    let new_tx_power = Self::from_bits(self as u8 + 1);
                    Self::set_power(new_tx_power);
                    new_tx_power
                }
            }
            _ => self,
        }
    }
}

/// Initiates a sequence of SPI1 transfers over DMA1_CHANNEL2(RX) and DMA1_CHANNEL3(TX).
/// The sequence of commands starts with the command given to this function
/// and continues with commands from [`COMMANDS`] until no commands remain.
#[inline]
fn send_command(command: &SyncSpiDescriptor, dp: &mut DevicePeripherals) {
    // Write memory address for SPI1 RX DMA channel
    dp.DMA1.ch2().mar().write(|w| unsafe { w.ma().bits(command.0.rx_buf_ptr as u32) });
    // Write memory address for SPI1 TX DMA channel
    dp.DMA1.ch3().mar().write(|w| unsafe { w.ma().bits(command.0.tx_buf_ptr as u32) });

    // Set DMA transfer size for SPI1 RX/TX
    dp.DMA1.ch2().ndtr().write(|w| w.ndt().set(command.0.transfer_length));
    dp.DMA1.ch3().ndtr().write(|w| w.ndt().set(command.0.transfer_length));

    // Enable DMA for SPI1 RX, TX
    dp.DMA1.ch2().cr().write(|w| w.minc().set_bit().tcie().set_bit().en().set_bit());
    dp.DMA1.ch3().cr().write(|w| w.minc().set_bit().dir().from_memory().tcie().set_bit().en().set_bit());
    // Enable SPI1
    dp.SPI1.cr1().write(|w| w.mstr().set_bit().spe().enabled());
}

#[inline]
fn enter_stop2(cp: &mut CorePeripherals, dp: &mut DevicePeripherals) {
    unsafe {
        cp.SCB.scr.write(SLEEPDEEP_ON);
        // Prepare to enter Stop 2: enter run mode range 2 with Stop 2 selected
        dp.PWR.cr1().write(|w| w.vos().bits(2).lpr().clear_bit().lpms().bits(2).dbp().set_bit());
    }
    STATE.0.set(State::Sleep);
    while dp.PWR.sr2().read().reglpf().bit_is_set() {}
}

/// Switch GNSS module ON, begin receiving data and begin GNSS pre-init timer
#[inline]
fn gnss_on(dp: &mut DevicePeripherals) {
    dp.USART1.cr1().write(|w| w.cmie().set_bit().ue().set_bit().re().set_bit().te().set_bit());
    dp.DMA1.ch5().ndtr().write(|w| w.ndt().set(NMEA_BUFFER.get().len() as u16));
    dp.DMA1.ch5().cr().write(|w| w.minc().set_bit().circ().set_bit().en().set_bit());
    dp.GPIOA.bsrr().write(|w| w.bs11().set_bit());
    dp.TIM6.cr1().write(|w| w.urs().counter_only().opm().set_bit().cen().set_bit());
}

/// Switch GNSS module OFF and stop receiving data
#[inline]
fn gnss_off(dp: &mut DevicePeripherals) {
    dp.GPIOA.bsrr().write(|w| w.br11().set_bit());
    dp.DMA1.ch5().cr().write(|w| w.minc().set_bit().circ().set_bit().en().clear_bit());
    dp.USART1.cr1().write(|w| w.cmie().set_bit().ue().clear_bit().re().clear_bit());
}

#[inline]
fn start_adc(dp: &mut DevicePeripherals) {
    // Switch on battery voltage divider
    dp.GPIOB.bsrr().write(|w| w.br4().set_bit());
    // Power up ADC1
    dp.ADC1.cr().write(|w| w.deeppwd().clear_bit().advregen().set_bit());
    // See datasheet table 63 "ADC voltage regulator start-up time" p.115 (20 us)
    unsafe {
        asm!("nop"); // A single nop should be 5 us
        asm!("nop");
        asm!("nop");
        asm!("nop");
    }
    // ADC calibration p.383
    dp.ADC1.cr().write(|w| w.deeppwd().clear_bit().advregen().set_bit().adcal().set_bit());
    while dp.ADC1.cr().read().adcal().bit_is_set() {}
    // Enable ADC
    dp.ADC1.cr().write(|w| w.deeppwd().clear_bit().advregen().set_bit().aden().set_bit());
}

#[inline]
fn initiate_cad(dp: &mut DevicePeripherals) {
    COMMANDS.get_mut().enqueue(&sx126x::SET_CAD);
    STATE.0.set(State::ScanAdCad(true));
    send_command(&sx126x::SET_STANDBY, dp);
}

/// System reset on panic
#[inline(never)]
#[panic_handler]
fn panic_sys_reset(_info: &PanicInfo) -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}

/// GNSS pre-init delay
#[interrupt]
fn TIM6_DACUNDER() {
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };

    // Configure GNSS
    dp.DMA1.ch4().ndtr().write(|w| w.ndt().set(GNSS_SET_SENTENCE_OUTPUT.len() as u16));
    dp.DMA1.ch4().cr().write(|w| w.minc().set_bit().dir().from_memory().tcie().set_bit().en().set_bit());

    dp.TIM6.sr().write(|w| w.uif().clear_bit());
}

/// When GNSS is ON, DMA1 CH5 is running in circular mode in the background.
/// This interrupt fires when the end of a NMEA sentence is detected.
#[interrupt]
fn USART1() {
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };
    let ndtr = dp.DMA1.ch5().ndtr().read().ndt().bits();
    let commands = COMMANDS.get_mut();

    dp.USART1.icr().write(|w| w.cmcf().clear_bit_by_one().orecf().clear_bit_by_one());

    // Sometimes the sentence is shifted by 1
    let sentence_begin_corrected = if unsafe { *NMEA_BUFFER.get().get_unchecked(SENTENCE_BEGIN.0.get()) } == b'$' {
        SENTENCE_BEGIN.0.get()
    } else {
        SENTENCE_BEGIN.0.get() + 1
    };
    // Update sentence position
    let sentence_length = calculate_sentence_length(ndtr, SENTENCE_BEGIN.0.get());
    SENTENCE_BEGIN.0.update(|s| (s + sentence_length) & (NMEA_BUFFER.get().len() - 1));

    if is_gga(NMEA_BUFFER.get(), sentence_begin_corrected) {
        unsafe {
            sx126x::PACKET.insert_position(NMEA_BUFFER.get(), sentence_begin_corrected);

            // Prepare packet header and transfer length
            sx126x::PACKET.set_header();

            commands.enqueue(&sx126x::WRITE_PACKET);
        }
        commands.enqueue(&sx126x::UNMASK_TXDONE);
        commands.enqueue(&sx126x::SET_TX);
        STATE.0.set(State::GnssTx(0));
        send_command(&sx126x::SET_PACKET_PARAMS, dp);
    }
}

/// Wake-up for CAD
#[interrupt]
fn RTC_WKUP() {
    let cp = unsafe { CORE_PERIPHERALS.assume_init_mut() };
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };

    dp.RTC.scr().write(|w| w.cwutf().set_bit());
    dp.EXTI.pr1().write(|w| w.pr20().clear_bit_by_one());

    unsafe {
        // Don't go back into Stop 2
        cp.SCB.scr.write(SLEEPDEEP_OFF);
        // Enter low-power run mode
        dp.PWR.cr1().write(|w| w.vos().bits(2).lpr().set_bit().lpms().bits(1).dbp().set_bit());
    }

    if INTERVAL_COUNTER.0.get() < SAMPLE_BAT_INTERVAL {
        INTERVAL_COUNTER.0.update(|c| c + 1);
        initiate_cad(dp);
    } else {
        INTERVAL_COUNTER.0.set(0);
        start_adc(dp);
    }
}

/// Sample divided battery voltage, if battery is too low, shutdown.
/// Otherwise, proceed with wake-up CAD sequence.
#[interrupt]
fn ADC1_2() {
    let cp = unsafe { CORE_PERIPHERALS.assume_init_mut() };
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };
    let isr = dp.ADC1.isr().read().bits();

    if isr & 1 == 1 {
        // ADC ready -> start ADC sample
        dp.ADC1.cr().write(|w| w.deeppwd().clear_bit().advregen().set_bit().aden().set_bit().adstart().set_bit());
        dp.ADC1.isr().write(|w| w.adrdy().clear_bit_by_one());
    } else if ((isr >> 2) & 1) == 1 {
        // EOC, conversion complete
        // Reading ADC1 DR resets EOC
        BAT_DIV_VOL.0.set(dp.ADC1.dr().read().rdata().bits() as u8);

        // Switch off battery divider
        dp.GPIOB.bsrr().write(|w| w.bs4().set_bit());
        // Disable and power down ADC1
        dp.ADC1.cr().write(|w| w.deeppwd().clear_bit().advregen().set_bit().aden().set_bit().addis().set_bit());
        while dp.ADC1.cr().read().aden().bit_is_set() {}
        dp.ADC1.cr().write(|w| w.deeppwd().set_bit());

        if BAT_DIV_VOL.0.get() < LOW_BATTERY_THRESHOLD {
            // Shutdown
            dp.RTC.cr().write(|w| w.wute().clear_bit()); // Disable wake-up timer
            dp.PWR.cr3().write(|w| w.apc().set_bit());
            // Pull-up NSS
            dp.PWR.pucrb().write(|w| w.pu0().set_bit());
            // Pull-down GNSS ON/OFF
            dp.PWR.pdcra().write(|w| w.pd11().set_bit());
            unsafe {
                cp.SCB.scr.write(SLEEPDEEP_ON);
                dp.PWR.cr1().write(|w| w.vos().bits(2).lpr().set_bit().lpms().bits(0b100).dbp().set_bit());
            }
        } else {
            initiate_cad(dp);
        }
    }
}

/// DIO1 handler (rising edge). Handles TXDONE, RXDONE and CADDONE
#[interrupt]
fn EXTI2() {
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };
    let commands = COMMANDS.get_mut();

    dp.EXTI.pr1().write(|w| w.pr2().clear_bit_by_one());

    match STATE.0.get() {
        State::ScanAdCad(first) => {
            // CAD DONE
            commands.enqueue(&sx126x::CLEAR_CAD);
            STATE.0.set(State::ScanAdCadGetIrq(first));
            send_command(&sx126x::GET_IRQ_STATUS, dp);
        }
        State::ScanAdWaitingSynRx => {
            // RX DONE
            commands.enqueue(&sx126x::READ_BUFFER);
            STATE.0.set(State::ScanAdProcessSynRx);
            send_command(&sx126x::CLEAR_RXDONE, dp);
        }
        State::ScanAdSynAckTx(first) => {
            // TX DONE
            if first {
                // Send 2 SynAcks
                commands.enqueue(&sx126x::SET_TX);
                STATE.0.set(State::ScanAdSynAckTx(false));
            } else {
                dp.RTC.cr().write(|w| w.wucksel().clock_spare().wutie().set_bit().wute().clear_bit());

                unsafe { sx126x::PACKET.insert_battery(BAT_DIV_VOL.0.get()) };
                gnss_on(dp);
                STATE.0.set(State::GnssContinue);
            }
            send_command(&sx126x::CLEAR_TXDONE, dp);
        }
        State::GnssTx(retries) => {
            // TX DONE
            // Listen for ACK
            commands.enqueue(&sx126x::UNMASK_RXDONE);
            commands.enqueue(&sx126x::SET_RX);
            STATE.0.set(State::GnssWaitingAck(retries));
            send_command(&sx126x::CLEAR_TXDONE, dp);
        }
        State::GnssWaitingAck(retries) => {
            // RX DONE
            commands.enqueue(&sx126x::READ_BUFFER);
            STATE.0.set(State::GnssProcessAck(retries));
            send_command(&sx126x::CLEAR_RXDONE, dp);
        }
        _ => (),
    }
}

/// DIO3 handler (rising edge). Handles TIMEOUT
#[interrupt]
fn EXTI9_5() {
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };
    let commands = COMMANDS.get_mut();

    dp.EXTI.pr1().write(|w| w.pr9().clear_bit_by_one());

    // TIMEOUT
    match STATE.0.get() {
        State::ScanAdWaitingSynRx => {
            commands.enqueue(&sx126x::UNMASK_CAD);
            commands.enqueue(&sx126x::SET_SLEEP);
            STATE.0.set(State::SleepContinue);
        }
        State::GnssWaitingAck(retries) => {
            if retries < MAX_TX_RETRIES {
                commands.enqueue(&sx126x::UNMASK_TXDONE);
                commands.enqueue(&sx126x::SET_TX);
                STATE.0.set(State::GnssTx(retries + 1));
            } else {
                gnss_off(dp);
                unsafe {
                    sx126x::PACKET.reset_flags();
                    sx126x::PACKET.set_header();
                }
                commands.enqueue(&sx126x::SET_PACKET_PARAMS);
                commands.enqueue(&sx126x::UNMASK_CAD);
                commands.enqueue(&sx126x::SET_SLEEP);
                STATE.0.set(State::SleepBegin);
            }
        }
        _ => (),
    }
    send_command(&sx126x::CLEAR_TIMEOUT, dp);
}

/// SPI1 RX DMA TC interrupt - SX1268 command complete
#[interrupt]
fn DMA1_CHANNEL2() {
    let cp = unsafe { CORE_PERIPHERALS.assume_init_mut() };
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };
    let commands = COMMANDS.get_mut();

    dp.DMA1.ifcr().write(|w| w.ctcif2().set_bit());

    // Disable DMA Ch2
    dp.DMA1.ch2().cr().write(|w| w.minc().set_bit().tcie().set_bit().en().clear_bit());

    // Disable SPI1
    dp.SPI1.cr1().write(|w| w.mstr().set_bit().spe().clear_bit());

    // Send next command
    if let Some(command) = commands.dequeue() {
        send_command(command, dp);
    } else {
        match STATE.0.get() {
            State::Init => {
                start_adc(dp);
            }
            State::ScanAdCadGetIrq(first) => {
                let irq_status = unsafe { sx126x::GET_IRQ_STATUS_BUFS.irq_status() };
                if irq_status.cad_detected() {
                    commands.enqueue(&sx126x::SET_RX_SYN);
                    STATE.0.set(State::ScanAdWaitingSynRx);
                    send_command(&sx126x::UNMASK_RXDONE, dp);
                } else if first {
                    STATE.0.set(State::ScanAdCad(false));
                    send_command(&sx126x::SET_CAD, dp);
                } else {
                    STATE.0.set(State::SleepContinue);
                    send_command(&sx126x::SET_SLEEP, dp);
                }
            }
            State::ScanAdProcessSynRx => {
                if unsafe { sx126x::ACK.get_address() } == GATEWAY_ADDRESS {
                    TxPower::set_power(TxPower::Dbm22);
                    TX_POWER.0.set(TxPower::Dbm22);
                    commands.enqueue(&sx126x::UNMASK_TXDONE);
                    commands.enqueue(&sx126x::SET_TX);
                    STATE.0.set(State::ScanAdSynAckTx(true));
                    send_command(unsafe { &sx126x::WRITE_PACKET }, dp);
                } else {
                    commands.enqueue(&sx126x::SET_SLEEP);
                    STATE.0.set(State::SleepContinue);
                    send_command(&sx126x::UNMASK_CAD, dp);
                }
            }
            State::GnssProcessAck(retries) => unsafe {
                if (sx126x::ACK.get_address() == GATEWAY_ADDRESS)
                    && (sx126x::ACK.get_packet_id() == sx126x::PACKET.get_packet_id())
                {
                    sx126x::PACKET.reset_flags();
                    TX_POWER.0.update(|tx_power| tx_power.adjust(sx126x::ACK.get_tx_power_control()));
                    STATE.0.set(State::GnssContinue);
                } else if retries < MAX_TX_RETRIES {
                    commands.enqueue(&sx126x::SET_TX);
                    STATE.0.set(State::GnssTx(retries + 1));
                    send_command(&sx126x::UNMASK_TXDONE, dp);
                } else {
                    gnss_off(dp);
                    sx126x::PACKET.reset_flags();
                    sx126x::PACKET.set_header();
                    commands.enqueue(&sx126x::UNMASK_CAD);
                    commands.enqueue(&sx126x::SET_SLEEP);
                    STATE.0.set(State::SleepBegin);
                    send_command(&sx126x::SET_PACKET_PARAMS, dp);
                }
            },
            State::SleepBegin => {
                // Enable wake-up timer
                dp.RTC.cr().write(|w| w.wucksel().clock_spare().wutie().set_bit().wute().set_bit());
                enter_stop2(cp, dp);
            }
            State::SleepContinue => {
                enter_stop2(cp, dp);
            }
            _ => (),
        }
    }
}

/// SPI1 TX DMA TC interrupt - Sent SX1268 command
#[interrupt]
fn DMA1_CHANNEL3() {
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };

    // Disable DMA Ch3
    dp.DMA1.ch3().cr().write(|w| w.minc().set_bit().dir().from_memory().tcie().set_bit().en().clear_bit());
    dp.DMA1.ifcr().write(|w| w.ctcif3().set_bit());
}

/// USART1(GNSS) TX DMA TC interrupt
#[interrupt]
fn DMA1_CHANNEL4() {
    let dp = unsafe { DEVICE_PERIPHERALS.assume_init_mut() };

    dp.DMA1.ch4().cr().write(|w| w.minc().set_bit().dir().from_memory().tcie().set_bit().en().clear_bit());
    dp.DMA1.ifcr().write(|w| w.ctcif4().set_bit());
}

/// Copy sections from load memory (flash) to virtual memory (SRAM1).
#[pre_init]
unsafe fn populate_sections_virtual_memory() {
    unsafe {
        asm!(
            // Copy vector_table to SRAM1
            "ldr r0, =__vector_table
            ldr r1, =__evector_table
            ldr r2, =__sivector_table
            0:
            cmp r1, r0
            beq 1f
            ldm r2!, {{r3}}
            stm r0!, {{r3}}
            b 0b
            1:",
            // Copy .rodata to SRAM1
            "ldr r0, =__srodata
            ldr r1, =__erodata
            ldr r2, =__sirodata
            0:
            cmp r1, r0
            beq 1f
            ldm r2!, {{r3}}
            stm r0!, {{r3}}
            b 0b
            1:"
        );
    }
}

#[entry]
fn main() -> ! {
    let (cp, dp) = unsafe {
        (
            CORE_PERIPHERALS.write(CorePeripherals::take().unwrap()),
            DEVICE_PERIPHERALS.write(DevicePeripherals::take().unwrap()),
        )
    };

    // ==== RCC configuration ====
    // Set system clock speed to 200 kHz
    dp.RCC.cr().write(|w| w.msirange().range200k().msirgsel().set_bit());

    // Enable LSI for RTC
    dp.RCC.csr().write(|w| w.lsion().set_bit());

    // Enable peripheral clocks
    dp.RCC.ahb1enr().write(|w| w.dma1en().set_bit());
    dp.RCC.ahb2enr().write(|w| w.gpioaen().set_bit().gpioben().set_bit().adcen().set_bit());
    dp.RCC.apb1enr1().write(|w| w.pwren().set_bit().rtcapben().set_bit().tim6en().set_bit());
    dp.RCC.apb2enr().write(|w| w.syscfgen().set_bit().spi1en().set_bit().usart1en().set_bit());

    // Ref manual p.234 - ADC needs clock source
    dp.RCC.ccipr().write(|w| w.adcsel().sysclk());

    // ==== FLASH configuration ====
    // Flash memory clock is gated off during sleep and stop
    dp.RCC.ahb1smenr().write(|w| w.flashsmen().clear_bit());
    // Power down flash during sleep
    dp.FLASH.acr().write(|w| w.sleep_pd().set_bit());

    // ==== TIM6 configuration ====
    // GNSS pre-init delay
    dp.TIM6.psc().write(|w| w.psc().set(19999)); // 200kHz / (19999 + 1) = 10 Hz (100ms per tick)
    dp.TIM6.arr().write(|w| w.arr().set(4)); // Works at 400ms
    dp.TIM6.cr1().write(|w| w.urs().counter_only());
    dp.TIM6.egr().write(|w| w.ug().set_bit());
    dp.TIM6.dier().write(|w| w.uie().set_bit());

    // ==== GPIOA configuration ====
    // A0: ADC1 C5
    // A2: DIO1 | A9: DIO3
    // A5: SCK | A6: MISO | A7: MOSI
    // A11: GNSS EN
    dp.GPIOA.moder().write(|w| {
        w.moder0()
            .analog()
            .moder2()
            .input()
            .moder5()
            .alternate()
            .moder6()
            .alternate()
            .moder7()
            .alternate()
            .moder9()
            .input()
            .moder11()
            .output()
    });
    dp.GPIOA.otyper().write(|w| w.ot11().push_pull());
    dp.GPIOA.bsrr().write(|w| w.br11().set_bit());
    dp.GPIOA.afrl().write(|w| w.afrl5().af5().afrl6().af5().afrl7().af5());

    // ==== GPIOB configuration ====
    // B0: NSS
    // B4: VIN_ADC_EN | B6: TX | B7: RX
    dp.GPIOB.moder().write(|w| w.moder0().alternate().moder4().output().moder6().alternate().moder7().alternate());
    dp.GPIOB.pupdr().write(|w| w.pupdr0().pull_up());
    dp.GPIOB.otyper().write(|w| w.ot4().open_drain());
    dp.GPIOB.bsrr().write(|w| w.bs4().set_bit()); // Disconnect voltage divider
    dp.GPIOB.afrl().write(|w| w.afrl0().af5().afrl6().af7().afrl7().af7());

    // ==== DMA1 C2,C3 + SPI1 configuration ====
    // DMA channel selection p.299
    // C2: SPI1 RX
    // C3: SPI1 TX
    // C4: USART1 TX
    // C5: USART1 RX
    dp.DMA1.cselr().write(|w| w.c2s().map1().c3s().map1().c4s().map2().c5s().map2());

    // DMA C2 SPI1 RX
    dp.DMA1.ch2().par().write(|w| unsafe { w.pa().bits(SPI1_DR) });

    // DMA C3 SPI1 TX
    dp.DMA1.ch3().par().write(|w| unsafe { w.pa().bits(SPI1_DR) });

    dp.SPI1.cr2().write(|w| w.frxth().set_bit().ssoe().enabled().txdmaen().set_bit().rxdmaen().set_bit());

    // ==== DMA1 C4,C5 configuration ====
    // DMA C4 USART1 TX
    dp.DMA1.ch4().par().write(|w| unsafe { w.pa().bits(USART1_TDR) });
    dp.DMA1.ch4().mar().write(|w| unsafe { w.ma().bits(GNSS_SET_SENTENCE_OUTPUT.as_ptr() as u32) });

    // DMA C5 USART1 RX
    dp.DMA1.ch5().par().write(|w| unsafe { w.pa().bits(USART1_RDR) });
    dp.DMA1.ch5().mar().write(|w| unsafe { w.ma().bits(NMEA_BUFFER.get().as_ptr() as u32) });

    // ==== USART1 configuration ====
    dp.USART1.brr().write(|w| w.brr().set(21)); // 200khz / 9600 approx. 21
    dp.USART1.cr2().write(|w| w.add().set(b'\n')); // Character detection
    dp.USART1.cr3().write(|w| w.dmar().set_bit().dmat().set_bit());

    // ==== ADC1 configuration ====
    // Set 8-bit resolution
    dp.ADC1.cfgr().write(|w| w.res().bits8());
    // Set sequence length to 1, first conversion to channel 5
    dp.ADC1.sqr1().write(|w| unsafe { w.l().set(0).sq1().bits(5) });
    // Sample time = 2.5 ADC cycles (shortest)
    dp.ADC1.smpr1().write(|w| w.smp5().cycles2_5());
    dp.ADC1.ier().write(|w| w.adrdyie().set_bit().eocie().set_bit());

    // ==== EXTI configuration ====
    // dp.SYSCFG.exticr2().write(|w| unsafe { w.exti4().bits(0b001) });
    dp.EXTI.rtsr1().write(|w| w.tr2().set_bit().tr9().set_bit().tr20().set_bit());
    dp.EXTI.imr1().write(|w| w.mr2().set_bit().mr9().set_bit().mr20().set_bit());

    // ==== PWR configuration ====
    unsafe {
        // Disable interrupt preemption?
        cp.SCB.aircr.write(0x05FA_0700);
        // Set SLEEPONEXIT
        cp.SCB.scr.write(SLEEPDEEP_OFF);
        // Set voltage scaling to range 2, low-power run mode and
        // remove write protection from BDCR and RTC registers
        dp.PWR.cr1().write(|w| w.vos().bits(2).lpr().set_bit().lpms().bits(1).dbp().set_bit());
    }

    // ==== RTC configuration ====
    // Configure RTC, set periodic wake up
    while dp.RCC.csr().read().lsirdy().bit_is_clear() {}
    dp.RCC.bdcr().write(|w| w.rtcsel().lsi().rtcen().set_bit());
    // Remove write protection from RTC registers
    dp.RTC.wpr().write(|w| w.key().deactivate1());
    dp.RTC.wpr().write(|w| w.key().deactivate2());
    // Enter init mode to set prescaler values
    dp.RTC.icsr().write(|w| w.init().set_bit());
    while dp.RTC.icsr().read().initf().bit_is_clear() {}
    // AN4759 p.12 - calendar unit = 1 Hz
    dp.RTC.prer().write(|w| w.prediv_a().set(127).prediv_s().set(249));
    dp.RTC.icsr().write(|w| w.init().clear_bit());
    // Turn off wake-up timer
    dp.RTC.cr().write(|w| w.wute().clear_bit());
    while dp.RTC.icsr().read().wutwf().bit_is_clear() {}
    // Write wake-up timer registers
    dp.RTC.wutr().write(|w| w.wut().set(WAKEUP_INTERVAL - 1));

    let commands = COMMANDS.get_mut();

    unsafe {
        sx126x::WRITE_PACKET.0.transfer_length = 5;

        commands.enqueue(&sx126x::SET_PACKET_TYPE);
        commands.enqueue(&sx126x::SET_RF_FREQUENCY);
        commands.enqueue(&sx126x::SET_BUFFER_ADDRESS);
        commands.enqueue(&sx126x::WRITE_SYNC_WORD);
        commands.enqueue(&sx126x::SET_MOD_PARAMS);
        commands.enqueue(&sx126x::SET_REG_MODE);
        commands.enqueue(&sx126x::SET_CAD_PARAMS);
        commands.enqueue(&sx126x::WRITE_PACKET);
        commands.enqueue(&sx126x::SET_PACKET_PARAMS);
        commands.enqueue(&sx126x::SET_DIO2_AS_RF_SWITCH_CTRL);
        commands.enqueue(&sx126x::UNMASK_CAD);

        // Unmask NVIC global interrupts
        cortex_m::peripheral::NVIC::unmask(Interrupt::ADC1_2); // EOC, ADRDY
        cortex_m::peripheral::NVIC::unmask(Interrupt::TIM6_DACUNDER); // UIF - GNSS pre-init delay
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CHANNEL2); // TC - SPI1 RX
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CHANNEL3); // TC - SPI1 TX
        cortex_m::peripheral::NVIC::unmask(Interrupt::USART1); // CM
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI2); // DIO1 RT
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI9_5); // DIO3 RT
        cortex_m::peripheral::NVIC::unmask(Interrupt::RTC_WKUP); // WUTE
        cortex_m::peripheral::NVIC::unmask(Interrupt::DMA1_CHANNEL4); // TC - USART1 TX
    }

    // Initialize SX1268
    dp.RTC.cr().write(|w| w.wucksel().clock_spare().wutie().set_bit().wute().set_bit());
    send_command(&sx126x::SET_STANDBY, dp);

    loop {
        unsafe { asm!("wfi") };
    }
}
