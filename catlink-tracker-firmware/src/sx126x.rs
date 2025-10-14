use crate::extract_gga::extract_gga;
use sx126x_spi_buffers::{
    commands::{
        Bw, CadExitMode, CadSymbolNum, ClearIrqStatus, Cr, GetIrqStatus, HeaderType, InvertIq, Irq, PacketType,
        RampTime, ReadBuffer, SetBufferBaseAddress, SetCad, SetCadParams, SetDio2AsRfSwitchCtrl, SetDioIrqParams,
        SetModulationParamsLora, SetPaConfig, SetPacketParams, SetPacketType, SetRegulatorMode, SetRfFrequency, SetRx,
        SetSleep, SetStandby, SetTx, SetTxParams, Sf, SpiDescriptor, StdbyConfig, WriteBuffer, WriteRegisters,
    },
    registers::LoraSyncWordMsb,
};

/// Sync wrapper for SpiDescriptor
pub struct SyncSpiDescriptor(pub SpiDescriptor);
unsafe impl Sync for SyncSpiDescriptor {}

// Non-mutable commands
static SET_STANDBY_BUFS: SetStandby = SetStandby::new(StdbyConfig::StdbyRc);
pub static SET_STANDBY: SyncSpiDescriptor = SyncSpiDescriptor(SET_STANDBY_BUFS.descriptor());

static SET_SLEEP_BUFS: SetSleep = SetSleep::new(true);
pub static SET_SLEEP: SyncSpiDescriptor = SyncSpiDescriptor(SET_SLEEP_BUFS.descriptor());

static SET_PACKET_TYPE_BUFS: SetPacketType = SetPacketType::new(PacketType::Lora);
pub static SET_PACKET_TYPE: SyncSpiDescriptor = SyncSpiDescriptor(SET_PACKET_TYPE_BUFS.descriptor());

static SET_RF_FREQUENCY_BUFS: SetRfFrequency = SetRfFrequency::new(455081984);
pub static SET_RF_FREQUENCY: SyncSpiDescriptor = SyncSpiDescriptor(SET_RF_FREQUENCY_BUFS.descriptor());

static WRITE_SYNC_WORD_BUFS: WriteRegisters<5> = WriteRegisters::<5>::new::<LoraSyncWordMsb>([0xAA, 0xF1]);
pub static WRITE_SYNC_WORD: SyncSpiDescriptor = SyncSpiDescriptor(WRITE_SYNC_WORD_BUFS.descriptor());

static SET_PA_CONFIG_22_BUFS: SetPaConfig = SetPaConfig::new(0x04, 0x07, 0);
pub static SET_PA_CONFIG_22: SyncSpiDescriptor = SyncSpiDescriptor(SET_PA_CONFIG_22_BUFS.descriptor());

static SET_PA_CONFIG_20_BUFS: SetPaConfig = SetPaConfig::new(0x03, 0x05, 0);
pub static SET_PA_CONFIG_20: SyncSpiDescriptor = SyncSpiDescriptor(SET_PA_CONFIG_20_BUFS.descriptor());

static SET_PA_CONFIG_17_BUFS: SetPaConfig = SetPaConfig::new(0x02, 0x03, 0);
pub static SET_PA_CONFIG_17: SyncSpiDescriptor = SyncSpiDescriptor(SET_PA_CONFIG_17_BUFS.descriptor());

static SET_PA_CONFIG_14_BUFS: SetPaConfig = SetPaConfig::new(0x04, 0x06, 0);
pub static SET_PA_CONFIG_14: SyncSpiDescriptor = SyncSpiDescriptor(SET_PA_CONFIG_14_BUFS.descriptor());

static SET_PA_CONFIG_10_BUFS: SetPaConfig = SetPaConfig::new(0x00, 0x03, 0);
pub static SET_PA_CONFIG_10: SyncSpiDescriptor = SyncSpiDescriptor(SET_PA_CONFIG_10_BUFS.descriptor());

static SET_TX_PARAMS_22_BUFS: SetTxParams = SetTxParams::new(22, RampTime::Ramp200U);
pub static SET_TX_PARAMS_22: SyncSpiDescriptor = SyncSpiDescriptor(SET_TX_PARAMS_22_BUFS.descriptor());

static SET_TX_PARAMS_15_BUFS: SetTxParams = SetTxParams::new(15, RampTime::Ramp200U);
pub static SET_TX_PARAMS_15: SyncSpiDescriptor = SyncSpiDescriptor(SET_TX_PARAMS_15_BUFS.descriptor());

static SET_BUFFER_ADDRESS_BUFS: SetBufferBaseAddress = SetBufferBaseAddress::new(0x00, 0x80);
pub static SET_BUFFER_ADDRESS: SyncSpiDescriptor = SyncSpiDescriptor(SET_BUFFER_ADDRESS_BUFS.descriptor());

static SET_MOD_PARAMS_BUFS: SetModulationParamsLora =
    SetModulationParamsLora::new(Sf::Sf9, Bw::Bw125, Cr::Cr4_5, false);
pub static SET_MOD_PARAMS: SyncSpiDescriptor = SyncSpiDescriptor(SET_MOD_PARAMS_BUFS.descriptor());

static SET_REG_MODE_BUFS: SetRegulatorMode = SetRegulatorMode::new(true);
pub static SET_REG_MODE: SyncSpiDescriptor = SyncSpiDescriptor(SET_REG_MODE_BUFS.descriptor());

static SET_CAD_PARAMS_BUFS: SetCadParams = SetCadParams::new(CadSymbolNum::CadOn4Symb, 23, 10, CadExitMode::CadOnly, 0);
pub static SET_CAD_PARAMS: SyncSpiDescriptor = SyncSpiDescriptor(SET_CAD_PARAMS_BUFS.descriptor());

static SET_DIO2_AS_RF_SWITCH_CTRL_BUFS: SetDio2AsRfSwitchCtrl = SetDio2AsRfSwitchCtrl::new(true);
pub static SET_DIO2_AS_RF_SWITCH_CTRL: SyncSpiDescriptor =
    SyncSpiDescriptor(SET_DIO2_AS_RF_SWITCH_CTRL_BUFS.descriptor());

static UNMASK_CAD_BUFS: SetDioIrqParams = SetDioIrqParams::new(
    Irq::new().with_cad_done(true).with_cad_detected(true),
    Irq::new().with_cad_done(true),
    Irq::new(),
    Irq::new(),
);
pub static UNMASK_CAD: SyncSpiDescriptor = SyncSpiDescriptor(UNMASK_CAD_BUFS.descriptor());

static UNMASK_TXDONE_BUFS: SetDioIrqParams = SetDioIrqParams::new(
    Irq::new().with_tx_done(true),
    Irq::new().with_tx_done(true),
    Irq::new(),
    Irq::new(),
);
pub static UNMASK_TXDONE: SyncSpiDescriptor = SyncSpiDescriptor(UNMASK_TXDONE_BUFS.descriptor());

static UNMASK_RXDONE_BUFS: SetDioIrqParams = SetDioIrqParams::new(
    Irq::new().with_rx_done(true).with_timeout(true),
    Irq::new().with_rx_done(true),
    Irq::new(),
    Irq::new().with_timeout(true),
);
pub static UNMASK_RXDONE: SyncSpiDescriptor = SyncSpiDescriptor(UNMASK_RXDONE_BUFS.descriptor());

static CLEAR_CAD_BUFS: ClearIrqStatus = ClearIrqStatus::new(Irq::new().with_cad_done(true).with_cad_detected(true));
pub static CLEAR_CAD: SyncSpiDescriptor = SyncSpiDescriptor(CLEAR_CAD_BUFS.descriptor());

static CLEAR_TXDONE_BUFS: ClearIrqStatus = ClearIrqStatus::new(Irq::new().with_tx_done(true));
pub static CLEAR_TXDONE: SyncSpiDescriptor = SyncSpiDescriptor(CLEAR_TXDONE_BUFS.descriptor());

static CLEAR_RXDONE_BUFS: ClearIrqStatus = ClearIrqStatus::new(Irq::new().with_rx_done(true));
pub static CLEAR_RXDONE: SyncSpiDescriptor = SyncSpiDescriptor(CLEAR_RXDONE_BUFS.descriptor());

static CLEAR_TIMEOUT_BUFS: ClearIrqStatus = ClearIrqStatus::new(Irq::new().with_timeout(true));
pub static CLEAR_TIMEOUT: SyncSpiDescriptor = SyncSpiDescriptor(CLEAR_TIMEOUT_BUFS.descriptor());

static SET_CAD_BUFS: SetCad = SetCad::new();
pub static SET_CAD: SyncSpiDescriptor = SyncSpiDescriptor(SET_CAD_BUFS.descriptor());

static SET_TX_BUFS: SetTx = SetTx::new(0);
pub static SET_TX: SyncSpiDescriptor = SyncSpiDescriptor(SET_TX_BUFS.descriptor());

static SET_RX_SYN_BUFS: SetRx = SetRx::new(15360); // 240ms timeout
pub static SET_RX_SYN: SyncSpiDescriptor = SyncSpiDescriptor(SET_RX_SYN_BUFS.descriptor());

static SET_RX_BUFS: SetRx = SetRx::new(7680); // 120ms timeout
pub static SET_RX: SyncSpiDescriptor = SyncSpiDescriptor(SET_RX_BUFS.descriptor());

// Mutable commands
pub static mut PACKET: Packet = Packet::new(crate::ADDRESS);
pub static mut WRITE_PACKET: SyncSpiDescriptor = SyncSpiDescriptor(unsafe { PACKET.cmd.descriptor() });

static mut SET_PACKET_PARAMS_BUFS: SetPacketParams =
    SetPacketParams::new(8, HeaderType::VariableLength, 3, false, InvertIq::Standard);
pub static SET_PACKET_PARAMS: SyncSpiDescriptor = SyncSpiDescriptor(unsafe { SET_PACKET_PARAMS_BUFS.descriptor() });

pub static mut GET_IRQ_STATUS_BUFS: GetIrqStatus = GetIrqStatus::new();
pub static GET_IRQ_STATUS: SyncSpiDescriptor = SyncSpiDescriptor(unsafe { GET_IRQ_STATUS_BUFS.descriptor() });

pub static mut ACK: Ack = Ack::new(0x80);
pub static READ_BUFFER: SyncSpiDescriptor = SyncSpiDescriptor(unsafe { ACK.cmd.descriptor() });

/// Bitmask for packet ID
const PACKET_ID_MASK: u8 = 0b0000_1100;

// A packet to send to the gateway
pub struct Packet {
    /// WriteBuffer command - 2 command bytes + 14 data bytes
    cmd: WriteBuffer<16>,
    /// Last 4 bits of address in the format 0bXXXX_0000
    header_lsbyte: u8,
    packet_id: u8,
    has_battery: bool,
    has_position: bool,
}
impl Packet {
    /// The position of the first payload byte after the header = 2 command bytes + 3 header bytes
    const PAYLOAD_BEGIN: usize = 5;

    #[inline]
    pub const fn new(address: u32) -> Self {
        // The max length of a packet is 14 bytes
        // header + position + battery = 3 + 10 + 1 = 14 bytes
        let mut write_buffer = WriteBuffer::<16>::new(0, [0; 14]);
        write_buffer.tx_buf[2] = ((address & 0xFF000) >> 12) as u8;
        write_buffer.tx_buf[3] = ((address & 0x00FF0) >> 4) as u8;
        write_buffer.tx_buf[4] = ((address & 0x0000F) << 4) as u8;
        Self {
            cmd: write_buffer,
            header_lsbyte: ((address & 0x0000F) << 4) as u8,
            packet_id: 0,
            has_battery: false,
            has_position: false,
        }
    }

    #[inline]
    pub const fn get_packet_id(&self) -> u8 {
        self.packet_id
    }

    /// Reset block flags
    #[inline]
    pub const fn reset_flags(&mut self) {
        self.has_battery = false;
        self.has_position = false;
    }

    /// Insert battery block into packet.
    /// It will always be the first block because of its block precendence.
    #[inline]
    pub const fn insert_battery(&mut self, battery: u8) {
        self.cmd.tx_buf[Self::PAYLOAD_BEGIN] = battery;
        self.has_battery = true;
    }

    /// Insert position block into packet.
    #[inline]
    pub fn insert_position(&mut self, buffer: &[u8; 1024], sentence_begin: usize) {
        let position_block_idx = if self.has_battery {
            Self::PAYLOAD_BEGIN + 1
        } else {
            Self::PAYLOAD_BEGIN
        };
        let position_block = unsafe { &mut *self.cmd.tx_buf.as_mut_ptr().add(position_block_idx).cast::<[u8; 10]>() };
        self.has_position = extract_gga(buffer, sentence_begin, position_block);
    }

    /// 1. Increment packet ID
    /// 2. Update the header in the bytes buffer
    /// 3. Update SetPacketParams
    /// 4. Update WriteBuffer transfer length
    #[inline]
    pub const fn set_header(&mut self) {
        // Increment packet ID
        self.packet_id = (self.packet_id + 0b0000_0100) & PACKET_ID_MASK;

        // Update header LSByte, other bytes don't change
        self.cmd.tx_buf[4] =
            self.header_lsbyte | self.packet_id | (self.has_position as u8) << 1 | self.has_battery as u8;

        // Update SetPacketParams and WriteBuffer transfer length
        let mut payload_length = 3; // Packet header length (min length)
        if self.has_battery {
            payload_length += 1;
        }
        if self.has_position {
            payload_length += 10;
        }
        unsafe {
            SET_PACKET_PARAMS_BUFS.tx_buf[4] = payload_length as u8;
            WRITE_PACKET.0.transfer_length = payload_length + 2; // +2 command bytes
        }
    }
}

/// An ACK received from the gateway
pub struct Ack {
    cmd: ReadBuffer<6>,
}
impl Ack {
    #[inline]
    pub const fn new(offset: u8) -> Self {
        Self {
            cmd: ReadBuffer::new(offset),
        }
    }
    #[inline]
    pub const fn get_address(&self) -> u32 {
        (self.cmd.rx_buf[3] as u32) << 12 | (self.cmd.rx_buf[4] as u32) << 4 | (self.cmd.rx_buf[5] >> 4) as u32
    }
    #[inline]
    pub const fn get_packet_id(&self) -> u8 {
        self.cmd.rx_buf[5] & PACKET_ID_MASK
    }
    #[inline]
    pub const fn get_tx_power_control(&self) -> u8 {
        self.cmd.rx_buf[5] & 3
    }
}
