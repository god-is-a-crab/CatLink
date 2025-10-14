/// Determine if the sentence is a GGA sentence.
#[inline]
pub fn is_gga(buffer: &[u8; 1024], sentence_begin: usize) -> bool {
    unsafe {
        *buffer.get_unchecked((sentence_begin + 3) & 1023) == b'G'
            && *buffer.get_unchecked((sentence_begin + 4) & 1023) == b'G'
            && *buffer.get_unchecked((sentence_begin + 5) & 1023) == b'A'
    }
}

#[inline]
pub fn calculate_sentence_length(ndtr: u16, sentence_begin: usize) -> usize {
    let ndtr: usize = ndtr as usize;
    if sentence_begin + ndtr < 1024 {
        1024 - ndtr - sentence_begin
    } else {
        (1024 - sentence_begin) + (1024 - ndtr)
    }
}

const POW10_10_DIGITS: [u32; 10] = [
    1000000000, 100000000, 10000000, 1000000, 100000, 10000, 1000, 100, 10, 1,
];

/// Extract position data from a GGA (Global Positioning System Fix Data) sentence in a circular buffer.
///
/// ### Arguments
/// * `buffer` - A 1024-byte circular buffer containing NMEA 0183 data.
/// * `sentence_begin` - Starting index of the GGA sentence in the buffer.
/// * `position_block` - Output buffer where parsed position data will be stored (10 bytes).
///
/// ### Returns
/// If the sentence contains a GNSS fix.
#[inline]
pub fn extract_gga(buffer: &[u8; 1024], sentence_begin: usize, position_block: &mut [u8; 10]) -> bool {
    unsafe {
        // Check time field
        if *buffer.get_unchecked((sentence_begin + 7) & 1023) == b',' {
            // No time field, assume no fix
            return false;
        }

        // Check latitude field
        if *buffer.get_unchecked((sentence_begin + 18) & 1023) == b',' {
            // No latitude field, no fix
            return false;
        }

        // Parse latitude
        let mut lat: u32 = 0;
        lat += ((*buffer.get_unchecked((sentence_begin + 18) & 1023) - b'0') as u32) * POW10_10_DIGITS[0];
        lat += ((*buffer.get_unchecked((sentence_begin + 19) & 1023) - b'0') as u32) * POW10_10_DIGITS[1];
        lat += ((*buffer.get_unchecked((sentence_begin + 20) & 1023) - b'0') as u32) * POW10_10_DIGITS[2];
        lat += ((*buffer.get_unchecked((sentence_begin + 21) & 1023) - b'0') as u32) * POW10_10_DIGITS[3];
        // Skip decimal point
        lat += ((*buffer.get_unchecked((sentence_begin + 23) & 1023) - b'0') as u32) * POW10_10_DIGITS[4];
        lat += ((*buffer.get_unchecked((sentence_begin + 24) & 1023) - b'0') as u32) * POW10_10_DIGITS[5];
        lat += ((*buffer.get_unchecked((sentence_begin + 25) & 1023) - b'0') as u32) * POW10_10_DIGITS[6];
        lat += ((*buffer.get_unchecked((sentence_begin + 26) & 1023) - b'0') as u32) * POW10_10_DIGITS[7];
        lat += ((*buffer.get_unchecked((sentence_begin + 27) & 1023) - b'0') as u32) * POW10_10_DIGITS[8];

        position_block[0] = (lat >> 24) as u8;
        position_block[1] = (lat >> 16) as u8;
        position_block[2] = (lat >> 8) as u8;
        position_block[3] = lat as u8;

        // Latitude hemisphere
        position_block[8] = ((*buffer.get_unchecked((sentence_begin + 29) & 1023) == b'N') as u8) << 1;

        // Parse longitude
        let mut lon: u32 = 0;
        lon += ((*buffer.get_unchecked((sentence_begin + 31) & 1023) - b'0') as u32) * POW10_10_DIGITS[0];
        lon += ((*buffer.get_unchecked((sentence_begin + 32) & 1023) - b'0') as u32) * POW10_10_DIGITS[1];
        lon += ((*buffer.get_unchecked((sentence_begin + 33) & 1023) - b'0') as u32) * POW10_10_DIGITS[2];
        lon += ((*buffer.get_unchecked((sentence_begin + 34) & 1023) - b'0') as u32) * POW10_10_DIGITS[3];
        lon += ((*buffer.get_unchecked((sentence_begin + 35) & 1023) - b'0') as u32) * POW10_10_DIGITS[4];
        // Skip decimal point
        lon += ((*buffer.get_unchecked((sentence_begin + 37) & 1023) - b'0') as u32) * POW10_10_DIGITS[5];
        lon += ((*buffer.get_unchecked((sentence_begin + 38) & 1023) - b'0') as u32) * POW10_10_DIGITS[6];
        lon += ((*buffer.get_unchecked((sentence_begin + 39) & 1023) - b'0') as u32) * POW10_10_DIGITS[7];
        lon += ((*buffer.get_unchecked((sentence_begin + 40) & 1023) - b'0') as u32) * POW10_10_DIGITS[8];
        lon += ((*buffer.get_unchecked((sentence_begin + 41) & 1023) - b'0') as u32) * POW10_10_DIGITS[9];

        position_block[4] = (lon >> 24) as u8;
        position_block[5] = (lon >> 16) as u8;
        position_block[6] = (lon >> 8) as u8;
        position_block[7] = lon as u8;

        // Longitude hemisphere
        position_block[8] |= (*buffer.get_unchecked((sentence_begin + 43) & 1023) == b'E') as u8;

        // Parse hdop
        if buffer[(sentence_begin + 51) & 1023] == b'.' {
            let mut hdop: u8 = 0;
            // Integer part is single digit
            hdop += (*buffer.get_unchecked((sentence_begin + 50) & 1023) - b'0') * 10;
            // Skip decimal point
            hdop += *buffer.get_unchecked((sentence_begin + 52) & 1023) - b'0';
            position_block[9] = hdop;
        } else {
            const ASCII_OFFSET: u16 = b'0' as u16;
            let mut hdop: u16 = 0;
            // Integer part is double digit
            hdop += (*buffer.get_unchecked((sentence_begin + 50) & 1023) as u16 - ASCII_OFFSET) * 100;
            hdop += (*buffer.get_unchecked((sentence_begin + 51) & 1023) as u16 - ASCII_OFFSET) * 10;
            // Skip decimal point
            hdop += *buffer.get_unchecked((sentence_begin + 53) & 1023) as u16 - ASCII_OFFSET;
            position_block[9] = if hdop < 256 { hdop as u8 } else { 255 };
        }
    }
    true
}
