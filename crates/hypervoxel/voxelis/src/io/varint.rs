use std::io::BufReader;

use byteorder::ReadBytesExt;

pub fn encode_varint_u32(mut value: u32) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(5);

    while value >= 0x80 {
        bytes.push((value as u8 & 0x7F) | 0x80);
        value >>= 7;
    }

    bytes.push(value as u8);

    bytes
}

pub fn encode_varint(mut value: usize) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(8);

    while value >= 0x80 {
        // Set the MSB to indicate more bytes follow
        bytes.push((value as u8 & 0x7F) | 0x80);
        value >>= 7;
    }

    // Last byte with MSB unset
    bytes.push(value as u8);

    bytes
}

pub fn decode_varint_u32(iter: &mut std::slice::Iter<u8>) -> Option<u32> {
    let mut result = 0u32;
    let mut shift = 0;

    loop {
        let byte = *iter.next()?;

        result |= ((byte & 0x7F) as u32) << shift;

        if byte & 0x80 == 0 {
            break;
        }

        shift += 7;

        if shift >= 32 {
            return None;
        }
    }

    Some(result)
}

pub fn decode_varint_u32_from_reader(reader: &mut BufReader<&[u8]>) -> Option<u32> {
    let mut result = 0u32;
    let mut shift = 0;

    loop {
        let byte = reader.read_u8().ok()?;

        result |= ((byte & 0x7F) as u32) << shift;

        if byte & 0x80 == 0 {
            break;
        }

        shift += 7;

        if shift >= 32 {
            return None;
        }
    }

    Some(result)
}

pub fn decode_varint(iter: &mut std::slice::Iter<u8>) -> Option<usize> {
    let mut result = 0usize;
    let mut shift = 0;

    loop {
        let byte = *iter.next()?;

        result |= ((byte & 0x7F) as usize) << shift;

        if byte & 0x80 == 0 {
            break;
        }

        shift += 7;
    }

    Some(result)
}
