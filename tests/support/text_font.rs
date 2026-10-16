//! License-clean synthetic outlines: a 100 x 100 square, a quadratic arch,
//! blank glyphs, and an intentionally incomplete glyph. No external font data.
#![allow(dead_code)]

fn put16(bytes: &mut [u8], offset: usize, value: u16) {
    bytes[offset..offset + 2].copy_from_slice(&value.to_be_bytes());
}
fn put32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_be_bytes());
}

fn outline(points: &[(i16, i16, bool)]) -> Vec<u8> {
    let mut bytes = vec![0; 14];
    put16(&mut bytes, 0, 1); // one contour
    put16(&mut bytes, 6, 100);
    put16(&mut bytes, 8, 100);
    put16(&mut bytes, 10, points.len() as u16 - 1);
    bytes.extend(points.iter().map(|point| u8::from(point.2)));
    for axis in 0..2 {
        let mut previous = 0;
        for &(x, y, _) in points {
            let value = if axis == 0 { x } else { y };
            bytes.extend_from_slice(&(value - previous).to_be_bytes());
            previous = value;
        }
    }
    bytes
}

/// R = square, Q = quadratic arch, space/B = blank, X = malformed.
/// The optional GPOS class-zero adjustment covers only R and moves by -200.
pub fn font(with_gpos: bool) -> Vec<u8> {
    let mut head = vec![0; 54];
    put32(&mut head, 0, 0x0001_0000);
    put32(&mut head, 12, 0x5f0f_3cf5);
    put16(&mut head, 18, 1000);
    put16(&mut head, 50, 1); // long loca
    let mut hhea = vec![0; 36];
    put32(&mut hhea, 0, 0x0001_0000);
    put16(&mut hhea, 4, 1000);
    put16(&mut hhea, 34, 5);
    let mut maxp = vec![0; 32];
    put32(&mut maxp, 0, 0x0001_0000);
    put16(&mut maxp, 4, 5);
    let mut hmtx = vec![0; 20];
    for i in 0..5 {
        put16(&mut hmtx, i * 4, 1000);
    }
    let mut cmap = vec![0; 274];
    put16(&mut cmap, 2, 1);
    put16(&mut cmap, 4, 0); // Unicode platform
    put32(&mut cmap, 8, 12);
    put16(&mut cmap, 14, 262); // format zero subtable length
    for (character, glyph) in [(b'R', 1), (b'Q', 2), (b' ', 3), (b'B', 3), (b'X', 4)] {
        cmap[18 + usize::from(character)] = glyph;
    }
    let square = outline(&[
        (0, 0, true),
        (100, 0, true),
        (100, 100, true),
        (0, 100, true),
    ]);
    let arch = outline(&[(0, 0, true), (50, 100, false), (100, 0, true)]);
    let mut glyf = square.clone();
    glyf.extend_from_slice(&arch);
    let end = glyf.len() as u32;
    // A contour header whose point data is missing must not become blank.
    glyf.extend_from_slice(&[0, 1, 0, 0, 0, 0, 0, 100, 0, 100]);
    let mut loca = Vec::new();
    for value in [0, 0, square.len() as u32, end, end, glyf.len() as u32] {
        loca.extend_from_slice(&value.to_be_bytes());
    }
    let mut tables = vec![
        (*b"head", head),
        (*b"hhea", hhea),
        (*b"maxp", maxp),
        (*b"hmtx", hmtx),
        (*b"cmap", cmap),
        (*b"glyf", glyf),
        (*b"loca", loca),
    ];
    if with_gpos {
        tables.push((*b"GPOS", gpos()));
    }
    sfnt(tables)
}

fn gpos() -> Vec<u8> {
    let mut bytes = Vec::new();
    // GPOS 1.0, empty Script/Feature lists, one Pair lookup at offset 18.
    for value in [1_u16, 0, 10, 12, 14, 0, 0, 1, 4, 2, 0, 1, 8] {
        bytes.extend_from_slice(&value.to_be_bytes());
    }
    // Format 2, coverage at 18, xAdvance in value1, class defs at 24/28,
    // 1x1 class-zero matrix containing -200. Coverage contains glyph R only.
    for value in [
        2_u16,
        18,
        4,
        0,
        24,
        28,
        1,
        1,
        (-200_i16) as u16,
        1,
        1,
        1,
        2,
        0,
        2,
        0,
    ] {
        bytes.extend_from_slice(&value.to_be_bytes());
    }
    bytes
}

fn sfnt(mut tables: Vec<([u8; 4], Vec<u8>)>) -> Vec<u8> {
    tables.sort_by_key(|entry| entry.0);
    let mut bytes = vec![0; 12 + tables.len() * 16];
    put32(&mut bytes, 0, 0x0001_0000);
    put16(&mut bytes, 4, tables.len() as u16);
    for (i, (tag, data)) in tables.into_iter().enumerate() {
        while !bytes.len().is_multiple_of(4) {
            bytes.push(0);
        }
        let offset = bytes.len();
        let row = 12 + 16 * i;
        bytes[row..row + 4].copy_from_slice(&tag);
        put32(&mut bytes, row + 8, offset as u32);
        put32(&mut bytes, row + 12, data.len() as u32);
        bytes.extend_from_slice(&data);
    }
    bytes
}

/// Corrupt a declared table length, retaining its presence in the directory.
pub fn truncate_table(bytes: &mut [u8], tag: [u8; 4], length: u32) {
    let count = u16::from_be_bytes([bytes[4], bytes[5]]) as usize;
    let row = (0..count)
        .map(|i| 12 + i * 16)
        .find(|&i| bytes[i..i + 4] == tag)
        .unwrap();
    put32(bytes, row + 12, length);
}
