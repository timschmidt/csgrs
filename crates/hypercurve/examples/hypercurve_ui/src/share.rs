use serde::{Serialize, de::DeserializeOwned};

const STATE_PARAM: &str = "state";

/// Serialize application state into a URL-safe percent-encoded JSON payload.
///
/// The demo state is deliberately plain JSON before encoding so shared links
/// remain debuggable while still being safe inside one query parameter.
pub fn encode_state<T: Serialize>(state: &T) -> Result<String, String> {
    let json = serde_json::to_string(state)
        .map_err(|error| format!("failed to serialize state: {error}"))?;
    Ok(percent_encode(json.as_bytes()))
}

/// Decode a URL-safe state payload produced by [`encode_state`].
pub fn decode_state<T: DeserializeOwned>(encoded: &str) -> Result<T, String> {
    let bytes = percent_decode(encoded)?;
    let json = String::from_utf8(bytes).map_err(|error| format!("state was not UTF-8: {error}"))?;
    serde_json::from_str(&json).map_err(|error| format!("failed to parse state: {error}"))
}

/// Load the optional `state` query parameter from the current browser URL.
#[cfg(target_arch = "wasm32")]
pub fn load_from_location<T: DeserializeOwned>() -> Result<Option<T>, String> {
    let Some(href) = current_href() else {
        return Ok(None);
    };
    let Some(encoded) = extract_state_param(&href) else {
        return Ok(None);
    };
    decode_state(encoded).map(Some)
}

/// Build a copyable URL for the current browser location and serialized state.
#[cfg(target_arch = "wasm32")]
pub fn share_url<T: Serialize>(state: &T) -> Result<String, String> {
    let href = current_href().ok_or("browser location is unavailable")?;
    Ok(set_state_param(&href, &encode_state(state)?))
}

#[cfg(target_arch = "wasm32")]
fn current_href() -> Option<String> {
    web_sys::window()?.location().href().ok()
}

fn extract_state_param(href: &str) -> Option<&str> {
    let before_hash = href.split_once('#').map_or(href, |(before, _)| before);
    let query = before_hash.split_once('?')?.1;
    query.split('&').find_map(|pair| {
        let (key, value) = pair.split_once('=').unwrap_or((pair, ""));
        (key == STATE_PARAM).then_some(value)
    })
}

fn set_state_param(href: &str, encoded_state: &str) -> String {
    let (before_hash, hash) = href
        .split_once('#')
        .map_or((href, ""), |(before, hash)| (before, hash));
    let (base, query) = before_hash
        .split_once('?')
        .map_or((before_hash, ""), |(base, query)| (base, query));
    let mut params: Vec<String> = query
        .split('&')
        .filter(|pair| {
            if pair.is_empty() {
                return false;
            }
            let (key, _) = pair.split_once('=').unwrap_or((*pair, ""));
            key != STATE_PARAM
        })
        .map(str::to_owned)
        .collect();
    params.push(format!("{STATE_PARAM}={encoded_state}"));

    let mut url = String::from(base);
    url.push('?');
    url.push_str(&params.join("&"));
    if !hash.is_empty() {
        url.push('#');
        url.push_str(hash);
    }
    url
}

fn percent_encode(bytes: &[u8]) -> String {
    let mut encoded = String::with_capacity(bytes.len());
    for &byte in bytes {
        if is_unreserved(byte) {
            encoded.push(byte as char);
        } else {
            encoded.push('%');
            encoded.push(hex_digit(byte >> 4));
            encoded.push(hex_digit(byte & 0x0f));
        }
    }
    encoded
}

fn percent_decode(encoded: &str) -> Result<Vec<u8>, String> {
    let mut decoded = Vec::with_capacity(encoded.len());
    let bytes = encoded.as_bytes();
    let mut index = 0;
    while index < bytes.len() {
        match bytes[index] {
            b'%' => {
                let high = bytes
                    .get(index + 1)
                    .copied()
                    .and_then(hex_value)
                    .ok_or("state contains an invalid percent escape")?;
                let low = bytes
                    .get(index + 2)
                    .copied()
                    .and_then(hex_value)
                    .ok_or("state contains an invalid percent escape")?;
                decoded.push((high << 4) | low);
                index += 3;
            }
            byte => {
                decoded.push(byte);
                index += 1;
            }
        }
    }
    Ok(decoded)
}

fn is_unreserved(byte: u8) -> bool {
    byte.is_ascii_alphanumeric() || matches!(byte, b'-' | b'.' | b'_' | b'~')
}

fn hex_digit(value: u8) -> char {
    match value {
        0..=9 => (b'0' + value) as char,
        10..=15 => (b'A' + value - 10) as char,
        _ => unreachable!("hex nibble must be in range"),
    }
}

fn hex_value(byte: u8) -> Option<u8> {
    match byte {
        b'0'..=b'9' => Some(byte - b'0'),
        b'a'..=b'f' => Some(byte - b'a' + 10),
        b'A'..=b'F' => Some(byte - b'A' + 10),
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use serde::{Deserialize, Serialize};

    use super::*;

    #[derive(Debug, Eq, PartialEq, Serialize, Deserialize)]
    struct RoundTrip {
        label: String,
        values: Vec<i32>,
    }

    #[test]
    fn state_round_trips_through_url_safe_encoding() {
        let state = RoundTrip {
            label: "curves & holes".to_owned(),
            values: vec![1, 2, 3],
        };
        let encoded = encode_state(&state).unwrap();
        assert!(!encoded.contains('&'));
        assert!(!encoded.contains('#'));
        assert_eq!(decode_state::<RoundTrip>(&encoded).unwrap(), state);
    }

    #[test]
    fn share_url_replaces_existing_state_and_keeps_hash() {
        let encoded = encode_state(&RoundTrip {
            label: "new".to_owned(),
            values: vec![9],
        })
        .unwrap();
        let url = set_state_param("https://example.test/demo?x=1&state=old#plot", &encoded);
        assert!(url.starts_with("https://example.test/demo?x=1&state="));
        assert!(url.ends_with("#plot"));
        assert_eq!(extract_state_param(&url), Some(encoded.as_str()));
    }
}
