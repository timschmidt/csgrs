use std::io::{Read, Write};

use base64::Engine;
use base64::engine::general_purpose::URL_SAFE_NO_PAD;
use lzma_rust2::{XzOptions, XzReader, XzWriter};
use serde::{Serialize, de::DeserializeOwned};

const STATE_PARAM: &str = "state";
const LZMA_PRESET: u32 = 9;
const MAX_ENCODED_STATE_CHARS: usize = 1_048_576;
const MAX_DECOMPRESSED_STATE_BYTES: u64 = 2 * 1024 * 1024;

/// Serialize application state into base64url-encoded LZMA-compressed JSON.
///
/// The payload uses an XZ/LZMA2 stream at preset 9, then RFC 4648 section 5's
/// URL-safe base64 alphabet without padding. The state remains structured JSON
/// before compression, while repeated field names and coordinate arrays shrink
/// aggressively for share links.
pub fn encode_state<T: Serialize>(state: &T) -> Result<String, String> {
    let json = serde_json::to_string(state)
        .map_err(|error| format!("failed to serialize state: {error}"))?;
    let compressed = compress_json(json.as_bytes())?;
    Ok(URL_SAFE_NO_PAD.encode(compressed))
}

/// Decode a URL-safe state payload produced by [`encode_state`].
pub fn decode_state<T: DeserializeOwned>(encoded: &str) -> Result<T, String> {
    if encoded.len() > MAX_ENCODED_STATE_CHARS {
        return Err(format!(
            "state is too large; encoded state is limited to {MAX_ENCODED_STATE_CHARS} characters"
        ));
    }
    let compressed = URL_SAFE_NO_PAD
        .decode(encoded)
        .map_err(|error| format!("state was not URL-safe base64: {error}"))?;
    let bytes = decompress_json(&compressed)?;
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

fn compress_json(json: &[u8]) -> Result<Vec<u8>, String> {
    let mut writer = XzWriter::new(Vec::new(), XzOptions::with_preset(LZMA_PRESET))
        .map_err(|error| format!("failed to start LZMA compressor: {error}"))?;
    writer
        .write_all(json)
        .map_err(|error| format!("failed to compress state: {error}"))?;
    writer
        .finish()
        .map_err(|error| format!("failed to finish compressed state: {error}"))
}

fn decompress_json(compressed: &[u8]) -> Result<Vec<u8>, String> {
    let reader = XzReader::new(compressed, false);
    let mut limited = reader.take(MAX_DECOMPRESSED_STATE_BYTES + 1);
    let mut decompressed = Vec::new();
    limited
        .read_to_end(&mut decompressed)
        .map_err(|error| format!("failed to decompress state: {error}"))?;
    if decompressed.len() as u64 > MAX_DECOMPRESSED_STATE_BYTES {
        return Err(format!(
            "state expands beyond {MAX_DECOMPRESSED_STATE_BYTES} bytes"
        ));
    }
    Ok(decompressed)
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
        assert!(!encoded.contains('%'));
        assert!(!encoded.contains('='));
        assert_eq!(decode_state::<RoundTrip>(&encoded).unwrap(), state);
    }

    #[test]
    fn invalid_base64_state_is_rejected() {
        assert!(decode_state::<RoundTrip>("not+url/safe").is_err());
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
