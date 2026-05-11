//! Matrix-vector transform support lives in [`super::core`].
//!
//! This covers ordinary `Matrix * Vector` dispatch, reusable transform handles,
//! vector batches, and the `Matrix4` point/direction fast paths that preserve
//! symbolic structure before approximation.
