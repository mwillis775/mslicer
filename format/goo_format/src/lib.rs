//! Elegoo format V3.0 (`.goo`).
//!
//! ## References
//!
//! - [Official Format Spec](https://github.com/elegooofficial/GOO)

mod encoding;
mod file;
mod header;
mod layer;
mod preview;

pub use encoding::{LayerDecoder, LayerEncoder};
pub use file::File;
pub use header::{ExposureDelayMode, Header};
pub use layer::Layer;
pub use preview::PreviewImage;

const ENDING_STRING: &[u8] = &[
    0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x44, 0x4C, 0x50, 0x00,
];
const MAGIC_TAG: &[u8] = &[0x07, 0x00, 0x00, 0x00, 0x44, 0x4C, 0x50, 0x00];
const DELIMITER: &[u8] = &[0xD, 0xA];

/// Placeholder written to every lift/retract distance and speed field when the
/// printer firmware controls motion itself (matches Elegoo's own files).
const FIRMWARE_MOTION_PLACEHOLDER: f32 = 0.05;

/// The Saturn 4 Ultra 16K separates layers with a firmware-driven tilting vat.
/// Elegoo/ChiTuBox files for it disable per-layer settings and only carry
/// placeholder lift values; with per-layer motion enabled the plate barely
/// rises between layers and prints come out flattened.
fn firmware_controls_motion(config: &common::slice::SliceConfig) -> bool {
    config.platform_resolution == nalgebra::Vector2::new(15120, 6230)
}
