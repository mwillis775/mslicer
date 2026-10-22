//! Live view of a printer's camera. The stream (RTSP H.264 on Elegoo printers)
//! is decoded by an `ffmpeg` child process into raw RGBA frames.

use std::{
    io::{ErrorKind, Read},
    process::{Child, Command, Stdio},
    sync::Arc,
    thread,
};

use egui::{ColorImage, Context, TextureHandle, TextureOptions};
use parking_lot::Mutex;
use tracing::{info, warn};

const WIDTH: usize = 960;
const HEIGHT: usize = 540;
const FPS: u32 = 10;

pub struct PrinterCamera {
    pub url: String,
    child: Child,
    shared: Arc<Mutex<Shared>>,
    texture: Option<TextureHandle>,
}

#[derive(Default)]
struct Shared {
    frame: Option<ColorImage>,
    has_frame: bool,
    error: Option<String>,
}

impl PrinterCamera {
    pub fn start(url: &str) -> Result<Self, String> {
        info!("Starting camera stream from {url}");
        let mut command = Command::new("ffmpeg");
        command.args([
            "-loglevel",
            "quiet",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
        ]);
        if url.starts_with("rtsp://") {
            // The printers reject RTSP over TCP.
            command.args(["-rtsp_transport", "udp"]);
        }
        let filter = format!("fps={FPS},scale={WIDTH}:{HEIGHT}");
        command
            .args(["-i", url, "-an", "-vf", &filter])
            .args(["-pix_fmt", "rgba", "-f", "rawvideo", "pipe:1"])
            .stdin(Stdio::null())
            .stdout(Stdio::piped())
            .stderr(Stdio::null());

        let mut child = command.spawn().map_err(|e| match e.kind() {
            ErrorKind::NotFound => "ffmpeg is not installed.".to_owned(),
            _ => format!("Failed to start ffmpeg: {e}"),
        })?;

        let shared = Arc::new(Mutex::new(Shared::default()));
        let mut stdout = child.stdout.take().unwrap();
        let thread_shared = shared.clone();
        thread::spawn(move || {
            let mut buffer = vec![0; WIDTH * HEIGHT * 4];
            loop {
                if let Err(e) = stdout.read_exact(&mut buffer) {
                    if e.kind() != ErrorKind::UnexpectedEof {
                        warn!("Camera stream read failed: {e}");
                    }
                    thread_shared.lock().error = Some("Camera stream ended.".into());
                    break;
                }

                let image = ColorImage::from_rgba_unmultiplied([WIDTH, HEIGHT], &buffer);
                let mut shared = thread_shared.lock();
                shared.frame = Some(image);
                shared.has_frame = true;
            }
        });

        Ok(Self {
            url: url.to_owned(),
            child,
            shared,
            texture: None,
        })
    }

    pub fn error(&self) -> Option<String> {
        self.shared.lock().error.clone()
    }

    pub fn has_frame(&self) -> bool {
        self.shared.lock().has_frame
    }

    pub fn aspect(&self) -> f32 {
        WIDTH as f32 / HEIGHT as f32
    }

    /// Uploads the newest decoded frame (if any) and returns the texture.
    pub fn texture(&mut self, ctx: &Context) -> Option<&TextureHandle> {
        if let Some(frame) = self.shared.lock().frame.take() {
            match &mut self.texture {
                Some(texture) => texture.set(frame, TextureOptions::LINEAR),
                None => {
                    let texture = ctx.load_texture("printer_camera", frame, TextureOptions::LINEAR);
                    self.texture = Some(texture);
                }
            }
        }

        self.texture.as_ref()
    }
}

impl Drop for PrinterCamera {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

pub enum CameraState {
    /// The printer has been asked to start streaming but hasn't sent a URL yet.
    WaitingForUrl,
    Streaming(PrinterCamera),
    Failed(String),
}
