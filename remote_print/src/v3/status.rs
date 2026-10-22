use serde::{Deserialize, Deserializer, Serialize};
use serde_json::Value;

use crate::shared::{PrintInfo, Response};

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct Message {
    #[serde(rename = "MainboardID")]
    pub mainboard_id: String,
    pub time_stamp: u64,
    pub topic: String,

    pub attributes: Option<Attributes>,
    pub status: Option<Status>,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct DiscoveryResponse {
    pub name: String,
    pub machine_name: String,
    pub brand_name: String,
    #[serde(rename = "MainboardIP")]
    pub mainboard_ip: String,
    #[serde(rename = "MainboardID")]
    pub mainboard_id: String,
    pub protocol_version: String,
    pub firmware_version: String,

    #[serde(default)]
    pub proxy: bool,
}

/// Sent by the elegoo-homeassistant proxy server for legacy Saturn support
#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct ProxyDiscoveryResponse {
    pub attributes: ProxyDiscoveryAttributes,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct ProxyDiscoveryAttributes {
    pub name: String,
    pub machine_name: String,
    pub brand_name: String,
    #[serde(rename = "MainboardIP")]
    pub mainboard_ip: String,
    #[serde(rename = "MainboardID")]
    pub mainboard_id: String,
    pub protocol_version: String,
    pub firmware_version: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct Attributes {
    pub name: String,
    pub machine_name: String,
    pub brand_name: String,
    pub protocol_version: String,
    pub firmware_version: String,
    pub capabilities: Vec<String>,
    pub support_file_type: Vec<String>,
    pub resolution: String,
    #[serde(rename = "XYZsize")]
    pub xyz_size: String,
    #[serde(rename = "MainboardIP")]
    pub mainboard_ip: String,
    #[serde(rename = "MainboardID")]
    pub mainboard_id: String,
}

impl Attributes {
    pub fn has_camera(&self) -> bool {
        self.capabilities.iter().any(|x| x == "VIDEO_STREAM")
    }

    pub fn supports_goo(&self) -> bool {
        (self.support_file_type.iter()).any(|x| x.eq_ignore_ascii_case("goo"))
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct Status {
    pub current_status: Vec<CurrentStatus>,
    pub print_info: PrintInfo,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub enum CurrentStatus {
    Idle,
    Printing,
    FileTransferring,
    ExposureTesting,
    DevicesTesting,
    /// The Saturn 4 Ultra briefly reports 8 at the end of a file transfer.
    Unknown(u8),
}

impl<'de> Deserialize<'de> for CurrentStatus {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        Ok(match u8::deserialize(deserializer)? {
            0 => Self::Idle,
            1 => Self::Printing,
            2 => Self::FileTransferring,
            3 => Self::ExposureTesting,
            4 => Self::DevicesTesting,
            x => Self::Unknown(x),
        })
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct Command<T> {
    pub id: String,
    pub data: CommandData<T>,
    pub topic: String,
}

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(rename_all = "PascalCase")]
pub struct CommandData<T> {
    pub cmd: u16,
    pub data: T,
    #[serde(rename = "RequestID")]
    pub request_id: String,
    #[serde(rename = "MainboardID")]
    pub mainboard_id: String,
    pub time_stamp: i64,
    pub from: u8,
}

/// Response to `sdcp/response/{MainboardID}`
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct ResponseMessage {
    pub data: ResponseData,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct ResponseData {
    pub cmd: u16,
    pub data: AckData,
    #[serde(rename = "MainboardID")]
    pub mainboard_id: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct AckData {
    #[serde(default)]
    pub ack: u8,
    /// Only sent in response to the video stream command.
    #[serde(rename = "VideoUrl")]
    pub video_url: Option<String>,
}

/// Response to `sdcp/error/{MainboardID}`
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct ErrorMessage {
    pub data: ErrorEnvelopeData,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct ErrorEnvelopeData {
    pub data: ErrorData,
    #[serde(rename = "MainboardID")]
    pub mainboard_id: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct ErrorData {
    pub error_code: Value,
}

/// Response to `sdcp/notice/{MainboardID}`
#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct NoticeMessage {
    pub data: NoticeEnvelopeData,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct NoticeEnvelopeData {
    pub data: NoticeData,
    #[serde(rename = "MainboardID")]
    pub mainboard_id: String,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "PascalCase")]
pub struct NoticeData {
    #[serde(rename = "Type")]
    pub notice_type: u8,
    pub message: Value,
}

/// Response to `POST /uploadFile/upload`
#[derive(Debug, Clone, Deserialize)]
pub struct UploadResponse {
    pub success: bool,
    #[serde(default)]
    pub messages: Value,
}

impl From<ProxyDiscoveryResponse> for DiscoveryResponse {
    fn from(proxy: ProxyDiscoveryResponse) -> Self {
        let attributes = proxy.attributes;
        DiscoveryResponse {
            name: attributes.name,
            machine_name: attributes.machine_name,
            brand_name: attributes.brand_name,
            mainboard_ip: attributes.mainboard_ip,
            mainboard_id: attributes.mainboard_id,
            protocol_version: attributes.protocol_version,
            firmware_version: attributes.firmware_version,
            proxy: true,
        }
    }
}

impl From<Response<ProxyDiscoveryResponse>> for Response<DiscoveryResponse> {
    fn from(response: Response<ProxyDiscoveryResponse>) -> Self {
        Response {
            id: response.id,
            data: response.data.into(),
        }
    }
}
