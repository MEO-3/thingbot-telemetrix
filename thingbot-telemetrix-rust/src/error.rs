use std::fmt;

pub type Result<T> = std::result::Result<T, Error>;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    EmptyPacket,
    InvalidPacketLength {
        declared: u8,
        actual: usize,
    },
    UnknownReport(u8),
    InvalidReportLength {
        report_id: u8,
        expected: &'static str,
        actual: usize,
    },
    ValueOutOfRange {
        field: &'static str,
        min: i32,
        max: i32,
        actual: i32,
    },
    Transport(String),
}

impl From<std::io::Error> for Error {
    fn from(error: std::io::Error) -> Self {
        Self::Transport(error.to_string())
    }
}

impl From<serialport::Error> for Error {
    fn from(error: serialport::Error) -> Self {
        Self::Transport(error.to_string())
    }
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyPacket => write!(f, "packet is empty"),
            Self::InvalidPacketLength { declared, actual } => {
                write!(
                    f,
                    "invalid packet length: declared {declared}, actual payload {actual}"
                )
            }
            Self::UnknownReport(report_id) => write!(f, "unknown report id {report_id}"),
            Self::InvalidReportLength {
                report_id,
                expected,
                actual,
            } => write!(
                f,
                "invalid report length for report {report_id}: expected {expected}, actual {actual}"
            ),
            Self::ValueOutOfRange {
                field,
                min,
                max,
                actual,
            } => write!(
                f,
                "{field} out of range: expected {min}..={max}, actual {actual}"
            ),
            Self::Transport(message) => write!(f, "transport error: {message}"),
        }
    }
}

impl std::error::Error for Error {}
