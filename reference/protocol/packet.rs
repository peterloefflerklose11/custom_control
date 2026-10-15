use std::io;

use super::error::Error;

use bytes::{BufMut, Bytes, BytesMut};
use tokio::io::{AsyncRead, AsyncReadExt, AsyncWrite, AsyncWriteExt};

pub const SUPPORTED_BAUD_RATES: &[u32] =
    &[1000000, 500000, 250000, 128000, 115200, 76800, 57600, 38400];

const HEADER: [u8; 2] = [0xFF, 0xFF];
pub const BROADCAST_ID: u8 = 0xFE;

#[allow(dead_code)]
pub enum ST3215Request {
    Ping { motor: u8 },
    Read { motor: u8, address: u8, length: u8 },
    Write { motor: u8, address: u8, data: Bytes },
    RegWrite { motor: u8, address: u8, data: Bytes },
    Action { motor: u8 },
    Reset { motor: u8 },
    SyncWrite { address: u8, data: Vec<(u8, Bytes)> },
}

impl ST3215Request {
    fn command(&self) -> u8 {
        match self {
            Self::Ping { .. } => 0x01,
            Self::Read { .. } => 0x02,
            Self::Write { .. } => 0x03,
            Self::RegWrite { .. } => 0x04,
            Self::Action { .. } => 0x05,
            Self::Reset { .. } => 0x06,
            Self::SyncWrite { .. } => 0x83,
        }
    }

    fn data_len(&self) -> u8 {
        match self {
            Self::Ping { .. } | Self::Action { .. } | Self::Reset { .. } => 0,
            Self::Read { .. } => 2,
            Self::Write { data, .. } | Self::RegWrite { data, .. } => data.len() as u8 + 1,
            Self::SyncWrite { data, .. } => {
                if data.is_empty() {
                    0
                } else {
                    let data_len = data[0].1.len() as u8;
                    2 + (data_len + 1) * (data.len() as u8)  // address + data_len + motor_data
                }
            }
        }
    }

    fn motor(&self) -> u8 {
        match self {
            Self::Ping { motor }
            | Self::Read { motor, .. }
            | Self::Write { motor, .. }
            | Self::RegWrite { motor, .. }
            | Self::Action { motor }
            | Self::Reset { motor } => *motor,
            Self::SyncWrite { .. } => BROADCAST_ID,
        }
    }

    pub fn to_bytes(&self) -> Bytes {
        let data_length = self.data_len();
        let mut packet = BytesMut::with_capacity(data_length as usize + 6);

        packet.extend_from_slice(&HEADER);
        packet.put_u8(self.motor());
        packet.put_u8(data_length + 2);
        packet.put_u8(self.command());

        match self {
            Self::Ping { .. } | Self::Action { .. } | Self::Reset { .. } => {}
            Self::Read {
                address, length, ..
            } => {
                packet.put_u8(*address);
                packet.put_u8(*length);
            }
            Self::Write { address, data, .. } | Self::RegWrite { address, data, .. } => {
                packet.put_u8(*address);
                packet.extend_from_slice(data);
            }
            Self::SyncWrite { address, data } => {
                if !data.is_empty() {
                    let data_len = data[0].1.len() as u8;
                    packet.put_u8(*address);
                    packet.put_u8(data_len);
                    for (id, d) in data {
                        packet.put_u8(*id);
                        packet.extend_from_slice(d);
                    }
                }
            }
        }

        let checksum: u8 = calculate_checksum(&packet[2..]);
        packet.put_u8(checksum);
        packet.freeze()
    }

    pub async fn async_write<W: AsyncWrite + Unpin>(
        &self,
        writer: &mut W,
        timeout_ms: u64,
    ) -> Result<(), Error> {
        let packet = self.to_bytes();

        tokio::time::timeout(std::time::Duration::from_millis(timeout_ms), async {
            writer.write_all(&packet).await?;
            writer.flush().await
        })
        .await
        .map_err(|_| Error::Timeout {
            source_packet: packet.clone(),
            reply_packet: Bytes::new(),
        })?
        .map_err(|e| Error::Io {
            error: e,
            source_packet: packet.clone(),
        })
    }

    pub async fn async_readwrite<RW: AsyncRead + AsyncWrite + Unpin>(
        &self,
        stream: &mut RW,
        timeout_ms: u64,
    ) -> Result<ST3215Response, Error> {
        // Write the request
        self.async_write(stream, timeout_ms).await?;

        // Read the response
        let response = ST3215Response::async_read(self, stream, timeout_ms).await?;

        // Validate response type matches request type
        let source_packet = self.to_bytes();
        let response_bytes = match &response {
            ST3215Response::Ping { source_bytes }
            | ST3215Response::Read { source_bytes, .. }
            | ST3215Response::Write { source_bytes }
            | ST3215Response::RegWrite { source_bytes }
            | ST3215Response::Action { source_bytes }
            | ST3215Response::Reset { source_bytes }
            | ST3215Response::SyncWrite { source_bytes } => source_bytes.clone(),
        };

        match (self, &response) {
            (ST3215Request::Ping { .. }, ST3215Response::Ping { .. })
            | (ST3215Request::Read { .. }, ST3215Response::Read { .. })
            | (ST3215Request::Write { .. }, ST3215Response::Write { .. })
            | (ST3215Request::RegWrite { .. }, ST3215Response::RegWrite { .. })
            | (ST3215Request::Action { .. }, ST3215Response::Action { .. })
            | (ST3215Request::Reset { .. }, ST3215Response::Reset { .. })
            | (ST3215Request::SyncWrite { .. }, ST3215Response::SyncWrite { .. }) => Ok(response),
            _ => {
                let expected = match self {
                    ST3215Request::Ping { .. } => "Ping",
                    ST3215Request::Read { .. } => "Read",
                    ST3215Request::Write { .. } => "Write",
                    ST3215Request::RegWrite { .. } => "RegWrite",
                    ST3215Request::Action { .. } => "Action",
                    ST3215Request::Reset { .. } => "Reset",
                    ST3215Request::SyncWrite { .. } => "SyncWrite",
                };
                let got = match &response {
                    ST3215Response::Ping { .. } => "Ping",
                    ST3215Response::Read { .. } => "Read",
                    ST3215Response::Write { .. } => "Write",
                    ST3215Response::RegWrite { .. } => "RegWrite",
                    ST3215Response::Action { .. } => "Action",
                    ST3215Response::Reset { .. } => "Reset",
                    ST3215Response::SyncWrite { .. } => "SyncWrite",
                };
                Err(Error::InvalidData {
                    msg: format!(
                        "Response type mismatch: expected {} response, got {} response",
                        expected, got
                    ),
                    source_packet,
                    reply_packet: response_bytes,
                })
            }
        }
    }
}

#[derive(Debug, PartialEq, Eq)]
pub enum ST3215Response {
    Ping { source_bytes: Bytes },
    Read { data: Bytes, source_bytes: Bytes },
    Write { source_bytes: Bytes },
    RegWrite { source_bytes: Bytes },
    Action { source_bytes: Bytes },
    Reset { source_bytes: Bytes },
    SyncWrite { source_bytes: Bytes },
}

impl ST3215Response {
    fn parse_data(
        request: &ST3215Request,
        request_bytes: Bytes,
        data: Bytes,
        source_bytes: Bytes,
    ) -> Result<Self, Error> {
        match request {
            ST3215Request::Ping { .. }
            | ST3215Request::Write { .. }
            | ST3215Request::RegWrite { .. }
            | ST3215Request::Action { .. }
            | ST3215Request::Reset { .. }
            | ST3215Request::SyncWrite { .. } => {
                if !data.is_empty() {
                    return Err(Error::InvalidData {
                        msg: "Invalid data length for status response".to_string(),
                        source_packet: request_bytes.clone(),
                        reply_packet: source_bytes.clone(),
                    });
                }
                match request {
                    ST3215Request::Ping { .. } => Ok(ST3215Response::Ping { source_bytes }),
                    ST3215Request::Write { .. } => Ok(ST3215Response::Write { source_bytes }),
                    ST3215Request::RegWrite { .. } => Ok(ST3215Response::RegWrite { source_bytes }),
                    ST3215Request::Action { .. } => Ok(ST3215Response::Action { source_bytes }),
                    ST3215Request::Reset { .. } => Ok(ST3215Response::Reset { source_bytes }),
                    ST3215Request::SyncWrite { .. } => {
                        Ok(ST3215Response::SyncWrite { source_bytes })
                    }
                    _ => unreachable!(),
                }
            }
            ST3215Request::Read { length, .. } => {
                if data.len() != *length as usize {
                    return Err(Error::InvalidData {
                        msg: format!(
                            "Invalid data length for Read response: expected {}, got {}",
                            length,
                            data.len()
                        ),
                        source_packet: request_bytes.clone(),
                        reply_packet: source_bytes,
                    });
                }
                Ok(ST3215Response::Read {
                    data,
                    source_bytes,
                })
            }
        }
    }

    pub async fn async_read<R: AsyncRead + Unpin>(
        request: &ST3215Request,
        reader: &mut R,
        timeout_ms: u64,
    ) -> Result<Self, Error> {
        let source_packet = request.to_bytes();
        let mut reply_buffer = BytesMut::with_capacity(256);

        // Read header, searching for 0xFF 0xFF to avoid misaligned reads
        tokio::time::timeout(std::time::Duration::from_millis(timeout_ms), async {
            let mut window = [0u8; 2];
            reader.read_exact(&mut window).await?;
            while window != HEADER {
                window[0] = window[1];
                reader.read_exact(&mut window[1..]).await?;
            }
            Ok(())
        })
        .await
        .map_err(|_| Error::Timeout {
            source_packet: source_packet.clone(),
            reply_packet: Bytes::new(),
        })?
        .map_err(|e| Error::Io {
            error: e,
            source_packet: source_packet.clone(),
        })?;

        reply_buffer.extend_from_slice(&HEADER);

        // Read ID, Length, Error
        reply_buffer.resize(5, 0);
        tokio::time::timeout(
            std::time::Duration::from_millis(timeout_ms),
            reader.read_exact(&mut reply_buffer[2..]),
        )
        .await
        .map_err(|_| Error::Timeout {
            source_packet: source_packet.clone(),
            reply_packet: reply_buffer.clone().freeze(),
        })?
        .map_err(|e| Error::Io {
            error: e,
            source_packet: source_packet.clone(),
        })?;

        let motor_id = reply_buffer[2];
        let length = reply_buffer[3];
        let error = reply_buffer[4];

        // Read params and checksum
        let remaining_len = if length >= 2 { length as usize - 1 } else { 0 };
        if remaining_len > 0 {
            reply_buffer.resize(5 + remaining_len, 0);
            tokio::time::timeout(
                std::time::Duration::from_millis(timeout_ms),
                reader.read_exact(&mut reply_buffer[5..]),
            )
            .await
            .map_err(|_| Error::Timeout {
                source_packet: source_packet.clone(),
                reply_packet: reply_buffer.clone().freeze(),
            })?
            .map_err(|e| Error::Io {
                error: e,
                source_packet: source_packet.clone(),
            })?;
        }

        let full_packet = reply_buffer.freeze();

        if full_packet.len() < 6 {
            return Err(Error::Io {  
                error: io::Error::new(
                    io::ErrorKind::InvalidData,
                    format!("Invalid packet length: expected at least 6 bytes, got {}", full_packet.len())
                ),
                source_packet,
            });
        }

        let response_data = full_packet.slice(5..full_packet.len() - 1);
        let received_checksum = full_packet[full_packet.len() - 1];

        let packet_for_checksum = &full_packet[2..full_packet.len() - 1];
        let calculated_checksum = calculate_checksum(packet_for_checksum);

        if motor_id != request.motor() && request.motor() != BROADCAST_ID {
            return Err(Error::MotorIdMismatch {
                expected: request.motor(),
                got: motor_id,
                source_packet,
                reply_packet: full_packet,
            });
        }

        if calculated_checksum != received_checksum {
            return Err(Error::ChecksumError {
                source_packet,
                reply_packet: full_packet,
            });
        }

        if let Some(errors) = super::error::ServoError::from_bits(error) {
            return Err(Error::Servo {
                errors,
                data: response_data.clone(),
                source_packet,
                response_data,
            });
        }

        Self::parse_data(request, source_packet, response_data, full_packet)
    }
}

fn calculate_checksum(packet: &[u8]) -> u8 {
    let sum: u16 = packet.iter().map(|&b| b as u16).sum();
    (!sum & 0xFF) as u8
}
