use super::packet;
use bytes::Bytes;

#[test]
fn test_ping_request_to_bytes() {
    let request = packet::ST3215Request::Ping { motor: 1 };
    let expected_bytes = Bytes::from_static(&[0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[tokio::test]
async fn test_ping_response_parsing() {
    let request = packet::ST3215Request::Ping { motor: 1 };
    let response_bytes = &[0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC];
    let mut reader = &response_bytes[..];
    let response = packet::ST3215Response::async_read(&request, &mut reader, 100)
        .await
        .unwrap();
    let expected_response = packet::ST3215Response::Ping {
        source_bytes: Bytes::from_static(response_bytes),
    };
    assert_eq!(response, expected_response);
}

#[test]
fn test_read_request_to_bytes() {
    let request = packet::ST3215Request::Read {
        motor: 1,
        address: 0x2A,
        length: 2,
    };
    let expected_bytes = Bytes::from_static(&[0xFF, 0xFF, 0x01, 0x04, 0x02, 0x2A, 0x02, 0xCC]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[tokio::test]
async fn test_read_response_parsing() {
    let request = packet::ST3215Request::Read {
        motor: 1,
        address: 0x2A,
        length: 2,
    };
    let response_data = Bytes::from_static(&[0x12, 0x34]);
    let response_bytes = &[0xFF, 0xFF, 0x01, 0x04, 0x00, 0x12, 0x34, 0xB4];
    let mut reader = &response_bytes[..];
    let response = packet::ST3215Response::async_read(&request, &mut reader, 100)
        .await
        .unwrap();
    let expected_response = packet::ST3215Response::Read {
        data: response_data,
        source_bytes: Bytes::from_static(response_bytes),
    };
    assert_eq!(response, expected_response);
}

#[test]
fn test_write_request_to_bytes() {
    let data = Bytes::from_static(&[0xAB, 0xCD]);
    let request = packet::ST3215Request::Write {
        motor: 1,
        address: 0x2A,
        data: data.clone(),
    };
    let expected_bytes =
        Bytes::from_static(&[0xFF, 0xFF, 0x01, 0x05, 0x03, 0x2A, 0xAB, 0xCD, 0x54]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_reg_write_request_to_bytes() {
    let data = Bytes::from_static(&[0xAB, 0xCD]);
    let request = packet::ST3215Request::RegWrite {
        motor: 1,
        address: 0x2A,
        data: data.clone(),
    };
    let expected_bytes =
        Bytes::from_static(&[0xFF, 0xFF, 0x01, 0x05, 0x04, 0x2A, 0xAB, 0xCD, 0x53]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_action_request_to_bytes() {
    let request = packet::ST3215Request::Action { motor: 1 };
    let expected_bytes = Bytes::from_static(&[0xFF, 0xFF, 0x01, 0x02, 0x05, 0xF7]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_reset_request_to_bytes() {
    let request = packet::ST3215Request::Reset { motor: 1 };
    let expected_bytes = Bytes::from_static(&[0xFF, 0xFF, 0x01, 0x02, 0x06, 0xF6]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_request_to_bytes() {
    // Based on Python reference data from actual working motors
    // Python: [255, 255, 254, 28, 131, 42, 2, 1, 247, 0, 2, 206, 2, 3, 38, 11, 4, 112, 2, 5, 42, 1, 6, 115, 8, 7, 206, 6, 8, 246, 13, 43]
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,  // ADDR_GOAL_POSITION
        data: vec![
            (1, Bytes::from_static(&[247, 0])),     // Motor 1: 247
            (2, Bytes::from_static(&[206, 2])),     // Motor 2: 718
            (3, Bytes::from_static(&[38, 11])),     // Motor 3: 2854
            (4, Bytes::from_static(&[112, 2])),     // Motor 4: 624
            (5, Bytes::from_static(&[42, 1])),      // Motor 5: 298
            (6, Bytes::from_static(&[115, 8])),     // Motor 6: 2163
            (7, Bytes::from_static(&[206, 6])),     // Motor 7: 1742
            (8, Bytes::from_static(&[246, 13])),    // Motor 8: 3574
        ],
    };

    // Expected: exact Python packet
    let expected_bytes = Bytes::from_static(&[
        255, 255, 254, 28, 131, 42, 2,
        1, 247, 0,
        2, 206, 2,
        3, 38, 11,
        4, 112, 2,
        5, 42, 1,
        6, 115, 8,
        7, 206, 6,
        8, 246, 13,
        43  // checksum from Python
    ]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_1() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[106, 0])),
            (2, Bytes::from_static(&[225, 2])),
            (3, Bytes::from_static(&[57, 11])),
            (4, Bytes::from_static(&[133, 2])),
            (5, Bytes::from_static(&[62, 1])),
            (6, Bytes::from_static(&[132, 8])),
            (7, Bytes::from_static(&[220, 6])),
            (8, Bytes::from_static(&[12, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 106, 0, 2, 225, 2, 3, 57, 11, 4, 133, 2, 5, 62, 1, 6, 132, 8, 7, 220, 6, 8, 12, 14, 51]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_2() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[122, 0])),
            (2, Bytes::from_static(&[245, 2])),
            (3, Bytes::from_static(&[76, 11])),
            (4, Bytes::from_static(&[154, 2])),
            (5, Bytes::from_static(&[82, 1])),
            (6, Bytes::from_static(&[150, 8])),
            (7, Bytes::from_static(&[234, 6])),
            (8, Bytes::from_static(&[34, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 122, 0, 2, 245, 2, 3, 76, 11, 4, 154, 2, 5, 82, 1, 6, 150, 8, 7, 234, 6, 8, 34, 14, 157]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_3() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[138, 0])),
            (2, Bytes::from_static(&[8, 3])),
            (3, Bytes::from_static(&[95, 11])),
            (4, Bytes::from_static(&[175, 2])),
            (5, Bytes::from_static(&[103, 1])),
            (6, Bytes::from_static(&[167, 8])),
            (7, Bytes::from_static(&[249, 6])),
            (8, Bytes::from_static(&[56, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 138, 0, 2, 8, 3, 3, 95, 11, 4, 175, 2, 5, 103, 1, 6, 167, 8, 7, 249, 6, 8, 56, 14, 6]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_4() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[153, 0])),
            (2, Bytes::from_static(&[28, 3])),
            (3, Bytes::from_static(&[114, 11])),
            (4, Bytes::from_static(&[196, 2])),
            (5, Bytes::from_static(&[123, 1])),
            (6, Bytes::from_static(&[185, 8])),
            (7, Bytes::from_static(&[7, 7])),
            (8, Bytes::from_static(&[78, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 153, 0, 2, 28, 3, 3, 114, 11, 4, 196, 2, 5, 123, 1, 6, 185, 8, 7, 7, 7, 8, 78, 14, 112]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_5() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[169, 0])),
            (2, Bytes::from_static(&[47, 3])),
            (3, Bytes::from_static(&[133, 11])),
            (4, Bytes::from_static(&[217, 2])),
            (5, Bytes::from_static(&[143, 1])),
            (6, Bytes::from_static(&[202, 8])),
            (7, Bytes::from_static(&[21, 7])),
            (8, Bytes::from_static(&[99, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 169, 0, 2, 47, 3, 3, 133, 11, 4, 217, 2, 5, 143, 1, 6, 202, 8, 7, 21, 7, 8, 99, 14, 221]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_6() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[185, 0])),
            (2, Bytes::from_static(&[67, 3])),
            (3, Bytes::from_static(&[153, 11])),
            (4, Bytes::from_static(&[239, 2])),
            (5, Bytes::from_static(&[164, 1])),
            (6, Bytes::from_static(&[219, 8])),
            (7, Bytes::from_static(&[36, 7])),
            (8, Bytes::from_static(&[121, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 185, 0, 2, 67, 3, 3, 153, 11, 4, 239, 2, 5, 164, 1, 6, 219, 8, 7, 36, 7, 8, 121, 14, 68]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_7() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[200, 0])),
            (2, Bytes::from_static(&[86, 3])),
            (3, Bytes::from_static(&[172, 11])),
            (4, Bytes::from_static(&[4, 3])),
            (5, Bytes::from_static(&[184, 1])),
            (6, Bytes::from_static(&[237, 8])),
            (7, Bytes::from_static(&[50, 7])),
            (8, Bytes::from_static(&[143, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 200, 0, 2, 86, 3, 3, 172, 11, 4, 4, 3, 5, 184, 1, 6, 237, 8, 7, 50, 7, 8, 143, 14, 175]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_8() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[216, 0])),
            (2, Bytes::from_static(&[106, 3])),
            (3, Bytes::from_static(&[191, 11])),
            (4, Bytes::from_static(&[25, 3])),
            (5, Bytes::from_static(&[204, 1])),
            (6, Bytes::from_static(&[254, 8])),
            (7, Bytes::from_static(&[64, 7])),
            (8, Bytes::from_static(&[165, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 216, 0, 2, 106, 3, 3, 191, 11, 4, 25, 3, 5, 204, 1, 6, 254, 8, 7, 64, 7, 8, 165, 14, 26]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_9() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[231, 0])),
            (2, Bytes::from_static(&[125, 3])),
            (3, Bytes::from_static(&[210, 11])),
            (4, Bytes::from_static(&[46, 3])),
            (5, Bytes::from_static(&[225, 1])),
            (6, Bytes::from_static(&[16, 9])),
            (7, Bytes::from_static(&[79, 7])),
            (8, Bytes::from_static(&[187, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 231, 0, 2, 125, 3, 3, 210, 11, 4, 46, 3, 5, 225, 1, 6, 16, 9, 7, 79, 7, 8, 187, 14, 131]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_variation_10() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[177, 0])),
            (2, Bytes::from_static(&[18, 3])),
            (3, Bytes::from_static(&[162, 11])),
            (4, Bytes::from_static(&[207, 2])),
            (5, Bytes::from_static(&[194, 1])),
            (6, Bytes::from_static(&[158, 8])),
            (7, Bytes::from_static(&[72, 7])),
            (8, Bytes::from_static(&[23, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 177, 0, 2, 18, 3, 3, 162, 11, 4, 207, 2, 5, 194, 1, 6, 158, 8, 7, 72, 7, 8, 23, 14, 241]);
    assert_eq!(request.to_bytes(), expected_bytes);
}

#[test]
fn test_sync_write_init_positions() {
    let request = packet::ST3215Request::SyncWrite {
        address: 0x2A,
        data: vec![
            (1, Bytes::from_static(&[91, 0])),
            (2, Bytes::from_static(&[145, 3])),
            (3, Bytes::from_static(&[229, 11])),
            (4, Bytes::from_static(&[67, 3])),
            (5, Bytes::from_static(&[245, 1])),
            (6, Bytes::from_static(&[33, 9])),
            (7, Bytes::from_static(&[93, 7])),
            (8, Bytes::from_static(&[209, 14])),
        ],
    };
    let expected_bytes = Bytes::from_static(&[255, 255, 254, 28, 131, 42, 2, 1, 91, 0, 2, 145, 3, 3, 229, 11, 4, 67, 3, 5, 245, 1, 6, 33, 9, 7, 93, 7, 8, 209, 14, 138]);
    assert_eq!(request.to_bytes(), expected_bytes);
}
