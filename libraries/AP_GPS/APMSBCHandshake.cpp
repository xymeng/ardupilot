/*
 * APMSBCHandshake.cpp
 *
 *  Created on: Jul 29, 2014
 *      Author: xymeng
 */

#include "APMSBCHandshake.h"

namespace quadstitch {

#if DEBUG_HANDSHAKE
#include <stdio.h>
#define Debug(fmt, args ... ) \
  do { printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ##args); } while(0)
#else
#define Debug(...)
#endif

APM_SBC_Handshake::APM_SBC_Handshake(HandshakeIO *io) {
  io_ = io;
  buffer_counter_ = 0;
  data_counter_ = 0;
  bytes_sum_ = 0;
  read_state_ = PREAMBLE1;

  sample_header.preamble1 = kPreamble1;
  sample_header.preamble2 = kPreamble2;
}

APM_SBC_Handshake::~APM_SBC_Handshake() {
}

bool APM_SBC_Handshake::SBCResetCamera(uint8_t* frames_per_second) {
  Message msg;
  MakeResetCameraMsg(&msg);
  SendMessageBlocking(msg);

  Debug("Sent reset camera msg\n");

  if (!WaitForAck(1000)) {
    Debug("Failed to receive ack\n");
    return false;
  }

  // Try a few times.
  int retry = 0;
  while (retry < 10) {
    if (!ReadMessageBlocking(500)) {
      Debug("Reading timeout\n");
      retry++;
      continue;
    }
    if (msg_.header.type == MSG_CAMERA_RESETTED) {
      Debug("Received reset complete msg.\n");
      *frames_per_second = msg_.msg_camera_resetted.frames_per_sec;
      MakeAckMsg(&msg, MSG_CAMERA_RESETTED);
      SendMessageBlocking(msg);
      return true;
    }
    // Received irrelevant packet.

    retry++;
  }

  return false;
}

bool APM_SBC_Handshake::WaitForResetMsg() {
  int retry = 0;
  while (retry < 10) {
    if (!ReadMessageBlocking(500)) {
      Debug("Reading timeout\n");
      retry++;
      continue;
    }
    if (msg_.header.type == MSG_RESET_CAMERA) {
      Debug("Received reset camera msg.\n");
      Message msg;
      MakeAckMsg(&msg, MSG_RESET_CAMERA);
      SendMessageBlocking(msg);

      return true;
    }
    // Received irrelevant packet.
    retry++;
  }
  return false;
}

bool APM_SBC_Handshake::APMCameraResetted(uint8_t frame_per_sec) {
  Message msg;
  MakeCameraResettedMsg(&msg, frame_per_sec);
  SendMessageBlocking(msg);

  Debug("Sent reset complete msg\n");

  if (!WaitForAck(5000)) {
    Debug("Failed to receive ack\n");
    return false;
  }

  return true;
}

bool APM_SBC_Handshake::SBCFrameSyncInfo(uint16_t frame_counter,
                                         uint32_t time_of_week_msec,
                                         uint16_t week) {
  Message msg;
  MakeFrameSyncInfoMsg(&msg, frame_counter, time_of_week_msec, week);
  SendMessageBlocking(msg);
  ReadMessageBlocking(500);
  if (msg_.header.type == MSG_ACK) {
    return true;
  }
  return false;
}

bool APM_SBC_Handshake::WaitFrameSyncInfo(uint16_t *frame_counter_offset,
                                          uint32_t *time_of_week_msec,
                                          uint16_t *week) {
  int retry = 0;
  while (retry < 10) {
    if (!ReadMessageBlocking(500)) {
      Debug("Reading timeout\n");
      retry++;
      continue;
    }
    if (msg_.header.type == MSG_SYNC_FRAME_INFO) {
      Debug("Received sync frame info msg.\n");
      Message msg;
      MakeAckMsg(&msg, MSG_SYNC_FRAME_INFO);
      SendMessageBlocking(msg);

      *frame_counter_offset = msg_.msg_frame_sync_info.frame_counter;
      *time_of_week_msec = msg_.msg_frame_sync_info.time_of_week_msec;
      *week = msg_.msg_frame_sync_info.week;

      return true;
    }
    // Received irrelevant packet.
    Debug("Received irrelevant packet\n");
    retry++;
  }
  return false;
}

bool APM_SBC_Handshake::WaitForAck(uint16_t timeout_msec) {
  if (ReadMessageBlocking(timeout_msec)) {
    if (msg_.header.type == MSG_ACK) {
      Debug("Received ack\n");
      return true;
    }
  }
  return false;
}

void APM_SBC_Handshake::AddChecksum(Message *msg) {
  uint16_t res = 0;
  uint16_t size = msg->header.len + sizeof(msg->header);

  for (int i = 0; i < size; i++) {
    res += msg->buffer[i];
  }

  msg->buffer[size] = (res & 0xff);
  msg->buffer[size + 1] = (res >> 8);
}

void APM_SBC_Handshake::MakeAckMsg(Message *msg, uint8_t ack_type) {
  msg->header = sample_header;
  msg->header.type = MSG_ACK;
  msg->header.len = 1;
  msg->msg_ack.acked_msg_type = ack_type;
  AddChecksum(msg);
}

void APM_SBC_Handshake::MakeResetCameraMsg(Message *msg) {
  msg->header = sample_header;
  msg->header.type = MSG_RESET_CAMERA;
  msg->header.len = sizeof(MsgResetCamera) - sizeof(MsgHeader);
  AddChecksum(msg);
}

void APM_SBC_Handshake::MakeCameraResettedMsg(Message *msg, uint8_t frame_per_sec) {
  msg->header = sample_header;
  msg->header.type = MSG_CAMERA_RESETTED;
  msg->header.len = sizeof(MsgCameraResetted) - sizeof(MsgHeader);
  msg->msg_camera_resetted.frames_per_sec = frame_per_sec;
  AddChecksum(msg);
}

void APM_SBC_Handshake::MakeFrameSyncInfoMsg(Message *msg,
                         uint16_t frame_counter,
                         uint32_t time_of_week_msec,
                         uint16_t week) {
  msg->header = sample_header;
  msg->header.type = MSG_SYNC_FRAME_INFO;
  msg->header.len = sizeof(MsgFrameSyncInfo) - sizeof(MsgHeader);
  msg->msg_frame_sync_info.frame_counter = frame_counter;
  msg->msg_frame_sync_info.time_of_week_msec = time_of_week_msec;
  msg->msg_frame_sync_info.week = week;
  AddChecksum(msg);
}

void APM_SBC_Handshake::SendMessageBlocking(const Message& msg) {
  int msg_size = msg.header.len + sizeof(msg.header) + 2;
  for (int i = 0; i < msg_size; i++) {
    while(!io_->WriteByte(msg.buffer[i]));
  }
  //Debug("Sent %d bytes\n", msg_size);
}

bool APM_SBC_Handshake::ReadMessageBlocking(uint16_t timeout_msec) {
  bool success = false;

#define BUFFER_RESET() \
  do { \
    buffer_counter_ = 0; \
    data_counter_ = 0; \
    bytes_sum_ = 0; \
    read_state_ = PREAMBLE1; \
  } while(0)

  while (!success) {
    uint8_t c;

    if (!io_->ReadByte(timeout_msec, &c)) {
      return false;
    }

    msg_.buffer[buffer_counter_ ] = c;
    buffer_counter_++;
    Debug("Received %d bytes\n", buffer_counter_);

    switch(read_state_) {

      case PREAMBLE1:
        if (c == kPreamble1) {
          Debug("Preamble 1");
          read_state_++;
          bytes_sum_ += c;
        } else {
          Debug("Preamble 1 match failed\n");
          BUFFER_RESET();
        }
        break;

      case PREAMBLE2:
        Debug("Preamble 2");
        if (c == kPreamble2) {
          read_state_++;
          bytes_sum_ += c;
        } else {
          Debug("Preamble 2 match failed\n");
          BUFFER_RESET();
        }
        break;

      case TYPE:
      case LENGTH_LO:
      case LENGTH_HI:
        Debug("Type and Length");
        if (read_state_ == LENGTH_HI) {
          Debug("Message length: %d\n", msg_.header.len);
        }
        read_state_++;
        bytes_sum_ += c;
        break;

      case DATA:
        if (data_counter_ == msg_.header.len) {
          // Current byte is checksum byte.
          // We do in this way to handle zero data length packet.
          read_state_++;
          goto LABEL_CHECKSUM;
        } else {
          Debug("DATA Bytes %d\n", data_counter_);
          bytes_sum_ += c;
          data_counter_++;
        }
        break;

LABEL_CHECKSUM:
      case CHECKSUM_LO:
        //Debug("Checksum lo");
        read_state_++;
        break;

      case CHECKSUM_HI:
      {
        //Debug("Checksum hi");
        uint8_t check_sum_lo = msg_.buffer[buffer_counter_ - 2];
        uint16_t check_sum = check_sum_lo + (((uint16_t)c) << 8);
        if (check_sum == bytes_sum_) {
          Debug("Received a message.\n");
          success = true;
        } else {
          Debug("Check sum failed\n");
          //PrintBuffer();
        }
        BUFFER_RESET();
        break;
      }

      default:
        //Debug("Bad data received\n");
        BUFFER_RESET();
    }
  }

  return true;
}

void APM_SBC_Handshake::PrintBuffer() {
#if DEBUG_HANDSHAKE
  for (int i = 0; i < buffer_counter_; i++) {
    Debug("%x ", msg_.buffer[i]);
  }
  Debug("\n");
#endif
}

void APM_SBC_Handshake::PrintMessage(const Message& msg) {
#if DEBUG_HANDSHAKE
  int msg_size = msg.header.len + sizeof(msg.header) + 2;
  for (int i = 0; i < msg_size; i++) {
    printf("%x ", msg.buffer[i]);
  }
  printf("\n");
#endif
}

}

