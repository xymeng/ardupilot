/*
 * APMSBCHandshake.h
 *
 *  Created on: Jul 29, 2014
 *      Author: xymeng
 */

#ifndef APMSBCHANDSHAKE_H_
#define APMSBCHANDSHAKE_H_

#include <stdint.h>

namespace quadstitch {

// Abstract class used to send/recv raw bytes.
// Different implementation will be used on different hardware.
class HandshakeIO {
 public:
  virtual ~HandshakeIO() {};
  virtual bool           WriteByte(uint8_t c)=0;
  virtual bool           ReadByte(uint16_t timeout_msec, uint8_t *c)=0;
};

class APM_SBC_Handshake {
 public:
  APM_SBC_Handshake(HandshakeIO *io);
  virtual ~APM_SBC_Handshake();

  // The following high level functions are called by either APM or SBC.
  // The first few capital letters denote the receiver of the message.
  // e.g. SBCResetCamera() is sent by APM, it tells SBC to reset camera.

  bool SBCResetCamera(uint8_t *frames_per_second);

  bool APMCameraResetted(uint8_t frames_per_sec);

  bool SBCFrameSyncInfo(uint16_t frame_counter,
                        uint32_t time_of_week_msec,
                        uint16_t week);

  bool WaitForResetMsg();
  bool WaitFrameSyncInfo(uint16_t *frame_counter_offset,
                         uint32_t *time_of_week_msec,
                         uint16_t *week);

 private:
  HandshakeIO *io_;

  struct __attribute__((__packed__)) MsgHeader {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t type;
    uint16_t len;
  };

  enum MsgType {
    MSG_ACK = 0x00,
    MSG_RESET_CAMERA = 0x01,
    MSG_CAMERA_RESETTED = 0x02,
    MSG_SYNC_FRAME_INFO = 0x03
  };

  struct __attribute__((__packed__)) MsgAck {
    MsgHeader header;
    uint8_t acked_msg_type;
  };

  struct __attribute__((__packed__)) MsgResetCamera {
    MsgHeader header;
  };

  struct __attribute__((__packed__)) MsgCameraResetted {
    MsgHeader header;
    uint8_t frames_per_sec;
  };

  struct __attribute__((__packed__)) MsgFrameSyncInfo {
    MsgHeader header;
    uint16_t frame_counter;
    uint32_t time_of_week_msec;
    uint16_t week;
  };

  static const uint8_t kPreamble1 = 0xAA;
  static const uint8_t kPreamble2 = 0xBB;

  static const int kBufferSize = 64;

  union Message {
    MsgHeader header;
    MsgAck msg_ack;
    MsgResetCamera msg_reset_camera;
    MsgCameraResetted msg_camera_resetted;
    MsgFrameSyncInfo msg_frame_sync_info;
    uint8_t buffer[kBufferSize];
  };

  Message msg_;

  enum ReadState {
    PREAMBLE1 = 0,
    PREAMBLE2,
    TYPE,
    LENGTH_LO,
    LENGTH_HI,
    DATA,
    CHECKSUM_LO,
    CHECKSUM_HI
  };

  MsgHeader sample_header;

  int buffer_counter_;
  int data_counter_;
  uint16_t bytes_sum_;

  int read_state_;

  void MakeAckMsg(Message *msg, uint8_t ack_type);
  void MakeResetCameraMsg(Message *msg);
  void MakeCameraResettedMsg(Message *msg, uint8_t frame_per_sec);
  void MakeFrameSyncInfoMsg(Message *msg,
                           uint16_t frame_counter,
                           uint32_t time_of_week_msec,
                           uint16_t week);

  bool WaitForAck(uint16_t timeout_msec);

  // Append uint16_t checksum to the end of the message.
  void AddChecksum(Message *msg);

  void SendMessageBlocking(const Message& msg);

  // Read ONE message.
  // Returns false if reading timeout happens.
  bool ReadMessageBlocking(uint16_t timeout_msec);

  void PrintBuffer();
  void PrintMessage(const Message&);
};

}

#endif /* APMSBCHANDSHAKE_H_ */
