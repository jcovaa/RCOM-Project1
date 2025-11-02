// Application layer protocol header.
// DO NOT CHANGE THIS FILE

#ifndef _APPLICATION_LAYER_H_
#define _APPLICATION_LAYER_H_

#include "link_layer.h"
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

// Application layer main function.
// Arguments:
//   serialPort: Serial port name (e.g., /dev/ttyS0).
//   role: Application role {"tx", "rx"}.
//   baudrate: Baudrate of the serial port.
//   nTries: Maximum number of frame retries.
//   timeout: Frame timeout.
//   filename: Name of the file to send / receive.
void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename);

int build_control_packet(uint8_t ctrl, const char *filename, uint32_t filesize,
                         unsigned char *out, int *out_len);

int parse_control_packet(const unsigned char *pkt, int pkt_len,
                         uint8_t *ctrl, char *filename_buf, size_t filename_buf_sz,
                         uint32_t *filesize);

int build_data_packet(const unsigned char *data, int len,
                      unsigned char *out, int *out_len);

int parse_data_packet(const unsigned char *pkt, int pkt_len,
                      const unsigned char **data_out, int *data_len);

uint32_t get_file_size(FILE *f);

#endif // _APPLICATION_LAYER_H_
