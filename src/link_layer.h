// Link layer header.
// DO NOT CHANGE THIS FILE

#ifndef _LINK_LAYER_H_
#define _LINK_LAYER_H_

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

typedef enum
{
    LlTx,
    LlRx,
} LinkLayerRole;

typedef struct
{
    char serialPort[50];
    LinkLayerRole role;
    int baudRate;
    int nRetransmissions;
    int timeout;
} LinkLayer;

// Control state for control frame
typedef enum
{
    CSTART_STATE,
    CFLAG_RCV,
    CA_RCV,
    CC_RCV,
    CBCC_OK,
    CSTOP_STATE
} ControlState;

// Information state for information frame
typedef enum
{
    ISTART_STAT,
    IFLAG_RCV,
    IA_RCV,
    IC_RCV,
    IBCC1_RCV,
    IDATA_RCV,
    IBCC2_RCV,
    ISTOP_STATE
} InformationState;

// Size of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer.
#define MAX_PAYLOAD_SIZE 1000

// MISC
#define FALSE 0
#define TRUE 1

// Open a connection using the "port" parameters defined in struct linkLayer.
// Return 0 on success or -1 on error.
int llopen(LinkLayer connectionParameters);

// Send data in buf with size bufSize.
// Return number of chars written, or -1 on error.
int llwrite(const unsigned char *buf, int bufSize);

// Receive data in packet.
// Return number of chars read, or -1 on error.
int llread(unsigned char *packet);

// Close previously opened connection and print transmission statistics in the console.
// Return 0 on success or -1 on error.
int llclose();

void alarmHandler(int signal);

int byteStuffing(const unsigned char *input, int input_len, unsigned char *output);
int readResponse(int expectedNr);
int byteDestuffing(const unsigned char *input, int input_len, unsigned char *output);

#endif // _LINK_LAYER_H_
