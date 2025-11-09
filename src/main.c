// Main file of the serial port project.
// DO NOT CHANGE THIS FILE

#define _POSIX_C_SOURCE 199309L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "application_layer.h"

#define N_TRIES 3
#define TIMEOUT 4

// ...existing code...

// Add these global statistics variables (add to header file)
extern unsigned long long total_bytes_sent;
extern unsigned long long total_bytes_received;
extern unsigned int total_i_frames_sent;
extern unsigned int total_i_frames_received;
extern unsigned int total_retransmissions;
extern unsigned int total_rejections;

// Arguments:
//   $1: /dev/ttySxx
//   $2: baud rate
//   $3: tx | rx
//   $4: filename
int main(int argc, char *argv[])
{
    if (argc < 5)
    {
        printf("Usage: %s /dev/ttySxx baudrate tx|rx filename\n", argv[0]);
        exit(1);
    }

    const char *serialPort = argv[1];
    const int baudrate = atoi(argv[2]);
    const char *role = argv[3];
    const char *filename = argv[4];

    // Validate baud rate
    switch (baudrate)
    {
    case 1200:
    case 1800:
    case 2400:
    case 4800:
    case 9600:
    case 19200:
    case 38400:
    case 57600:
    case 115200:
        break;
    default:
        printf("Unsupported baud rate (must be one of 1200, 1800, 2400, 4800, 9600, 19200, 38400, 57600, 115200)\n");
        exit(2);
    }

    // Validate role
    if (strcmp("tx", role) != 0 && strcmp("rx", role) != 0)
    {
        printf("ERROR: Role must be \"tx\" or \"rx\"\n");
        exit(3);
    }

    printf("Starting link-layer protocol application\n"
           "  - Serial port: %s\n"
           "  - Role: %s\n"
           "  - Baudrate: %d\n"
           "  - Number of tries: %d\n"
           "  - Timeout: %d\n"
           "  - Filename: %s\n",
           serialPort,
           role,
           baudrate,
           N_TRIES,
           TIMEOUT,
           filename);

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);

    applicationLayer(serialPort, role, baudrate, N_TRIES, TIMEOUT, filename);

    clock_gettime(CLOCK_MONOTONIC, &end);

    double elapsed = (end.tv_sec - start.tv_sec) +
                     (end.tv_nsec - start.tv_nsec) / 1e9;

    printf("\n========== STATISTICS ==========\n");
    printf("Execution time: %.3f seconds\n", elapsed);

    if (strcmp(role, "tx") == 0)
    {
        printf("Total I-frames sent: %u\n", total_i_frames_sent);
        printf("Total bytes sent: %llu\n", total_bytes_sent);
        printf("Total retransmissions: %u\n", total_retransmissions);
        printf("Total REJ received: %u\n", total_rejections);
    }
    else
    {
        printf("Total I-frames received: %u\n", total_i_frames_received);
        printf("Total data bytes received: %llu\n", total_bytes_received);
    }
    printf("================================\n");

    return 0;
}