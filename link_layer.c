// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E

int alarmEnabled = FALSE;
int alarmCount = 0;
int UA_received = FALSE;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {

    // Transmitter
    if (connectionParameters.role == LlTx) {
        if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0) {
            perror("openSerialPort");
            return -1;
        }

        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;

        if (sigaction(SIGALRM, &act, NULL) == -1) {
            perror("sigaction");
            return -1;
        }

        unsigned char SET[5] = {0x7E, 0x03, 0x03, 0x03 ^ 0x03, 0x7E};
        unsigned char UA;

        while (alarmCount < connectionParameters.nRetransmissions && !UA_received) {
            if (!alarmEnabled) {
                printf("Sending SET frame (attempt %d)\n", alarmCount + 1);
                writeBytesSerialPort(SET, 5);
                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

            ControlState state = START_STATE;
            while (state != STOP_STATE) {
                int res = readByteSerialPort(&UA);
                if (res <= 0)
                    continue;
                
                switch (state) {
                    case START_STATE:
                        if (UA == 0x7E)
                            state = FLAG_RCV;
                        break;
                    case FLAG_RCV:
                        if (UA == 0x03)
                            state = A_RCV;
                        else if (UA == FLAG)
                            state = FLAG_RCV;
                        else 
                            state = START_STATE;
                        break;
                    case A_RCV:
                        if (UA == 0x03)
                            state = C_RCV;
                        else if (UA == FLAG)
                            state = FLAG_RCV;
                        else 
                            state = START_STATE;
                        break;
                    case C_RCV:
                        if (UA == (0x03 ^ 0x03))
                            state = BCC_OK;
                        else if (UA == FLAG) 
                            state = FLAG_RCV;
                        else 
                            state = START_STATE;
                        break;
                    case BCC_OK:
                        if (UA == FLAG)
                            state = STOP_STATE;
                        else 
                            state = START_STATE;
                        break;
                    default:
                        state = START_STATE;
                        break;
                }
            }
        }
    }   

    // Receiver
    else if (connectionParameters.role == LlRx) {

    }

    else {
        printf("error determining the role using llopen.\n");
        return -1;
    }

    return 0;
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    // TODO: Implement this function

    return 0;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose() {
    // TODO: Implement this function

    return 0;
}

void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
}