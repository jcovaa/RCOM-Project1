// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E
#define A_SET 0x03
#define A_UA 0x01
#define C_SET 0x03
#define C_UA 0x07

#define A_I 0x03
#define C_I(Ns) ((Ns) << 7) // Test with << 6
#define ESC 0x7D
#define STUFF_XOR 0x20
#define C_RR(Nr) (0x05 | ((Nr) << 7))
#define C_REJ(Nr) (0x01 | ((Nr) << 7))

int timeout = 0;
int nrTries = 0;

int alarmEnabled = FALSE;
int alarmCount = 0;
int UA_received = FALSE;
int DISC_received = FALSE;

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{

    alarmEnabled = FALSE;
    alarmCount = 0;
    UA_received = FALSE;
    DISC_received = FALSE;

    timeout = connectionParameters.timeout;
    nrTries = connectionParameters.nRetransmissions;

    // Transmitter
    if (connectionParameters.role == LlTx)
    {
        if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
        {
            perror("openSerialPort");
            return -1;
        }

        printf("Serial port %s opened\n", connectionParameters.serialPort);

        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1)
        {
            perror("sigaction");
            return -1;
        }

        unsigned char SET[5] = {FLAG, A_SET, C_SET, A_SET ^ C_SET, FLAG};

        while (alarmCount < connectionParameters.nRetransmissions && !UA_received)
        {
            if (!alarmEnabled)
            {
                printf("Sending SET frame (attempt %d)\n", alarmCount + 1);
                writeBytesSerialPort(SET, 5);
                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

            unsigned char byte;
            ControlState state = CSTART_STATE;
            while (state != CSTOP_STATE)
            {
                int res = readByteSerialPort(&byte);
                if (res <= 0)
                    continue;

                switch (state)
                {
                case CSTART_STATE:
                    if (byte == FLAG)
                        state = CFLAG_RCV;
                    break;
                case CFLAG_RCV:
                    if (byte == 0x01)
                        state = CA_RCV;
                    else if (byte == FLAG)
                        state = CFLAG_RCV;
                    else
                        state = CSTART_STATE;
                    break;
                case CA_RCV:
                    if (byte == 0x07)
                        state = CC_RCV;
                    else if (byte == FLAG)
                        state = CFLAG_RCV;
                    else
                        state = CSTART_STATE;
                    break;
                case CC_RCV:
                    if (byte == (0x01 ^ 0x07))
                        state = CBCC_OK;
                    else if (byte == FLAG)
                        state = CFLAG_RCV;
                    else
                        state = CSTART_STATE;
                    break;
                case CBCC_OK:
                    if (byte == FLAG)
                        state = CSTOP_STATE;
                    else
                        state = CSTART_STATE;
                    break;
                default:
                    state = CSTART_STATE;
                    break;
                }
            }

            printf("UA frame received correctly.\n");
            UA_received = TRUE;
            alarm(0);
            alarmEnabled = FALSE;
            break;
        }

        if (!UA_received)
        {
            printf("Failed to receive UA after %d attempts\n", connectionParameters.nRetransmissions);
            return -1;
        }

        return 0;
    }

    // Receiver
    else if (connectionParameters.role == LlRx)
    {
        if (openSerialPort(connectionParameters.serialPort, connectionParameters.baudRate) < 0)
        {
            perror("openSerialPort");
            return -1;
        }

        printf("Serial port %s opened\n", connectionParameters.serialPort);

        unsigned char byte;
        ControlState state = CSTART_STATE;
        while (state != CSTOP_STATE)
        {
            int res = readByteSerialPort(&byte);
            if (res <= 0)
                continue;

            switch (state)
            {
            case CSTART_STATE:
                if (byte == FLAG)
                    state = CFLAG_RCV;
                break;
            case CFLAG_RCV:
                if (byte == 0x03)
                    state = CA_RCV;
                else if (byte == FLAG)
                    state = CFLAG_RCV;
                else
                    state = CSTART_STATE;
                break;
            case CA_RCV:
                if (byte == 0x03)
                    state = CC_RCV;
                else if (byte == FLAG)
                    state = CFLAG_RCV;
                else
                    state = CSTART_STATE;
                break;
            case CC_RCV:
                if (byte == (0x03 ^ 0x03))
                    state = CBCC_OK;
                else if (byte == FLAG)
                    state = CFLAG_RCV;
                else
                    state = CSTART_STATE;
                break;
            case CBCC_OK:
                if (byte == FLAG)
                    state = CSTOP_STATE;
                else
                    state = CSTART_STATE;
                break;
            default:
                state = CSTART_STATE;
                break;
            }
        }

        printf("SET frame received correctly.\n");

        unsigned char UA[5] = {FLAG, A_UA, C_UA, A_UA ^ C_UA, FLAG};
        int bytes_sent = writeBytesSerialPort(UA, 5);
        printf("%d bytes written to serial port (UA frame)\n", bytes_sent);

        sleep(1);
        return 0;
    }

    else
    {
        printf("error determining the role using llopen.\n");
        return -1;
    }
}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    static int Ns = 0;
    alarmCount = 0;
    alarmEnabled = FALSE;

    unsigned char tempFrame[bufSize + 6]; // maximum size needed for FLAG + A + C + BCC1 + DATA + BCC2 + FLAG
    int idx = 0;

    tempFrame[idx++] = FLAG;
    tempFrame[idx++] = A_I;
    tempFrame[idx++] = C_I(Ns);
    tempFrame[idx++] = A_I ^ C_I(Ns); // BCC1

    unsigned char BCC2 = 0x00;
    for (int i = 0; i < bufSize; i++)
        BCC2 ^= buf[i];

    // Copy data payload from the application buffer into the frame (after header)
    memcpy(&tempFrame[idx], buf, bufSize);
    idx += bufSize;

    tempFrame[idx++] = BCC2;
    tempFrame[idx++] = FLAG;

    unsigned char stuffedFrame[2 * (bufSize + 6)];
    int headerLen = 4;

    // Copy header (FLAG, A, C, BCC1) to stuffed frame without modifications
    memcpy(stuffedFrame, tempFrame, headerLen);
    // Apply byte stuffing only to DATA + BCC2 + FLAG
    int stuffedDataLen = byteStuffing(tempFrame + headerLen, bufSize + 2, stuffedFrame + headerLen);
    int frameSize = headerLen + stuffedDataLen;

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    while (alarmCount < nrTries)
    {
        if (!alarmEnabled)
        {
            // printf("Sending I frame with Ns=%d (attempt %d)\n", Ns, alarmCount + 1);
            writeBytesSerialPort(stuffedFrame, frameSize);
            alarm(timeout);
            alarmEnabled = TRUE;
        }

        int response = readResponse((Ns + 1) % 2);

        if (response == 1)
        {
            printf("RR received, frame acknowledged.\n");
            alarm(0);
            alarmEnabled = FALSE;
            Ns = (Ns + 1) % 2; // Toggle Ns 0 -> 1 or 1 -> 0
            return bufSize;
        }
        else if (response == -1)
        {
            printf("REJ received, retransmitting frame.\n");
            alarm(0);
            alarmEnabled = FALSE;
        }
    }

    printf("Failed to receive RR after 3 attempts, llwrite failed.\n");
    return -1;
}

int byteStuffing(const unsigned char *input, int input_len, unsigned char *output)
{
    int j = 0;
    for (int i = 0; i < input_len; i++)
    {
        if (input[i] == FLAG)
        {
            output[j++] = ESC;
            output[j++] = FLAG ^ STUFF_XOR;
        }
        else if (input[i] == ESC)
        {
            output[j++] = ESC;
            output[j++] = ESC ^ STUFF_XOR;
        }
        else
        {
            output[j++] = input[i];
        }
    }

    return j;
}

int byteDestuffing(const unsigned char *input, int input_len, unsigned char *output)
{
    int j = 0;
    for (int i = 0; i < input_len; i++)
    {
        if (input[i] == ESC)
        {
            i++;
            output[j++] = input[i] ^ STUFF_XOR;
        }
        else
        {
            output[j++] = input[i];
        }
    }
    return j;
}

int readResponse(int expectedNr)
{
    unsigned char byte;
    ControlState state = CSTART_STATE;
    unsigned char C_field = 0;

    // True because llwrite already handles with alarm
    while (TRUE)
    {
        int res = readByteSerialPort(&byte);
        if (res <= 0)
            continue;

        switch (state)
        {
        case CSTART_STATE:
            if (byte == FLAG)
                state = CFLAG_RCV;
            break;
        case CFLAG_RCV:
            if (byte == 0x01)
                state = CA_RCV;
            else if (byte == FLAG)
                state = CFLAG_RCV;
            else
                state = CSTART_STATE;
            break;
        case CA_RCV:
            if (byte == C_RR(expectedNr))
            {
                C_field = byte;
                state = CC_RCV;
            }
            else if (byte == C_REJ(expectedNr))
            {
                C_field = byte;
                state = CC_RCV;
            }
            else if (byte == FLAG)
                state = CFLAG_RCV;
            else
                state = CSTART_STATE;
            break;
        case CC_RCV:
            if (byte == (0x01 ^ C_field))
                state = CBCC_OK;
            else if (byte == FLAG)
                state = CFLAG_RCV;
            else
                state = CSTART_STATE;
            break;
        case CBCC_OK:
            if (byte == FLAG)
            {
                if ((C_field & 0x01) == 0x01)
                    return -1; // REJ
                else
                    return 1; // RR
            }
            else
                state = CSTART_STATE;
            break;
        default:
            state = CSTART_STATE;
            break;
        }
    }
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    static int expectedNs = 0;
    unsigned char frame[MAX_PAYLOAD_SIZE + 6];
    unsigned char byte;
    int idx = 0;

    do
    {
        int res = readByteSerialPort(&byte);

        if (res > 0)
        {
            if (idx == 0 && byte != FLAG)
            {
                continue;
            }

            frame[idx++] = byte;

            if (idx >= MAX_PAYLOAD_SIZE + 6)
            {
                printf("Frame is too large.\n");
                return -1;
            }
        }
    } while (!(idx > 1 && byte == FLAG));

    printf("Frame received (%d bytes): ", idx);
    for (int i = 0; i < idx; i++)
        printf("%02X ", frame[i]);
    printf("\n");

    int destuffedLen = byteDestuffing(frame + 1, idx - 2, frame + 1);

    printf("Destuffed frame (%d bytes): ", destuffedLen + 1);
    for (int i = 0; i < destuffedLen + 1; i++)
        printf("%02X ", frame[i]);
    printf("\n");

    unsigned char A = frame[1], C = frame[2], BCC1 = frame[3];

    if ((A ^ C) != BCC1)
    {
        printf("BCC1 error: expected %02X, got %02X. Sending REJ.\n", (A ^ C), BCC1);
        unsigned char rej[5] = {FLAG, A_I, C_REJ(expectedNs), A_I ^ C_REJ(expectedNs), FLAG};
        writeBytesSerialPort(rej, 5);
        printf("REJ sent.\n");
        return -1;
    }

    int dataLen = destuffedLen - 4;
    unsigned char BCC2 = 0x00;
    for (int i = 0; i < dataLen; i++)
    {
        BCC2 ^= frame[4 + i];
    }

    if (BCC2 != frame[4 + dataLen])
    {
        printf("BCC2 error: expected %02X, got %02X. Sending REJ.\n", BCC2, frame[4 + dataLen]);
        unsigned char rej[5] = {FLAG, A_I, C_REJ(expectedNs), A_I ^ C_REJ(expectedNs), FLAG};
        writeBytesSerialPort(rej, 5);
        printf("REJ sent.\n");
        return -1;
    }

    memcpy(packet, frame + 4, dataLen);

    unsigned char rr[5] = {FLAG, A_I, C_RR((expectedNs + 1) % 2), A_I ^ C_RR((expectedNs + 1) % 2), FLAG};
    writeBytesSerialPort(rr, 5);
    printf("RR sent, expecting Ns=%d next.\n", (expectedNs + 1) % 2);

    printf("Packet received correctly (Ns=%d, %d bytes).\n", expectedNs, dataLen);

    expectedNs = (expectedNs + 1) % 2;
    return dataLen;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(LinkLayer connectionParameters)
{
    alarmEnabled = FALSE;
    alarmCount = 0;
    DISC_received = FALSE;
    UA_received = FALSE;

    unsigned char byte;
    ControlState state;

    // Transmitter
    if (connectionParameters.role == LlTx)
    {
        struct sigaction act = {0};
        act.sa_handler = &alarmHandler;
        if (sigaction(SIGALRM, &act, NULL) == -1)
        {
            perror("sigaction");
            closeSerialPort();
            return -1;
        }

        unsigned char DISC_tx[5] = {FLAG, 0x03, 0x0B, 0x03 ^ 0x0B, FLAG};
        unsigned char DISC_A_rx = 0x01, DISC_C_rx = 0x0B;

        unsigned char UA_tx[5] = {FLAG, 0x01, 0x07, 0x01 ^ 0x07, FLAG};

        alarmEnabled = FALSE;
        alarmCount = 0;

        while (alarmCount < connectionParameters.nRetransmissions && !DISC_received)
        {
            if (!alarmEnabled)
            {
                sleep(1);
                printf("Sending DISC frame (attempt %d)\n", alarmCount + 1);
                writeBytesSerialPort(DISC_tx, 5);
                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

            state = CSTART_STATE;
            while (state != CSTOP_STATE)
            {
                int res = readByteSerialPort(&byte);
                if (res <= 0)
                {
                    continue;
                }

                switch (state)
                {
                case CSTART_STATE:
                    if (byte == FLAG)
                    {
                        state = CFLAG_RCV;
                    }
                    break;
                case CFLAG_RCV:
                    if (byte == DISC_A_rx)
                    {
                        state = CA_RCV;
                    }
                    else if (byte == FLAG)
                    {
                        state = CFLAG_RCV;
                    }
                    else
                    {
                        state = CSTART_STATE;
                    }
                    break;
                case CA_RCV:
                    if (byte == DISC_C_rx)
                    {
                        state = CC_RCV;
                    }
                    else if (byte == FLAG)
                    {
                        state = CFLAG_RCV;
                    }
                    else
                    {
                        state = CSTART_STATE;
                    }
                    break;
                case CC_RCV:
                    if (byte == (DISC_A_rx ^ DISC_C_rx))
                    {
                        state = CBCC_OK;
                    }
                    else if (byte == FLAG)
                    {
                        state = CFLAG_RCV;
                    }
                    else
                    {
                        state = CSTART_STATE;
                    }
                    break;
                case CBCC_OK:
                    if (byte == FLAG)
                    {
                        state = CSTOP_STATE;
                    }
                    else
                    {
                        state = CSTART_STATE;
                    }
                    break;
                default:
                    state = CSTART_STATE;
                    break;
                }
            }

            printf("DISC frame from receiver received correctly.\n");
            DISC_received = 1;
            alarm(0);
            alarmEnabled = FALSE;

            int bytes_sent = writeBytesSerialPort(UA_tx, 5);
            printf("%d bytes written to serial port (UA frame)\n", bytes_sent);
        }

        if (!DISC_received)
        {
            printf("Failed to receive DISC after %d attempts\n", connectionParameters.nRetransmissions);
            sleep(1);
            closeSerialPort();
            return -1;
        }

        if (closeSerialPort() < 0)
        {
            perror("closeSerialPort");
            return -1;
        }

        printf("Serial port %s closed (llclose - Tx)\n", connectionParameters.serialPort);
        return 0;
    }

    // Receiver
    else if (connectionParameters.role == LlRx)
    {
        unsigned char DISC_reply[5] = {FLAG, 0x01, 0x0B, (0x01 ^ 0x0B), FLAG};
        unsigned char DISC_A_tx = 0x03, DISC_C_tx = 0x0B;

        state = CSTART_STATE;
        while (state != CSTOP_STATE)
        {
            int res = readByteSerialPort(&byte);
            if (res <= 0)
            {
                continue;
            }

            switch (state)
            {
            case CSTART_STATE:
                if (byte == FLAG)
                {
                    state = CFLAG_RCV;
                }
                break;
            case CFLAG_RCV:
                if (byte == DISC_A_tx)
                {
                    state = CA_RCV;
                }
                else if (byte == FLAG)
                {
                    state = CFLAG_RCV;
                }
                else
                {
                    state = CSTART_STATE;
                }
                break;
            case CA_RCV:
                if (byte == DISC_C_tx)
                {
                    state = CC_RCV;
                }
                else if (byte == FLAG)
                {
                    state = CFLAG_RCV;
                }
                else
                {
                    state = CSTART_STATE;
                }
                break;
            case CC_RCV:
                if (byte == (DISC_A_tx ^ DISC_C_tx))
                {
                    state = CBCC_OK;
                }
                else if (byte == FLAG)
                {
                    state = CFLAG_RCV;
                }
                else
                {
                    state = CSTART_STATE;
                }
                break;
            case CBCC_OK:
                if (byte == FLAG)
                {
                    state = CSTOP_STATE;
                }
                else
                {
                    state = CSTART_STATE;
                }
                break;
            default:
                state = CSTART_STATE;
                break;
            }
        }

        printf("DISC frame from transmitter received correctly.\n");

        int bytes_sent = writeBytesSerialPort(DISC_reply, 5);
        printf("%d bytes written to serial port (DISC reply)\n", bytes_sent);

        state = CSTART_STATE;
        unsigned char UA_A = 0x01, UA_C = 0x07;

        while (state != CSTOP_STATE)
        {
            int res = readByteSerialPort(&byte);
            if (res <= 0)
            {
                continue;
            }

            switch (state)
            {
            case CSTART_STATE:
                if (byte == FLAG)
                {
                    state = CFLAG_RCV;
                }
                break;
            case CFLAG_RCV:
                if (byte == UA_A)
                {
                    state = CA_RCV;
                }
                else if (byte == FLAG)
                {
                    state = CFLAG_RCV;
                }
                else
                {
                    state = CSTART_STATE;
                }
                break;
            case CA_RCV:
                if (byte == UA_C)
                {
                    state = CC_RCV;
                }
                else if (byte == FLAG)
                {
                    state = CFLAG_RCV;
                }
                else
                {
                    state = CSTART_STATE;
                }
                break;
            case CC_RCV:
                if (byte == (UA_A ^ UA_C))
                {
                    state = CBCC_OK;
                }
                else if (byte == FLAG)
                {
                    state = CFLAG_RCV;
                }
                else
                {
                    state = CSTART_STATE;
                }
                break;
            case CBCC_OK:
                if (byte == FLAG)
                {
                    state = CSTOP_STATE;
                }
                else
                {
                    state = CSTART_STATE;
                }
                break;
            default:
                state = CSTART_STATE;
                break;
            }
        }

        printf("UA frame received correctly.\n");

        sleep(1);

        if (closeSerialPort() < 0)
        {
            perror("closeSerialPort");
            return -1;
        }

        printf("Serial port %s closed (llclose - Rx)\n", connectionParameters.serialPort);
        return 0;
    }

    printf("error determining the role using llclose.\n");
    return -1;
}

void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;
}
