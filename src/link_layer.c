// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FLAG 0x7E

#define A_TR_R 0x03
#define A_R_TR 0x01

#define C_SET 0x03
#define C_UA 0x07
#define C_DISC 0x0B

#define C_I(Ns) (Ns << 7)
#define C_RR(Nr) ((Nr) ? 0xAB : 0xAA)
#define C_REJ(Nr) ((Nr) ? 0x55 : 0x54)

#define ESC 0x7D
#define STUFF_XOR 0x20

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

        unsigned char SET[5] = {FLAG, A_TR_R, C_SET, A_TR_R ^ C_SET, FLAG};

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
                    if (byte == A_TR_R)
                        state = CA_RCV;
                    else if (byte == FLAG)
                        state = CFLAG_RCV;
                    else
                        state = CSTART_STATE;
                    break;
                case CA_RCV:
                    if (byte == C_UA)
                        state = CC_RCV;
                    else if (byte == FLAG)
                        state = CFLAG_RCV;
                    else
                        state = CSTART_STATE;
                    break;
                case CC_RCV:
                    if (byte == (A_TR_R ^ C_UA))
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
                if (byte == A_TR_R)
                    state = CA_RCV;
                else if (byte == FLAG)
                    state = CFLAG_RCV;
                else
                    state = CSTART_STATE;
                break;
            case CA_RCV:
                if (byte == C_SET)
                    state = CC_RCV;
                else if (byte == FLAG)
                    state = CFLAG_RCV;
                else
                    state = CSTART_STATE;
                break;
            case CC_RCV:
                if (byte == (A_TR_R ^ C_SET))
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

        unsigned char UA[5] = {FLAG, A_TR_R, C_UA, A_TR_R ^ C_UA, FLAG};
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
    tempFrame[idx++] = A_TR_R;
    tempFrame[idx++] = C_I(Ns);
    tempFrame[idx++] = A_TR_R ^ C_I(Ns); // BCC1

    unsigned char BCC2 = 0x00;
    for (int i = 0; i < bufSize; i++)
    {
        BCC2 ^= buf[i];
    }

    // Copy data payload
    memcpy(&tempFrame[idx], buf, bufSize);
    idx += bufSize;
    tempFrame[idx++] = BCC2;

    unsigned char stuffedFrame[2 * (bufSize + 6)];
    stuffedFrame[0] = FLAG;

    int stuffedLen = byteStuffing(tempFrame + 1, idx - 1, stuffedFrame + 1);

    stuffedFrame[1 + stuffedLen] = FLAG;
    int frameSize = 1 + stuffedLen + 1;

    struct sigaction act = {0};
    act.sa_handler = &alarmHandler;
    if (sigaction(SIGALRM, &act, NULL) == -1)
    {
        perror("sigaction");
        return -1;
    }

    int attempts = 0;
    while (attempts < nrTries)
    {
        if (!alarmEnabled)
        {
            printf("Sending I frame with Ns=%d (attempt %d)\n", Ns, alarmCount + 1);
            writeBytesSerialPort(stuffedFrame, frameSize);
            alarm(timeout);
            alarmEnabled = TRUE;
        }

        int response = readResponse(Ns);

        if (response == 1)
        {
            printf("RR received, frame acknowledged.\n");
            alarm(0);
            alarmEnabled = FALSE;
            Ns = (Ns + 1) % 2; // Toggle Ns
            return bufSize;
        }
        else if (response == -1)
        {
            printf("REJ received, retransmitting frame.\n");
            alarm(0);
            alarmEnabled = FALSE;
            attempts++;
        }
        else if (response == 0)
        {
            printf("Timeout waiting for RR/REJ, retransmitting frame.\n");
            attempts++;
        }
    }

    printf("Failed to receive RR after %d attempts, llwrite failed.\n", nrTries);
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

int readResponse(int Ns)
{
    unsigned char byte;
    ControlState state = CSTART_STATE;
    unsigned char C_field = 0;
    int expectedNr = (Ns + 1) % 2;
    int startAlamrs = alarmCount;

    while (TRUE)
    {
        if (!alarmEnabled && alarmCount > startAlamrs)
            return 0;

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
            if (byte == A_R_TR)
                state = CA_RCV;
            else if (byte == FLAG)
                state = CFLAG_RCV;
            else
                state = CSTART_STATE;
            break;
        case CA_RCV:
            // RR with Nr = (Ns + 1) % 2
            if (byte == C_RR(expectedNr))
            {
                C_field = byte;
                state = CC_RCV;
            }
            // REJ with Nr = Ns
            else if (byte == C_REJ(Ns))
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
            if (byte == (A_R_TR ^ C_field))
                state = CBCC_OK;
            else if (byte == FLAG)
                state = CFLAG_RCV;
            else
                state = CSTART_STATE;
            break;
        case CBCC_OK:
            if (byte == FLAG)
            {
                if (C_field == C_REJ(Ns))
                    return -1; // REJ received
                else
                    return 1; // RR received
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

    for (;;)
    {
        unsigned char byte;
        InformationState state = ISTART_STATE;

        unsigned char stuffedFrame[MAX_PAYLOAD_SIZE * 2 + 10];
        int stuffedLen = 0;

        while (state != ISTOP_STATE)
        {
            int res = readByteSerialPort(&byte);
            if (res <= 0)
                continue;

            switch (state)
            {
            case ISTART_STATE:
                if (byte == FLAG)
                {
                    state = IFLAG_RCV;
                    stuffedLen = 0;
                }
                break;
            case IFLAG_RCV:
                if (byte == FLAG)
                    state = IFLAG_RCV;
                else
                {
                    stuffedFrame[stuffedLen++] = byte;
                    state = IDATA_RCV;
                }
                break;
            case IDATA_RCV:
                if (byte == FLAG)
                    state = ISTOP_STATE;
                else
                    stuffedFrame[stuffedLen++] = byte;
                break;
            default:
                state = ISTART_STATE;
                break;
            }
        }

        unsigned char destuffedFrame[MAX_PAYLOAD_SIZE + 10];
        int destuffedLen = byteDestuffing(stuffedFrame, stuffedLen, destuffedFrame);

        if (destuffedLen < 4)
        {
            printf("Frame too short after destuffing (%d bytes).\n", destuffedLen);
            continue;
        }

        unsigned char A_field = destuffedFrame[0];
        unsigned char C_field = destuffedFrame[1];
        unsigned char BCC1 = destuffedFrame[2];

        if (A_field != A_TR_R)
        {
            printf("Invalid A field: 0x%02X\n", A_field);
            continue; // silently discard
        }

        if (C_field != C_I(0) && C_field != C_I(1))
        {
            continue;
        }

        // Validate BCC1
        if (BCC1 != (A_field ^ C_field))
        {
            printf("BCC1 validation failed, frame discarded.\n");
            continue;
        }

        int dataLen = destuffedLen - 4;
        unsigned char receivedBCC2 = destuffedFrame[destuffedLen - 1];

        unsigned char calculatedBCC2 = 0x00;
        for (int i = 0; i < dataLen; i++)
        {
            calculatedBCC2 ^= destuffedFrame[3 + i];
        }

        int receivedNs = (C_field >> 7) & 0x01;
        if (receivedBCC2 != calculatedBCC2)
        {
            printf("BCC2 error detected (expected 0x%02X, got 0x%02X), sending REJ(%d).\n",
                   calculatedBCC2, receivedBCC2, expectedNs);
            unsigned char REJ[5] = {FLAG, A_R_TR, C_REJ(expectedNs), A_R_TR ^ C_REJ(expectedNs), FLAG};
            writeBytesSerialPort(REJ, 5);
            continue; // wait for retransmission
        }

        if (receivedNs == expectedNs)
        {
            printf("Valid I frame received with Ns=%d, sending RR(%d).\n", receivedNs, (expectedNs + 1) % 2);

            memcpy(packet, destuffedFrame + 3, dataLen);

            int nextNr = (expectedNs + 1) % 2;
            unsigned char RR[5] = {FLAG, A_R_TR, C_RR(nextNr), A_R_TR ^ C_RR(nextNr), FLAG};
            writeBytesSerialPort(RR, 5);

            expectedNs = nextNr;

            return dataLen;
        }
        else
        {
            printf("Duplicate I frame received with Ns=%d, re-sending RR(%d).\n", receivedNs, expectedNs);

            unsigned char RR[5] = {FLAG, A_R_TR, C_RR(expectedNs), A_R_TR ^ C_RR(expectedNs), FLAG};
            writeBytesSerialPort(RR, 5);

            continue; // duplicate
        }
    }
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

        unsigned char DISC_tx[5] = {FLAG, A_TR_R, C_DISC, A_TR_R ^ C_DISC, FLAG};
        unsigned char DISC_A_rx = A_R_TR, DISC_C_rx = C_DISC;

        unsigned char UA_tx[5] = {FLAG, A_R_TR, C_UA, A_R_TR ^ C_UA, FLAG};

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
