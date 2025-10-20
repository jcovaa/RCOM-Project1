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

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    // TODO: Implement this function

    return 0;
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
