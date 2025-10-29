// Application layer protocol implementation

#include "application_layer.h"

#define C_START 1
#define C_DATA 2
#define C_END 3

#define T_FILE_SIZE 0
#define T_FILE_NAME 1

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters;
    memset(&connectionParameters, 0, sizeof(connectionParameters));

    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.role = (role && strcmp(role, "tx") == 0) ? LlTx : LlRx;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (llopen(connectionParameters) == -1)
    {
        printf("llopen failed.\n");
        return;
    }

    int result = 0;

    if (connectionParameters.role == LlTx)
    {
        // Transmitter
        if (!filename)
        {
            printf("Transmitter: missing filename.\n");
            result = -1;
        }
        else
        {
            FILE *f = fopen(filename, "rb");
            if (!f)
            {
                perror("fopen");
                result = -1;
            }
            else
            {
                uint32_t fsize = get_file_size(f);

                // START control packet
                unsigned char pkt[MAX_PAYLOAD_SIZE];
                int pkt_len = 0;
                if (build_control_packet(C_START, filename, fsize, pkt, &pkt_len) != 0)
                {
                    printf("Failed to build START packet.\n");
                    result = -1;
                }
                else if (llwrite(pkt, pkt_len) < 0)
                {
                    printf("llwrite START failed.\n");
                    result = -1;
                }
                else
                {
                    // DATA packets
                    const int chunk = MAX_PAYLOAD_SIZE - 3;
                    unsigned char buf[4096];
                    uint32_t sent = 0;

                    while (!feof(f))
                    {
                        size_t to_read = (chunk < (int)sizeof(buf)) ? (size_t)chunk : sizeof(buf);
                        size_t n = fread(buf, 1, to_read, f);
                        if (n == 0)
                        {
                            if (ferror(f))
                            {
                                perror("fread");
                                result = -1;
                            }
                            break;
                        }

                        int data_pkt_len = 0;
                        if (build_data_packet(buf, (int)n, pkt, &data_pkt_len) != 0)
                        {
                            printf("Failed to build DATA packet.\n");
                            result = -1;
                            break;
                        }
                        if (llwrite(pkt, data_pkt_len) < 0)
                        {
                            printf("llwrite DATA failed.\n");
                            result = -1;
                            break;
                        }
                        sent += (uint32_t)n;
                    }

                    if (result == 0)
                    {
                        // END control packet
                        if (build_control_packet(C_END, filename, fsize, pkt, &pkt_len) != 0)
                        {
                            printf("Failed to build END packet.\n");
                            result = -1;
                        }
                        else if (llwrite(pkt, pkt_len) < 0)
                        {
                            printf("llwrite END failed.\n");
                            result = -1;
                        }
                    }

                    if (sent != fsize)
                    {
                        printf("Warning: sent %u bytes but advertised %u.\n", sent, fsize);
                    }
                }

                fclose(f);
            }
        }
    }
    else
    {
        // Receiver
        unsigned char pkt[MAX_PAYLOAD_SIZE];
        int n = llread(pkt);
        if (n < 0)
        {
            printf("llread START failed.\n");
            result = -1;
        }
        else
        {
            uint8_t ctrl = 0;
            char start_name[256] = {0};
            uint32_t start_size = 0;
            if (parse_control_packet(pkt, n, &ctrl, start_name, sizeof(start_name), &start_size) != 0 || ctrl != C_START)
            {
                printf("Expected START control packet.\n");
                result = -1;
            }
            else
            {
                const char *out_name = NULL;
                if (filename && *filename)
                    out_name = filename;
                else if (*start_name)
                    out_name = start_name;
                else
                    out_name = "received_file.bin";

                FILE *f = fopen(out_name, "wb");
                if (!f)
                {
                    perror("fopen");
                    result = -1;
                }
                else
                {
                    uint32_t written = 0;
                    for (;;)
                    {
                        n = llread(pkt);
                        if (n < 0)
                        {
                            printf("llread failed during transfer.\n");
                            result = -1;
                            break;
                        }
                        if (n == 0)
                            continue;

                        if (pkt[0] == C_DATA)
                        {
                            const unsigned char *data = NULL;
                            int data_len = 0;
                            if (parse_data_packet(pkt, n, &data, &data_len) != 0)
                            {
                                printf("Invalid DATA packet, ignoring.\n");
                                continue;
                            }
                            size_t w = fwrite(data, 1, (size_t)data_len, f);
                            if ((int)w != data_len)
                            {
                                perror("fwrite");
                                result = -1;
                                break;
                            }
                            written += (uint32_t)w;
                        }
                        else if (pkt[0] == C_END)
                        {
                            uint8_t end_ctrl = 0;
                            char end_name[256] = {0};
                            uint32_t end_size = 0;
                            if (parse_control_packet(pkt, n, &end_ctrl, end_name, sizeof(end_name), &end_size) != 0 || end_ctrl != C_END)
                            {
                                printf("Invalid END control packet.\n");
                                result = -1;
                            }
                            else if (end_size != start_size ||
                                     ((*start_name || *end_name) && strcmp(start_name, end_name) != 0))
                            {
                                printf("END mismatch.\n");
                                result = -1;
                            }
                            break;
                        }
                        else
                        {
                            // Ignore unknown packets
                            continue;
                        }
                    }

                    fclose(f);
                    if (result == 0)
                    {
                        if (written != start_size)
                            printf("Warning: wrote %u bytes but advertised %u.\n", written, start_size);
                        else
                            printf("Received file '%s' (%u bytes).\n", out_name, written);
                    }
                }
            }
        }
    }

    if (llclose(connectionParameters) == -1)
    {
        printf("llclose failed.\n");
        return;
    }

    if (result == 0)
        printf("Application transfer completed successfully.\n");
    else
        printf("Application transfer failed.\n");
}

int build_control_packet(uint8_t ctrl, const char *filename, uint32_t filesize,
                         unsigned char *out, int *out_len)
{
    if (!out || !out_len)
        return -1;

    int idx = 0;
    out[idx++] = ctrl;

    out[idx++] = T_FILE_SIZE;
    out[idx++] = 4;
    out[idx++] = (filesize >> 24) & 0xFF;
    out[idx++] = (filesize >> 16) & 0xFF;
    out[idx++] = (filesize >> 8) & 0xFF;
    out[idx++] = (filesize) & 0xFF;

    // File name
    size_t name_len = filename ? strlen(filename) : 0;
    out[idx++] = T_FILE_NAME;
    out[idx++] = (uint8_t)name_len;
    if (name_len > 0)
    {
        memcpy(&out[idx], filename, name_len);
        idx += (int)name_len;
    }

    *out_len = idx;
    if (*out_len > MAX_PAYLOAD_SIZE)
        return -1;
    return 0;
}

int parse_control_packet(const unsigned char *pkt, int pkt_len,
                         uint8_t *ctrl, char *filename_buf, size_t filename_buf_sz,
                         uint32_t *filesize)
{
    if (!pkt || pkt_len < 1)
        return -1;
    int idx = 0;
    *ctrl = pkt[idx++];

    uint32_t size = 0;
    char name_local[256] = {0};
    int have_size = 0, have_name = 0;

    while (idx + 2 <= pkt_len)
    {
        uint8_t T = pkt[idx++];
        uint8_t L = pkt[idx++];
        if (idx + L > pkt_len)
            return -1;

        if (T == T_FILE_SIZE && L == 4)
        {
            size = ((uint32_t)pkt[idx] << 24) |
                   ((uint32_t)pkt[idx + 1] << 16) |
                   ((uint32_t)pkt[idx + 2] << 8) |
                   (uint32_t)pkt[idx + 3];
            have_size = 1;
        }
        else if (T == T_FILE_NAME)
        {
            size_t cpy = (L < sizeof(name_local) - 1) ? L : (sizeof(name_local) - 1);
            memcpy(name_local, &pkt[idx], cpy);
            name_local[cpy] = '\0';
            have_name = 1;
        }
        idx += L;
    }

    if (filesize)
        *filesize = size;
    if (filename_buf && filename_buf_sz > 0)
    {
        if (have_name)
        {
            strncpy(filename_buf, name_local, filename_buf_sz - 1);
            filename_buf[filename_buf_sz - 1] = '\0';
        }
        else
        {
            filename_buf[0] = '\0';
        }
    }
    return (have_size ? 0 : -1);
}

int build_data_packet(const unsigned char *data, int len,
                      unsigned char *out, int *out_len)
{
    if (!data || !out || !out_len || len < 0)
        return -1;
    if (len > MAX_PAYLOAD_SIZE - 3)
        return -1; // C + L2 + L1 overhead

    out[0] = C_DATA;
    out[1] = (uint8_t)((len >> 8) & 0xFF);
    out[2] = (uint8_t)(len & 0xFF);
    memcpy(out + 3, data, len);
    *out_len = 3 + len;
    return 0;
}

int parse_data_packet(const unsigned char *pkt, int pkt_len,
                      const unsigned char **data_out, int *data_len)
{
    if (!pkt || pkt_len < 3)
        return -1;
    if (pkt[0] != C_DATA)
        return -1;
    int K = ((int)pkt[1] << 8) | pkt[2];
    if (K != pkt_len - 3)
        return -1;
    if (data_out)
        *data_out = pkt + 3;
    if (data_len)
        *data_len = K;
    return 0;
}

uint32_t get_file_size(FILE *f)
{
    if (!f)
        return 0;
    long cur = ftell(f);
    if (cur < 0)
        cur = 0;
    if (fseek(f, 0, SEEK_END) != 0)
        return 0;
    long sz = ftell(f);
    if (sz < 0)
        sz = 0;
    (void)fseek(f, cur, SEEK_SET);
    if (sz > 0xFFFFFFFFL)
        sz = 0xFFFFFFFFL;
    return (uint32_t)sz;
}