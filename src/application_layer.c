// Application layer protocol implementation

#include "application_layer.h"

LinkLayerRole getRoleFromString(const char *roleStr)
{
    if (strcmp(roleStr, "TRANSMITTER") == 0)
        return LlTx;
    else if (strcmp(roleStr, "RECEIVER") == 0)
        return LlRx;
    else
    {
        fprintf(stderr, "Invalid role: %s\n", roleStr);
        exit(EXIT_FAILURE);
    }
}

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{

    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    if (strcmp(role, "tx") == 0)
    {
        connectionParameters.role = LlTx;
    }
    else
    {
        connectionParameters.role = LlRx;
    }
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.timeout = timeout;

    printf("Testing llopen\n");
    if (llopen(connectionParameters) == -1)
    {
        printf("Error occured\n");
        return;
    }

    printf("Testing llclose\n");
    if (llclose(connectionParameters) == -1)
    {
        printf("Error occured\n");
        return;
    }

    printf("\nEverything right\n");
    return;
}
