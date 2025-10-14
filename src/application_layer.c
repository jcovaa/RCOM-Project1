// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include <string.h>

#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{

    LinkLayer connectionParameters;
    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    strcpy(connectionParameters.role, role);
    strcpy(connectionParameters.serialPort, serialPort);
    connectionParameters.timeout = timeout;

    printf("Testing llopen\n");
    if (llopen(connectionParameters) == -1) {
        printf("Error occured\n");
        return;
    }

    printf("\nEverything right\n");
    return;
}
