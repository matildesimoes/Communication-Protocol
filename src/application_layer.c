// Application layer protocol implementation

#include "application_layer.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "link_layer.h"

#define DATA_PACKET 1
#define CONTROL_PACKET_START 2
#define CONTROL_PACKET_END 3

#define CONTROL_PACKET_FILE_SIZE 0
#define CONTROL_PACKET_FILE_NAME 1

#define MAX_DATA_SIZE 256

// Imprime "Application Layer" seguido do título e do conteúdo
void printAL(char *title, unsigned char *content, int contentSize) {
    // DEBUG
    printf("\nApplication Layer\n");
    for (int i = 0; title[i] != '\0'; i++) printf("%c", title[i]);
    printf("\n");
    for (int i = 0; i < contentSize; i++) printf("0x%x ", content[i]);
    printf("\n");
}

// Calcula o logaritmo de base 2 de n
char logaritmo2(int n) {
    char res = -1;
    while (n > 0) {
        n /= 2;
        res++;
    }
    return res;
}

// Constrói e retorna um pacote de controlo de tipo (START/END) dado por 'controlField', com o tamanho do ficheiro e o nome do ficheiro
unsigned char *buildControlPacket(unsigned char controlField, long int fileSize, const char *fileName, int *packetSize) {
    unsigned char fileSizeLength = 1 + (logaritmo2(fileSize) / 8);  // número de bits necessários para representar o tamanho do ficheiro
    unsigned char fileNameLength = strlen(fileName);                // comprimento do nome do ficheiro

    *packetSize = 5 + fileSizeLength + fileNameLength;  // 5 -> C + T1 + L1 + T2 + L2
    unsigned char *controlPacket = (unsigned char *)malloc(*packetSize);

    controlPacket[0] = controlField;              // C
    controlPacket[1] = CONTROL_PACKET_FILE_SIZE;  // T1
    controlPacket[2] = fileSizeLength;            // L1

    int index;
    for (index = 3; index < (fileSizeLength + 3); index++) {
        // V1 - tamanho do ficheiro
        unsigned leftmost = fileSize & 0xFF << (fileSizeLength - 1) * 8;
        leftmost >>= (fileSizeLength - 1) * 8;
        fileSize <<= 8;
        controlPacket[index] = leftmost;
    }

    controlPacket[index++] = CONTROL_PACKET_FILE_NAME;        // T2
    controlPacket[index++] = fileNameLength;                  // L2
    memcpy(controlPacket + index, fileName, fileNameLength);  // V2 - nome do ficheiro

    printAL("Pacote de Controlo Construído", controlPacket, *packetSize);  // DEBUG

    return controlPacket;
}

// Constrói e retorna um pacote de dados
unsigned char *buildDataPacket(int dataSize, unsigned char *data, int *packetSize) {
    *packetSize = dataSize + 3;  // 3 -> C + L2 + L1
    unsigned char *dataPacket = (unsigned char *)malloc(*packetSize);

    dataPacket[0] = DATA_PACKET;     // C
    dataPacket[1] = dataSize / 256;  // L1
    dataPacket[2] = dataSize % 256;  // L2
    // dataSize = 256 * L2 + L1

    memcpy(dataPacket + 3, data, dataSize);  // dados

    printAL("Pacote de Dados Construído", dataPacket, *packetSize);  // DEBUG

    return dataPacket;
}

// Envia um pacote de dados com 'size' dados do conteúdo do ficheiro
void sendDataPacket(int size, unsigned char *fileContent) {
    unsigned char *data;
    data = (unsigned char *)malloc(size);
    memcpy(data, fileContent, size);  // dados

    int dataPacketSize;
    unsigned char *dataPacket = buildDataPacket(size, data, &dataPacketSize);

    if (llwrite(dataPacket, dataPacketSize) < 0) {
        printf("Erro a enviar um pacote de dados com %d bytes\n", dataPacketSize);
        exit(-1);
    }

    free(data);
}

// Lê e interpreta um pacote de controlo, retirando o tamanho do ficheiro e retornando o nome do ficheiro
char *parseControlPacket(unsigned char *packet, int *fileSize) {
    unsigned char fileSizeLength = packet[2];
    *fileSize = 0;
    for (unsigned char i = 3; i < (fileSizeLength + 3); i++) {
        *fileSize |= packet[i];
        *fileSize <<= 8;
    }
    *fileSize >>= 8;
    unsigned char fileNameLength = packet[fileSizeLength + 4];
    char *fileName = (char *)malloc(fileNameLength + 1);
    memcpy(fileName, packet + fileSizeLength + 5, fileNameLength);

    printAL("Pacote de Controlo Recebido", packet, fileSizeLength + fileNameLength + 5);  // DEBUG

    return fileName;
}

void applicationLayer(const char *serialPort, const char *role, int baudRate, int nTries, int timeout, const char *filename) {
    LinkLayer connectionParameters;
    strcpy(connectionParameters.serialPort, serialPort);
    if (strcmp(role, "tx") == 0)  // role == "tx"
        connectionParameters.role = LlTx;
    else if (strcmp(role, "rx") == 0)  // role == "rx"
        connectionParameters.role = LlRx;
    else {
        printf("role = %s (deve ser tx ou rx)\n", role);
        exit(-1);
    }

    connectionParameters.baudRate = baudRate;
    connectionParameters.nRetransmissions = nTries;
    connectionParameters.timeout = timeout;

    if (llopen(connectionParameters) < 0) {
        printf("Erro a estabelecer a ligação\n");
        exit(-1);
    }

    if (connectionParameters.role == LlTx) {
        FILE *file = fopen(filename, "rb");
        if (file == NULL) {
            printf("Erro a abrir o ficheiro %s para ler\n", filename);
            exit(-1);
        }

        long int fileSize;
        struct stat st;
        if (stat(filename, &st) == 0) {
            fileSize = st.st_size;
            printf("O tamanho do ficheiro é %ld bytes\n", fileSize);  // DEBUG
        } else {
            printf("Erro a obter o tamanho do ficheiro\n");
            exit(-1);
        }

        // Construir e enviar pacote de controlo 'start'
        int startControlPacketSize;
        unsigned char *startControlPacket = buildControlPacket(CONTROL_PACKET_START, fileSize, filename, &startControlPacketSize);
        if (llwrite(startControlPacket, startControlPacketSize) < 0) {
            printf("Erro a enviar pacote de controlo 'start'\n");
            exit(-1);
        }

        unsigned char *fileContent = (unsigned char *)malloc(fileSize * sizeof(unsigned char));
        fread(fileContent, sizeof(unsigned char), fileSize, file);

        int completePackets = fileSize / MAX_DATA_SIZE;
        int incompletePacketSize = fileSize % MAX_DATA_SIZE;

        // Enviar pacotes de dados 'completos'
        for (int i = 0; i < completePackets; i++) {
            sendDataPacket(MAX_DATA_SIZE, fileContent);
            fileContent += MAX_DATA_SIZE;
        }

        // Enviar pacote de dados 'incompleto' (caso exista)
        if (incompletePacketSize != 0) {
            sendDataPacket(incompletePacketSize, fileContent);
        }

        // Construir e enviar pacote de controlo 'end'
        int endControlPacketSize;
        unsigned char *endControlPacket = buildControlPacket(CONTROL_PACKET_END, fileSize, filename, &endControlPacketSize);
        if (llwrite(endControlPacket, endControlPacketSize) < 0) {
            printf("Erro a enviar pacote de controlo 'end'\n");
            exit(-1);
        }

        fclose(file);
    } else if (connectionParameters.role == LlRx) {
        FILE *newFile = fopen(filename, "wb");
        if (newFile == NULL) {
            printf("Erro a abrir o ficheiro %s para escrever\n", filename);
            exit(-1);
        }

        unsigned char *packet = (unsigned char *)malloc(MAX_DATA_SIZE);
        while (TRUE) {
            if (llread(packet) > 0) {
                if (packet[0] == CONTROL_PACKET_START) {
                    int fileSize;
                    char *newFileName = parseControlPacket(packet, &fileSize);
                    printf("Início da receção do ficheiro %s (%d bytes)\n", newFileName, fileSize);
                } else if (packet[0] == DATA_PACKET) {
                    int dataSize = packet[1] * 256 + packet[2];
                    fwrite(packet + 3, sizeof(unsigned char), dataSize, newFile);

                    printAL("Pacote de Dados Recebido", packet, dataSize + 3);  // DEBUG
                } else if (packet[0] == CONTROL_PACKET_END) {
                    int fileSize;
                    char *newFileName = parseControlPacket(packet, &fileSize);
                    printf("Fim da receção do ficheiro %s (%d bytes)\n", newFileName, fileSize);
                    break;
                }
            }
        }

        fclose(newFile);
        free(packet);
    }

    if (llclose(TRUE) < 0) {
        printf("Erro a concluir a ligação\n");
        exit(-1);
    }
}
