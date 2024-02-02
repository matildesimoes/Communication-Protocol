// Link layer protocol implementation

#include "link_layer.h"

#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

// MISC
#define _POSIX_SOURCE 1  // POSIX compliant source

#define FLAG 0x7E
#define A 0x03
#define A_CLOSE 0x01
#define C_SET 0x03
#define C_UA 0x07
#define C_RR(r) (((r) << 7) | 0x05)
#define C_REJ(r) (((r) << 7) | 0x01)
#define C_DISC 0x0B

#define N(s) ((s) << 6)

#define ESC 0x7D
#define FLAG_ESCAPED 0x5E
#define ESC_ESCAPED 0x5D

typedef enum {
    START_STATE,
    FLAG_RCV_STATE,
    A_RCV_STATE,
    C_RCV_STATE,
    BCC_OK_STATE,
    STOP_STATE
} State;

int fd;
int alarmEnabled = FALSE;
int alarmCount = 0;
int nRetransmissions;
int timeout;
LinkLayerRole role;

// Estatísticas
clock_t start;
int totalTramas = 0;
int totalTramasI = 0;
int totalTramasSU = 0;
int totalSET = 0;
int totalUA = 0;
int totalRR = 0;
int totalREJ = 0;
int totalDISC = 0;
int totalBytes = 0;
int totalRetransmissions = 0;
int totalBCC1 = 0;
int totalBCC2 = 0;
int totalDuplicados = 0;
int totalOpen = 0;
int totalWrite = 0;
int totalRead = 0;
int totalClose = 0;
int totalStuffed = 0;
int totalFlagStuffed = 0;
int totalEscStuffed = 0;

// Imprime "Link Layer" seguido do título e do conteúdo
void printLL(char *title, unsigned char *content, int contentSize) {
    // DEBUG
    /*printf("\nLink Layer\n");
    for (int i = 0; title[i] != '\0'; i++) printf("%c", title[i]);
    printf("\n");
    for (int i = 0; i < contentSize; i++) printf("0x%x ", content[i]);
    printf("\n");*/
}

// Converte int em speed_t
speed_t get_baudrate(int baudrate) {
    switch (baudrate) {
        case 50:
            return B50;
        case 75:
            return B75;
        case 110:
            return B110;
        case 134:
            return B134;
        case 150:
            return B150;
        case 200:
            return B200;
        case 300:
            return B300;
        case 600:
            return B600;
        case 1200:
            return B1200;
        case 2400:
            return B2400;
        case 4800:
            return B4800;
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 500000:
            return B500000;
        case 576000:
            return B576000;
        case 921600:
            return B921600;
        case 1000000:
            return B1000000;
        case 1152000:
            return B1152000;
        case 1500000:
            return B1500000;
        case 2000000:
            return B2000000;
        case 2500000:
            return B2500000;
        case 3000000:
            return B3000000;
        case 3500000:
            return B3500000;
        case 4000000:
            return B4000000;
        default:
            return B0;
    }
}

// Lida com uma interrupção do alarme: desativa-o, incrementa um contador e imprime "ALARM"
void alarmHandler(int signal) {
    alarmEnabled = FALSE;
    alarmCount++;
    printf("\nALARM\n");
}

/**
 * Máquina de estados que processa cada byte lido da porta série
 * @param a valor esperado no campo A
 * @param c1 um dos possíveis valores esperados no campo C
 * @param c2 outro dos possíveis valores esperados no campo C
 * @param aCheck valor lido do campo A
 * @param cCheck valor lido do campo C
 * @param state estado atual
 *
 * @details
 * A existência dos parâmetros c1 e c2 permite aproveitar a mesma máquina de estados para llopen, llwrite, llread e llclose
 * Em llopen, só existe um valor esperado para o campo C (C_SET ou C_UA), pelo que c1 = c2
 * Em llread, existem dois valores esperados para o campo C (C_RR e C_REJ), pelo que c1 != c2
 * Em llwrite, existem dois valores esperados para o campo C (N(0) e N(1)), pelo que c1 != c2
 * Em llclose, só existe um valor esperado para o campo C (C_DISC), pelo que c1 = c2
 */
void processByte(unsigned char a, unsigned char c1, unsigned char c2, unsigned char *aCheck, unsigned char *cCheck, State *state) {
    unsigned char byteRead;

    if (read(fd, &byteRead, sizeof(byteRead)) == sizeof(byteRead)) {
        totalBytes++;
        printLL("Byte Lido", &byteRead, sizeof(byteRead));  // DEBUG
        switch (*state) {
            case START_STATE:
                if (byteRead == FLAG)
                    *state = FLAG_RCV_STATE;
                else
                    *state = START_STATE;
                break;
            case FLAG_RCV_STATE:
                if (byteRead == FLAG)
                    *state = FLAG_RCV_STATE;
                else if (byteRead == a) {
                    *aCheck = byteRead;
                    *state = A_RCV_STATE;
                } else
                    *state = START_STATE;
                break;
            case A_RCV_STATE:
                if (byteRead == FLAG)
                    *state = FLAG_RCV_STATE;
                else if (byteRead == c1 || byteRead == c2) {
                    *cCheck = byteRead;
                    *state = C_RCV_STATE;
                } else
                    *state = START_STATE;
                break;
            case C_RCV_STATE:
                if (byteRead == FLAG)
                    *state = FLAG_RCV_STATE;
                else if (byteRead == (*aCheck ^ *cCheck))
                    *state = BCC_OK_STATE;
                else {
                    totalBCC1++;
                    *state = START_STATE;
                }
                break;
            case BCC_OK_STATE:
                if (byteRead == FLAG)
                    *state = STOP_STATE;
                else
                    *state = START_STATE;
                break;
            default:
                break;
        }
    }
}

////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters) {
    totalOpen++;
    start = clock();
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("Erro a abrir a porta série %s\n", connectionParameters.serialPort);
        return -1;
    }

    struct termios oldtio;
    struct termios newtio;

    if (tcgetattr(fd, &oldtio) == -1) {
        printf("Erro a usar tcgetattr\n");
        return -1;
    }

    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = get_baudrate(connectionParameters.baudRate) | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0;
    newtio.c_cc[VMIN] = 0;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &newtio) == -1) {
        printf("Erro a usar tcsetattr\n");
        return -1;
    }

    nRetransmissions = connectionParameters.nRetransmissions;
    timeout = connectionParameters.timeout;
    role = connectionParameters.role;

    State state = START_STATE;
    unsigned char aCheck;
    unsigned char cCheck;

    if (connectionParameters.role == LlTx) {
        (void)signal(SIGALRM, alarmHandler);
        int tries = nRetransmissions;
        unsigned char set[5] = {FLAG, A, C_SET, A ^ C_SET, FLAG};

        do {
            printLL("LLOPEN - enviado SET", set, sizeof(set));  // DEBUG
            totalTramas++;
            totalTramasSU++;
            totalSET++;
            write(fd, set, sizeof(set));
            alarm(timeout);
            alarmEnabled = TRUE;
            while (alarmEnabled == TRUE && state != STOP_STATE) {
                // Enquanto o alarme não tiver disparado e estado não for o final, processa os bytes da porta série (um de cada vez)
                processByte(A, C_UA, C_UA, &aCheck, &cCheck, &state);  // espera um UA
            }
            if (state == STOP_STATE) {
                // O estado final foi alcançado, pelo que o alarme pode ser desativado
                alarm(0);
                alarmEnabled = FALSE;
            } else {
                // O alarme tocou, pelo que ocorreu timeout e deve haver retransmissão (se ainda não tiver sido excedido o número máximo de tentativas)
                tries--;
                totalRetransmissions++;
            }
        } while (tries >= 0 && state != STOP_STATE);

        if (state != STOP_STATE) {
            // Foi excedido o número máximo de tentativas de retransmissão
            totalRetransmissions--;
            printf("LLOPEN - UA não foi recebido\n");
            return -1;
        }
    } else if (connectionParameters.role == LlRx) {
        while (state != STOP_STATE) {
            // Processa os bytes da porta série (um de cada vez)
            processByte(A, C_SET, C_SET, &aCheck, &cCheck, &state);  // espera um SET
        }
        unsigned char ua[5] = {FLAG, A, C_UA, A ^ C_UA, FLAG};
        printLL("LLOPEN - enviado UA", ua, sizeof(ua));  // DEBUG
        totalTramas++;
        totalTramasSU++;
        totalUA++;
        write(fd, ua, sizeof(ua));  // quando receber o SET, responde com UA
    } else {
        printf("Erro em connectionParameters.role\n");
        return -1;
    }

    return 1;
}

float totalTprop = 0.0;
float nTprop = 0.0;

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    totalWrite++;
    unsigned char bcc2 = buf[0];
    for (int i = 1; i < bufSize; i++) {
        // Cálculo do BCC2
        bcc2 ^= buf[i];
    }

    unsigned char *dataBcc2 = (unsigned char *)malloc(2 * bufSize + 2);  // aloca memória dinâmica para o pior caso: ter de fazer stuffing de todos os bytes de dados e do BCC2
    int index = 0;
    for (int i = 0; i < bufSize; i++) {
        // Stuffing dos dados
        if (buf[i] == FLAG) {
            totalStuffed++;
            totalFlagStuffed++;
            dataBcc2[index++] = ESC;
            dataBcc2[index++] = FLAG_ESCAPED;
        } else if (buf[i] == ESC) {
            totalStuffed++;
            totalEscStuffed++;
            dataBcc2[index++] = ESC;
            dataBcc2[index++] = ESC_ESCAPED;
        } else {
            dataBcc2[index++] = buf[i];
        }
    }

    // Stuffing do BCC2
    if (bcc2 == FLAG) {
        totalStuffed++;
        totalFlagStuffed++;
        dataBcc2[index++] = ESC;
        dataBcc2[index++] = FLAG_ESCAPED;
    } else if (bcc2 == ESC) {
        totalStuffed++;
        totalEscStuffed++;
        dataBcc2[index++] = ESC;
        dataBcc2[index++] = ESC_ESCAPED;
    } else {
        dataBcc2[index++] = bcc2;
    }

    static unsigned char tramaI = 0;
    unsigned char n = N(tramaI);
    unsigned char bcc1 = A ^ n;

    unsigned char next = (tramaI + 1) % 2;

    // Construção do frame a transmitir
    unsigned char *frame = malloc(index + 5);  // 5 -> F A C BCC1 F;
    frame[0] = FLAG;
    frame[1] = A;
    frame[2] = n;
    frame[3] = bcc1;
    for (int i = 4; i < (index + 4); i++) {
        frame[i] = dataBcc2[i - 4];
    }
    frame[index + 4] = FLAG;

    int size = index + 5;  // 5 -> F A C BCC1 F

    State state = START_STATE;
    unsigned char aCheck;
    unsigned char cCheck;
    unsigned char accepetedCheck;
    unsigned char rejectedCheck;

    int tries = nRetransmissions;

    do {
        clock_t startTprop = clock();
        printLL("LL WRITE - frame enviado", frame, size);  // DEBUG
        totalTramas++;
        totalTramasI++;
        write(fd, frame, size);
        alarm(timeout);
        alarmEnabled = TRUE;
        accepetedCheck = FALSE;
        rejectedCheck = FALSE;
        while (alarmEnabled == TRUE && rejectedCheck == FALSE && accepetedCheck == FALSE) {
            state = START_STATE;
            while (state != STOP_STATE && alarmEnabled == TRUE) {
                // Enquanto o alarme não tiver disparado e estado não for o final, processa os bytes da porta série (um de cada vez)
                processByte(A, C_RR(next), C_REJ(tramaI), &aCheck, &cCheck, &state);  // espera um RR ou REJ
            }

            if (state == STOP_STATE) {
                // O estado final foi alcançado, pelo que o alarme pode ser desativado
                alarm(0);
                alarmEnabled = FALSE;
            } else {
                // O alarme tocou, pelo que ocorreu timeout e deve haver retransmissão (se ainda não tiver sido excedido o número máximo de tentativas)
                tries--;
                totalRetransmissions++;
                continue;
            }

            // Interpretação da Resposta
            if (cCheck == C_RR(next)) {
                // O frame enviado foi recebido e aceite - o recetor está pronto para receber o próximo frame
                accepetedCheck = TRUE;
                tramaI = next;
            } else if (cCheck == C_REJ(tramaI)) {
                // O frame enviado foi rejeitado - deve ser retransmitido (se ainda não tiver sido excedido o número máximo de tentativas)
                rejectedCheck = TRUE;
            }
        }
        clock_t endTprop = clock();
        float seconds = (float)(endTprop - startTprop) / CLOCKS_PER_SEC;
        totalTprop += seconds;
        nTprop += 1.0;
    } while (tries >= 0 && accepetedCheck == FALSE);

    if (state != STOP_STATE) {
        // Foi excedido o número máximo de tentativas de retransmissão
        totalRetransmissions--;
        printf("LLWRITE - não foi recebida resposta\n");
        return -1;
    }

    free(dataBcc2);
    free(frame);
    return size;
}

float totalTframe = 0.0;
float nTframe = 0.0;

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    totalRead++;
    static unsigned char tramaI = 0;

    State state = START_STATE;
    unsigned char byteRead;
    unsigned char aCheck;
    unsigned char cCheck;
    unsigned char escFound = FALSE;

    int index = 0;
    int size;

    while (state != BCC_OK_STATE) {
        // Enquanto o estado não for o BCC OK, processa os bytes da porta série (um de cada vez)
        processByte(A, N(0), N(1), &aCheck, &cCheck, &state);
    }
    clock_t startTframe = clock();
    if (cCheck != N(tramaI)) {
        // Recebeu uma trama de que não estava à espera (duplicada)
        totalDuplicados++;
        while (byteRead != FLAG) {
            // Lê os bytes da porta série (um de cada vez), mas ignora-os - apenas para limpar
            totalBytes++;
            read(fd, &byteRead, sizeof(byteRead));
        }
        // Responde com a indicação de qual é o índice da trama que está pronto para receber
        unsigned char n = C_RR(tramaI);
        unsigned char rr[5] = {FLAG, A, n, A ^ n, FLAG};
        printLL("LL WRITE - RR enviado", rr, sizeof(rr));  // DEBUG
        totalTramas++;
        totalTramasSU++;
        totalRR++;
        write(fd, rr, sizeof(rr));
        return -1;
    }

    while (state != STOP_STATE) {
        // Enquanto o estado não for o final, processa os bytes da porta série (um de cada vez)
        if (state == BCC_OK_STATE && read(fd, &byteRead, sizeof(byteRead)) == sizeof(byteRead)) {
            totalBytes++;
            // Destuffing dos dados e do BCC2
            if (escFound) {
                if (byteRead == FLAG_ESCAPED) {
                    totalStuffed++;
                    totalFlagStuffed++;
                    packet[index++] = FLAG;
                } else if (byteRead == ESC_ESCAPED) {
                    totalStuffed++;
                    totalEscStuffed++;
                    packet[index++] = ESC;
                }
                escFound = FALSE;
            } else if (byteRead == ESC) {
                escFound = TRUE;
            } else if (byteRead == FLAG) {
                size = index + 5;  // 5 -> F A C BCC1 F
                unsigned char bcc2 = packet[index - 1];
                index--;
                packet[index] = '\0';                                 // retira o BCC2 do pacote de dados
                printLL("LLWRITE - pacote recebido", packet, index);  // DEBUG
                unsigned char bcc2Acc = packet[0];
                for (int i = 1; i < index; i++) {
                    // Cálculo do BCC2
                    bcc2Acc ^= packet[i];
                }
                if (bcc2 == bcc2Acc) {
                    // O valor de BCC2 está correto, pelo que a trama foi recebida com sucesso e o recetor está pronto para a próxima
                    tramaI = (tramaI + 1) % 2;
                    unsigned char n = C_RR(tramaI);
                    unsigned char rr[5] = {FLAG, A, n, A ^ n, FLAG};
                    printLL("LLWRITE - RR enviado", rr, sizeof(rr));  // DEBUG
                    totalTramas++;
                    totalTramasSU++;
                    totalRR++;
                    write(fd, rr, sizeof(rr));
                    state = STOP_STATE;
                } else {
                    // O valor de BCC está incorreto, pelo que a trama deve ser retransmitida
                    totalBCC2++;
                    unsigned char n = C_REJ(tramaI);
                    unsigned char rej[5] = {FLAG, A, n, A ^ n, FLAG};
                    printLL("LLWRITE - REJ enviado", rej, sizeof(rej));  // DEBUG
                    totalTramas++;
                    totalTramasSU++;
                    totalREJ++;
                    write(fd, rej, sizeof(rej));
                    state = STOP_STATE;
                    return -1;
                }
            } else {
                packet[index++] = byteRead;
            }
        }
    }

    clock_t endTframe = clock();
    float seconds = (float)(endTframe - startTframe) / CLOCKS_PER_SEC;
    totalTframe += seconds;
    nTframe += 1.0;
    
    return size;
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics) {
    totalClose++;
    State state = START_STATE;
    unsigned char aCheck;
    unsigned char cCheck;

    int tries = nRetransmissions;

    if (role == LlTx) {
        unsigned char disc[5] = {FLAG, A, C_DISC, A ^ C_DISC, FLAG};
        do {
            printLL("LLCLOSE - enviado DISC", disc, sizeof(disc));  // DEBUG
            totalTramas++;
            totalTramasSU++;
            totalDISC++;
            write(fd, disc, sizeof(disc));
            alarm(timeout);
            alarmEnabled = TRUE;
            while (alarmEnabled == TRUE && state != STOP_STATE) {
                // Enquanto o alarme não tiver disparado e estado não for o final, processa os bytes da porta série (um de cada vez)
                processByte(A_CLOSE, C_DISC, C_DISC, &aCheck, &cCheck, &state);  // espera um DISC
            }
            if (state == STOP_STATE) {
                // O estado final foi alcançado, pelo que o alarme pode ser desativado
                alarm(0);
                alarmEnabled = FALSE;
            } else {
                // O alarme tocou, pelo que ocorreu timeout e deve haver retransmissão (se ainda não tiver sido excedido o número máximo de tentativas)
                tries--;
                totalRetransmissions++;
            }
        } while (tries >= 0 && state != STOP_STATE);

        if (state != STOP_STATE) {
            // Foi excedido o número máximo de tentativas de retransmissão
            totalRetransmissions--;
            printf("LLCLOSE - DISC não foi recebido\n");
            return -1;
        }

        unsigned char ua[5] = {FLAG, A_CLOSE, C_UA, A_CLOSE ^ C_UA, FLAG};
        printLL("LLCLOSE - enviado UA", ua, sizeof(ua));  // DEBUG
        totalTramas++;
        totalTramasSU++;
        totalUA++;
        write(fd, ua, sizeof(ua));  // quando receber o DISC, rsponde com UA
        printf("\nTotal Tprop: %f\n", totalTprop); 
        printf("N Tprop: %f\n", nTprop); 
        printf("Média Tprop: %f\n", totalTprop/nTprop);  
    } else if (role == LlRx) {
        while (state != STOP_STATE) {
            // Enquanto o estado não for o final, processa os bytes da porta série (um de cada vez)
            processByte(A, C_DISC, C_DISC, &aCheck, &cCheck, &state);  // espera um DISC
        }
        unsigned char disc[5] = {FLAG, A_CLOSE, C_DISC, A_CLOSE ^ C_DISC, FLAG};
        printLL("LLCLOSE - enviado DISC", disc, sizeof(disc));  // DEBUG
        totalTramas++;
        totalTramasSU++;
        totalDISC++;
        write(fd, disc, sizeof(disc));  // quando receber o DISC, responde com DISC
        printf("\nTotal Tframe: %f\n", totalTframe);
        printf("N Tframe: %f\n", nTframe);
        printf("Média Tframe: %f\n", totalTframe/nTframe);        
    } else {
        printf("Erro em connectionParameters.role\n");
        return -1;
    }

    close(fd);

    if (showStatistics) {
        clock_t end = clock();
        float seconds = (float)(end - start) / CLOCKS_PER_SEC;
        printf("\n---------- Estatísticas ----------\n");
        printf("\nTempo de Execução: %f segundos\n", seconds);
        printf("\nInvocações a llopen: %d\n", totalOpen);
        printf("Invocações a llwrite: %d\n", totalWrite);
        printf("Invocações a llread: %d\n", totalRead);
        printf("Invocações a llclose: %d\n", totalClose);
        printf("\nTramas Enviadas: %d\n", totalTramas);
        printf("\nTramas de Informação: %d\n", totalTramasI);
        printf("Tramas de Supervisão/Não Numeradas: %d\n", totalTramasSU);
        printf("\nTramas SET: %d\n", totalSET);
        printf("Tramas UA: %d\n", totalUA);
        printf("Tramas RR: %d\n", totalRR);
        printf("Tramas REJ: %d\n", totalREJ);
        printf("Tramas DISC: %d\n", totalDISC);
        printf("\nBytes Recebidos: %d\n", totalBytes);
        printf("\nBytes Stuffed/Destuffed: %d\n", totalStuffed);
        printf("FLAG Stuffed/Destuffed: %d\n", totalFlagStuffed);
        printf("ESC Stuffed/Destuffed: %d\n", totalEscStuffed);
        printf("\nAlarmes: %d\n", alarmCount);
        printf("Retransmissões: %d\n", totalRetransmissions);
        printf("Erros no BCC1: %d\n", totalBCC1);
        printf("Erros no BCC2: %d\n", totalBCC2);
        printf("Tramas Duplicadas: %d\n", totalDuplicados);
    }

    return 1;
}
