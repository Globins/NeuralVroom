#define _WIN32_WINNT 0x501

#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>




#define DEFAULT_BUFLEN 1500
#define DEFAULT_PORT "1986"

#define MAX_SOCKETS 10

int main(int argc, char **argv) 
{
    WSADATA wsaData;
    SOCKET ConnectSocket[MAX_SOCKETS];
    struct addrinfo *result = NULL,
                    *ptr = NULL,
                    hints;
    char recvbuf[DEFAULT_BUFLEN];
    int iResult;
    int recvbuflen = DEFAULT_BUFLEN;
    
    // Validate the parameters
    if (argc != 2) {
        printf("usage: %s server-name\n", argv[0]);
        return 1;
    }

    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory( &hints, sizeof(hints) );
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    // Resolve the server address and port
    iResult = getaddrinfo(argv[1], DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Attempt to connect to an address until one succeeds
    for(unsigned i = 0; i < MAX_SOCKETS; i++){
        for(ptr=result; ptr != NULL ;ptr=ptr->ai_next) {

            // Create a SOCKET for connecting to server
            ConnectSocket[i] = socket(ptr->ai_family, ptr->ai_socktype, 
                ptr->ai_protocol);
            if (ConnectSocket[i] == INVALID_SOCKET) {
                printf("socket failed with error: %ld\n", WSAGetLastError());
                WSACleanup();
                return 1;
            }

            // Connect to server.
            iResult = connect( ConnectSocket[i], ptr->ai_addr, (int)ptr->ai_addrlen);
            if (iResult == SOCKET_ERROR) {
                closesocket(ConnectSocket[i]);
                ConnectSocket[i] = INVALID_SOCKET;
                continue;
            }
            break;
        }

        if (ConnectSocket[i] == INVALID_SOCKET) {
            printf("Unable to connect to server!\n");
            WSACleanup();
            return 1;
        }

    }
    

    freeaddrinfo(result);

    

    // Send an initial buffer
    for(int i = 0; i < MAX_SOCKETS; i++){
        char temp[64];
        snprintf(temp, sizeof temp, "%i (15,10,45) (15,%i,30)",i, 15+i*3);
        const char *sendbuf = temp;

        iResult = send( ConnectSocket[i], sendbuf, (int)strlen(sendbuf), 0 );
        if (iResult == SOCKET_ERROR) {
            printf("send failed with error: %d\n", WSAGetLastError());
            closesocket(ConnectSocket[i]);
            WSACleanup();
            return 1;
        }

        printf("Bytes Sent: %ld\n", iResult);
        // shutdown the connection since no more data will be sent
        iResult = shutdown(ConnectSocket[i], SD_SEND);
        if (iResult == SOCKET_ERROR) {
            printf("shutdown failed with error: %d\n", WSAGetLastError());
            closesocket(ConnectSocket[i]);
            WSACleanup();
            return 1;
        }
    }
    
    
    for(int i = 0; i < MAX_SOCKETS; i++){
    // Receive until the peer closes the connection
        do {

            iResult = recv(ConnectSocket[i], recvbuf, recvbuflen, 0);
            if ( iResult > 0 ){
                printf("Bytes received: %d\n", iResult);
                printf(recvbuf);
            }else if ( iResult == 0 )
                printf("Connection closed\n");
            else
                printf("recv failed with error: %d\n", WSAGetLastError());

        } while( iResult > 0 );
        closesocket(ConnectSocket[i]);
    }

    // cleanup
    
    WSACleanup();

    return 0;
}



