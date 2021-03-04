// sever libraries
#define _WIN32_WINNT 0x501


#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
  

#define DEFAULT_BUFLEN 1500
#define DEFAULT_PORT "1986"

#define MAX_THREADS 10

//pathfinder libraries
#include "include/pathfinder/hybridAStar.hpp"
#include "include/mapGenerator.hpp"
#include "include/pathrouter/evolutionManager.hpp"
#include <iostream>
#include <fstream>
#include <map>
#include <string.h>

typedef struct threadData{
    Grid* grid;
    SOCKET ClientSocket;

}THREADDATA, *PTHREADDATA;


DWORD WINAPI threadFunc(LPVOID lpParameter ){
    PTHREADDATA pThreadData = (PTHREADDATA) lpParameter;
    int iResult;
    int iSendResult;

    char recvbuf[64];
    int recvbuflen = 64;

    char sendbuf[DEFAULT_BUFLEN];
    int sendbuflen = DEFAULT_BUFLEN;

    char* vehicleId;
    char* currentPos;
    char* endPos;
    char* obsUpdate;

    iResult = recv(pThreadData->ClientSocket, recvbuf, recvbuflen, 0);

    if (iResult > 0) {
        printf("Bytes received: %d\n", iResult);

        vehicleId = strtok(recvbuf, " ");
        currentPos = strtok(NULL, " ");
        endPos = strtok(NULL, " ");
        obsUpdate = strtok(NULL, " ");

        if(obsUpdate != NULL){
            pThreadData->grid->addObstacle(atoi(strtok(obsUpdate, "(,)")), atoi(strtok(NULL, "(,)")) );
        }
        
        HybridAStar pathfinder = HybridAStar(pThreadData->grid);

        Vehicle* v = new Vehicle(1, 1);

        VehicleState start = VehicleState{atof(strtok(currentPos, "(,)")), atof(strtok(NULL, "(,)")), deg2rad(atof(strtok(NULL, "(,)"))), Forward, Straight};
        VehicleState end = VehicleState{atof(strtok(endPos, "(,)")), atof(strtok(NULL, "(,)")), deg2rad(atof(strtok(NULL, "(,)"))), Forward, Straight};
        
        v->current_path = pathfinder.run(start, end, *v, true);
        
        
        char fileName[32] = "";
        strcat(fileName,"v");
        strcat(fileName,vehicleId);
        strcat(fileName, ".txt");

        ofstream myfile;
        myfile.open(fileName);

        myfile << vehicleId  << "\n";

        for(VehicleState state : v->current_path)
        {
            char temp[64];
            snprintf(temp, sizeof temp, "(%f,%f,%f)",state.posX, state.posY, state.ori);
            strcat(sendbuf, temp);
            myfile << "(" << state.posX << ", " << state.posY << ", " << state.ori << ", " << state.steer << ", " << state.gear << "),";
        }
        
        myfile.close();

    // Echo the buffer back to the sender
        iSendResult = send( pThreadData->ClientSocket, sendbuf, DEFAULT_BUFLEN, 0 );
        if (iSendResult == SOCKET_ERROR) {
            printf("send failed with error: %d\n", WSAGetLastError());
            closesocket(pThreadData->ClientSocket);
            WSACleanup();
            return 1;
        }
        printf("Bytes sent: %d\n", iSendResult);
    }
    else if (iResult == 0)
        printf("Connection closing...\n");
    else  {
        printf("recv failed with error: %d\n", WSAGetLastError());
        closesocket(pThreadData->ClientSocket);
        WSACleanup();
        return 1;
    }


    iResult = shutdown(pThreadData->ClientSocket, SD_SEND);
    if (iResult == SOCKET_ERROR) {
        printf("shutdown failed with error: %d\n", WSAGetLastError());
        closesocket(pThreadData->ClientSocket);
        WSACleanup();
        return 1;
    }
    
    return 0;
}

int main(){
    WSADATA wsaData;
    int iResult;

    SOCKET ListenSocket = INVALID_SOCKET;
    SOCKET ClientSocket = INVALID_SOCKET;

    struct addrinfo *result = NULL;
    struct addrinfo hints;

    HANDLE  hThreadArray[MAX_THREADS];
    PTHREADDATA pDataArray[MAX_THREADS];
    
    // Initialize Winsock
    iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("WSAStartup failed with error: %d\n", iResult);
        return 1;
    }

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    // Resolve the server address and port
    iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
    if ( iResult != 0 ) {
        printf("getaddrinfo failed with error: %d\n", iResult);
        WSACleanup();
        return 1;
    }

    // Create a SOCKET for connecting to server
    ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (ListenSocket == INVALID_SOCKET) {
        printf("socket failed with error: %ld\n", WSAGetLastError());
        freeaddrinfo(result);
        WSACleanup();
        return 1;
    }

    // Setup the TCP listening socket
    iResult = bind( ListenSocket, result->ai_addr, (int)result->ai_addrlen);
    if (iResult == SOCKET_ERROR) {
        printf("bind failed with error: %d\n", WSAGetLastError());
        freeaddrinfo(result);
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    freeaddrinfo(result);

    iResult = listen(ListenSocket, SOMAXCONN);
    if (iResult == SOCKET_ERROR) {
        printf("listen failed with error: %d\n", WSAGetLastError());
        closesocket(ListenSocket);
        WSACleanup();
        return 1;
    }

    //check if evo file exsists
        //run evo code if it does not
    mapGenerator mTest = mapGenerator(50, 50, vector<int>{3,3}, 7, 3);
    EvolutionManager trainer = EvolutionManager();
    NeuralNet nn = trainer.train({24, 12, 6, 4, 2}, 100000, false, mTest);
    mapGenerator m = mapGenerator(50, 50, vector<int>{3,3}, 7, 0);
    Grid* grid = m.getGrid();

    for(unsigned i = 0; i < MAX_THREADS; i++){
        ClientSocket = accept(ListenSocket, NULL, NULL);
        if (ClientSocket == INVALID_SOCKET) {
            printf("accept failed with error: %d\n", WSAGetLastError());
            closesocket(ListenSocket);
            WSACleanup();
            return 1;
        }
        pDataArray[i] = (PTHREADDATA) HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, sizeof(THREADDATA));
        pDataArray[i]->grid = grid;
        pDataArray[i]->ClientSocket = ClientSocket;
        DWORD dwThreadId;
        hThreadArray[i] = CreateThread(NULL, 0, threadFunc, (LPVOID)pDataArray[i], 0, &dwThreadId);
    }

    WaitForMultipleObjects(MAX_THREADS, hThreadArray, TRUE, INFINITE);
    // No longer need server socket
    closesocket(ListenSocket);
    

    // cleanup
    closesocket(ClientSocket);
    WSACleanup();

    return 0;
}