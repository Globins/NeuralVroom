all: client sever

client: client.cpp
	gcc -o client client.cpp -lws2_32 -std=c++11 -Wall

sever: sever.cpp
	gcc -o sever sever.cpp -lws2_32 -std=c++11 -Wall