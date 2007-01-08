#ifndef TCP_CHANNEL_H
#define TCP_CHANNEL_H
#include "CommunicationChannel.h"
#include "TCPAddress.h"


class TCPServer:public Runnable
{
	CommunicationChannel *channel;
	int socket;
	Thread thread;

public:
	TCPServer(CommunicationChannel *channel, int socket)
	{
		this->channel = channel;
		this->socket = socket;
	}
	void run(void *args);
};

class TCPHandler:public Runnable
{
	CommunicationChannel *channel;
	int socket;

public:
	TCPHandler(CommunicationChannel *channel, int socket)
	{
		this->channel = channel;
		this->socket = socket;
	}
	void run(void *args);
};



class TCPChannel:public CommunicationChannel
{
	int tcpSocket;
	static bool initialized;
	Thread thread;


public:
	TCPChannel(int idx);
	bool connectSender(ChannelAddress *addr);
	bool connectReceiver(ChannelAddress *addr);
	bool sendMessage(ChannelAddress *addr, char *buf, int bufLen, char type);
	char *receiveMessage(ChannelAddress *addr, int *retLen, char *retType);
	unsigned int fromNative(int n);
	int toNative(unsigned int l);
};


#endif
