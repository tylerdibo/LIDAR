/*
 * Serial.cpp
 *
 *  Created on: Oct 29, 2016
 *      Author: tyler
 */

#include "Serial.h"
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <queue>

Serial::Serial() {
	uart0_filestream = -1;
	stopThreads = false;
}

Serial::~Serial() {
	// TODO Auto-generated destructor stub
}

void Serial::threadReceive(){
	while(!stopThreads && uart0_filestream != -1){
		unsigned char bytesRead[32768];
		int nbrBytes = read(uart0_filestream, (void*)bytesRead, 32767);
		for(int i=0; i < nbrBytes; i++){
			rxQueue.push(bytesRead[i]);
		}

		usleep(5000);
	}
}

static void threadCaller(Serial* serialPtr){
	serialPtr->threadReceive();
}

int Serial::available(){
	return !rxQueue.empty();
}

bool Serial::begin(char *port, int baud){
	uart0_filestream = open(port, O_RDONLY | O_NOCTTY | O_NDELAY);
	if(uart0_filestream == -1){
		printf("Error. Unable to open UART.");
		return false;
	}

	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

	std::thread receive(threadCaller, this);

	return true;
}

int Serial::sread(){
	if(rxQueue.empty()) return 0;
	unsigned char data = rxQueue.front();
	rxQueue.pop();
	return data;
}

void Serial::end(){
	stopThreads = true;
	close(uart0_filestream);
}
