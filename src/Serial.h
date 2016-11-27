/*
 * Serial.h
 *
 *  Created on: Oct 29, 2016
 *      Author: tyler
 */

#ifndef SERIAL_H_
#define SERIAL_H_

#include <queue>

class Serial {
public:
	Serial();
	~Serial();
	bool begin(char *port, int baud);
	int available();
	int sread();
	void end();
	void threadReceive();

private:
	int uart0_filestream;
	bool stopThreads;
	std::queue<int> rxQueue;
};

#endif /* SERIAL_H_ */
