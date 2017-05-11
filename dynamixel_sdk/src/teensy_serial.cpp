#include "dynamixel_sdk/teensy_serial.h"

cserial_board::cserial_board(char* port,int tspeed, int bytes_IN, int bytes_OUT){
	fd=new int[1];
	SIZE_IN=new uint32_t[1];
	*SIZE_IN=(uint32_t) bytes_IN;
	SIZE_OUT=new uint32_t[1];
	*SIZE_OUT=(uint32_t) bytes_OUT;
	speed=new int[1];
	*speed=tspeed;
	serial_address=new char[50];
	strcpy(serial_address,port);
	poll_fd=(struct pollfd*) calloc(1,sizeof(struct pollfd));
	ch_in=new unsigned char [(*SIZE_IN)+1];
	ch_out=new unsigned char [(*SIZE_OUT)+1];
}

bool cserial_board::cserial_open(){
	struct termios options;
	*fd=open(serial_address, O_RDWR);
	while(!(*fd)){
	*fd=open(serial_address, O_RDWR);
	fprintf(stderr,"Problem opening serial port in %s \n",serial_address);
	}
	printf("Serial port in %s open \n",serial_address);
	tcgetattr(*fd,&options);
	sleep(2); //required? to make flush work, for some reason
        tcflush(*fd,TCIFLUSH);
	sleep(2); //required? to make flush work, for some reason
	cfmakeraw(&options);
        //options.c_iflag=0;
        //options.c_lflag=0; //non canonical, no echo
        //options.c_oflag=0;
        //options.c_cflag |= (CLOCAL | CREAD | CS8);
	tcsetattr(*fd, TCSANOW, &options);
	poll_fd->fd=*fd;
	poll_fd->events=POLLIN;
        options.c_cc[VTIME]=1; //inter-character timer 1->0.1s
	options.c_cc[VMIN]=*SIZE_IN; //blocks read until VMIN bytes received
	tcsetattr(*fd, TCSANOW, &options);	
	return true;
}

bool cserial_board::cserial_read(){
	//if(poll(poll_fd,1,1000)>0 && read(*fd,ch_in,*SIZE_IN)>=*SIZE_IN) return true;
	if(read(*fd,ch_in,*SIZE_IN)>=*SIZE_IN) return true;
	return false;	
}

bool cserial_board::cserial_write(){
	if(write(*fd,ch_out,2)>0){return true;}
	return false;
}

void cserial_board::cserial_close(){
	close(*fd);
}

bool cserial_board::cserial_update(){
	if(this->cserial_write() && this->cserial_read()) return true;
	return false;
}

cserial_board::~cserial_board(){
	this->cserial_close();
	delete fd;
	delete SIZE_IN;
	delete SIZE_OUT;
	delete speed;
	delete serial_address;
	delete poll_fd;
	delete ch_in;
	delete ch_out;
}
