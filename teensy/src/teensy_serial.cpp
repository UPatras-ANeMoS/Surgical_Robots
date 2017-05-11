#include "teensy_serial.h"
using namespace std;

cserial_board::cserial_board(char* port,int tspeed, int bytes_IN, int bytes_OUT)
{
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

int cserial_board::cserial_open(){
	
	struct termios options;
	//bzero(&options,sizeof(options));
	cfmakeraw(&options);
	/*
	bzero(&options,sizeof(options));
	options.c_iflag=0;
	options.c_lflag=0; //non canonical, no echo
	options.c_oflag=0;
	options.c_cflag |= (CLOCAL | CREAD | CS8);
	*/
	
	*fd=open(serial_address, O_RDWR | O_NOCTTY | O_NDELAY); //O_NDELAY// | O_NOCTTY );//| O_NONBLOCK);
	if (!(*fd)){
		*fd=open(serial_address, O_RDWR | O_NOCTTY | O_NDELAY);// | O_NOCTTY);// | O_NONBLOCK);
		if (!(*fd)){
			cout<< "Problem opening serial port"<<endl;;
			exit(-1);
		}
	}

	if (*fd>0){

		sleep(2); //required? to make flush work, for some reason
		tcflush(*fd,TCIFLUSH);
		tcsetattr(*fd, TCSANOW, &options);
		
		poll_fd->fd=*fd;
		poll_fd->events=POLLIN;
	
		options.c_cc[VTIME]=1; //inter-character timer 1->0.1s
		options.c_cc[VMIN]=*SIZE_IN; //blocks read until VMIN bytes received
		tcsetattr(*fd, TCSANOW, &options);	
		
	return 1;
	}
	else{
	return -1;
	}
}

bool cserial_board::cserial_read(){
	if(poll(poll_fd,1,1000)>0){
	if(read(*fd,ch_in,*SIZE_IN)==*SIZE_IN){
		//for(uint8_t len=0; len<*SIZE_IN;len++)
		//cout<<(int) (ch_in)[len] <<" " ;
		////cout << endl;
		return true;
	}
	else{
		cout<< "read Problem after polling" << endl;
		return false;		
	}
	}
	else{
		return false;	
	}
}

bool cserial_board::cserial_write(){
	//write one byte to teensy to init read
	if(write(*fd,ch_out,1)==1){
		//cout<< "Teensy Data written"<<endl;
		return true;
	}
	else{
		cout<< "Teensy Data write problem"<<endl;
		sleep(1);
		return false;
	}
}

void cserial_board::cserial_close(){
	close(*fd);
	cout << "serial closed" << endl;
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
