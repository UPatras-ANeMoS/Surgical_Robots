//#include "../include/serial_board.h"
#include "../include/serial_board.h"
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

bool cserial_board::cserial_open(){
	struct termios options;
	bzero(&options,sizeof(options));
	//cfmakeraw(&options);
	*fd=open(serial_address, O_RDWR); //O_NDELAY// | O_NOCTTY );//| O_NONBLOCK);
	if (*fd<0){
		*fd=open(serial_address, O_RDWR);// | O_NOCTTY);// | O_NONBLOCK);
		if (*fd<0){
			cout<< "Problem opening serial port"<<endl;;
		}
	}
	if (*fd>=0){
		//sleep(1); //required? to make flush work, for some reason
		tcflush(*fd,TCIFLUSH);	
		//sleep(1);
		options.c_cflag = B115200|CS8|CLOCAL|CSTOPB;
		options.c_cc[VTIME]=10; //inter-character timer 1->0.1s
		options.c_cc[VMIN]=*SIZE_IN; //blocks read until VMIN bytes received
		tcsetattr(*fd, TCSANOW, &options);
		poll_fd->fd=*fd;
		poll_fd->events=POLLIN;
	return true;
	}
	return false;
}

bool cserial_board::cserial_read(){
  //  cout<< " read "<<endl;	
	int len=0;
	int i=2;

	
	while(i-- && len<*SIZE_IN){
	  if(poll(poll_fd,1,1000)>0){
	    len+=read(*fd,&ch_in[len],*SIZE_IN);
	  }
	  else{
	    cout<<"Poll error"<<endl;
	  }
	}
	if(len!=*SIZE_IN){
	  cout<< "read Problem A" << len << endl;
	return false;
	}
 	else{
//  	      //len=read(*fd,ch_in,1000);
// 	      if(len!=*SIZE_IN)
//  			cout<< "read 2nd case " << len << endl;
//  			//cout<<"flush" <<endl;
//  						//tcflush(*fd,TCIFLUSH);
// // 			return false;
// // 		}
 		
 	}
	return true; 
}

bool cserial_board::cserial_write(){
   //for(uint8_t i=0;i<ch_out[1]; i++)
   //   printf("%d ", ch_out[i]);
   //   cout<<endl;	
	if (ch_out[1]>=3 && write(*fd,ch_out,ch_out[1])==ch_out[1]){
	//  cout<< " write "<<endl;	
		return true;
		//usleep(500); //sleep 0.5ms in case 1 byte for data to arrive
	}
	cout<< "Data to write has problem"<<endl;		
	return false;
}

void cserial_board::cserial_close(){
	tcflush(*fd,TCIOFLUSH);	
	close(*fd);
	//cout << "serial closed" << endl;
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
