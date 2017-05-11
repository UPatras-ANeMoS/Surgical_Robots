//FUTURE WORK USE IFNDEF FOR NON EXCIECTENT LIBS-HEAders

#include "dynamixel_sdk/dxl_defs.h"
//DXL STUFF
//#include "dynamixel_sdk/dynamixel_sdk.h"
#include "ros/ros.h"
#include <time.h>

#define SLEEP usleep(50)

//PRINT ERROR
inline bool check_error(int dxl_comm_result, dynamixel::PacketHandler* packHandler, uint8_t dxl_error){
	if (dxl_comm_result != COMM_SUCCESS){packHandler->printTxRxResult(dxl_comm_result); return false;}
    	else if (dxl_error != 0) {packHandler->printRxPacketError(dxl_error); return false;}
        return true;
}

//SET UINT8_T PARAMETER
inline bool write_uint8t(dynamixel::PacketHandler* packHandler, dynamixel::PortHandler* portHandler,uint8_t motor_ID,uint8_t PARAM,uint8_t val,uint8_t *error){
	uint8_t dxl_comm_result=packHandler->write1ByteTxRx(portHandler,motor_ID,PARAM,val, error);
	SLEEP;
	if(check_error(dxl_comm_result,packHandler,*error)) return true;
	return false;
}

//SET UINT16_T PARAMETER
inline bool write_uint16t(dynamixel::PacketHandler* packHandler, dynamixel::PortHandler* portHandler,uint8_t motor_ID,uint16_t PARAM,uint16_t val,uint8_t* error){
	uint8_t dxl_comm_result=packHandler->write2ByteTxRx(portHandler,motor_ID,PARAM,val, error);
	SLEEP;
	if(check_error(dxl_comm_result,packHandler,*error)) return true;
	return false;
}
//READ UINT8_T PARAMETER
inline bool read_uint8t(dynamixel::PacketHandler* packHandler, dynamixel::PortHandler* portHandler,uint8_t motor_ID,uint8_t PARAM,uint8_t* val,uint8_t* error){
	uint8_t dxl_comm_result=packHandler->read1ByteTxRx(portHandler,motor_ID,PARAM,val, error);
	SLEEP;
	if(check_error(dxl_comm_result,packHandler,*error)) return true;
	return false;
}

//READ UINT16_T PARAMETER
inline bool read_uint16t(dynamixel::PacketHandler* packHandler, dynamixel::PortHandler* portHandler,uint8_t motor_ID,uint16_t PARAM,uint16_t* val,uint8_t* error){
	uint8_t dxl_comm_result=packHandler->read2ByteTxRx(portHandler,motor_ID,PARAM,val,error);
	SLEEP;
	if(check_error(dxl_comm_result,packHandler,*error)) return true;
	return false;
}

//SYNC_WRITE PARAMETER LOW_LEVEL
inline bool write_sync(dynamixel::GroupSyncWrite* sWrite){
	uint8_t comm_result=sWrite->txPacket();
	SLEEP;
	if(comm_result!=COMM_SUCCESS) {sWrite->getPacketHandler()->printTxRxResult(comm_result); return false;}
	return true;
}

//SYNC WRITE PARAMETER GENERIC
inline bool SYNC_WRITE(uint8_t* num_motor, uint8_t sum, uint16_t param_addr, uint16_t param_size , uint16_t* data,dynamixel_motor* dxl_mtr){
	dynamixel::GroupSyncWrite groupwrite(dxl_mtr[num_motor[0]].portHandler,dxl_mtr[num_motor[0]].packetHandler, param_addr, param_size);
	bool dxl_add_param=true;
	uint8_t dat[param_size];
	for(uint8_t i=0;i<sum;i++){
		if(param_size==1) dat[0]=(uint8_t) data[i]; 
		else{
		  dat[0]=DXL_LOBYTE(data[i]); 
		  dat[1]=DXL_HIBYTE(data[i]); 
		}
		dxl_add_param= dxl_add_param && groupwrite.addParam(dxl_mtr[num_motor[i]].ID, dat); 
	}
	if(dxl_add_param && write_sync(&groupwrite)) return true;
	ROS_INFO("Problem with sync_write parameter creation");
	return false;
}

//BULK_READ PARAMETER LOW_LEVEL
inline bool read_bulk(dynamixel::GroupBulkRead* sBRead){
	uint8_t comm_result=sBRead->txRxPacket();
	SLEEP;
	if(comm_result!=COMM_SUCCESS) {sBRead->getPacketHandler()->printTxRxResult(comm_result);return false;}
	return true;
}

//BULK READ PARAMETER GENERIC
inline bool BULK_READ(uint8_t* num_motor, uint8_t sum, uint16_t* param_addr, uint16_t* param_sizes , uint32_t* data,dynamixel_motor* dxl_mtr){
	bool ret=false;
	dynamixel::GroupBulkRead groupBRead(dxl_mtr[num_motor[0]].portHandler,dxl_mtr[num_motor[0]].packetHandler);
	bool dxl_add_param=true;
	for(uint8_t i=0;i<sum;i++) dxl_add_param=dxl_add_param && groupBRead.addParam(dxl_mtr[num_motor[i]].ID,param_addr[i],param_sizes[i]);
	if(dxl_add_param && read_bulk(&groupBRead)){
	    for(uint8_t i=0;i<sum;i++){
		    if(groupBRead.isAvailable(dxl_mtr[num_motor[i]].ID,param_addr[i],param_sizes[i]))
			data[i]=groupBRead.getData(dxl_mtr[num_motor[i]].ID,param_addr[i],param_sizes[i]);
		    else {ROS_INFO("Motor %d groupBulkRead nodata failed", num_motor[i]); data[i]=1000000;}
	    }
	    return true;	
	}
	ROS_INFO("Problem with bulk read");
	return false;	
}


//POSITION CONTROL
void set_position_control_single(uint8_t num_motor,dynamixel_motor* dxl_mtr){
	uint8_t error;
	while(!write_uint16t(dxl_mtr[num_motor].packetHandler, dxl_mtr[num_motor].portHandler, dxl_mtr[num_motor].ID,P_CCW ,4095 , &error)){sleep(1);}
	while(!write_uint16t(dxl_mtr[num_motor].packetHandler, dxl_mtr[num_motor].portHandler, dxl_mtr[num_motor].ID,P_CW ,0 , &error)){sleep(1);}
	dxl_mtr[num_motor].mode=POSITION_CONTROL;
}

void set_position_control_sync(uint8_t* num_motor, uint8_t sum, dynamixel_motor* dxl_mtr){
	uint16_t pos_conv[2]={4095,4095};
	while(!SYNC_WRITE(num_motor,sum,P_CCW,2,pos_conv,dxl_mtr)){sleep(1);}
	pos_conv[0]=pos_conv[1]=0;
	while(!SYNC_WRITE(num_motor,sum,P_CW,2,pos_conv,dxl_mtr)){sleep(1);}
	for(uint8_t i=0;i<sum;i++) dxl_mtr[num_motor[sum]].mode=POSITION_CONTROL;
}

inline bool write_position_single(uint8_t num_motor, dynamixel_motor* dxl_mtr){
	if(dxl_mtr[num_motor].mode!=POSITION_CONTROL) set_position_control_single(num_motor,dxl_mtr);
	uint8_t error;
	if(write_uint16t(dxl_mtr[num_motor].packetHandler,dxl_mtr[num_motor].portHandler, dxl_mtr[num_motor].ID,P_GOAL_POSITION_L ,dxl_mtr[num_motor].goal_position, &error)) 
	    return true;
	ROS_INFO("Problem write position to motor ID %d",dxl_mtr[num_motor].ID);
	return false;
}

inline bool write_position_sync(uint8_t* num_motor, uint8_t sum, dynamixel_motor* dxl_mtr){
	uint8_t num_motor2[sum];
	uint8_t sum2=0;
	for(uint8_t i=0;i<sum;i++){
	  if(dxl_mtr[num_motor[i]].mode!=POSITION_CONTROL) num_motor2[sum2++]=num_motor[i];
	}
	if(sum2!=0) set_position_control_sync(num_motor2,sum2,dxl_mtr);
	uint16_t pos_conv[sum];
	for(uint8_t i=0;i<sum;i++) pos_conv[i]=dxl_mtr[num_motor[i]].goal_position;
	if(SYNC_WRITE(num_motor,sum,P_GOAL_POSITION_L,2,pos_conv,dxl_mtr))return true;
	ROS_INFO("Problem with sync_write parameter creation");
	return false;
}

inline bool read_position_single(uint8_t num_motor,dynamixel_motor* dxl_mtr){
	uint8_t error=0;
	if(read_uint16t(dxl_mtr[num_motor].packetHandler,dxl_mtr[num_motor].portHandler, dxl_mtr[num_motor].ID, P_PRESENT_POSITION_L, &dxl_mtr[num_motor].measured_position,&error))
	    return true;
	ROS_INFO("Motor %d with ID %d read position problem",num_motor,dxl_mtr[num_motor].ID);	
	return false;
}

inline bool read_position_bulk(uint8_t* num_motor, uint8_t sum, dynamixel_motor* dxl_mtr){
	uint16_t param_addr[sum];
	uint16_t param_size[sum];
	uint32_t data[sum];
	for(uint8_t i=0;i<sum;i++){
		param_addr[i]=P_PRESENT_POSITION_L;
		param_size[i]=2;
	}
	if(BULK_READ(num_motor,sum,param_addr,param_size,data,dxl_mtr)){
	for(uint8_t i=0;i<sum;i++){if(data[i]!=1000000) dxl_mtr[num_motor[i]].measured_position=data[i];}
	return true;
	}
	ROS_INFO("Bulk read position error sum is %d and first motor has ID %d",sum,dxl_mtr[num_motor[0]].ID);
	return false;
}

//VELOCITY CONTROL
inline void set_velocity_control_single(uint8_t num_motor,dynamixel_motor* dxl_mtr){
	uint8_t error;
	while(!write_uint16t(dxl_mtr[num_motor].packetHandler, dxl_mtr[num_motor].portHandler, dxl_mtr[num_motor].ID,P_CCW ,0 , &error)){sleep(1);}
	while(!write_uint16t(dxl_mtr[num_motor].packetHandler, dxl_mtr[num_motor].portHandler, dxl_mtr[num_motor].ID,P_CW ,0 , &error)){sleep(1);}
	dxl_mtr[num_motor].mode=VELOCITY_CONTROL;
}

inline void set_velocity_control_sync(uint8_t* num_motor, uint8_t sum,dynamixel_motor* dxl_mtr){
	uint16_t pos_conv[2]={0,0};
	while(!SYNC_WRITE(num_motor,sum,P_CCW,2,pos_conv,dxl_mtr)){sleep(1);}
	while(!SYNC_WRITE(num_motor,sum,P_CW,2,pos_conv,dxl_mtr)){sleep(1);}
	for(uint8_t i=0;i<sum;i++) dxl_mtr[num_motor[i]].mode=VELOCITY_CONTROL;
}

inline bool write_velocity_single(uint8_t num_motor,dynamixel_motor* dxl_mtr){
	while(dxl_mtr[num_motor].mode!=VELOCITY_CONTROL) set_velocity_control_single(num_motor,dxl_mtr);
	uint8_t error;
	if(write_uint16t(dxl_mtr[num_motor].packetHandler,dxl_mtr[num_motor].portHandler, dxl_mtr[num_motor].ID, P_SPEED_L ,dxl_mtr[num_motor].goal_speed, &error)) 
	    return true;
	ROS_INFO("Problem write velocity %d to motor ID %d",dxl_mtr[num_motor].goal_speed, dxl_mtr[num_motor].ID);
	return false;
}

inline bool write_velocity_sync(uint8_t* num_motor, uint8_t sum, dynamixel_motor* dxl_mtr){
	uint8_t num_motor2[sum];
	uint8_t sum2=0;
	for(uint8_t i=0;i<sum;i++){
	  if(dxl_mtr[num_motor[i]].mode!=VELOCITY_CONTROL){num_motor2[sum2]=num_motor[i]; sum2++;}
	}
	if(sum2>1){set_velocity_control_sync(num_motor2,sum2,dxl_mtr);}
	else if(sum2>0) set_velocity_control_single(num_motor2[sum2],dxl_mtr);
	uint16_t pos_conv[sum];
	for(uint8_t i=0;i<sum;i++) pos_conv[i]=dxl_mtr[num_motor[i]].goal_speed;
	if(SYNC_WRITE(num_motor,sum,P_SPEED_L,2,pos_conv,dxl_mtr))return true;
	ROS_INFO("Problem with sync_write");
	return false;
}

inline bool read_velocity_single(uint8_t num_motor, dynamixel_motor* dxl_mtr){
	uint8_t error=0;
	if(read_uint16t(dxl_mtr[num_motor].packetHandler,dxl_mtr[num_motor].portHandler, dxl_mtr[num_motor].ID, P_PRESENT_SPEED_L, &dxl_mtr[num_motor].current_velocity,&error))
	    return true;
	ROS_INFO("Motor with ID %d read velocity problem",dxl_mtr[num_motor].ID);	
	return false;
}

inline bool read_velocity_bulk(uint8_t* num_motor, uint8_t sum, dynamixel_motor* dxl_mtr){
	uint16_t param_addr[sum];
	uint16_t param_size[sum];
	uint32_t data[sum];
	for(uint8_t i=0;i<sum;i++){
		param_addr[i]=P_PRESENT_SPEED_L;
		param_size[i]=2;
	}
	if(BULK_READ(num_motor,sum,param_addr,param_size,data,dxl_mtr)){
	for(uint8_t i=0;i<sum;i++){if(data[i]!=1000000) dxl_mtr[num_motor[i]].current_velocity=data[i];}
	return true;
	}
	ROS_INFO("Bulk read velocity error");
	return false;
}

//ENABLE/DISABLE TORQUE
inline bool torqueONOFF_single(uint8_t num_motor, uint8_t ONOFF, dynamixel_motor* dxl_mtr,uint8_t* error){
	while(!write_uint8t(dxl_mtr[num_motor].packetHandler,dxl_mtr[num_motor].portHandler,dxl_mtr[num_motor].ID,ADDR_MX_TORQUE_ENABLE ,ONOFF,error)){
	  ROS_INFO("Motor with ID %d problem setting torque on",dxl_mtr[num_motor].ID);
	}
	  return true;		
}

void torqueONOFF_sync(uint8_t* ONOFF, uint8_t* num_motor, uint8_t sum,dynamixel_motor* dxl_mtr){
	uint16_t torque_conv[sum];
	for(uint8_t i=0;i<sum;i++) torque_conv[i]=uint16_t (ONOFF[i]);
	while(!SYNC_WRITE(num_motor,sum,ADDR_MX_TORQUE_ENABLE,1,torque_conv,dxl_mtr)){ROS_INFO("Problem with sync_write torque creation");}
}



