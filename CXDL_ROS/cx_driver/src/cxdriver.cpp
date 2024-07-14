#include "cx_driver/controlcan.h"
#include <iostream>
#include "cx_driver/cxdriver.h"
#include "unistd.h"


//Frame
Frame::Frame(){
	vci_can_obj.ID=0x00;
	vci_can_obj.SendType=0;
	vci_can_obj.RemoteFlag=0;
	vci_can_obj.ExternFlag=0;
	vci_can_obj.DataLen=8;
	for(int i;i<vci_can_obj.DataLen;i++)
	{vci_can_obj.Data[i]=0x00;}
};
Frame::~Frame(){};
void Frame::setID(UINT byte){vci_can_obj.ID=byte;}
void Frame::setSendType(BYTE byte){vci_can_obj.SendType=byte;}
void Frame::setRemoteFlag(BYTE byte){vci_can_obj.RemoteFlag=byte;}
void Frame::setExternFlag(BYTE byte){vci_can_obj.ExternFlag=byte;}
void Frame::setDataLen(BYTE byte){vci_can_obj.DataLen=byte;}
void Frame::setData(int i,int byte){vci_can_obj.Data[i]=byte;}
ULONG Frame::transmit(double interval,DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len)
{ULONG error = VCI_Transmit(DeviceType,DeviceInd,CANInd,pSend,Len);
usleep(interval);
return error;}
ULONG Frame::transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len)
{ULONG error = VCI_Transmit(DeviceType,DeviceInd,CANInd,pSend,Len);
return error;}
VCI_CAN_OBJ& Frame::getFrame(){return vci_can_obj;}
BYTE Frame::getDataLen(){return vci_can_obj.DataLen;}
BYTE Frame::getData(int i){return vci_can_obj.Data[i];}
UINT Frame::getID(){return vci_can_obj.ID;}
BYTE Frame::getSendType(){return vci_can_obj.SendType;}
BYTE Frame::getRemoteFlag(){return vci_can_obj.RemoteFlag;}
BYTE Frame::getExternFlag(){return vci_can_obj.ExternFlag;}


DWORD receive(DWORD DevType, DWORD DevIndex, DWORD CANIndex,PVCI_CAN_OBJ pReceive, ULONG Len, INT WaitTime)
{ULONG error =VCI_Receive(DevType, DevIndex,CANIndex,pReceive,Len, WaitTime);
return error;}

Motor::Motor(){}
Motor::~Motor(){}
void Motor::setSdoID(BYTE sdo_id){SDO_ID = sdo_id;}
BYTE Motor::getSdoID(){return SDO_ID;}
void Motor::setPdoID(BYTE pdo_id){PDO_ID = pdo_id;}
BYTE Motor::getPdoID(){return PDO_ID;}
void Motor::setEncodingRate(BYTE encoding_rate){Encoding_rate = encoding_rate;}
BYTE Motor::getEncodingRate(){return Encoding_rate;}




