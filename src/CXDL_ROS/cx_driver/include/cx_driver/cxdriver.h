#ifndef _CX_DRIVER_H_
#define _CX_DRIVER_H_


#include "cx_driver/controlcan.h"

enum class MotorType {EVO,ZHUOYU,HAOKONG,SHENGJIANG,DIPAN};

class Frame
{
    public:
    Frame();
    ~Frame();
    void setID(UINT byte);
    void setSendType(BYTE byte);
    void setRemoteFlag(BYTE byte);
    void setExternFlag(BYTE byte);
    void setDataLen(BYTE byte);
    void setData(int i,int byte);
    ULONG transmit(double interval,DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len);
    ULONG transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len);
    VCI_CAN_OBJ& getFrame();
    BYTE getDataLen();
    BYTE getData(int i);
    UINT getID();
    BYTE getSendType();
    BYTE getRemoteFlag();
    BYTE getExternFlag();

    private:
    VCI_CAN_OBJ vci_can_obj;

};

class Motor : public Frame
{
    public:
        Motor();
        Motor(MotorType motortype,int sdo_id,int pdo_id,int encoding_rate);
        Motor(MotorType motortype,int sdo_id,int pdo_id);
        Motor(MotorType motortype,int sdo_id);
        ~Motor();
        void setSdoID(int sdo_id);
        int getSdoID();
        void setPdoID(int pdo_id);
        int getPdoID();
        void setEncodingRate(int encoding_rate);
        int getEncodingRate();
        double getVelocity();
        void setVelocity(double velocity);
        double getAcceleration();
        void setAcceleration(double acceleration);
        MotorType getMotortype();
        void setMotortype(MotorType motorType);


    private:
        int SDO_ID;
        int PDO_ID_1;
        int PDO_ID_2;
        int Encoding_rate;
        double Velocity;
        double Acceleration;
        MotorType Motortype;
       


};

class EVO_MOTOR :public Motor
{

};
class ZHUOYU_MOTOR :public Motor
{
    
};
class HAOKONG_MOTOR :public Motor
{
    
};
class SHENGJIANG_MOTOR :public Motor
{
    
};
class DIPAN_MOTOR :public Motor
{
    
};


DWORD receive(DWORD DevType, DWORD DevIndex, DWORD CANIndex,PVCI_CAN_OBJ pReceive, ULONG Len, INT WaitTime);
ULONG transmit(DWORD DeviceType,DWORD DeviceInd,DWORD CANInd,PVCI_CAN_OBJ pSend,UINT Len);




#endif 