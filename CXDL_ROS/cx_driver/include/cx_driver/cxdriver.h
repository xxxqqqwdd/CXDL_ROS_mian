#ifndef _CX_DRIVER_H_
#define _CX_DRIVER_H_


#include "cx_driver/controlcan.h"


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
        ~Motor();
        void setSdoID(BYTE sdo_id);
        BYTE getSdoID();
        void setPdoID(BYTE pdo_id);
        BYTE getPdoID();
        void setEncodingRate(BYTE encoding_rate);
        BYTE getEncodingRate();


    private:
        BYTE SDO_ID;
        BYTE PDO_ID;
        int Encoding_rate;


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

enum MotorType {EVO,ZHUOYU,HAOKONG};

DWORD receive(DWORD DevType, DWORD DevIndex, DWORD CANIndex,PVCI_CAN_OBJ pReceive, ULONG Len, INT WaitTime);





#endif 