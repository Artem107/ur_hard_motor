#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>                 //Работа Read и write
#include <fcntl.h>                  //открытие файла
#include <termios.h>                //Работа с портом
#include <stdint.h>                 //Uint8_t
#include <cstring>
#include <time.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define PORTUSB "/dev/ttyUSB0"
//#define PORTUSB "/dev/AVRBLUE"
#define SPEEDSERIAL B115200
#define STARTBYTE 0xAA

using namespace ros;


Publisher SACP;
Subscriber sub;

extern int USBDiscr;
extern char* port;


int initSerial();
int serialComun(double speed, double vel);
uint8_t hashSumm(uint8_t* buf, size_t ln);
int readSerial(uint8_t *buf, size_t ln);
uint8_t readSerial();
void send(const geometry_msgs::Twist&);
void connectDevice();
void speedControl();



int USBDiscr = 0;
char* port = "/dev/AVRBLUE";
geometry_msgs::Twist SendM;


//Структура для отправляемых сообщений
union packetMove{
    #pragma pack(push,1)
    struct msgOut{
        uint8_t Start[2];
        uint8_t len;
        uint16_t speed;
        uint16_t vel;
        uint8_t CRC;
    };
    #pragma pack(pop)
    msgOut MSG;
    uint8_t buffer[64];
}packetSend;

//Структура для принемаемых сообщений
struct msgIn{
    int len;
    uint8_t MSG[64];
    uint8_t CRC;
    uint8_t verifine;
}MSGIN;
//msgIn MSGIN;


void send(const geometry_msgs::Twist& move){
    ROS_INFO("Forward [%f], angle [%f]",move.linear.x,move.angular.z);
    serialComun(move.linear.x,move.angular.z);
}


int main(int argc, char* argv[])
{
    connectDevice();
    init(argc, argv, "Ur_hard_motor");
    init(argc, argv, "SACP");
    NodeHandle m;
    SACP = m.advertise<geometry_msgs::Twist>("SACP", 1000);
    serialComun(0,0);
    sub = m.subscribe("/unior2/twist",1000, send);
    spin();
    return 0;
}

int initSerial(){
    struct termios tio;
    int tty_fd;
    memset(&tio,0,sizeof(tio));
    cfmakeraw(&tio);
    tio.c_cflag=CS8|CREAD|CLOCAL; // 8n1, see termios.h for more information
    tio.c_cc[VMIN]=1;
    tio.c_cc[VTIME]=0;
    tty_fd=open(port, O_RDWR | O_NONBLOCK | O_NOCTTY);
    cfsetospeed(&tio,SPEEDSERIAL);
    cfsetispeed(&tio,SPEEDSERIAL);
    usleep(1000);
    tcsetattr(tty_fd,TCSANOW,&tio);
    if(tty_fd<=0) return -1;
    USBDiscr=tty_fd;
    return 0;
}

int serialComun(double speed,double vel){
    if(access(port,0)){
                close(USBDiscr);
                connectDevice();
            }
    uint8_t buf[2]={0,0};;
    packetSend.MSG.Start[0]=STARTBYTE;
    packetSend.MSG.Start[1]=STARTBYTE;
    packetSend.MSG.len=4;
    packetSend.MSG.speed=int16_t(speed*250);
    packetSend.MSG.vel=int16_t(vel*250);
    packetSend.MSG.CRC=hashSumm(packetSend.buffer,4);
    ROS_INFO("Forward_1 [%d], angle_1 [%d]",packetSend.MSG.speed,packetSend.MSG.vel);
    write(USBDiscr,packetSend.buffer,8);
    tcflush(USBDiscr, TCIOFLUSH);
    readSerial(buf,2);
    if(buf[0]==0xAA && buf[1]==0xAA){
        MSGIN.len=readSerial();
        readSerial(MSGIN.MSG,size_t(MSGIN.len));
        for(int k = 0; k < MSGIN.len/2; k++){
            ROS_INFO("Byte %d [%d]",k ,int16_t(MSGIN.MSG[k*2]|(MSGIN.MSG[k*2+1]<<8)));
        }
        ROS_INFO("Length: [%d]", MSGIN.len);
        MSGIN.CRC=readSerial();
        uint8_t CRC=hashSumm(MSGIN.MSG,size_t(MSGIN.len));
        MSGIN.verifine=readSerial();
    }
    else return -1;
    speedControl();
    return 0;
}

uint8_t hashSumm(uint8_t* buf, size_t ln){
    uint16_t CRC = 0;
    for(int i = 0; i < int(ln); i++){
        CRC += buf[i+3]*211;
        CRC ^= CRC>>8;
    }
    return uint8_t(CRC);
}

int readSerial(uint8_t* buf, size_t ln){
    for(int k=0; k<100; k++){
        usleep(100);
        ssize_t x = read(USBDiscr ,buf ,ln);
        if(x==ln){
            return 0;
        }
    }
}

uint8_t readSerial(){
    uint8_t buf;
    for(int k=0; k<100; k++){
        usleep(100);
        ssize_t x = read(USBDiscr ,&buf ,1);
        if(x==1){
            return buf;
        }
    }
}

void connectDevice(){
    if(initSerial()<0){
        ROS_INFO("No connect device");
        usleep(100000);
        connectDevice();
    }
    ROS_INFO("Succes connect device");
}

void speedControl(){
        ROS_INFO("SpeedM: %d AngularM: %d", int16_t(MSGIN.MSG[0]|(MSGIN.MSG[1]<<8)), int16_t(MSGIN.MSG[2]|(MSGIN.MSG[3]<<8)));
        SendM.angular.z = int16_t(MSGIN.MSG[2]|(MSGIN.MSG[3]<<8))/10;
        SendM.linear.x = int16_t(MSGIN.MSG[0]|(MSGIN.MSG[1]<<8))/10;
        SACP.publish(SendM);
        if(int16_t(MSGIN.MSG[8]|(MSGIN.MSG[8+1]<<8))>1500){
            usleep(35000);
            serialComun(0,0);
        }
        //usleep(10000);
    }
