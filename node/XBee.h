#ifndef XBee_H
#define XBee_H

#include "queue.h"

#define u_char unsigned char
#define u_int unsigned int
#define u_long  unsigned long

#define MSG_SIZE (200) // Maximum message size
#define MAX_MSGQ (20)
#define RSSSZ (10)

#define RTS PIN_P54
#define CTS PIN_P55
#define RESET PIN_P51

#define wd(x,y) ((u_int)(x<<8) + (u_int)(y))
#define hb(x) (u_char)((x & 0xFF00) >> 8)
#define lb(x) (u_char)(x & 0xFF)
#define bs(x) ((u_int)((x & 0xFF00) >> 8) + (u_int)(x & 0xFF))

enum STATE {
    uninitialized,
    self_realizing,
    rest,
    deploy
};

typedef struct
{
   u_int addr;
   u_char rssi[RSSSZ];
   u_char nrssi[RSSSZ];
   u_char ri;
   u_char ni;
   u_char rl;
   u_char nl;
   u_long lstime;
}neighbor;

typedef struct
{
    u_char message[MSG_SIZE];
    u_char len;
    u_char frameid;
    u_char retries;
    u_long senttime;
    bool active;
}outgoingmsg;

class XBee
{
public:
    XBee();
    void Init();
    void Rx();
    void Tx(u_char *msg, u_int len);
    bool Pulse(u_long now);
    
private:
    bool ParseXBee(u_char *message, u_char length);
    bool ATcmd(u_int cmd, u_char *data);
    bool ParseRx();

    void MY();
    void Bx();
    void LostRearBx();
    void LostRearAck(u_char addr);
    void NRSS();
    bool RSSReport();
    bool FwdRSSReport(u_char *data, u_char length);
    void LostNodeNotice(u_char addr);
    void ACK(u_int addr, u_char sfid);

    bool PingOut(u_char pid, u_int naddr, u_char *data);
    bool PingIn(u_char pid, u_int naddr); 

    void CtrlOut();

    bool MsgAudit();    
    bool MsgQueue(u_char*s, int len, u_char frameid);
    bool MsgRetry(u_char frameid);
    bool MsgMark(u_char frameid);

    void ResetNeighbor(neighbor *nb, u_int addr);
    bool NeighborUpdate(u_int addr, u_char rss);
    bool NeighborUpdate(u_int addr, u_char rss, u_char nrss);
    void NeighborAudit();

    u_char rssavg(u_char *rss, u_int len);
    void iterrssi(neighbor *nb);
    void iternrssi(neighbor *nb);
    u_char rearrssi();
    u_char frontrssi();

    void ServoMgr();
    int CalcCtrl(); 

    u_char fid();
	u_int  escape(u_char *msg, u_int len);
	u_char checksum(u_char *msg, u_int len);
	
    void toggleled(int led);

    Queue Rx_q;
    outgoingmsg m_outmsgs[MAX_MSGQ];
    neighbor m_fnb;
    neighbor m_rnb;
    STATE   m_state;
    u_int m_addr;
    u_char m_frameid;
    u_int m_ticks;
    u_long m_now;
    int m_CtrlIn;
    bool m_bfrontlost;
    bool m_brearlost;
};
#endif
