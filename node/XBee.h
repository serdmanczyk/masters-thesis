#ifndef XBee_H
#define XBee_H

#include "queue.h"

#define u_char unsigned char
#define u_int unsigned int
#define u_long  unsigned long

#define MSG_SIZE (100) // Maximum message size
#define MAX_NEIGHBORS (5) // Maximum neighbors to remember
#define MAX_MSGQ (20)

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
    nominal_op
};

typedef struct
{
   u_int addr;
   u_char rss;
   u_char nrss;
   u_char rssi[10];
   u_char nrssi[10];
   u_char i;
   u_char rlen;
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
    bool Pulse(unsigned long now);
    
private:
    // void SetRTS();
    // void BD();
    // void WR();
    bool ParseXBee(u_char *message, u_int length);

    void MY();
    void Bx();
    void NRSS();
    bool RSSReport();
    bool FwdRSSReport(u_int saddr, u_char srss, u_int naddr, u_char nrss);
    void ACK(u_int addr, u_char sfid);

    bool PingOut(u_char pid, u_int naddr);
    bool PingIn(u_char pid, u_int naddr); 

    bool MsgAudit();    
    bool MsgQueue(u_char*s, int len, u_char frameid);
    bool MsgRetry(u_char frameid);
    bool MsgMark(u_char frameid);

    bool UpdateNeighbor(u_int addr, u_char rss);
    bool UpdateNeighbor(u_int addr, u_char rss, u_char nrss);

    u_char navg(u_char *rss, u_int len);
    void NRSSIAudit();

    u_char fid();
	u_int  escape(u_char *msg, u_int len);
	u_char checksum(u_char *msg, u_int len);

    bool ATcmd(u_int cmd, u_char *data);
    bool ParseRx();
	
    void altleds();
    void rotateleds();
    void toggleled(int led);

    Queue Rx_q;
    neighbor m_neighbors[MAX_NEIGHBORS];
    outgoingmsg m_outmessages[MAX_MSGQ];
    u_int m_addr;
    u_int m_faddr;
    u_int m_raddr;
    u_char m_frameid;
    u_char m_currled;
    u_int m_ticks;
    u_long m_now;
    STATE   m_state;
};
#endif
