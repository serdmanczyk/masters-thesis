#ifndef XBee_H
#define XBee_H

#include "queue.h"

#define u_char unsigned char
#define u_int unsigned int
#define u_long  unsigned long

#define MSG_SIZE (20) // Maximum neighbors to remember
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
    // void Tx();
    bool Pulse(unsigned long now);
    

private:
    
    bool ImmediateTx(u_char *msg, u_int len);
    bool ParseRx();
    bool ParseXBee(u_char *message);
    bool ProcATcmd(u_int cmd, u_char *data);
    
    void ReqAddr();
    // void SetRTS();
    // void BD();
    // void WR();

    void BxNeighborCheck();
    void BxNeighborResponse();
    bool ReportRSSItoBaseStation();
    bool ForwardRSSReporttoBaseStation(u_int saddr, u_char srss, u_int naddr, u_char nrss);

    bool PassPingOut(u_char pid, u_int naddr);
    bool PassPingIn(u_char pid, u_int naddr);
    
    bool Q_msgout(u_char*s, int len, u_char frameid);
    bool RetryOutMsg(u_char frameid);
    bool MarkOutMsg(u_char frameid);
    bool AuditOutMsgs();
    void AuditNRSSI();
    
    bool UpdateNeighborInfo(u_int addr, u_char rss);
    bool UpdateNeighborInfo(u_int addr, u_char rss, u_char nrss);

    u_char navg(u_char *rss, u_int len);

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
