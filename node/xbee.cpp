#include <rxduino.h>
#include <string.h>
#include "queue.h"
#include "xbee.h"

XBee::XBee()
: m_addr(0x00),
m_faddr(0x00),
m_raddr(0x00),
m_frameid(0),
m_ticks(0),
m_now(0),
m_currled(0),
m_state(uninitialized)
{
   for (u_int i=0;i<MAX_NEIGHBORS;i++)
   {
      m_neighbors[i].addr = 0xFFFF;
      m_neighbors[i].rss = 0x00;
      m_neighbors[i].neighrss = 0x00;
   }

   for (u_int i=0;i<MAX_MSGQ;i++)
   {
      memset(m_outmessages[i].message, 0, 100);
      m_outmessages[i].len = 0;
      m_outmessages[i].frameid = 0;
      m_outmessages[i].senttime = 0;
      m_outmessages[i].active = false;
   }
}

void XBee::Init()
{
   pinMode(PIN_LED0, OUTPUT); 
   pinMode(PIN_LED1, OUTPUT); 
   pinMode(PIN_LED2, OUTPUT); 
   pinMode(PIN_LED3, OUTPUT); 
   pinMode(RESET, OUTPUT);
   pinMode(RTS, OUTPUT);
   pinMode(CTS, INPUT);

   digitalWrite(PIN_LED0, 1); 
   digitalWrite(PIN_LED1, 1);

   Serial.begin(57600, SCI_SCI2B); // port of XBee GR-SAKURA 
   digitalWrite(RESET, 0); // Reset XBee 
   delay(200);  
   digitalWrite(RESET, 1); 
   digitalWrite(RTS, 0);

   // digitalWrite(PIN_LED0, 0); 
   digitalWrite(PIN_LED1, 0);
   delay(200);  
   m_state = self_realizing;

   ReqAddr();
}

void XBee::Rx()
{
   if (Serial.available() && !Rx_q.Full())
   {
      toggleled(PIN_LED0);
   }

   while (Serial.available() && !Rx_q.Full()) // Send to XBee received characters from PC 
   { 
      Rx_q.Enqueue(Serial.read());
   }
}

bool XBee::Pulse(u_long now)
{
   m_ticks++;  // increments of 100ms
   m_now = now;

   // ---- schedule
   // 100 ms Parse Rx buff
   // 1500 ms NeighborBx
   // 3000 ms RSSReport 

   Rx();
   ParseRx();

   if (m_state == nominal_op)
   {
      if ((m_ticks % 40) == 0) // 200ms
      {
         AuditOutMsgs();
      }

      if ((m_ticks % 300) == 0) //1.5s
      {
         BxNeighborCheck();
         BxNeighborResponse();
      }

      if ((m_ticks % 600) == 0) // 3s
      {
         ReportRSSItoBaseStation();
      }
   }

   if (((m_ticks % 600) == 0) && (m_state == uninitialized)) {ReqAddr();} // re-request
   if ((m_ticks % 600) == 0) {m_ticks = 0;} // reset

   return true;
}


bool XBee::Q_msgout(u_char*s, int len, u_char frameid)
{
   u_int id = 0;

   for (u_int i=0;i<MAX_MSGQ;i++)
   {
      if (!m_outmessages[i].active) 
      {
         id = i;
         break;
      }
   }

   memcpy(m_outmessages[id].message, s, len);
   m_outmessages[id].len = len;
   m_outmessages[id].frameid = frameid;
   m_outmessages[id].senttime = m_now;
   m_outmessages[id].active = true;
   m_outmessages[id].retries = 0;

   Serial.write(s, len);

   return true;
}

bool XBee::MarkOutMsg(u_char frameid)
{
   bool marked = false;

   for (u_char i=0;i<MAX_MSGQ;i++)
   {
      if (m_outmessages[i].frameid == frameid) 
      {
         memset(m_outmessages[i].message, 0, 100);
         m_outmessages[i].frameid = 0;
         m_outmessages[i].senttime = 0;
         m_outmessages[i].retries = 0;
         m_outmessages[i].active = false;
         marked = true;
         break;
      }
   }

   return marked;
}

bool XBee::RetryOutMsg(u_char frameid)
{
   bool marked = false;
   for (u_char i=0;i<MAX_MSGQ;i++)
   {
      if (m_outmessages[i].frameid == frameid)
      {
         Serial.write(m_outmessages[i].message, m_outmessages[i].len);
         m_outmessages[i].retries++;
         m_outmessages[i].senttime = m_now;
         marked = true;
      }
   }

   return marked;
}

bool XBee::AuditOutMsgs()
{
   for (u_char i=0;i<MAX_MSGQ;i++)
   {
      if (m_outmessages[i].active)
      {
         if ((m_now - m_outmessages[i].senttime) > 200)
         {
            Serial.write(m_outmessages[i].message, m_outmessages[i].len);
            m_outmessages[i].retries++;
            m_outmessages[i].senttime = m_now;
         }

         if (m_outmessages[i].retries > 3)
         {
            memset(m_outmessages[i].message, 0, 100);
            m_outmessages[i].len = 0;
            m_outmessages[i].frameid = 0;
            m_outmessages[i].senttime = 0;
            m_outmessages[i].retries = 0;
            m_outmessages[i].active = false;
         }
      }
   }

   return true;
}

bool XBee::ParseXBee(u_char *message)
{
   if (message[0] == 0x81)  // received message
   {
      u_char addr = wd(message[1],message[2]);
      u_char rss = message[3];
      u_char opt = message[4];

      switch(message[5])
      {
      case 0x10:{
         toggleled(PIN_LED0);
         UpdateNeighborInfo(addr, rss);
         break;
         }

      case 0x12:{
         u_char neighrss = message[6];

         toggleled(PIN_LED1);
         UpdateNeighborInfo(addr, rss, neighrss);
         break;
         }

      case 0x22:{ 
         u_char saddr = wd(message[6],message[7]);
         u_char srss = message[8];
         u_char naddr = wd(message[9],message[10]);
         u_char nrss = message[11];

         toggleled(PIN_LED2);
         ForwardRSSReporttoBaseStation(saddr, srss, naddr, nrss);
         break;
         }

      case 0x24:{
         u_char pid = message[6];
         u_char naddr = wd(message[7],message[8]);

         toggleled(PIN_LED3);
         if (m_addr == naddr)
         {
            PassPingIn(pid, naddr);
         }
         else
         {
            PassPingOut(pid, naddr);
         }
         break;
         }

      case 0x26:{
         u_char pid = message[6];
         u_char naddr = wd(message[7],message[8]);

         PassPingIn(pid, naddr);
         break;
         }
      }
   }
   else if (message[0] == 0x88) // AT Command Response
   {
      u_char frameid = message[1];
      u_int cmd = wd(message[2],message[3]);
      u_char status = message[4];
      u_char *data = &message[5];

      ProcATcmd(cmd, data);
      // 0 = OK
      // 1 = Error
      // 2 = Invalid command
      // 3 = Invalid parameter
   }
   else if (message[0] == 0x8A) // Modem Status
   {
      u_char data = message[1];
      // 0 = Hardware reset
      // 1 = Watchdog timer reset
      // 2 = Associated
      // 3 = Disassociated
      // 4 = Synchronization Lost
      //    (Beacon-enabled only )
      // 5 = Coordinator realignment
      // 6 = Coordinator started
   }
   else if (message[0] == 0x89) // Transmit status
   {
      u_char frameid = message[1];
      u_char status = message[2];

      if (status == 0) 
      {
         MarkOutMsg(frameid);
      }
      if (status & 3) // 1 (No ACK), 2 (CCA Failure), 3 (Purged)
      {
         RetryOutMsg(frameid);
      }
   }  

   return true;
}

bool XBee::ProcATcmd(u_int cmd, u_char *data)
{
   if (cmd == wd('M','Y'))
   {
      m_addr = wd(data[0],data[1]);

      if (m_addr == 0x0001)
      {
         m_raddr = 0x0000;
         m_faddr = 0x0003;
      }
      else if (m_addr == 0x0003)
      {
         m_raddr = 0x0001;
         m_faddr = 0x0004;
      }

      if (m_state == self_realizing)
      {
         m_state = nominal_op;
         digitalWrite(PIN_LED0, 0);
      }
   }

   return true;
}
   
bool XBee::UpdateNeighborInfo(u_int addr, u_char rss)
{
   bool found = false;

   for (u_char i=0;i<MAX_NEIGHBORS;i++)
   {
      if (m_neighbors[i].addr == addr)
      {
         m_neighbors[i].rss = rss;
         found = true;
      }
   }

   if (!found)
   {
      for (u_char i=0;i<MAX_NEIGHBORS;i++)
      {
         if (m_neighbors[i].addr == 0xFFFF)
         {
            m_neighbors[i].addr = addr;
            m_neighbors[i].rss = rss;
            found = true;
            break;
         }
      }
   }

   return found;
}

bool XBee::UpdateNeighborInfo(u_int addr, u_char rss, u_char nrss)
{
   bool found = false;

   for (u_char i=0;i<MAX_NEIGHBORS;i++)
   {
      if (m_neighbors[i].addr == addr)
      {
         m_neighbors[i].rss = rss;
         m_neighbors[i].neighrss = nrss;
         found = true;
         break;
      }
   }

   if (!found)
   {
      for (u_char i=0;i<MAX_NEIGHBORS;i++)
      {
         if (m_neighbors[i].addr == 0xFFFF)
         {
            m_neighbors[i].addr = addr;
            m_neighbors[i].rss = rss;
            m_neighbors[i].neighrss = nrss;
            found = true;
            break;
         }
      }
   }

   return found;
}

// void XBee::SetRTS(){Serial.write((u_char*)"\x7e\x00\x06\x08\x01\x44\x36\x00\x01\x7b", 10);}
// void XBee::WR(){Serial.write((u_char*)"\x7e\x00\x06\x08\x01\x57\x52\x00\x00\x4d", 10);}
// void XBee::BD(){Serial.write((u_char*)"\x7e\x00\x06\x08\x01\x42\x44\x00\x06\x6a", 10);}
void XBee::ReqAddr(){Serial.write((u_char*)"\x7e\x00\x04\x08\x01\x4d\x59\x50", 8);}

void XBee::BxNeighborCheck(){Serial.write((u_char *)"\x7E\x00\x06\x01\x00\xFF\xFF\x00\x10\xf0", 10);}

void XBee::BxNeighborResponse()
{
   for (u_char i=0;i<MAX_NEIGHBORS;i++)
   {         
      if (m_neighbors[i].addr != 0xFFFF)
      {
         u_char msg[11];

         memcpy(msg, "\x7E\x00\x07\x01\xFF\xFF\xFF\x00\x12\xFF\xFF", 11);
         msg[4] = m_frameid++; // frame id
         msg[5] = hb(m_neighbors[i].addr); // neighbor address high
         msg[6] = lb(m_neighbors[i].addr); // neighbor address low
         msg[9] = m_neighbors[i].rss;  // neighbor rss
         msg[10] = (u_char)(0xFF - ((int)(0x13 + (m_frameid-1) + bs(m_neighbors[i].addr) + m_neighbors[i].rss)&0xFF)); // checksum

         Q_msgout(msg, 11, m_frameid-1);
      }
   }
}

bool XBee::ReportRSSItoBaseStation()
{
   for (u_char i=0;i<MAX_NEIGHBORS;i++)
   {
      if (m_neighbors[i].addr != 0xFFFF)
      {
         u_char msg[16];

         memcpy(msg, "\x7E\x00\x0C\x01\xFF\xFF\xFF\x00\x22\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 16);
         msg[4] = m_frameid++; // frame id
         msg[5] = hb(m_raddr); // address high byte
         msg[6] = lb(m_raddr); // address low byte
         msg[9] = hb(m_addr); // source address high
         msg[10] = lb(m_addr); // source address low
         msg[11] = m_neighbors[i].rss; // source rss
         msg[12] = hb(m_neighbors[i].addr); // source neighbor address high
         msg[13] = lb(m_neighbors[i].addr); // source neighbor address low
         msg[14] = m_neighbors[i].neighrss; // source neighbor rss    
         msg[15] = (u_char)(0xFF - ((int)(0x23 + (m_frameid-1) + bs(m_raddr) + bs(m_addr) + bs(m_neighbors[i].addr) + m_neighbors[i].rss + m_neighbors[i].neighrss)&0xFF)); // checksum

         Q_msgout(msg, 16, m_frameid-1);
      }
   }

   return true;
}

bool XBee::ForwardRSSReporttoBaseStation(u_int saddr, u_char srss, u_int naddr, u_char nrss)
{
   u_char msg[16];

   memcpy(msg, "\x7E\x00\x0C\x01\xFF\xFF\xFF\x00\x22\xFF\xFF\xFF\xFF\xFF\xFF\xFF", 16);
   msg[4] = m_frameid++; // frame id
   msg[5] = hb(m_raddr); // address high byte
   msg[6] = lb(m_raddr); // address low byte
   msg[9] = hb(saddr); // source address high
   msg[10] = lb(saddr); // source address low
   msg[11] = srss; // source rss
   msg[12] = hb(naddr); // source neighbor address high
   msg[13] = lb(naddr); // source neighbor address low
   msg[14] = nrss; // source neighbor rss
   msg[15] = (u_char)(0xFF - ((int)(0x23 + (m_frameid-1) + bs(m_raddr) + bs(saddr) + bs(naddr) + srss + nrss)&0xFF)); // checksum

   Q_msgout(msg, 16, m_frameid-1);

   return true;
}

bool XBee::PassPingOut(u_char pid, u_int naddr)
{
   u_char msg[14];

   memcpy(msg, "\x7E\x00\x09\x01\xFF\xFF\xFF\x00\x24\xFF\xFF\xFF\xFF", 14);
   msg[4] = m_frameid++; // frame id
   msg[5] = hb(m_faddr); // address high byte
   msg[6] = lb(m_faddr); // address low byte
   msg[9] = pid; // packet id
   msg[10] = hb(naddr); // node address high
   msg[11] = lb(naddr); // node address low
   msg[12] = (u_char)(0xFF - ((int)(0x25 + (m_frameid-1) + bs(m_faddr) + bs(naddr) + pid)&0xFF)); // checksum

   Q_msgout(msg, 14, m_frameid-1);

   return true;
}

bool XBee::PassPingIn(u_char pid, u_int naddr)
{
   u_char msg[14];

   // rotateleds();
   memcpy(msg, "\x7E\x00\x09\x01\xFF\xFF\xFF\x00\x26\xFF\xFF\xFF\xFF", 14);
   msg[4] = m_frameid++; // frame id
   msg[5] = hb(m_raddr); // address high byte
   msg[6] = lb(m_raddr); // address low byte
   msg[9] = pid; // packet id
   msg[10] = hb(naddr); // node address high
   msg[11] = lb(naddr); // node address low
   msg[12] = (u_char)(0xFF - ((int)(0x27 + (m_frameid-1) + bs(m_raddr) + bs(naddr) + pid)&0xFF)); // checksum

   Q_msgout(msg, 14, m_frameid-1);

   return true;
}

void XBee::toggleled(int led)
{
   digitalWrite(led, !digitalRead(led));
}

 void XBee::rotateleds()
 {
   switch(m_currled){
   case 0:
      digitalWrite(PIN_LED0, 0);
      digitalWrite(PIN_LED1, 1);
      break;
   case 1:
      digitalWrite(PIN_LED1, 0);
      digitalWrite(PIN_LED2, 1);
      break;
   case 2:
      digitalWrite(PIN_LED2, 0);
      digitalWrite(PIN_LED3, 1);
      break;
   case 3:
      digitalWrite(PIN_LED3, 0);
      digitalWrite(PIN_LED0, 1);
      break;
   }

   m_currled++;
   if (m_currled > 3)
      m_currled = 0;
 }

 void XBee::altleds()
 {
   digitalWrite(PIN_LED0, !digitalRead(PIN_LED0));
   digitalWrite(PIN_LED1, !digitalRead(PIN_LED1));
   digitalWrite(PIN_LED2, !digitalRead(PIN_LED2));
   digitalWrite(PIN_LED3, !digitalRead(PIN_LED3));
 }

// Thanks to XBee for their elegant API message format
// which allows this function to be so simple
bool XBee::ParseRx()
{
   u_char message[100];
   u_int str = 0;
   u_int msb = 0;
   u_int lsb = 0;
   u_int start = 0; 
   u_int length = 0; 
   u_int chksum = 0;
   u_int delpos = 0;
   int state = 0;
   bool escaped = false;
   bool found = false;

   for (u_char i=0; i<Rx_q.Size(); i++){
      if (Rx_q.Peek(i) == 0x7E){
         if ((Rx_q.Size() - i) < 4){
            // not enough room for a message, wait for more
            break;
         }
         memset(message, 0, 100);
         delpos = i;
         state++;
         continue;
      }
      else if (state == 0){
         delpos = i;
      }
      if (state == 1){
         msb = Rx_q.Peek(i);
         state++;
      }else if (state == 2){
         lsb = Rx_q.Peek(i);
         start = i + 1 + msb;
         length = lsb - msb;
         if (!Rx_q.DataAt(start+length)){
            break;
         }
         state++;
      }else if (state == 3){
         if ((i + str) >=start){
            if (Rx_q.Peek(i) == 0x7D){
               escaped = true;
               length++;

               if (!Rx_q.DataAt(start+length)){
                  break;
               }else{
                  continue;
               }
            }
            if (!escaped){
               message[i-start] = Rx_q.Peek(i);
               chksum += Rx_q.Peek(i);
            }else{
               message[i-start] = (Rx_q.Peek(i)^0x20);
               chksum += (Rx_q.Peek(i)^0x20);
               escaped = false;
            }
            if ((i - start) == length){
               if (((chksum&0xFF) == 0xFF)){
                  ParseXBee(message);
               }
               delpos = i+1;
               state = 0;
            }
         }
      }
   }
   if (delpos){
      // delete Queue up to character behind pos
      Rx_q.Clear(delpos);
   }

   return found;
}