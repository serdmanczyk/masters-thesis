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
      m_neighbors[i].nrss = 0x00;
      m_neighbors[i].i = 0;
      m_neighbors[i].rlen = 0;
      memset(m_neighbors[i].rssi, 0, 10);
      memset(m_neighbors[i].nrssi, 0, 10);
   }

   for (u_int i=0;i<MAX_MSGQ;i++)
   {
      memset(m_outmessages[i].message, 0, MSG_SIZE);
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

   MY();
}

void XBee::Rx()
{
   u_char msg[MSG_SIZE];
   u_int size = 0;

   size = Serial.available();
   if (size > 0)
   {
      if (size > MSG_SIZE)
         size = MSG_SIZE;

      toggleled(PIN_LED0);
      for (u_int i=0;i<size;i++)
      {
         msg[i] = Serial.read();
      }

      Rx_q.QueueString(msg, size);
   }
}

void XBee::Tx(u_char *msg, u_int len)
{
   u_char out[MSG_SIZE];
   u_int olen = 0;

   memset(out, 0, MSG_SIZE);
   memcpy(out, msg, len);

   out[len] = checksum(out, len);
   olen = escape(out, len+1);

	Serial.write(out, olen);
}

bool XBee::Pulse(u_long now)
{
   m_ticks++;  // increments of 5ms
   m_now = now;

   // ---- schedule
   // 1500 ms NeighborBx
   // 3000 ms RSSReport 

   Rx();
   ParseRx();

   if (m_state == nominal_op)
   {
      if ((m_ticks % 40) == 0) // 200ms  
      {
         NRSSIAudit();
         MsgAudit();
      }

      if ((m_ticks % 240) == 0) //.6s
      {
         Bx();
      }

      if ((m_ticks % 240) == 0) //.8s
      {
         NRSS();
      }

      if ((m_ticks % 300) == 0) //1s
      {
         NRSS();
      }
   }

   if (((m_ticks % 300) == 0) && (m_state == uninitialized)) {MY();} // re-request
   if ((m_ticks % 300) == 0) {m_ticks = 0;} // Reset

   return true;
}

bool XBee::ParseXBee(u_char *message, u_int length)
{
   if (message[0] == 0x81)  // received message
   {
      u_int addr = wd(message[1],message[2]);
      u_char rss = message[3];
      u_char opt = message[4];
      u_char id = message[5];

      switch(message[6])
      {
         case 0x10:{
            NeighborUpdate(addr, rss);
            break;
         }
         case 0x12:{
            u_char nrss = message[7];

            NeighborUpdate(addr, rss, nrss);
            break;
         }
         case 0x22:{ 
            u_int saddr = wd(message[7],message[8]);
            u_char srss = message[9];  
            u_int naddr = wd(message[10],message[11]);
            u_char nrss = message[12];

            toggleled(PIN_LED1);
            FwdRSSReport(saddr, srss, naddr, nrss);
            ACK(addr, id);
            break;
         }
         case 0x24:{
            u_char pid = message[7];
            u_int naddr= wd(message[8],message[9]);

            toggleled(PIN_LED2);
            if (m_addr == naddr)
            {
               PingIn(pid, naddr);
            }
            else
            {
               PingOut(pid, naddr);
            }
            ACK(addr, id);
            break;
         }
         case 0x26:{
            u_char pid = message[7];
            u_int naddr = wd(message[8],message[9]);

            PingIn(pid, naddr);
            ACK(addr, id);
            break;
         }
         case 0x27:{
            toggleled(PIN_LED3);
            MsgMark(id);
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

      ATcmd(cmd, data);
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

      // if (status == 0) 
      // {
      //    MsgMark(frameid);
      // }
      if (status & 3) // 1 (No ACK), 2 (CCA Failure), 3 (Purged)
      {
         MsgRetry(frameid);
      }
   }  

   return true;
}

void XBee::MY(){Serial.write((u_char*)"\x7e\x00\x04\x08\x01\x4d\x59\x50", 8);}
void XBee::Bx(){Serial.write((u_char *)"\x7E\x00\x07\x01\x00\xFF\xFF\x00\x00\x10\xf0", 11);}

void XBee::ACK(u_int addr, u_char sfid)
{
   u_char msg[10];

   memcpy(msg, "\x7E\x00\x07\x01\x00\xFF\xFF\x00\xFF\x27", 10);
   msg[5] = hb(addr); // neighbor address high
   msg[6] = lb(addr); // neighbor address low
   msg[8] = sfid;  // frame id (for ack)

   Tx(msg, 10);
}

void XBee::NRSS()
{
   for (u_char i=0;i<MAX_NEIGHBORS;i++)
   {         
      if (m_neighbors[i].addr != 0xFFFF)
      {
         u_char msg[11];

         memcpy(msg, "\x7E\x00\x08\x01\x00\xFF\xFF\x00\x00\x12\xFF", 11);
         msg[5] = hb(m_neighbors[i].addr); // neighbor address high
         msg[6] = lb(m_neighbors[i].addr); // neighbor address low
         msg[10] = m_neighbors[i].rss;  // neighbor rss

         Tx(msg, 11);
      }
   }
}

bool XBee::RSSReport()
{
   for (u_char i=0;i<MAX_NEIGHBORS;i++)
   {
      if (m_neighbors[i].addr != 0xFFFF)
      {
         u_char msg[17];
         u_char rss = navg(m_neighbors[i].rssi, m_neighbors[i].rlen);
         u_char nrss = navg(m_neighbors[i].nrssi, m_neighbors[i].rlen);
         u_char frid = fid();

         memcpy(msg, "\x7E\x00\x0E\x01\xFF\xFF\xFF\x00\xFF\x22\xFF\xFF\xFF\xFF\xFF\xFF\x00", 17);
         msg[4] = frid; // frame id
         msg[5] = hb(m_raddr); // address high byte
         msg[6] = lb(m_raddr); // address low byte
         msg[8] = frid;
         msg[10] = hb(m_addr); // source address high
         msg[11] = lb(m_addr); // source address low
         msg[12] = rss; // source rss
         msg[13] = hb(m_neighbors[i].addr); // source neighbor address high
         msg[14] = lb(m_neighbors[i].addr); // source neighbor address low
         msg[15] = nrss; // source neighbor rss    

         MsgQueue(msg, 17, frid);
      }
   }

   return true;
}

bool XBee::FwdRSSReport(u_int saddr, u_char srss, u_int naddr, u_char nrss)
{
   u_char msg[17];
   u_char frid = fid();

   memcpy(msg, "\x7E\x00\x0E\x01\xFF\xFF\xFF\x00\xFF\x22\xFF\xFF\xFF\xFF\xFF\xFF", 17);
   msg[4] = frid; // frame id
   msg[5] = hb(m_raddr); // address high byte
   msg[6] = lb(m_raddr); // address low byte
   msg[8] = frid;
   msg[10] = hb(saddr); // source address high
   msg[11] = lb(saddr); // source address low
   msg[12] = srss; // source rss
   msg[13] = hb(naddr); // source neighbor address high
   msg[14] = lb(naddr); // source neighbor address low
   msg[15] = nrss; // source neighbor rss

   MsgQueue(msg, 17, frid);

   return true;
}

bool XBee::PingOut(u_char pid, u_int naddr)
{
   u_char msg[14];
   u_char frid = fid();

   memcpy(msg, "\x7E\x00\x0B\x01\xFF\xFF\xFF\x00\xFF\x24\xFF\xFF\xFF\x00", 14);
   msg[4] = frid; // frame id
   msg[5] = hb(m_faddr); // address high byte
   msg[6] = lb(m_faddr); // address low byte
   msg[8] = frid;
   msg[10] = pid; // packet id
   msg[11] = hb(naddr); // node address high
   msg[12] = lb(naddr); // node address low

   MsgQueue(msg, 14, frid);

   return true;
}

bool XBee::PingIn(u_char pid, u_int naddr)
{
   u_char msg[14];
   u_char frid = fid();

   memcpy(msg, "\x7E\x00\x0B\x01\xFF\xFF\xFF\x00\xFF\x26\xFF\xFF\xFF\x00", 14);
   msg[4] = frid; // frame id
   msg[5] = hb(m_raddr); // address high byte
   msg[6] = lb(m_raddr); // address low byte
   msg[8] = frid;
   msg[10] = pid; // packet id
   msg[11] = hb(naddr); // node address high
   msg[12] = lb(naddr); // node address low

   MsgQueue(msg, 14, frid);

   return true;
}

bool XBee::NeighborUpdate(u_int addr, u_char rss)
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

bool XBee::NeighborUpdate(u_int addr, u_char rss, u_char nrss)
{
   bool found = false;

   for (u_char i=0;i<MAX_NEIGHBORS;i++)
   {
      if (m_neighbors[i].addr == addr)
      {

         m_neighbors[i].rss = rss;
         m_neighbors[i].nrss = nrss;
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
            m_neighbors[i].nrss = nrss;
            found = true;
            break;
         }
      }
   }

   return found;
}

bool XBee::MsgQueue(u_char*s, int len, u_char frameid)
{
   u_int id = 0;
   bool found = false;

   for (u_int i=0;i<MAX_MSGQ;i++)
   {
      if (!m_outmessages[i].active) 
      {
         id = i;
         found = true;
         break;
      }
   }

   memcpy(m_outmessages[id].message, s, len);
   m_outmessages[id].len = len;
   m_outmessages[id].frameid = frameid;
   m_outmessages[id].senttime = m_now;
   m_outmessages[id].active = true;
   m_outmessages[id].retries = 0;

   Tx(s, len);

   return true;
}

bool XBee::MsgMark(u_char frameid)
{
   bool marked = false;

   for (u_char i=0;i<MAX_MSGQ;i++)
   {
      if (m_outmessages[i].frameid == frameid) 
      {
         memset(m_outmessages[i].message, 0, MSG_SIZE);
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

bool XBee::MsgRetry(u_char frameid)
{
   bool marked = false;
   for (u_char i=0;i<MAX_MSGQ;i++)
   {
      if (m_outmessages[i].frameid == frameid)
      {
         Tx(m_outmessages[i].message, m_outmessages[i].len);
         m_outmessages[i].retries++;
         m_outmessages[i].senttime = m_now;
         marked = true;
      }
   }

   return marked;
}

bool XBee::MsgAudit()
{
   for (u_char i=0;i<MAX_MSGQ;i++)
   {
      if (m_outmessages[i].active)
      {
         if ((m_now - m_outmessages[i].senttime) > 100)
         {
            Tx(m_outmessages[i].message, m_outmessages[i].len);
            m_outmessages[i].retries++;
            m_outmessages[i].senttime = m_now;
         }

         if (m_outmessages[i].retries > 3)
         {
            memset(m_outmessages[i].message, 0, MSG_SIZE);
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

u_char XBee::navg(u_char *rss, u_int len)
{
   u_char rssi = 0x5A; // -90 dbm
   u_int sum = 0;

   for (u_char i=0; i<len;i++)
   {
      sum += (int)rss[i];
   }

   rssi = (u_char)(sum / len);

   return rssi;
}


void XBee::NRSSIAudit()
{
   for (u_char i=0;i<MAX_NEIGHBORS;i++)
   {
      m_neighbors[i].rssi[m_neighbors[i].i] = m_neighbors[i].rss;
      m_neighbors[i].nrssi[m_neighbors[i].i++] = m_neighbors[i].nrss;

      if (m_neighbors[i].i > 9)
         m_neighbors[i].i = 0;
      if (m_neighbors[i].rlen < 10)
         m_neighbors[i].rlen++;
   }
}

u_char XBee::fid()
{
   m_frameid++;

   switch(m_frameid){
      case 0x7D:
         m_frameid += 2;
         break;
      case 0x7E:
      case 0x11:
      case 0x13:
      case 0x0a:
      case 0x0d:
         m_frameid++;
         break;
      default:
         break;
   }

   return m_frameid;
}

u_int XBee::escape(u_char *msg, u_int len)
{
   u_int size = len;
   u_int pos = 3;
   u_char out[MSG_SIZE];

   memcpy(out, msg, 3);
   for (u_int i=3;i<(len-1);i++)
   {
      switch(msg[i])
      {
      case 0x7D:
      case 0x7E:
      case 0x11:
      case 0x13:
         out[pos] = 0x7D;
         out[pos+1] = msg[i] ^0x20;
         pos++;
         size++;
         break;
      default:
         out[pos] = msg[i];
         break;
      }
      pos++;
   }

   out[size-1] = msg[len-1];

   memcpy(msg, out, size);

   return size;
}

u_char XBee::checksum(u_char *msg, u_int len)
{
   u_int chksum = 0;

   for (u_int i=3;i<len;i++)
   {
      chksum += msg[i];
   }

   return 0xff - (chksum&0xFF);
}

bool XBee::ATcmd(u_int cmd, u_char *data)
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

// Thanks to XBee for their elegant API message format
// which allows this function to be so simple
bool XBee::ParseRx()
{
   u_char message[MSG_SIZE];
   u_int str = 0;
   u_int msb = 0;
   u_int lsb = 0;
   u_int start = 0; 
   u_int length = 0; 
   u_int chksum = 0;
   u_int delpos = 0;
   u_int ei = 0;
   int state = 0;
   bool escaped = false;
   bool found = false;

   for (u_char i=0; i<Rx_q.Size(); i++){
      if (Rx_q.Peek(i) == 0x7E){
         if ((Rx_q.Size() - i) < 4){
            // not enough room for a message, wait for more
            break;
         }
         memset(message, 0, MSG_SIZE);
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
               ei++;

               if (!Rx_q.DataAt(start+length)){
                  break;
               }else{
                  continue;
               }
            }
            if (!escaped){
               message[i-start-ei] = Rx_q.Peek(i);
               chksum += Rx_q.Peek(i);
            }else{
               message[i-start-ei] = (Rx_q.Peek(i)^0x20);
               chksum += (Rx_q.Peek(i)^0x20);
               escaped = false;
            }
            if ((i - start) == length){
               if (((chksum&0xFF) == 0xFF)){
                  ParseXBee(message, length);
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