#include <rxduino.h>
#include <string.h>
#include "queue.h"
#include "xbee.h"

XBee::XBee()
: m_addr(0x00),
m_frameid(0),
m_ticks(0),
m_now(0),
m_CtrlIn(90),  // Stay still
m_state(uninitialized)
{
   ResetNeighbor(&m_fnb, 0xFFFF);
   ResetNeighbor(&m_rnb, 0x0000);

   for (u_int i=0;i<MAX_MSGQ;i++)
   {
      memset(m_outmsgs[i].message, 0, MSG_SIZE);
      m_outmsgs[i].len = 0;
      m_outmsgs[i].frameid = 0;
      m_outmsgs[i].senttime = 0;
      m_outmsgs[i].active = false;
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

   Rx();
   ParseRx();


   if (((m_ticks % 180) == 0) && m_state != deploy) // 600ms
      Bx();

   if (m_state == deploy)
   {
      if (m_ticks % 20) // 100ms  
         MsgAudit();

      if ((m_ticks % 40) == 0) // 200ms  
         ServoMgr();

      if ((m_ticks % 80) == 0) // 400ms
         NRSSIAudit();

      if (m_ticks == 240) // 800ms
         NRSS();

      if (m_ticks == 300) // 1s
         RSSReport();
   }

   if ((m_ticks >= 300) && (m_state == uninitialized)) {MY();} // re-request
   if (m_ticks >= 300) {m_ticks = 0;} // Reset

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

      if (id != 0)
         ACK(addr, id);

      NeighborUpdate(addr, rss);

      switch(message[6])
      {
         case 0x12:{ // rss
            u_char nrss = message[7];

            NeighborUpdate(addr, rss, nrss);
            break;
         }
         case 0x22:{ // rss report
            FwdRSSReport(&message[7]);
            break;
         }
         case 0x24:{ // outward ping
            u_char pid = message[7];
            u_int naddr= wd(message[8],message[9]);

            if (m_addr == naddr)
            {
               PingIn(pid, naddr);
            }
            else
            {
               PingOut(pid, naddr);
            }
            break;
         }
         case 0x26:{ // inward ping
            u_char pid = message[7];
            u_int naddr = wd(message[8],message[9]);

            PingIn(pid, naddr);
            break;
         }
         case 0x27:{ // acknowledge message
            u_char appmsid = message[7];
            MsgMark(appmsid);
            break;
         }
         case 0x28:{ // address assign
            u_int raddr = wd(message[7],message[8]);
            u_int faddr = wd(message[9],message[10]);

            ResetNeighbor(&m_fnb, faddr);
            ResetNeighbor(&m_rnb, raddr);
            break;
         }
         case 0x30:{ // deploy
            m_state = deploy;
            digitalWrite(PIN_LED0, 1);
            digitalWrite(PIN_LED1, 1);
            digitalWrite(PIN_LED2, 1);
            digitalWrite(PIN_LED3, 1);
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
   u_char msg[11];

   memcpy(msg, "\x7E\x00\x08\x01\x00\xFF\xFF\x00\x00\x27\x00", 11);
   msg[5] = hb(addr); // neighbor address high
   msg[6] = lb(addr); // neighbor address low
   msg[10] = sfid;  // frame id (for ack)

   Tx(msg, 11);
}

void XBee::NRSS()
{
   neighbor *nodes[2];

   nodes[0] = &m_fnb;
   nodes[1] = &m_rnb;
   for (u_char i=0;i<2;i++)
   {         
      if (nodes[i]->addr != 0xFFFF)
      {
         u_char msg[11];

         memcpy(msg, "\x7E\x00\x08\x01\x00\xFF\xFF\x00\x00\x12\xFF", 11);
         msg[5] = hb(nodes[i]->addr); // neighbor address high
         msg[6] = lb(nodes[i]->addr); // neighbor address low
         msg[10] = nodes[i]->rss;  // neighbor rss

         Tx(msg, 11);
      }
   }
}

bool XBee::RSSReport()
{
   u_char msg[21];
   u_char frid = fid();

   memcpy(msg, "\x7E\x00\x12\x01\xFF\xFF\xFF\x00\xFF\x22\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x00", 21);
   msg[4] = frid; // frame id
   msg[5] = hb(m_rnb.addr); // address high byte
   msg[6] = lb(m_rnb.addr); // address low byte
   msg[8] = frid;
   msg[10] = hb(m_addr); // source address high
   msg[11] = lb(m_addr); // source address low
   msg[12] = hb(m_fnb.addr); 
   msg[13] = lb(m_fnb.addr); 
   msg[14] = navg(m_fnb.rssi, m_fnb.rlen);
   msg[15] = navg(m_fnb.nrssi, m_fnb.rlen);
   msg[16] = hb(m_rnb.addr); 
   msg[17] = lb(m_rnb.addr); 
   msg[18] = navg(m_rnb.rssi, m_rnb.rlen);
   msg[19] = navg(m_rnb.nrssi, m_rnb.rlen);

   MsgQueue(msg, 21, frid);

   return true;
}

bool XBee::FwdRSSReport(u_char *data)
{
   u_char msg[21];
   u_char frid = fid();

   memcpy(msg, "\x7E\x00\x12\x01\xFF\xFF\xFF\x00\xFF\x22\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x00", 21);
   msg[4] = frid; // frame id
   msg[5] = hb(m_rnb.addr); // address high byte
   msg[6] = lb(m_rnb.addr); // address low byte
   msg[8] = frid;
   memcpy(&msg[10], data, 10);

   MsgQueue(msg, 21, frid);

   return true;
}

bool XBee::PingOut(u_char pid, u_int naddr)
{
   u_char msg[14];
   u_char frid = fid();

   memcpy(msg, "\x7E\x00\x0B\x01\xFF\xFF\xFF\x00\xFF\x24\xFF\xFF\xFF\x00", 14);
   msg[4] = frid; // frame id
   msg[5] = hb(m_fnb.addr); // address high byte
   msg[6] = lb(m_fnb.addr); // address low byte
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
   msg[5] = hb(m_rnb.addr); // address high byte
   msg[6] = lb(m_rnb.addr); // address low byte
   msg[8] = frid;
   msg[10] = pid; // packet id
   msg[11] = hb(naddr); // node address high
   msg[12] = lb(naddr); // node address low

   MsgQueue(msg, 14, frid);

   return true;
}


void XBee::ResetNeighbor(neighbor *nb, u_int addr)
{
   nb->addr = addr;
   nb->rss = 0x00;
   nb->nrss = 0x00;
   nb->i = 0;
   nb->rlen = 0;
   memset(nb->rssi, 0, 10);
   memset(nb->nrssi, 0, 10);
}


bool XBee::NeighborUpdate(u_int addr, u_char rss)
{
   bool found = false;
   neighbor *nodes[2];

   nodes[0] = &m_fnb;
   nodes[1] = &m_rnb;

   for (u_char i=0;i<2;i++)
   {
      if (nodes[i]->addr == addr)
      {
         nodes[i]->rss = rss;
         found = true;
      }
   }

   return found;
}

bool XBee::NeighborUpdate(u_int addr, u_char rss, u_char nrss)
{
   bool found = false;
   neighbor *nodes[2];

   nodes[0] = &m_fnb;
   nodes[1] = &m_rnb;

   for (u_char i=0;i<2;i++)
   {
      if (nodes[i]->addr == addr)
      {

         nodes[i]->rss = rss;
         nodes[i]->nrss = nrss;
         found = true;
         break;
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
      if (!m_outmsgs[i].active) 
      {
         id = i;
         found = true;
         break;
      }
   }

   memcpy(m_outmsgs[id].message, s, len);
   m_outmsgs[id].len = len;
   m_outmsgs[id].frameid = frameid;
   m_outmsgs[id].senttime = m_now;
   m_outmsgs[id].active = true;
   m_outmsgs[id].retries = 0;

   Tx(s, len);

   return true;
}

bool XBee::MsgMark(u_char frameid)
{
   bool marked = false;

   for (u_char i=0;i<MAX_MSGQ;i++)
   {
      if (m_outmsgs[i].frameid == frameid) 
      {
         memset(m_outmsgs[i].message, 0, MSG_SIZE);
         m_outmsgs[i].frameid = 0;
         m_outmsgs[i].senttime = 0;
         m_outmsgs[i].retries = 0;
         m_outmsgs[i].active = false;
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
      if (m_outmsgs[i].frameid == frameid)
      {
         Tx(m_outmsgs[i].message, m_outmsgs[i].len);
         m_outmsgs[i].retries++;
         m_outmsgs[i].senttime = m_now;
         marked = true;
      }
   }

   return marked;
}

bool XBee::MsgAudit()
{
   for (u_char i=0;i<MAX_MSGQ;i++)
   {
      if (m_outmsgs[i].active)
      {
         if ((m_now - m_outmsgs[i].senttime) > 100)
         {
            Tx(m_outmsgs[i].message, m_outmsgs[i].len);
            m_outmsgs[i].retries++;
            m_outmsgs[i].senttime = m_now;
         }

         if (m_outmsgs[i].retries > 3)
         {
            memset(m_outmsgs[i].message, 0, MSG_SIZE);
            m_outmsgs[i].len = 0;
            m_outmsgs[i].frameid = 0;
            m_outmsgs[i].senttime = 0;
            m_outmsgs[i].retries = 0;
            m_outmsgs[i].active = false;
         }
      }
   }

   return true;
}

void XBee::ServoMgr()
{
   // V is modeled for a continuous servo
   //  Where V = 90 means standing still
   //        V = 0 means full speed backwards
   //        V = 180 means full speed forwards
   int V = CalcCtrl();

   //  LED output demo
   if (V == 90)
   {
      //  Stay still and/or letting rear catch up
      digitalWrite(PIN_LED0, 1);
      digitalWrite(PIN_LED1, 1);
      digitalWrite(PIN_LED2, 1);
      digitalWrite(PIN_LED3, 1);
   }

   if (V < 90)
   {
      //  Back up fastest
      digitalWrite(PIN_LED0, 0);
      digitalWrite(PIN_LED1, 1);
      digitalWrite(PIN_LED2, 1);
      digitalWrite(PIN_LED3, 1);
   }

   if (V > 90)
   {
      //  Speed up (connection to rear too strong)
      digitalWrite(PIN_LED0, 1);
      digitalWrite(PIN_LED1, 1);
      digitalWrite(PIN_LED2, 1);
      digitalWrite(PIN_LED3, 0);
   }
}

int XBee::CalcCtrl()
{
   u_char RSSthh = 80;
   u_char RSSthl = RSSthh - 10;
   u_char rrssi = rearrssi();
   u_char frssi = frontrssi();
   int Vir = 0, Vif = 0;
   int Vo = 0;


   //  adjust servo signal 3-1 to RSS beyond threshold

   if (rrssi > RSSthh)  //  too far from rear
      Vir = -3 * (rrssi - RSSthh);
   else if (rrssi < RSSthl)  // too close to rear
      Vir = 3 * (RSSthl - rrssi);

   Vo = m_CtrlIn + Vir;

   if (frssi < 40)  // too close to front
      Vo = 90;

   if (Vo > 180)
      Vo = 180;

   if (Vo < 0)
      Vo = 0;

   return Vo; 
}

u_char XBee::navg(u_char *nbr, u_int len)
{
   u_char rssi = 0x5A; // -90 dbm
   u_int sum = 0;

   for (u_char i=0; i<len;i++)
   {
      sum += (int)nbr[i];
   }

   rssi = (u_char)(sum / len);

   return rssi;
}


void XBee::NRSSIAudit()
{
   neighbor *nodes[2];

   nodes[0] = &m_fnb;
   nodes[1] = &m_rnb;

   for (u_char i=0;i<2;i++)
   {
      if (nodes[i]->addr != 0xFFFF)
      {
         nodes[i]->rssi[nodes[i]->i] = nodes[i]->rss;
         nodes[i]->nrssi[nodes[i]->i] = nodes[i]->nrss;
         nodes[i]->i++;

         if (nodes[i]->i > 9)
            nodes[i]->i = 0;
         if (nodes[i]->rlen < 10)
            nodes[i]->rlen++;
      }
   }
}

u_char XBee::rearrssi()
{
   u_char rrssi = 70;
   u_char nrssi = 70;

   rrssi = navg(m_rnb.rssi, m_rnb.rlen);
   nrssi = navg(m_rnb.nrssi, m_rnb.rlen);

   return rrssi > nrssi ? nrssi : rrssi;
}

u_char XBee::frontrssi()
{
   u_char rrssi = 70;
   u_char nrssi = 70;

   rrssi = navg(m_fnb.rssi, m_rnb.rlen);
   nrssi = navg(m_fnb.nrssi, m_rnb.rlen);

   return rrssi > nrssi ? nrssi : rrssi;
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

      if (m_state == self_realizing)
      {
         m_state = rest;
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