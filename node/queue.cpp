#include "Queue.h"
#include <string.h>

Queue::Queue() 
: m_Head(0),
  m_Tail(0),
  m_Size(0)
{
  memset(m_Data, 0, Q_SIZE);
}

Queue::~Queue()
{
  memset(m_Data, 0, Q_SIZE);
}

bool Queue::Enqueue(unsigned char d) 
{
    if (!Full())
    {
        m_Data[m_Tail++] = d;
        m_Tail %= Q_SIZE;
        m_Size++;
        return true;
    }else{
        return false;
    }
}

unsigned char Queue::Dequeue()
{
    unsigned char t=0;

    if (!Empty()) 
    {
        t = m_Data[m_Head];
        m_Data[m_Head++] = 0;
        m_Head %= Q_SIZE;
        m_Size--;
    }
    return t;
}

int Queue::QueueString(unsigned char * s, int len)
{
    int sendlen = 0;

    for (int i=0; i<len; i++)
    {
        if (Enqueue(*s))
        {
            s++;
            sendlen++;
        }else{ 
            break;
        }
    }

    return sendlen;
}


bool Queue::DataAt(unsigned int pos)
{
   pos += m_Head;
   pos %= Q_SIZE;

   if ((m_Head + m_Size) > Q_SIZE)
   {
      if ((pos >= m_Head) || (pos < m_Tail))
      {
        return true;
      }
  }
  else
  {
     if (pos >= m_Tail)
     {
        return false;
     }
  }

   return true;
}

bool Queue::WriteAt(unsigned int pos, unsigned char c)
{
   pos += m_Head;
   pos %= Q_SIZE;

   if (pos <= m_Tail)
   {
      m_Data[pos] = c;
      return true;
   }

   return false;
}

unsigned char Queue::Peek(unsigned int pos)
{
   unsigned char c = 0;

   pos += m_Head;
   pos %= Q_SIZE;

   if ((m_Head + m_Size) > Q_SIZE)
   {
      if ((pos >= m_Head) || (pos < m_Tail))
      {
        c = m_Data[pos];
      }
  }
  else
  {
     if (pos < m_Tail)
     {
        c = m_Data[pos];
     }
  }

   return c;
}

bool Queue::Clear(unsigned int pos)
{
   pos += m_Head;
   pos %= Q_SIZE;

   if (pos > m_Tail)
   { 
      // delete entire thing
      for (unsigned int i=0; i<Q_SIZE; i++) 
      {
         m_Data[i] = 0;
      }

      m_Head = 0;
      m_Tail = 0;
      m_Size = 0;
      return true;
   }

   while(m_Head != pos)
   {
      m_Data[m_Head++] = 0;
      m_Head %= Q_SIZE;
      m_Size--;
   }

   return true;
}
