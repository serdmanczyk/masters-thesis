// Copyright (c) 2012 Brad Luyster
//
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
// copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in 
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT
// OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef Task_H
#define Task_H

typedef int (*TaskFunction)(unsigned long);

#define TaskScheduleSize 11

struct TaskItem {
    TaskFunction fPtr;
    unsigned long next;
    unsigned long recur;
    char itemName[8];
};

class Task
{
private:
    unsigned int _TaskStart;
    unsigned int _TaskEnd;
    unsigned int _itemsInTask;
    TaskItem _schedule[TaskScheduleSize];

    int _TaskGetTop(TaskItem &item);
    int _addToTask(TaskItem item);

public:
    Task();

    int scheduleFunction(TaskFunction func, const char * id, unsigned long initialRun, unsigned long recur);
    int scheduleRemoveFunction(const char * id);
    int scheduleChangeFunction(const char * id, unsigned long nextRunTime, unsigned long newRecur);

    int Run(unsigned long now);
    /* data */
};
#endif