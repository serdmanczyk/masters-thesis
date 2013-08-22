//#include <rxduino.h>
#include <string.h>
#include "Task.h"

Task::Task()
{
    _itemsInTask = 0;
    _TaskStart = 0;
    _TaskEnd = 0;
}

int Task::scheduleFunction(TaskFunction func, const char * id, unsigned long initialRun, unsigned long recur)
{
    int rv = 0;

    if(strlen(id) > 7)
    {
        rv = -1;
    } else {

        TaskItem newItem;
        newItem.fPtr = func;
        memset(newItem.itemName, 0, 8);
        memcpy(newItem.itemName, id, strlen(id));
        newItem.recur = recur;
        newItem.next = initialRun;

        rv = _addToTask(newItem);
    }

    return rv;
}

int Task::scheduleRemoveFunction(const char * id)
{
    TaskItem target;
    int rv = -1;
    for (int i = 0; i < _itemsInTask; ++i)
    {
        if(_TaskGetTop(target) == 0)
        {
            if(strcmp(target.itemName, id) == 0)
            {
                rv = 0;
            } else {
                _addToTask(target);
            }
        } else {
            rv = -1;
            break;
        }
    }

    return rv;
}

int Task::scheduleChangeFunction(const char * id, unsigned long nextRunTime, unsigned long newRecur)
{
    TaskItem target;
    int rv = -1;
    for (int i = 0; i < _itemsInTask; ++i)
    {
        if(_TaskGetTop(target) == 0)
        {
            if(strcmp(target.itemName, id) == 0)
            {
                target.next = nextRunTime;
                target.recur = newRecur;
                rv = 0;
            }
            _addToTask(target);
        } else {
            rv = -1;
            break;
        }
    }

    return rv;
}

int Task::Run(unsigned long now)
{
    TaskItem target;
    int rv = 0;
    if(_itemsInTask == 0)
    {
        rv = -1;
    }
    for (int i = 0; i < _itemsInTask; ++i)
    {
        if(_TaskGetTop(target)==0)
        {
            if(target.next <= now)
            {
                int tRv;
                tRv = (target.fPtr)(now);
                if(tRv == 0)
                {
                    rv++;
                }
                if(target.recur != 0)
                {
                    target.next = now + target.recur;
                    _addToTask(target);
                }
            } else {
                _addToTask(target);
            }
        } else {
            rv = -1;
            break;
        }
    }

    return rv;
}

int Task::_TaskGetTop(TaskItem &item)
{
    int rv = 0;
    //Remove the top item, stuff it into item
    if (_TaskEnd != _TaskStart) {
            TaskItem tempTaskItem = _schedule[_TaskStart];
            //This Algorithm also from Wikipedia.
            _TaskStart = (_TaskStart + 1) % TaskScheduleSize;
            item = tempTaskItem;
            _itemsInTask--;
    } else {
    //if the buffer is empty, return an error code
        rv = -1;
    }

    return rv;  
}

int Task::_addToTask(TaskItem item)
{
    //This is just a circular buffer, and this algorithm is stolen from wikipedia
    int rv = 0;
    if ((_TaskEnd + 1) % TaskScheduleSize != _TaskStart) {
        _schedule[_TaskEnd] = item;
        _TaskEnd = (_TaskEnd + 1) % TaskScheduleSize;
        _itemsInTask++;
    } else {
        //if buffer is full, error
        rv = -1;
    }
    return rv;
}