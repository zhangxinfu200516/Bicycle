#ifndef __TASK_H__  
#define __TASK_H__

#define NEW
//#define OLD

enum Enum_Direction {FORWARD, BACKWARD};

extern void Task_Init();
extern void Task_Loop();
void Balance_Init();
void Balance_Task();
#endif