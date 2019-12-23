/****************************************
 * Author : Quentin C			*
 * Mail : quentin.chateau@gmail.com	*
 * Date : 29/11/13			*
 ****************************************/
#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

void ControlPrepareNewGoal(void);
void ControlReset(void); // TODO unused ?
void ControlSetStop(int mask);
void ControlUnsetStop(int mask);

void ControlInit(void);
void ControlCompute(void);
extern uint16_t lastReachedID;
#ifdef __cplusplus
}
#endif

#endif
