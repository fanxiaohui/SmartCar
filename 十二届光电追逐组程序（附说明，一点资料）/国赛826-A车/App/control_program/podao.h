#ifndef _PODAO_H_
#define _PODAO_H_

#define UP_FLAG_TEMP_delay_time1   25
#define DOWN_FLAG_TEMP_delay_time1  25
#define DOWN_FLAG_TEMP_delay_time2  100
#define UP_HILL_YUZHI    50
#define DOWN_HILL_YUZHI  -50
#define UP_HILL_SUM      200
#define DOWN_HILL_SUM   -200
extern void podao_jc();
extern void podao_get();
extern uint8 jiance;
extern uint8 up_hill,down_hill;
extern uint8 down_flag;
extern uint8 up_flag;
extern uint8 up_enable_flag;
extern uint8 po_loop_able;
extern int16 UP_FLAG_TEMP_1;
extern int16 DOWN_FLAG_TEMP_1;
extern int16 DOWN_FLAG_TEMP_2;
extern int16 DOWN_FLAG_TEMP_cnt2;
extern int16 po_cnt;
#endif