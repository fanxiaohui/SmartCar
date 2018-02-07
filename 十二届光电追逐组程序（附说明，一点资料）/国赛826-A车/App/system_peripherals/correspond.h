#ifndef _CORRESPOND_H_
#define _CORRESPOND_H_
#define NRF_SEND_DELAY_TIME1    5
extern void correspond_init();
extern void correspond_send();
extern void correspond_read();

extern void  A_send_B();

extern void  B_send_A();

extern void  A_read_B();
extern void  B_read_A();

extern void  A_send_B0();
extern void  B_send_A0();
extern void  A_read_B0();
extern void  B_read_A0();

extern uint8 nrf_flag;
extern  uint8 nrf_send_finish_flag;
extern uint8 nrf_read_finish_flag;
extern uint8 buff_tx[DATA_PACKET];
extern uint8 buff_rx[DATA_PACKET];
extern uint8 nrf_send_delay_flag1;
extern int16 nrf_send_delay_cnt1;
extern uint8 STOP_OUT_able;
extern uint8 Q_loop_able;

#define clean_buff_rx           memset(buff_rx,0,sizeof(buff_rx));

#endif