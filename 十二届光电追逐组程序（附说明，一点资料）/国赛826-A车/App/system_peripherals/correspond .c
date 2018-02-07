#include "include.h"

void correspond_init();
void correspond_send();
void correspond_read();
void  A_send_B();
void  B_send_A();
void  A_read_B();
void  B_read_A();


uint32 i=0;
uint8 relen;
uint8 buff_tx[DATA_PACKET];//(1,0,0,1)为前车发给后车,(1,0,1,0)为旧前车发给新前车,(0,1,1,0)为后车发给前车,
//第五位为坡道超车控制位，第六位为直道控制位,第七位为避障,第八位为环道超车,第九位为停车，第十位为出界停车
uint8 buff_rx[DATA_PACKET];
uint8 nrf_flag=0;
uint8 nrf_send_finish_flag=0;
uint8 nrf_read_finish_flag=0;

uint8 nrf_send_delay_flag1;
int16 nrf_send_delay_cnt1;
uint8 STOP_OUT_able=0;
uint8 Q_loop_able=0;

void correspond_init()
{
      while(!nrf_init())                 //初始化NRF24L01+ ,等待初始化成功为止
      {
            Site_t site = {10,20};   //x = 10 ,y = 20
            LCD_str(site,"NRF error!!!", BLUE,RED);
      }
      nrf_flag = 1;
      
}
void correspond_send()
{
      
      
      if(nrf_tx(buff_tx,DATA_PACKET) == 1 )          //发送一个数据包：buff（包为32字节）
      {
            while(nrf_tx_state() == NRF_TXING);//等待发送完成
            if( NRF_TX_OK == nrf_tx_state () )
            {
                  nrf_send_finish_flag=1;
            }
            else
            {
                  nrf_send_finish_flag=0;
            }
      }
      else
      {
            nrf_send_finish_flag=0;
      }
}
void correspond_read()
{    
      nrf_read_finish_flag=0;  
      relen = nrf_rx(buff_rx,DATA_PACKET);               //等待接收一个数据包，数据存储在buff里
      if(relen != 0)
      {
            nrf_read_finish_flag=1;            //打印接收到的数据
      } 
}

void  A_send_B()
{
      uint8 temp[DATA_PACKET]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};
      temp[0]=1;temp[1]=0;temp[2]=0;temp[3]=1;
      
      if(Q_H_overtake_po_flag)//坡道超车
      {temp[4]=1;}
      else
      {temp[4]=0;}
      if(Q_H_overtake_cross_flag)//十字超车
      {temp[5]=1;}
      else if(Q_H_stop_ok)//十字前车靠边成功
      {temp[5]=2;}
      else
      {temp[5]=0;}
      if(Q_H_obstacle_flag)//有障碍
      {temp[6]=1;}
      else
      {temp[6]=0;}
      if(Q_H_overtake_ring_flag)//前车已停
      {temp[7]=1;}
      else
      {temp[7]=0;}
      
      if(STOP_PATTERN)
      {temp[8]=1;}
      else
      {temp[8]=0;}
      if(QAQ_STOP)//出界停车
      {temp[9]=1;}
      else
      {temp[9]=0;}
      
      if(LOOP_TEMP)//使能后车环道检测
      {temp[17]=1;}
      else
      {temp[17]=2;}
      if(LOR_turn_flag1)//靠边停车方向
      {temp[18]=1;}
      else
      {temp[18]=2;}
      
      memcpy(buff_tx,temp, DATA_PACKET* sizeof( uint8));
      correspond_send();
}
void  B_send_A()//超车，超车成功
{
      uint8 temp[DATA_PACKET]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,};
      
      temp[0]=0;temp[1]=1;temp[2]=1;temp[3]=0;
      
      if(H_Q_overtake_ring_flag)//后车出环
      {temp[7]=1;}
      else if(H_Q_overtake_outrow_flag)//后车看到出环口
      {temp[7]=2;}
      else
      {temp[7]=0;}
      
      if(QAQ_STOP)//出界停车
      {temp[9]=1;}
      else
      {temp[9]=0;}
      
      temp[10]=(((uint16) car_distance) & 0xFF00)>>8;
      temp[11]=((uint16) car_distance) & 0x00FF;
      if(chaoshengbo_flag)
      {temp[12]=1;}
      else
      {temp[12]=2;}
      if((car_extract_row>0) || (END_LINE_TEMP &&(abs(speedout_val)>=4*10000)))
      {temp[13]=1;}
      else
      {temp[13]=2;}
      
      if(overtake_ok_flag)//起跑超车成功
      {temp[28]=1;}
      else
      {temp[28]=0;}
      if(overtake_start_ok)//起跑障碍超车成功
      {temp[29]=1;}
      else
      {temp[29]=0;}
      if(overtake_cross_ok)//十字超车成功
      {temp[30]=1;}
      else
      {temp[30]=0;}
      if(overtake_po_ok)//坡道超车成功
      {temp[31]=1;}
      else
      {temp[31]=0;}
      
      memcpy(buff_tx,temp, DATA_PACKET* sizeof( uint8));
      correspond_send();
}
void  A_read_B()
{
      correspond_read();
      uint16 i=0,j=0;
      //------------------------------------超声波数据-----------------------------------//
      if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[12]==1))
      {
            H_csb_get=1;
            if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0))
            {
                  i=((uint16)buff_rx[10])<<8;
                  j=buff_rx[11];
                  Q_distance=i + j;
            }
      }
      else if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[12]==2))
      {
            H_csb_get=0;
      }
      if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[13]==1))
      {
            H_QC_get=1;
      }
      else if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[13]==2))
      {
            H_QC_get=0;
      }
      //------------------------------------起跑超车成功-----------------------------------//
      if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[28]==1))
      {
            overtake_start_flag=0;//退出超车模式
            
            speedout_val0_flag=0;
            START_LINE_TEMP=0;//清标志
            
            car_stop_flag=0;//启动，中间行驶
            car_R_pull_over_flag=0;
            car_L_pull_over_flag=0;
             keep_side=0;
            car_flag=1;//变后车
            gpio_set(PORT_CSB, 0);
            barrier_able=0;//关障碍检测
            
            nrf_read_finish_flag=1;
      }
      else
            nrf_read_finish_flag=0;
      //-----------------------------------起跑障碍超车成功------------------------------------//
      if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[29]==1))
      {
            overtake_start_flag=0;
            LOR_turn_send_ok=0;
            
            car_stop_flag=0;//启动，中间行驶
            car_R_pull_over_flag=0;
            car_L_pull_over_flag=0;
            keep_side=0;
            car_flag=1;//变后车
            keep_side=0;
            gpio_set(PORT_CSB, 0);
            barrier_able=0;//关障碍检测
            
            nrf_read_finish_flag=1;
      }
      else
            nrf_read_finish_flag=0;
      //-----------------------------------十字超车成功------------------------------------//
      if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[30]==1))
      {
            overtake_cross_flag=0;
            overtake_cross_stop=0;
            overtake_cross_stop_over=1;
            bangbang_tingcnt=0;
            Q_overtake_cross=0;
            speedout_val_cross_flag=0;
            speedout_val_cross_ting_flag=0;
            Q_H_stop_ok=0;
            
            car_stop_ing_flag=0;
            car_stop_flag=0;
            cross_jian=0; 
            qibu_flag=1;
            car_R_pull_over_flag=0;
            car_L_pull_over_flag=0;
            keep_side=0;
            car_flag=1;
            gpio_set(PORT_CSB, 0);
            barrier_able=0;//关障碍检测
            
            overtake_cross_sum--;
            
            in_curve_flag=1;
            if(L_TURN )
            {
                  in_L_curve_flag=1;
            }
            else
            {
                  in_R_curve_flag=1;
            }
      }
      else
            nrf_read_finish_flag=0;
      //-----------------------------------坡道超车成功------------------------------------//
      if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[31]==1))
      {
            car_stop_ing_flag=0;
            car_stop_flag=0;
            overtake_po_stop=0;
            overtake_po_stop_over=1;
            qibu_flag=1;
            overtake_po_flag=0;//启动，退出模式
            
            speedout_val_po_flag=0;
            speedout_val_po_flag=0;//清标志位
            car_R_pull_over_flag=0;
            car_L_pull_over_flag=0;
            keep_side=0;       
            po_pull_over=0;
            car_flag=1;//交换前后车
            gpio_set(PORT_CSB, 0);
            barrier_able=0;
            
            overtake_po_sum--;//超车次数自减
            down_flag=0;
      }
      else
            nrf_read_finish_flag=0;
      
      //-------------------------------------------------------------------------//
      if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[7]==1))//后车出环
      {
            overtake1_ring_ok_flag=1;
            nrf_read_finish_flag=1;
      }
      else if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[7]==2))//后车看到出环口
      {
            overtake_outrow_flag=1;
            nrf_read_finish_flag=1;
      }
      else
            nrf_read_finish_flag=0;
      
      //-------------------------------------------------------------------------//
      if((buff_rx[0]==0)&&(buff_rx[1]==1)&&(buff_rx[2]==1)&&(buff_rx[3]==0)&&(buff_rx[9]==1))//出界停车
      {
            if(STOP_OUT_able)
            {
                  car_status = CAR_STOPP;
            }
            else
            {
                  double_car_flag=0;
                  car_L_pull_over_flag=0;
                  car_R_pull_over_flag=0;
                  car_stop_flag=0;
                  car_stop_ing_flag=0;
                  car_status = CAR_RUN;
            }
      }
      
      clean_buff_rx;
}

void  B_read_A()
{
      correspond_read();
      
      //-----------------------------------使能后车环道检测------------------------------------//
      if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[17]==1))
      {
            Q_loop_able=1;
      }
      else if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[17]==2))
      {
            Q_loop_able=0;
      }
      //-----------------------------------靠边停车方向------------------------------------//
      if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[18]==1))
      {
            LOR_turn_flag2=1;
      }
      else if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[18]==2))
      {
            LOR_turn_flag2=0;
      }
      //----------------------------------坡道超车-------------------------------------//
      if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[4]==1))
      {
            overtake_po_flag=1;
            nrf_read_finish_flag=1;
      }
      else
            nrf_read_finish_flag=0;
      //-----------------------------------十字超车------------------------------------//
      if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[5]==1))
      {
            overtake_cross_flag=1;
            nrf_read_finish_flag=1;
      }
      else if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[5]==2))//十字前车靠边成功
      {
            Q_stop_ok=1;
            nrf_read_finish_flag=1;
      }
      else
            nrf_read_finish_flag=0;
      //-----------------------------------开启避障------------------------------------//
      if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[6]==1))
      {
            barrier_able_flag=1;//开启避障
            nrf_read_finish_flag=1;
      }
      else
            nrf_read_finish_flag=0;
      //-----------------------------------环道，前车已停------------------------------------//
      if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[7]==1))
      {
            ringout_able_flag=1;
            nrf_read_finish_flag=1;
      }
      else
            nrf_read_finish_flag=0;
      
      //-------------------------------------停车-------------------------------------//
      if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[8]==1))
      {
            STOP_PATTERN=1;
      }
      else
            nrf_read_finish_flag=0;
      
#if 1      //--------------------------------出界停车----------------------------------//
      if((buff_rx[0]==1)&&(buff_rx[1]==0)&&(buff_rx[2]==0)&&(buff_rx[3]==1)&&(buff_rx[9]==1))
      {
            if(STOP_OUT_able)
            {
                  car_status = CAR_STOPP;
            }
            else
            {
                  double_car_flag=0;
                  car_L_pull_over_flag=0;
                  car_R_pull_over_flag=0;
                  car_stop_flag=0;
                  car_stop_ing_flag=0;
                  car_status = CAR_RUN;
            }
      }
#endif      
      clean_buff_rx;
}

//-----------------------------发车通信协议--------------------------------//
#if 1
uint32 A_send_B0_flag=0,B_read_A0_flag=0,B_send_A0_flag=0,A_read_B0_flag=0;

void  A_send_B0()
{
      uint8 temp[DATA_PACKET]={0,0,1,1};//请求发车
      A_send_B0_flag++;
      memcpy(buff_tx,temp, 4* sizeof( uint8));
      correspond_send();
}
void  B_read_A0()
{
      B_read_A0_flag++;
      correspond_read();
      if((buff_rx[0]==0)&&(buff_rx[1]==0)&&(buff_rx[2]==1)&&(buff_rx[3]==1))
      {
            while(!nrf_send_finish_flag)
            {
                  send_start_flag=1;
                  B_send_A0();
                  if(send_error_flag && !nrf_send_finish_flag)
                  {
                        nrf_read_finish_flag=0;
                        break;
                  }
            }
            send_start_flag=0;
            send_error=0;
            send_error_flag=0;
            if(nrf_send_finish_flag)
                  car_start_flag=0;
      }
      else
            nrf_read_finish_flag=0;
      clean_buff_rx;
}   
void  B_send_A0()
{
      uint8 temp[DATA_PACKET]={1,1,0,0};//1,0前车，0,1保持后车，1超车成功
      B_send_A0_flag++;
      memcpy(buff_tx,temp, 4 * sizeof( uint8));
      correspond_send();
}
void  A_read_B0()
{
      A_read_B0_flag++;
      correspond_read();
      if((buff_rx[0]==1)&&(buff_rx[1]==1)&&(buff_rx[2]==0)&&(buff_rx[3]==0))
      {
            car_start_flag=0;
      }
      else
            nrf_read_finish_flag=0;
      clean_buff_rx;
}
#endif