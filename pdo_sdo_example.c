//接受回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t count = 0;
    // debugPrintPending("ID====%d -----------------------\r\n",debugCanRxMsg.msg.header.StdId );
    /* Get RX message */
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &debugCanRxMsg.msg.header, RxData) != HAL_OK)
    {
        /* Reception Error */
        //Error_Handler();
    }
		
		
    if(debugCanRxMsg.msg.header.StdId == 0x180 + motor_1.deviceId)
    {
        if(motor_1.init_time<5){
			motor_1.initPosition=(int32_t)((RxData[3] << 24) + (RxData[2] << 16)  + (RxData[1] << 8) + RxData[0]);
			motor_1.positionActual=0;
			motor_1.init_time++;
		}else{
		motor_1.positionActual =(int32_t)((RxData[3] << 24) + (RxData[2] << 16)  + (RxData[1] << 8) + RxData[0])-motor_1.initPosition;
        motor_1.currentActual = (int16_t)((RxData[7] << 8) + RxData[6]);
        //motor_1.stateWord.raw = (uint16_t)((RxData[5] << 8) + RxData[4]);
        parseStateWord(&motor_1.stateWord, (uint16_t)((RxData[5] << 8) + RxData[4]));
//		debugPrintPending("Send positionActual= %d\r\n",motor_1.positionActual );
//		debugPrintPending("Send Current1= %d\r\n",motor_1.currentActual );
		}
    }
    if(debugCanRxMsg.msg.header.StdId == 0x280 + motor_1.deviceId)
    {
        motor_1.velocityActual = (int32_t)((RxData[3] << 24) + (RxData[2] << 16)  + (RxData[1] << 8) + RxData[0]);
        motor_1.torqueActual = (int16_t)((RxData[7] << 8) + RxData[6]);
        //motor_1.stateWord.raw = (uint16_t)((RxData[5] << 8) + RxData[4]);
        parseStateWord(&motor_1.stateWord, (uint16_t)((RxData[5] << 8) + RxData[4]));
//		debugPrintPending("Send velocity1= %d\r\n",motor_1.velocityActual);
    }

    if(debugCanRxMsg.msg.header.StdId == 0x180 + motor_2.deviceId)
    {
		if(motor_2.init_time<5){
			motor_2.initPosition=(int32_t)((RxData[3] << 24) + (RxData[2] << 16)  + (RxData[1] << 8) + RxData[0]);
			motor_2.positionActual=0;
			motor_2.init_time++;
		}else{
			motor_2.positionActual = (int32_t)((RxData[3] << 24) + (RxData[2] << 16)  + (RxData[1] << 8) + RxData[0])-motor_2.initPosition;
			motor_2.currentActual = (int16_t)((RxData[7] << 8) + RxData[6]);
			//motor_2.stateWord.raw = (uint16_t)((RxData[5] << 8) + RxData[4]);
			parseStateWord(&motor_2.stateWord, (uint16_t)((RxData[5] << 8) + RxData[4]));
			//debugPrintPending("Send Current2= %d\r\n",motor_2.positionActual);
		}
    }

    if(debugCanRxMsg.msg.header.StdId == 0x280 + motor_2.deviceId)
    {
        motor_2.velocityActual = (int32_t)((RxData[3] << 24) + (RxData[2] << 16)  + (RxData[1] << 8) + RxData[0]);
        motor_2.torqueActual = (int16_t)((RxData[7] << 8) + RxData[6]);
        //motor_1.stateWord.raw = (uint16_t)((RxData[5] << 8) + RxData[4]);
        parseStateWord(&motor_2.stateWord, (uint16_t)((RxData[5] << 8) + RxData[4]));
//		debugPrintPending("Send Current1= %d\r\n",motor_1.currentActual );
    }


#if CAN_MSG_DEBUG == 1
    /* debug *********************************************************************************/
  debugCanRxMsg.msg.timeStamp = TIMER_BASE_TICK;
  for(count = 0;count < 8;count++)
  {
    debugCanRxMsg.msg.canData[count] = RxData[count];
  }

  /* Add mag to debug buf */
  addDebugMsg(&uartDebug, debugCanRxMsg.data, sizeof(DebugCanRxMsg), sendCanRxMsgByUart);
  /****************************************************************************************/
#endif

}

//发送函数
//循环发送电流
void setMotorCurrent(unsigned int device_id, short int current)
{
    //set target torque
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x71;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=current>>(0*8)& 0xFF;
    CAN_SendData[5]=current>>(1*8)& 0xFF;
    CAN_SendData[6]=0x00;
    CAN_SendData[7]=0x00;
    canSendMessageNoDelay(id,len,CAN_SendData);

    //execute the motion
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x0f;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessageNoDelay(id,len,CAN_SendData);
}

void initMotor(void)
{
    motor_1.deviceId = 0x7f; //DEC:127 ankle
    motor_1.polarity = 0; //1000 0000
    motor_1.currentRated = 7000;//unit: millamp
    motor_1.currentSlope = 1000000;//unit: per thousand of rated current per second
    motor_2.deviceId = 0x7e; //Knee
    motor_2.polarity = 0;//0x80//position polarity value = -1
    motor_2.currentRated = 7000;//unit: millamp
    motor_2.currentSlope = 1000000;//unit: per thousand of rated current per second       //1000
    initMotorTpdo_1(motor_1.deviceId);
    initMotorTpdo_2(motor_1.deviceId);
    initMotorRatedCurrent(motor_1.deviceId, motor_1.currentRated);
    initMotorPolarity(motor_1.deviceId, motor_1.polarity);
    initMotorIpMode(motor_1.deviceId);
    initMotorPtMode(motor_1.deviceId, motor_1.currentRated, motor_1.currentSlope);
    //  initMotorPpMode(motor_1.deviceId);
    initMotorTpdo_1(motor_2.deviceId);
    initMotorTpdo_2(motor_2.deviceId);
    initMotorRatedCurrent(motor_2.deviceId, motor_2.currentRated);
    initMotorPolarity(motor_2.deviceId, motor_2.polarity);
    initMotorIpMode(motor_2.deviceId);
    initMotorPtMode(motor_2.deviceId, motor_2.currentRated, motor_2.currentSlope);
    //  initMotorPpMode(motor_2.deviceId);
}
void initMotorPolarity(unsigned int device_id, unsigned char polarity)
{
    //set polarity
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x7e;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=polarity;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);
}

void initMotorRatedCurrent(unsigned int device_id, unsigned int current)
{
    //set rated current
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x75;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=current>>(0*8)& 0xFF;
    CAN_SendData[5]=current>>(1*8)& 0xFF;
    CAN_SendData[6]=current>>(2*8)& 0xFF;
    CAN_SendData[7]=current>>(3*8)& 0xFF;
    canSendMessageNoDelay(id,len,CAN_SendData);
}

void initMotorPtMode(unsigned int device_id, unsigned int current, unsigned int slope)
{
    //set node to operational state
    id=0x00;
    len=2;
    CAN_SendData[0]=1;
    CAN_SendData[1]=device_id;CAN_SendData[1]=0x00;CAN_SendData[2]=0x00;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set current slope
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x87;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=slope>>(0*8)& 0xFF;
    CAN_SendData[5]=slope>>(1*8)& 0xFF;   //�˴�ĿǰȡֵΪ   slope=1000
    CAN_SendData[6]=slope>>(2*8)& 0xFF;
    CAN_SendData[7]=slope>>(3*8)& 0xFF;
    canSendMessage(id,len,CAN_SendData);

    //set rated current
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x75;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=current>>(0*8)& 0xFF;
    CAN_SendData[5]=current>>(1*8)& 0xFF;
    CAN_SendData[6]=current>>(2*8)& 0xFF;
    CAN_SendData[7]=current>>(3*8)& 0xFF;
    canSendMessageNoDelay(id,len,CAN_SendData);

    //READY 2 SWITCH ON
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x06;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //SWITCH ON
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x07;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //mode of operation -> profiled torque
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x60;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x04;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set target torque
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x71;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessageNoDelay(id,len,CAN_SendData);

    //START MO=1
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x0F;CAN_SendData[5]=0x01;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessageNoDelay(id,len,CAN_SendData);
}

void initMotorTpdo_1(unsigned int device_id)
{
    //set heart beat time -> 1 second
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x17;CAN_SendData[2]=0x10;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0xe8;CAN_SendData[5]=0x03;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set node to pre operational state
    id=0x00;
    len=2;
    CAN_SendData[0]=0x80;CAN_SendData[1]=device_id;CAN_SendData[2]=0x00;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set pdo parameter subindex 1 -> set bit31 to 1, not valid
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x00;CAN_SendData[2]=0x18;CAN_SendData[3]=0x01;
    CAN_SendData[4]=0x80+device_id;CAN_SendData[5]=0x01;CAN_SendData[6]=0x00;CAN_SendData[7]=0xc0;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 0 -> 0
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x00;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 1 -> position actual value
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x00;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x01;
    CAN_SendData[4]=0x20;CAN_SendData[5]=0x00;CAN_SendData[6]=0x64;CAN_SendData[7]=0x60;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 2 -> status word
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x00;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x02;
    CAN_SendData[4]=0x10;CAN_SendData[5]=0x00;CAN_SendData[6]=0x41;CAN_SendData[7]=0x60;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 3 -> current actual value
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x00;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x03;
    CAN_SendData[4]=0x10;CAN_SendData[5]=0x00;CAN_SendData[6]=0x78;CAN_SendData[7]=0x60;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 0 -> 3
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x00;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x03;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set pdo parameter subindex 2 -> cyclic synchronization
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x00;CAN_SendData[2]=0x18;CAN_SendData[3]=0x02;
    CAN_SendData[4]=0x01;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set pdo parameter subindex 1 -> set bit31 to 0, valid
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x00;CAN_SendData[2]=0x18;CAN_SendData[3]=0x01;
    CAN_SendData[4]=0x80+device_id;CAN_SendData[5]=0x01;CAN_SendData[6]=0x00;CAN_SendData[7]=0x40;
    canSendMessage(id,len,CAN_SendData);

}

void initMotorTpdo_2(unsigned int device_id)
{
//  set heart beat time -> 1 second
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x17;CAN_SendData[2]=0x10;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0xe8;CAN_SendData[5]=0x03;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set node to pre operational state
    id=0x00;
    len=2;
    CAN_SendData[0]=0x80;CAN_SendData[1]=device_id;CAN_SendData[2]=0x00;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set pdo parameter subindex 1 -> set bit31 to 1, not valid
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x01;CAN_SendData[2]=0x18;CAN_SendData[3]=0x01;
    CAN_SendData[4]=0x80+device_id;CAN_SendData[5]=0x02;CAN_SendData[6]=0x00;CAN_SendData[7]=0xc0;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 0 -> 0
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x01;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 1 -> Velocity actual value
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x01;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x01;
    CAN_SendData[4]=0x20;CAN_SendData[5]=0x00;CAN_SendData[6]=0x6C;CAN_SendData[7]=0x60;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 2 -> status word
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x01;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x02;
    CAN_SendData[4]=0x10;CAN_SendData[5]=0x00;CAN_SendData[6]=0x41;CAN_SendData[7]=0x60;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 3 -> Torque actual value
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x01;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x03;
    CAN_SendData[4]=0x10;CAN_SendData[5]=0x00;CAN_SendData[6]=0x77;CAN_SendData[7]=0x60;
    canSendMessage(id,len,CAN_SendData);

    //set pdo map sub index 0 -> 3
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x01;CAN_SendData[2]=0x1a;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x03;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set pdo parameter subindex 2 -> cyclic synchronization
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x01;CAN_SendData[2]=0x18;CAN_SendData[3]=0x02;
    CAN_SendData[4]=0x01;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set pdo parameter subindex 1 -> set bit31 to 0, valid
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x01;CAN_SendData[2]=0x18;CAN_SendData[3]=0x01;
    CAN_SendData[4]=0x80+device_id;CAN_SendData[5]=0x02;CAN_SendData[6]=0x00;CAN_SendData[7]=0x40;
    canSendMessage(id,len,CAN_SendData);

}


void initMotorIpMode(unsigned int device_id)
{
    uint32_t ac=5000000,dc=5000000;

    //set node to operational state
    id=0x00;
    len=2;
    CAN_SendData[0]=1;
    CAN_SendData[1]=device_id;CAN_SendData[1]=0x00;CAN_SendData[2]=0x00;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //clear all data records -> set 0
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0xc4;CAN_SendData[2]=0x60;CAN_SendData[3]=0x06;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //enable access to buffer -> set 1
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0xc4;CAN_SendData[2]=0x60;CAN_SendData[3]=0x06;
    CAN_SendData[4]=0x01;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set sub mode -> 0
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0xc0;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //actual buf size -> 1
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0xc4;CAN_SendData[2]=0x60;CAN_SendData[3]=0x02;
    CAN_SendData[4]=0x01;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //buffer organization -> 0(FIFO)
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0xc4;CAN_SendData[2]=0x60;CAN_SendData[3]=0x03;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //interpolation period -> 10ms
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0xc2;CAN_SendData[2]=0x60;CAN_SendData[3]=0x01;
    CAN_SendData[4]=0x0a;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //extrapolation cycles timeout -> 2 cycles
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x75;CAN_SendData[2]=0x2f;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x02;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set acceleration -> ac
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x83;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=ac>>(0*8)& 0xFF;
    CAN_SendData[5]=ac>>(1*8)& 0xFF;
    CAN_SendData[6]=ac>>(2*8)& 0xFF;
    CAN_SendData[7]=ac>>(3*8)& 0xFF;
    canSendMessage(id,len,CAN_SendData);

    //set deceleration -> dc
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x84;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=dc>>(0*8)& 0xFF;
    CAN_SendData[5]=dc>>(1*8)& 0xFF;
    CAN_SendData[6]=dc>>(2*8)& 0xFF;
    CAN_SendData[7]=dc>>(3*8)& 0xFF;
    canSendMessage(id,len,CAN_SendData);

    //READY 2 SWITCH ON
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x06;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //set position to px = 0
    id=0x300 + device_id;
    len=8;
    CAN_SendData[0]=0x50;CAN_SendData[1]=0x78;CAN_SendData[2]=0x00;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //change the PWM freq -> xp[2]=6
    CAN_SendData[0]=0x78;CAN_SendData[1]=0x50;CAN_SendData[2]=0x02;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x06;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //READY 2 SWITCH ON
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x06;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //SWITCH ON
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x07;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //mode of operation -> interpolation position
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x60;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x07;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //START MO=1
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x1F;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessageNoDelay(id,len,CAN_SendData);
}

void initMotorPpMode(unsigned int device_id)
{
    id=0x00;
    len=2;
    CAN_SendData[0]=1;
    CAN_SendData[1]=device_id;CAN_SendData[1]=0x00;CAN_SendData[2]=0x00;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //mode of operation -> Cyclic synchronous position
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x60;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x01;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //acc -> 1e6
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x83;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x40;CAN_SendData[5]=0x42;CAN_SendData[6]=0x0f;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //dec -> 1e6
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x84;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x40;CAN_SendData[5]=0x42;CAN_SendData[6]=0x0f;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //change the PWM freq -> xp[2]=6
    CAN_SendData[0]=0x78;CAN_SendData[1]=0x50;CAN_SendData[2]=0x02;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x06;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //READY 2 SWITCH ON
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x06;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //SWITCH ON
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x07;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //PX=0
    id=0x300 + device_id;
    len=8;
    CAN_SendData[0]=0x50;CAN_SendData[1]=0x78;CAN_SendData[2]=0x00;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x00;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //speed -> 80000
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x81;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x10;CAN_SendData[5]=0x27;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);

    //START MO=1
    id=0x600 + device_id;
    len=8;
    CAN_SendData[0]=0x22;CAN_SendData[1]=0x40;CAN_SendData[2]=0x60;CAN_SendData[3]=0x00;
    CAN_SendData[4]=0x0F;CAN_SendData[5]=0x00;CAN_SendData[6]=0x00;CAN_SendData[7]=0x00;
    canSendMessage(id,len,CAN_SendData);
}
