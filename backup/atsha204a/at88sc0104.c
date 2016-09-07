#include "at88sc0104.h"

#define BYTE char

unsigned char ucCM_InsBuff[4];

unsigned char g_pass_word1[8] = {0xAA, 0x34, 0x45, 0x99, 0xF0, 0xE4, 0xBB, 0x00};
unsigned char g_pass_word2[8] = {0x45, 0x84, 0x4F, 0xA9, 0x74, 0x95, 0x83, 0x74};
unsigned char pucBuffer1[8]   = {0xAB, 0x35, 0x75, 0x89, 0x75, 0xF4, 0xBA, 0x08};
unsigned char pucBuffer2[8]   = {0xA4, 0x15, 0x95, 0x6D, 0x78, 0x44, 0xCA, 0x99};
unsigned char pucBuffer3[8]   = {0xA6, 0x65, 0xB5, 0x9F, 0x7A, 0x54, 0xB9, 0x34};
unsigned char pucBuffer4[8]   = {0x3B, 0x05, 0xC5, 0x3A, 0x95, 0x94, 0xB6, 0xA8};

void cm_Delay(uchar ucDelay)
{
	udelay(ucDelay*30);
}

#if 0
char __gpio_get( void )
{
    if(DV_GPIO_cm_I2C_Date_Input() == 1)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
#endif

#if 1
static void gpio_init(void)
{
    CM_DIR_INIT;
}
#endif

void cm_ClockCycle(void)
{
    cm_Delay(1);
    CM_CLK_LO;
    cm_Delay(2);
    CM_CLK_HI;
    cm_Delay(1);
}

void cm_ClockCycles(uchar ucCount)
{
    uchar i;

    for (i = 0; i < ucCount; i++)
    {
        cm_ClockCycle();
    }

    CM_CLK_LO;
}

void cm_PowerOn(void)
{
    CM_CLK_LO;

    CM_DATA_HI;
    cm_Delay(2);
    CM_CLK_HI;
    cm_Delay(8);

    cm_ClockCycles(CM_PWRON_CLKS);
}

void cm_Clockhigh(void)
{
    cm_Delay(1);
    CM_CLK_HI;
    cm_Delay(1);
}

void cm_Clocklow(void)
{
    cm_Delay(1);
    CM_CLK_LO;
    cm_Delay(1);
}

void cm_Start(void)
{
	CM_DATA_OUT;                         // Data line must be an output to send a start sequence
    cm_Clocklow();
    CM_DATA_HI;
    cm_Delay(4);
    cm_Clockhigh();
    cm_Delay(4);
    CM_DATA_LO;
    cm_Delay(8);
    cm_Clocklow();
    cm_Delay(8);
}

void cm_Stop(void)
{
	CM_DATA_OUT;                         // Data line must be an output to send a start sequence
	cm_Clocklow();
	CM_DATA_LO;
	cm_Delay(4);
	cm_Clockhigh();
	cm_Delay(8);
	CM_DATA_HI;
	cm_Delay(4);
}

void cm_AckNak(uchar ucAck)
{
	CM_DATA_OUT;                         // Data line must be an output to send a start sequence
	cm_Clocklow();

	if (ucAck) CM_DATA_LO;          // Low on data line indicates an ACK
	else       CM_DATA_HI;          // High on data line indicates an NACK

	cm_Delay(2);
	cm_Clockhigh();
	cm_Delay(8);
	cm_Clocklow();
}


uchar cm_Write(uchar ucData)
{
    uchar i;
	CM_DATA_OUT;                         // Data line must be an output to send a start sequence

    for(i=0; i<8; i++)
    {
        cm_Clocklow();               // Send 8 bits of data

        if (ucData&0x80) CM_DATA_HI;
        else             CM_DATA_LO;

        cm_Clockhigh();

        ucData = ucData<<1;
    }

    cm_Clocklow();

#if 1
	CM_DATA_IN;
#else	
    CM_DATA_HI;
    WriteTW88(GPIO_OE0, ReadTW88(GPIO_OE0) & 0xFD);  /*  立即将date脚配置成输入*/
#endif
    cm_Delay(8);

    cm_Clockhigh();

    while(i > 1)
    {                               // loop waiting for ack (loop above left i == 8)
        cm_Delay(2);

        if (CM_DATA_RD) i--;        // if SDA is high level decrement retry counter
        else            i = 0;
    }
    cm_Clocklow();
	CM_DATA_OUT;

    return i;

}

uchar cm_Read(void)
{
	uchar i;
	uchar rByte = 0;

	CM_DATA_IN;
	CM_DATA_HI;

    //WriteTW88(GPIO_OE0, ReadTW88(GPIO_OE0) & 0xFD); /*  立即将date脚配置成输入*/

	for(i=0x80; i; i = i >> 1)
	{
		cm_ClockCycle();

		if (CM_DATA_RD)
		{
            rByte |= i;
		}

		cm_Clocklow();
	}
	CM_DATA_OUT;
	return rByte;
}

void cm_WaitClock(uchar loop)
{
	uchar i, j;

	CM_DATA_LO;

	for(j=0; j<loop; j++)
	{
		cm_Start();

		for(i = 0; i < 15; i++)
        {
    		cm_ClockCycle();
        }

		cm_Stop();
	}
}

uchar cm_SendCommand(puchar pucInsBuff)
{
    uchar i, ucCmd;

    i = 100;

    ucCmd = pucInsBuff[0];

    while(i)
    {
        cm_Start();

        if (cm_Write(ucCmd) == 0) break;

        if (--i == 0) return FAIL_CMDSTART;
    }

    for(i = 1; i< 4; i++)
	{	
    
        if (cm_Write(pucInsBuff[i]) != 0)
        {
			return FAIL_CMDSEND;
        }
    }

    return SUCCESS;
}

uchar cm_SendData(puchar pucSendBuf, uchar ucLen)
{
	int i;

	for(i = 0; i < ucLen; i++)
	{
		if (cm_Write(pucSendBuf[i]) == 1)
		{
			return FAIL_WRDATA;
		}
	}

	cm_Stop();

	return SUCCESS;
}

uchar cm_WriteCommand(puchar pucInsBuff, puchar pucSendVal, uchar ucLen)
{
    uchar ucReturn;

    if ((ucReturn = cm_SendCommand(pucInsBuff)) != SUCCESS)
    {
		return ucReturn;
    }

    return cm_SendData(pucSendVal, ucLen);
}


uchar cm_ReceiveData(puchar pucRecBuf, uchar ucLen)
{
    int i;

    for(i = 0; i < (ucLen-1); i++)
    {
        pucRecBuf[i] = cm_Read();

        cm_AckNak(TRUE);
    }

    pucRecBuf[i] = cm_Read();

    cm_AckNak(FALSE);

    cm_Stop();

    return SUCCESS;
}

uchar cm_ReadCommand(puchar pucInsBuff, puchar pucRetVal, uchar ucLen)
{
    uchar ucReturn;

    if ((ucReturn = cm_SendCommand(pucInsBuff)) != SUCCESS)
    {
        return ucReturn;
    }

    return cm_ReceiveData(pucRetVal, ucLen);
}

uchar cm_ReadFuse(puchar pucFuze)
{
	uchar ucReturn;
    uchar ucCmdRdFuze[4] = {0xb6, 0x01, 0x00, 0x01};

	if((ucReturn = cm_ReadCommand(ucCmdRdFuze,pucFuze,1)) != SUCCESS) return ucReturn;

	return SUCCESS;
}

char cm_Init( void )
{
	BYTE ucReturn = 0;
    BYTE ucFuze   = 0xFF;

	gpio_init();

	cm_PowerOn();

    ucReturn = cm_ReadFuse(&ucFuze);

    //Printf("\r\n cm_Init0_ucReturn = %d", (WORD)ucReturn);

    if (ucReturn != SUCCESS) return 1;

    return 0;
}
#if 1
BYTE cm_OpenConfigZone( void ) /* 打开配置区*/
{
    BYTE temp[3];
	BYTE ucReturn = 0;

    ucCM_InsBuff[0] = 0xBA;
    ucCM_InsBuff[1] = 0x07;
    ucCM_InsBuff[2] = 0x00;
    ucCM_InsBuff[3] = 0x03;

    temp[0] = 0xDD;
    temp[1] = 0x42;
    temp[2] = 0x97;

    ucReturn = cm_WriteCommand(ucCM_InsBuff, temp, 3);

    cm_WaitClock(3);

    if (ucReturn != SUCCESS) return 2;

    return 0;
}

BYTE cm_UseZoneConfigure( BYTE en ) /* 配置用户区使用模式*/
{
    BYTE temp[2];
	BYTE ucReturn = 0;

    temp[0] = 0xFF;
    temp[1] = 0x61;

    if(en == 1)
    {
        temp[0] = 0xDF;
        temp[1] = 0x63;
    }

    ucCM_InsBuff[0] = 0xB4;
    ucCM_InsBuff[1] = 0x00;
    ucCM_InsBuff[2] = 0x22;
    ucCM_InsBuff[3] = 0x02;

    ucReturn = cm_WriteCommand(ucCM_InsBuff, temp, 0x02);

    if (ucReturn != SUCCESS) return 3;

    return 0;
}
#endif

BYTE cm_SetUseZone( BYTE dat ) /* 选择用户区*/
{
	BYTE ucReturn = 0;

    ucCM_InsBuff[0] = 0xB4;
    ucCM_InsBuff[1] = 0x03;
    ucCM_InsBuff[2] = dat;             /* 0~3用户区*/
    ucCM_InsBuff[3] = 0x00;

    ucReturn = cm_SendCommand(ucCM_InsBuff);

    if (ucReturn != SUCCESS) return 4;

    return 0;
}

#if 1
uchar cm_WriteSmallZone( void ) /*  写用户区*/
{
	uchar ucReturn;

	ucCM_InsBuff[0] = 0xB0;
    ucCM_InsBuff[1] = 0x00;
    ucCM_InsBuff[2] = 0x00;
    ucCM_InsBuff[3] = 0x08;

    ucReturn = cm_WriteCommand(ucCM_InsBuff, pucBuffer2, 0x08);

    if (ucReturn != SUCCESS) return 5;

    return 0;
}
#endif


uchar cm_ReadSmallZone( void ) /* 读用户区*/
{
    uchar i;
	uchar ucReturn;
	uchar receive[8];

    ucCM_InsBuff[0] = 0xB2;
    ucCM_InsBuff[1] = 0x00;
    ucCM_InsBuff[2] = 0x00;
    ucCM_InsBuff[3] = 0x08;

    ucReturn = cm_ReadCommand(ucCM_InsBuff, receive, 0x08);

    //Printf("\r\n RECEIVE1 = %d", (WORD)receive[0]);
    //Printf("\r\n RECEIVE2 = %d", (WORD)receive[1]);
    //Printf("\r\n RECEIVE3 = %d", (WORD)receive[2]);
    //Printf("\r\n RECEIVE4 = %d", (WORD)receive[3]);
    //Printf("\r\n RECEIVE5 = %d", (WORD)receive[4]);
    //Printf("\r\n RECEIVE6 = %d", (WORD)receive[5]);
    //Printf("\r\n RECEIVE7 = %d", (WORD)receive[6]);
    //Printf("\r\n RECEIVE8 = %d", (WORD)receive[7]);

    for(i = 0; i < 8; i++)
    {
        if(receive[i] != pucBuffer2[i])
        {
            return 5;
        }
    }

    if (ucReturn != SUCCESS) return 6;

    return 0;
}

#if 1
BYTE cm_SetDCR( void )
{
	uchar ucReturn;
	uchar uctemp[2];

    uctemp[0] = 0xBF;

    ucCM_InsBuff[0] = 0xB4;
    ucCM_InsBuff[1] = 0x00;
    ucCM_InsBuff[2] = 0x18;
    ucCM_InsBuff[3] = 0x01;

    ucReturn = cm_WriteCommand(ucCM_InsBuff, uctemp, 0x01);

    if (ucReturn != SUCCESS) return 7;

    return 0;
}


BYTE cm_WriteSecretSeedG0_3( void ) /* 选择密钥*/
{
	uchar ucReturn;

    ucCM_InsBuff[0] = 0xB4;
    ucCM_InsBuff[1] = 0x00;
    ucCM_InsBuff[2] = 0x98;
    ucCM_InsBuff[3] = 0x08;

    ucReturn = cm_WriteCommand(ucCM_InsBuff, g_pass_word1, 0x08);

    if (ucReturn != SUCCESS) return 8;

    return 0;
}


BYTE cm_CheckReadPassword( void ) /* 校验密码*/
{
    BYTE temp[3];
	uchar ucReturn;

    ucCM_InsBuff[0] = 0xBA;
    ucCM_InsBuff[1] = 0x11;
    ucCM_InsBuff[2] = 0x00;
    ucCM_InsBuff[3] = 0x03;

    temp[0] = 0x12;
    temp[1] = 0x30;
    temp[2] = 0xA9;

    ucReturn = cm_WriteCommand(ucCM_InsBuff, temp, 0x03);

    cm_WaitClock(10);

    if (ucReturn != SUCCESS) return 8;

    return 0;
}


char cm_BurnFuse( void ) /*熔断操作*/
{
    uchar ucReturn;
    uchar ucCmdWrFuze[4] = {0xb4, 0x01, 0x00, 0x00};

    ucCmdWrFuze[2] = (BYTE)CM_FAB;
    if((ucReturn = cm_SendCommand(ucCmdWrFuze))!= SUCCESS) return ucReturn;

    ucCmdWrFuze[2] = (BYTE)CM_CMA;
    if((ucReturn = cm_SendCommand(ucCmdWrFuze))!= SUCCESS) return ucReturn;

    ucCmdWrFuze[2] = (BYTE)CM_PER;
    if((ucReturn = cm_SendCommand(ucCmdWrFuze))!= SUCCESS) return ucReturn;

    return  SUCCESS;
}
#endif

#if 1
/* -------------------------------   认证部分-----------------------------------------------*/
uchar ucGpaRegisters[Gpa_Regs];

// Reset the cryptographic state
void cm_ResetCrypto(void)
{
    uchar i;

    for (i = 0; i < Gpa_Regs; ++i) ucGpaRegisters[i] = 0;
}

// Generate next value
uchar cm_GPAGen(uchar Datain)
{
	uchar Din_gpa;
	uchar Ri, Si, Ti;
	uchar R_sum, S_sum, T_sum;

	// Input Character
	Din_gpa = Datain^Gpa_byte;

	Ri = Din_gpa & 0x1f;   			                         //Ri[4:0] = Din_gpa[4:0]
	Si = ((Din_gpa << 3) & 0x78)|((Din_gpa >> 5) & 0x07);   //Si[6:0] = {Din_gpa[3:0], Din_gpa[7:5]}
	Ti = (Din_gpa >> 3) & 0x1f;  		                     //Ti[4:0] = Din_gpa[7:3];

	//R polynomial
	R_sum = cm_Mod(RD, cm_RotR(RG), CM_MOD_R);
	RG = RF;
	RF = RE;
	RE = RD;
	RD = RC^Ri;
	RC = RB;
	RB = RA;
	RA = R_sum;

	//S ploynomial
	S_sum = cm_Mod(SF, cm_RotS(SG), CM_MOD_S);
	SG = SF;
	SF = SE^Si;
	SE = SD;
	SD = SC;
	SC = SB;
	SB = SA;
	SA = S_sum;

	//T polynomial
	T_sum = cm_Mod(TE,TC,CM_MOD_T);
	TE = TD;
	TD = TC;
	TC = TB^Ti;
	TB = TA;
	TA = T_sum;

    // Output Stage
    Gpa_byte =(Gpa_byte<<4)&0xF0;                                      // shift gpa_byte left by 4

    Gpa_byte |= ((((RA^RE)&0x1F)&(~SA))|(((TA^TD)&0x1F)&SA))&0x0F; // concat 4 prev bits and 4 new bits

	return Gpa_byte;
}


// Clock some zeros into the state machine
void cm_GPAGenN(uchar Count)
{
    while(Count--) cm_GPAGen(0x00);
}


// Do authenticate/encrypt chalange encryption
void cm_AuthenEncryptCal(uchar *Ci, uchar *G_Sk, uchar *Q, uchar *Ch)
{
    uchar i, j;

    // Reset all registers
    cm_ResetCrypto();

    // Setup the cyptographic registers
    for(j = 0; j < 4; j++) {
	    for(i = 0; i<3; i++) cm_GPAGen(Ci[2*j]);
	    for(i = 0; i<3; i++) cm_GPAGen(Ci[2*j+1]);
	    cm_GPAGen(Q[j]);
    }

    for(j = 0; j<4; j++ ) {
	    for(i = 0; i<3; i++) cm_GPAGen(G_Sk[2*j]);
	    for(i = 0; i<3; i++) cm_GPAGen(G_Sk[2*j+1]);
	    cm_GPAGen(Q[j+4]);
    }

    // begin to generate Ch
    cm_GPAGenN(6);                    // 6 0x00s
    Ch[0] = Gpa_byte;

    for (j = 1; j<8; j++) {
	    cm_GPAGenN(7);                // 7 0x00s
	    Ch[j] = Gpa_byte;
    }

    // then calculate new Ci and Sk, to compare with the new Ci and Sk read from eeprom
    Ci[0] = 0xff;		              // reset AAC
    for(j = 1; j<8; j++) {
	    cm_GPAGenN(2);                // 2 0x00s
	    Ci[j] = Gpa_byte;
    }

    for(j = 0; j<8; j++) {
	     cm_GPAGenN(2);                // 2 0x00s
	     G_Sk[j] = Gpa_byte;
    }

	cm_GPAGenN(3);                    // 3 0x00s
}

// Low quality random number generator
void cm_RandGen(puchar pucRanddat)
{
	uchar i;

	for(i = 0; i < 8; i++)
    {
        pucRanddat[i] = (uchar)(i%6);
    }
}

//ucReturn = cm_ActiveSecurity(1, passwd, NULL, FALSE);

uchar ucCM_Q_Ch[16], ucCM_Ci2[8];

// Common code for both activating authentication and encryption
static BYTE cm_AuthenEncrypt(uchar ucCmd1, uchar ucAddrCi, puchar pucRandom)
{
    unsigned char i;
	BYTE ucReturn;
	BYTE pucG_Sk[8];
    BYTE pucCi[8];

    // Generate chalange data
    if (pucRandom) for (i = 0; i < 8; ++i)
    {
        ucCM_Q_Ch[i] = pucRandom[i];
    }
    else
    {
        cm_RandGen(ucCM_Q_Ch);
    }

    for (i = 0; i < 8; ++i)
    {
        pucG_Sk[i] = g_pass_word1[i];
    }

    ucCM_InsBuff[0] = 0xB6;
    ucCM_InsBuff[1] = 0x00;
    ucCM_InsBuff[2] = ucAddrCi;
    ucCM_InsBuff[3] = 0x08;
    ucReturn = cm_ReadCommand(ucCM_InsBuff, pucCi, 0x08);

    cm_AuthenEncryptCal(pucCi, pucG_Sk, ucCM_Q_Ch, &ucCM_Q_Ch[8]);

    // Send chalange
    ucCM_InsBuff[0] = 0xB8;
    ucCM_InsBuff[1] = ucCmd1;
    ucCM_InsBuff[2] = 0x00;
    ucCM_InsBuff[3] = 0x10;
    if ((ucReturn = cm_WriteCommand(ucCM_InsBuff, ucCM_Q_Ch, 16)) != SUCCESS) return ucReturn;

    // Give chips some clocks to do calculations    3-10
    cm_WaitClock(10);

    // Verify result
    ucCM_InsBuff[0] = 0xB6;
    ucCM_InsBuff[1] = 0x00;
    ucCM_InsBuff[2] = ucAddrCi;
    ucCM_InsBuff[3] = 0x08;

    if ((ucReturn = cm_ReadCommand(ucCM_InsBuff, ucCM_Ci2, 8)) != SUCCESS) return ucReturn;

    for(i=0; i<8; i++)
    {
        if (pucCi[i]!=ucCM_Ci2[i]) return FAILED;
    }

    // Done
    return SUCCESS;
}
/* -------------------------------   认证部分-----------------------------------------------*/
#endif

BYTE at88sc_checkpswd( void )
{
    BYTE ucPrint = 0xFF;

    gpio_init();
	//WriteTW88(GPIO_EN0, ReadTW88(GPIO_EN0) | 0x02);  //---enable gpio function

    ucPrint = cm_Init();
    //Printf("\r\n cm_Init0 = %d", (WORD)ucPrint);
    if (ucPrint != SUCCESS) return 1;

    ucPrint = cm_SetUseZone(0x01);
   //Printf("\r\n cm_Init3 = %d", (WORD)ucPrint);
    if (ucPrint != SUCCESS) return 2;

    ucPrint = cm_AuthenEncrypt(0x01, 0x60, NULL);
    //Printf("\r\n cm_AuthenEncrypt_at88sc_checkpswd = %d", (WORD)ucPrint);
    if (ucPrint != SUCCESS) return 3;

    ucPrint = cm_ReadSmallZone();
    //Printf("\r\n cm_ReadSmallZone = %d", (WORD)ucPrint);

    if (ucPrint != SUCCESS) return 4;

    return 0;
}

#define Printf printk
void Test( void )
{
    BYTE ucPrint = 0xFF;
    //BYTE temp[3];

	gpio_init();
    //WriteTW88(GPIO_EN0, ReadTW88(GPIO_EN0) | 0x02);  //---enable gpio function

    ucPrint = cm_Init();
    Printf("\r\n cm_Init0 = %d", (WORD)ucPrint);

    ucPrint = cm_OpenConfigZone();
    Printf("\r\n cm_Init1 = %d", (WORD)ucPrint);

    ucPrint = cm_UseZoneConfigure(OFF);
    Printf("\r\n cm_Init2 = %d", (WORD)ucPrint);

    ucPrint = cm_WriteSecretSeedG0_3();
    Printf("\r\n cm_WriteSecretSeedG1_3 = %d", (WORD)ucPrint);

    ucPrint = cm_SetUseZone(0x01);
    Printf("\r\n cm_Init3 = %d", (WORD)ucPrint);

    ucPrint = cm_WriteSmallZone();
    Printf("\r\n cm_Init4 = %d", (WORD)ucPrint);

    ucPrint = cm_ReadSmallZone();
    Printf("\r\n cm_Init5 = %d", (WORD)ucPrint);

    ucPrint = cm_SetDCR();
    Printf("\r\n cm_SetDCR_cm_Init5 = %d", (WORD)ucPrint);

    //设置需要验证
    ucPrint = cm_UseZoneConfigure(ON);
    Printf("\r\n cm_UseZoneConfigure = %d", (WORD)ucPrint);

    //熔断操作
    cm_BurnFuse();
}

static int at88_open(struct inode *inode,struct file *filp)
{
	uchar ret, psw[8];

	//at88sc_test_2();
	//at88sc_test();
	//read_test();

	printk("get in at88_open success\n");

	return 0;

	printk("get in at88_open success\n");
	//PINMUX0 = PINMUX0 &0xFFFFF3FF; //set gpio[8] and gpio[9] is gpio mode
	cm_PowerOn();
	#if 1
	psw[0] = 0xDD;
	psw[1] = 0x42;
	psw[2] = 0x97;
	#else
	psw[0] = 0x11;
	psw[1] = 0x23;
	psw[2] = 0x56;
	#endif
	ret = cm_VerifySecureCode(psw);
	if(ret != SUCCESS)
		printk("cm_VerifySecureCode failed!\n");
	else
		printk("cm_VerifySecureCode success!\n");

	printk("<dbk>psw-1[0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x]\n",psw[0],psw[1],psw[2],psw[3],psw[4],psw[5],psw[6],psw[7]);

	ret = cm_ReadConfigZone(0xB0,psw,8);
	printk("<dbk>psw-2[0x%02x,0x%02x,0x%02x,0x%02x][%c,%c,%c,%c]\n",psw[0],psw[1],psw[2],psw[3],psw[0],psw[1],psw[2],psw[3]);
	if((psw[1] == 'V')&&(psw[2] == 'A')&&(psw[3] == 'N')){
		printk("the fuse has burn 12.................\n");
		//at88_myinit();
	}else
	   //	at88_myinit();
	printk("leave the at88_open.\n");

    return 0;
}
static int at88_release(struct inode *inode,struct file *filp)
{
	printk("get in at88_release success\n");
	//power off the chip
	//cm_PowerOff();
	printk("leave the at88_release\n");
	return 0;
}

static ssize_t at88_read (struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	char ret = 0;
#if 1
	printk("at88sc read here\n");	
	ret = at88sc_checkpswd();
	printk("at88sc read value %d:\n",ret);	
	
	return ret;	
#else
	uchar ucReturn;
	uchar ucData[16],ucG[8];
	int i;
	printk("get in at88_read success\n");
	ucReturn = cm_SetUserZone(0, FALSE);
	if(ucReturn != SUCCESS)
		printk("cm_SetUserZone failed!\n");
	else
		printk("cm_SetUserZone success!\n");
	ucG[0] = 'V';
	ucG[1] = 'A';
	ucG[2] = 'N';

	ucReturn = cm_VerifySecureCode(ucG);
	if(ucReturn != SUCCESS)
		printk("cm_VerifySecureCode  userzone 0 failed!\n");
	else
		printk("cm_VerifySecureCode  userzone 0 success!\n");
	for (i = 0; i < 16; ++i) ucData[i] = 0x00;
	ucReturn = cm_ReadSmallZone(0, ucData, 6);
	if(ucReturn != SUCCESS)
		printk("cm_ReadSmallZone failed!\n");
	else
		printk("cm_ReadSmallZone success!\n");
	for(i = 0; i < count; i++)
		buf[i] = ucData[i];
	return 0;
#endif
}

static ssize_t at88_write (struct file *filp, const char *buf, size_t count, loff_t *f_pos)
{


#if  1
	Test();
	printk(" at88sc write open here\n");
	return 0;
#else
	uchar ucReturn;
	uchar ucData[16],ucG[8];
	int i;
	printk("get in at88_write success!\n");
	ucReturn = cm_SetUserZone(0, FALSE);
	if(ucReturn != SUCCESS)
		printk("cm_SetUserZone failed!\n");
	else
		printk("cm_SetUserZone success!\n");
	ucG[0] = 'V';
	ucG[1] = 'A';
	ucG[2] = 'N';

	ucReturn = cm_VerifySecureCode(ucG);
	if(ucReturn != SUCCESS)
		printk("cm_VerifySecureCode  userzone 0 failed!\n");
	else
		printk("cm_VerifySecureCode  userzone 0 success!\n");
	ucReturn = cm_WriteSmallZone(0, (uchar *)buf, 16);
	if(ucReturn != SUCCESS)
		printk("cm_WriteSmallZone failed!\n");
	else
		printk("cm_WriteSmallZone success!\n");
	printk("leave at88_write success!\n");
	return 0;
#endif
}

static long at88_ioctl(struct file *file,
                unsigned int cmd,  unsigned long arg)
{
	return 0;
}
/*
static struct file_operations at88_fops = {
	.open		= at88_open,
	.release	= at88_release,
	.read		= at88_read,
	.write		= at88_write,
	.ioctl		= at88_ioctl
};*/

static const struct file_operations at88_fops = {
    .owner          = THIS_MODULE,
    //.ioctl          = at88_ioctl,
    .unlocked_ioctl = at88_ioctl,
    .open           = at88_open,
    .release        = at88_release,
    .read		= at88_read,
	.write		= at88_write,
};


static  struct miscdevice at88_dev = {
   .minor		= MISC_DYNAMIC_MINOR,
   .name		= "at88sc",
   .fops        = &at88_fops,
};

#if 1
static int __init davinci_at88_init(void)
{
	int ret;

	/* Register the character device (atleast try) */
	printk("at88sc Crypto module init. \n");
/*
	res = register_chrdev(CM_MAJOR, "cm0", &at88_fops);

	if(res < 0) {
		printk("device register failed with %d.\n",res);
		return res;
	}
*/
	ret = misc_register(&at88_dev);
    if(0 != ret)
    {
    	return ret;
    }
#if 1
	gpio_init();
#else
	*((volatile unsigned int *)IO_ADDRESS(0x200f0000 + 0xC4)) = 0;//gpio1_3
	*((volatile unsigned int *)IO_ADDRESS(0x200f0000 + 0xC8)) = 0;//gpio1_4
#endif
	//gpio_set_direction(4,GIO_DIR_INPUT);
	//CM_DATA_OUT;
	//CM_DATA_HI;

	//CM_CLK_OUT;
	//CM_CLK_LO;

	//at88_open(NULL,NULL);
	return 0;
}

static void __exit davinci_at88_exit(void)
{
	//unregister_chrdev(CM_MAJOR,"cm0");
	misc_deregister(&at88_dev);
	printk("at88sc release success.\n");
}

module_init(davinci_at88_init);
module_exit(davinci_at88_exit);

MODULE_AUTHOR("hecong cong.he@qualvision.cn");
MODULE_DESCRIPTION("at88sc driver expander");
MODULE_LICENSE("GPL");



#endif

