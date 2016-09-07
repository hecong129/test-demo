
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>

#include <linux/init.h>
#include <linux/delay.h>
#include <linux/proc_fs.h>
#include <linux/workqueue.h>

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>

/*
#include <linux/types.h>
#include <linux/init.h>
#include <linux/mm.h>
#include <linux/module.h>
//#include <linux/spi/ads7846.h>
#include <linux/utsname.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/times.h>
#include <linux/interrupt.h>
//#include <linux/module.h>
#include <linux/errno.h>
#include <linux/miscdevice.h>
#include <linux/fcntl.h>
*/

//#include <asm/hardware.h>
//#include <asm/setup.h>
//#include <asm/mach-types.h>
//#include <asm/irq.h>
 /*
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>

#include <asm/arch/gio.h>
*/

#include <asm/uaccess.h>
#include <asm/system.h>
#include <asm/io.h>

#include <mach/gpio.h>
#include <mach/ak37-gpio.h>


#define gpio_set_pin_as_gpio(pin)		ak_setpin_as_gpio(pin)

// Basic Datatypes
typedef unsigned char  uchar;
typedef unsigned char *puchar;
typedef signed char    schar;
typedef signed char    BYTE;
typedef signed char   *pschar;
//typedef unsigned int   uint;
typedef unsigned int  *puint;
typedef signed int     sint;
typedef signed int     WORD;
typedef signed int    *psint;


// -------------------------------------------------------------------------------------------------

// High Level Function Prototypes

// Select Chip
uchar cm_SelectChip(uchar ucChipId);

// Activate Security
uchar cm_ActiveSecurity(uchar ucKeySet, puchar pucKey, puchar pucRandom, uchar ucEncrypt);

// Deactivate Security
uchar cm_DeactiveSecurity(void);

// Verify Password
uchar cm_VerifyPassword(puchar pucPassword, uchar ucSet, uchar ucRW);

// Reset Password
uchar cm_ResetPassword(void);

// Verify Secure Code
#define cm_VerifySecureCode(CM_PW) cm_VerifyPassword(CM_PW, 7, CM_PWWRITE)

// Read Configuration Zone
uchar cm_ReadConfigZone(uchar ucCryptoAddr, puchar pucBuffer, uchar ucCount);

// Write Configuration Zone
uchar cm_WriteConfigZone(uchar ucCryptoAddr, puchar pucBuffer, uchar ucCount, uchar ucAntiTearing);

// Set User Zone
uchar cm_SetUserZone(uchar ucZoneNumber, uchar ucAntiTearing);

// Read User Zone
uchar cm_ReadLargeZone(uint uiCryptoAddr, puchar pucBuffer, uchar ucCount);

// Read Small User Zone
uchar cm_ReadSmallZone(void);

// Write User Zone
char cm_WriteLargeZone(uint uiCryptoAddr, puchar pucBuffer, uchar ucCount);

// Write Small User Zone
#if 1
uchar cm_WriteSmallZone( void );
#else
uchar cm_WriteSmallZone(uchar ucCryptoAddr, puchar pucBuffer, uchar ucCount);
#endif
// Send Checksum
uchar cm_SendChecksum(puchar pucChkSum);

// Read Checksum
uchar cm_ReadChecksum(puchar pucChkSum);

// Read Fuse Byte
uchar cm_ReadFuse(puchar pucFuze);

// Burn Fuse
char cm_BurnFuse( void );

// -------------------------------------------------------------------------------------------------
// Configuration Structures
// -------------------------------------------------------------------------------------------------

// CryptoMemory Low Level Linkage
//
typedef struct{
    uchar (*Carddetect)(void);
    void (*PowerOff)(void);
    void (*PowerOn)(void);
    uchar (*SendCommand)(puchar pucCommandBuffer);
    uchar (*ReceiveRet)(puchar pucReceiveData, uchar ucLength);
    uchar (*SendData)(puchar pucSendData, uchar ucLength);
    void (*RandomGen)(puchar pucRandomData);
    void (*WaitClock)(uchar ucLoop);
    uchar (*SendCmdByte)(uchar ucCommand);
} cm_low_level;

// CryptoMemory Low Level Configuration
//
// If any of the supplied CryptoMemory low level library functions are used, this structure must be
// present in the user code. For a detailed description of the elements in the structure, please
// see the "CryptoMemory Library User Manual".
//
typedef struct{
    uchar ucChipSelect;
    uchar ucClockPort;
    uchar ucClockPin;
    uchar ucDataPort;
    uchar ucDataPin;
    uchar ucCardSensePort;
    uchar ucCardSensePin;
    uchar ucCardSensePolarity;
    uchar ucPowerPort;
    uchar ucPowerPin;
    uchar ucPowerPolarity;
    uchar ucDelayCount;
    uchar ucStartTries;
} cm_port_cfg;

// -------------------------------------------------------------------------------------------------
// Externals for Configuration Structures
// -------------------------------------------------------------------------------------------------

extern cm_low_level CM_LOW_LEVEL;
extern cm_port_cfg  CM_PORT_CFG;

// -------------------------------------------------------------------------------------------------
// Other Externals
// -------------------------------------------------------------------------------------------------

extern uchar ucCM_Encrypt;
extern uchar ucCM_Authenticate;
extern uchar ucCM_UserZone;
extern uchar ucCM_AntiTearing;
extern uchar ucCM_InsBuff[4];

// end of multiple inclusion protection

// Macros for all of the registers
#define RA       (ucGpaRegisters[0])
#define RB       (ucGpaRegisters[1])
#define RC       (ucGpaRegisters[2])
#define RD       (ucGpaRegisters[3])
#define RE       (ucGpaRegisters[4])
#define RF       (ucGpaRegisters[5])
#define RG       (ucGpaRegisters[6])
#define TA       (ucGpaRegisters[7])
#define TB       (ucGpaRegisters[8])
#define TC       (ucGpaRegisters[9])
#define TD       (ucGpaRegisters[10])
#define TE       (ucGpaRegisters[11])
#define SA       (ucGpaRegisters[12])
#define SB       (ucGpaRegisters[13])
#define SC       (ucGpaRegisters[14])
#define SD       (ucGpaRegisters[15])
#define SE       (ucGpaRegisters[16])
#define SF       (ucGpaRegisters[17])
#define SG       (ucGpaRegisters[18])
#define Gpa_byte (ucGpaRegisters[19])
#define Gpa_Regs (20)


// Defines for constants used
#define CM_MOD_R (0x1F)
#define CM_MOD_T (0x1F)
#define CM_MOD_S (0x7F)

// Macros for common operations
#define cm_Mod(x,y,m) ((x+y)>m?(x+y-m):(x+y))
#define cm_RotT(x)    (((x<<1)&0x1e)|((x>>4)&0x01))
#define cm_RotR(x)    (((x<<1)&0x1e)|((x>>4)&0x01))
#define cm_RotS(x)    (((x<<1)&0x7e)|((x>>6)&0x01))

// Externals
extern uchar ucGpaRegisters[Gpa_Regs];

// Function Prototypes
void cm_ResetCrypto(void);
uchar cm_GPAGen(uchar Datain);
void cm_CalChecksum(uchar *Ck_sum);
void cm_AuthenEncryptCal(uchar *Ci, uchar *G_Sk, uchar *Q, uchar *Ch);
void cm_GPAGenN(uchar Count);
void cm_GPAGenNF(uchar Count, uchar DataIn);
void cm_GPAcmd2(puchar pucInsBuff);
void cm_GPAcmd3(puchar pucInsBuff);
void cm_GPAdecrypt(uchar ucEncrypt, puchar pucBuffer, uchar ucCount);
void cm_GPAencrypt(uchar ucEncrypt, puchar pucBuffer, uchar ucCount);
// -------------------------------------------------------------------------------------------------
// I/O Port Grouping
//
// Note: the "PORTx" in the header is the last address in the group of three port address
//
// -------------------------------------------------------------------------------------------------
#define IO_PORT_IN  (-2)
#define IO_PORT_DIR (-1)
#define IO_PORT_OUT (0)

// -------------------------------------------------------------------------------------------------
// Macros
// -------------------------------------------------------------------------------------------------
#define GIO_DIR_OUTPUT	1
#define GIO_DIR_INPUT	2
#define GIO_STATE_HIGH	3
#define GIO_STATE_LOW	4

#define GPIO_DIR_INPUT		1
#define GPIO_DIR_OUTPUT	    0
#define GPIO_LEVEL_LOW		0
#define GPIO_LEVEL_HIGH		1

static void gpio_set_direction(int pin, int dir)
{
	if(dir == GIO_DIR_INPUT)
		ak_gpio_cfgpin(pin,GPIO_DIR_INPUT);
	else
		ak_gpio_cfgpin(pin,GPIO_DIR_OUTPUT);
}

static void __gpio_set(int pin, int level)
{
	if(level == GIO_STATE_HIGH)
		ak_gpio_setpin(pin,GPIO_LEVEL_HIGH);
	else
		ak_gpio_setpin(pin,GPIO_LEVEL_LOW);
}

static int __gpio_get(int pin)
{
	 return ak_gpio_getpin(pin);
}

// Define control of the secure memory in terms of the pins defined in CM_PORT_CFG
//
//#define CM_CLK_PD     (*(volatile unsigned long *)(CM_PORT_CFG.ucClockPort))
//#define CM_CLK_PO     (*(volatile unsigned long *)(CM_PORT_CFG.ucClockPort)
//#define CM_CLK_PIN    (CM_PORT_CFG.ucClockPin)
#define CM_CLK_OUT  gpio_set_direction(AK_GPIO_23,GIO_DIR_OUTPUT)
#define CM_CLK_HI  __gpio_set(AK_GPIO_23,GIO_STATE_HIGH)
#define CM_CLK_LO   __gpio_set(AK_GPIO_23,GIO_STATE_LOW)
//#define CM_CLK_OUT    CM_CLK_PD|=(1<<CM_CLK_PIN)
//#define CM_CLK_LO     CM_CLK_PO&=~(1<<CM_CLK_PIN)
//#define CM_CLK_HI     CM_CLK_PO|=(1<<CM_CLK_PIN)

//#define CM_DATA_PI    (*(volatile unsigned long *)(CM_PORT_CFG.ucDataPort))
//#define CM_DATA_PD    (*(volatile unsigned long *)(CM_PORT_CFG.ucDataPort))
//#define CM_DATA_PO    (*(volatile unsigned long *)(CM_PORT_CFG.ucDataPort))
//#define CM_DATA_PIN   (CM_PORT_CFG.ucDataPin)
#define CM_DATA_OUT gpio_set_direction(AK_GPIO_79,GIO_DIR_OUTPUT)
#define CM_DATA_IN    gpio_set_direction(AK_GPIO_79,GIO_DIR_INPUT)
#define CM_DATA_HI __gpio_set(AK_GPIO_79,GIO_STATE_HIGH)
#define CM_DATA_LO  __gpio_set(AK_GPIO_79,GIO_STATE_LOW)
#define CM_DATA_RD   __gpio_get(AK_GPIO_79)
//#define CM_DATA_OUT   CM_DATA_PD|=(1<<CM_DATA_PIN)
//#define CM_DATA_IN    CM_DATA_PD&=~(1<<CM_DATA_PIN)
//#define CM_DATA_HI    CM_DATA_PO|=(1<<CM_DATA_PIN)
//#define CM_DATA_LO    CM_DATA_PO&=~(1<<CM_DATA_PIN)
//#define CM_DATA_RD    (CM_DATA_PI&(1<<CM_DATA_PIN))
//#define CM_DATA_BIT   ((CM_DATA_PI>>CM_DATA_PIN)&1)

#if 0
#define CM_DETECT_PI  (*(volatile unsigned char *)(CM_PORT_CFG.ucCardSensePort+IO_PORT_IN))
#define CM_DETECT_PD  (*(volatile unsigned char *)(CM_PORT_CFG.ucCardSensePort+IO_PORT_DIR))
#define CM_DETECT_PIN (CM_PORT_CFG.ucCardSensePin)
#define CM_DETECT_POL (CM_PORT_CFG.ucCardSensePolarity)
#define CM_DETECT_IN  CM_DETECT_PD&=~(1<<CM_DETECT_PIN)
#define CM_DETECT_RD  CM_DETECT_PI&(1<<CM_DETECT_PIN)

#define CM_POWER_PD  (*(volatile unsigned char *)(CM_PORT_CFG.ucPowerPort+IO_PORT_DIR))
#define CM_POWER_PO  (*(volatile unsigned char *)(CM_PORT_CFG.ucPowerPort+IO_PORT_OUT))
#define CM_POWER_PIN (CM_PORT_CFG.ucPowerPin)
#define CM_POWER_POL (CM_PORT_CFG.ucPowerPolarity)
#define CM_POWER_OUT CM_POWER_PD|=(1<<CM_POWER_PIN)
#define CM_POWER_HI  CM_POWER_PO|=(1<<CM_POWER_PIN)
#define CM_POWER_LO  CM_POWER_PO&=~(1<<CM_POWER_PIN)
#else//findt
#define CM_DETECT_PI  1
#define CM_DETECT_PD  1
#define CM_DETECT_PIN 1
#define CM_DETECT_POL 1
#define CM_DETECT_IN  1
#define CM_DETECT_RD  1

#define CM_POWER_PD 1
#define CM_POWER_PO  1
#define CM_POWER_PIN 1
#define CM_POWER_POL 1
#define CM_POWER_OUT 1
#define CM_POWER_HI  1
#define CM_POWER_LO  1

#endif
//#define CM_DIR_INIT   CM_CLK_PD|=(1<<CM_CLK_PIN);CM_DATA_PD|=(1<<CM_DATA_PIN)
#define CM_DIR_INIT  gpio_set_pin_as_gpio(AK_GPIO_23);gpio_set_pin_as_gpio(AK_GPIO_79);gpio_set_direction(AK_GPIO_23,GIO_DIR_OUTPUT);gpio_set_direction(AK_GPIO_79,GIO_DIR_OUTPUT)
#define CM_TIMER      (CM_PORT_CFG.ucDelayCount)
#define CM_START_TRIES (CM_PORT_CFG.ucStartTries)

// -------------------------------------------------------------------------------------------------
// Macros that replace small common function
// -------------------------------------------------------------------------------------------------

#define CM_CLOCKHIGH  cm_Delay(1);(CM_CLK_HI);cm_Delay(1)
#define CM_CLOCKLOW   cm_Delay(1);(CM_CLK_LO);cm_Delay(1)
#define CM_CLOCKCYCLE cm_Delay(1);(CM_CLK_LO);cm_Delay(2);(CM_CLK_HI);cm_Delay(1)

// -------------------------------------------------------------------------------------------------
// Low Level Function Prototypes
// -------------------------------------------------------------------------------------------------

// Placeholder function that always returns TRUE
uchar cm_TRUE(void);

// Placeholder function that always returns SUCCESS
uchar cm_SUCCESS(void);

// Card Detect
uchar cm_CardDetect(void);

// Power On Functions
void cm_FullPowerOn(void);
void cm_PowerOn(void);

// Power Off Functions
void cm_FullPowerOff(void);
void cm_PowerOff(void);

// Send Command
uchar cm_SendCommand(puchar pucCommandBuffer);

// Receive Data
uchar cm_ReceiveData(puchar pucReceiveData, uchar ucLength);

// Send Data
uchar cm_SendData(puchar pucSendData, uchar ucLength);

// Random
void cm_RandGen(puchar pucRandomData);

// Wait Clock
void cm_WaitClock(uchar ucLoop);

// Send Command Byte
uchar cm_SendCmdByte(uchar ucCommand);

// end of multiple inclusion protection

// -------------------------------------------------------------------------------------------------
// Other includes required by this header file
// -------------------------------------------------------------------------------------------------

// Constants used in low level functions
// Power on clocks (spec call for 5, but California Card uses 15)
#define CM_PWRON_CLKS (15)

// Mid-Level Functions
uchar cm_ReadCommand(puchar pucInsBuff, puchar pucRetVal, uchar ucLen);
uchar cm_WriteCommand(puchar pucInsBuff, puchar pucSendVal, uchar ucLen);

// Functions in CM_I2C.C used internally by other low level functions
void cm_Clockhigh(void);
void cm_Clocklow(void);
void cm_ClockCycle(void);
void cm_ClockCycles(uchar ucCount);
void cm_Start(void);
void cm_Stop(void);
uchar cm_Write(uchar ucData);
void cm_Ack(void);
void cm_N_Ack(void);
uchar cm_Read(void);
void cm_WaitClock(uchar loop);
uchar cm_SendCommand(puchar pucInsBuff);
uchar cm_ReceiveRet(puchar pucRecBuf, uchar ucLen);
uchar cm_SendDat(puchar pucSendBuf, uchar ucLen);
void cm_RandomGen(puchar pucRanddat);

// functions in CM_LOW.C used internally by other low level functions
void cm_Delay(uchar ucDelay);

// end of multiple inclusion protection

// Definations
// -------------------------------------------------------------------------------------------------

// Basic Definations (if not available elsewhere)
#ifndef FALSE
#define FALSE (0)
#define TRUE  (!FALSE)
#endif
#ifndef NULL
#define NULL ((void *)0)
#endif

// Device Configuration Register
#define DCR_ADDR      (0x18)
#define DCR_SME       (0x80)
#define DCR_UCR       (0x40)
#define DCR_UAT       (0x20)
#define DCR_ETA       (0x10)
#define DCR_CS        (0x0F)

// Cryptograms
#define CM_Ci         (0x50)
#define CM_Sk         (0x58)
#define CM_G          (0x90)

// Fuses
#define CM_FAB        (0x06)
#define CM_CMA        (0x04)
#define CM_PER        (0x00)

// Password
#define CM_PSW        (0xB0)
#define CM_PWREAD     (1)
#define CM_PWWRITE    (0)

// Return Code Defination
#define SUCCESS       (0)
#define FAILED        (1)
#define FAIL_CMDSTART (2)
#define FAIL_CMDSEND  (3)
#define FAIL_WRDATA   (4)
#define FAIL_RDDATA   (5)
// note: additional specific error codes may be added in the future

#define OFF 0
#define ON  1



