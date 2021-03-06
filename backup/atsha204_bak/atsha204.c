
/* Copyright (C) 
 * 2016 - 何聪:
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 * 
 */


/**
 * @file atsha204.c
 * 函数: 
 * @author 何聪:
 * @version V1.0
 * @date 2016-08-31
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <linux/slab.h>

#include <linux/io.h>
#include "sha256.h"


#define ATSHA204_DEBUG_EN      1
#if(ATSHA204_DEBUG_EN == 1)

	#define atsh204_dbg  printk
//	#define atsh204_dbg(x,arg...) printk( KERN_ERR x,##arg)
	#define atsh204_err(x,arg...) printk(KERN_ERR"[ATSHA204]"x,##arg)
	#define atsh204_print(x,arg...) printk(KERN_ERR "[ATSHA204]"x,##arg)
#else
	#define atsh204_dbg(x,arg...)
	#define atsh204_err(x,arg...) 
	#define atsh204_print(x,arg...)  
#endif

#define ATSHA204_I2C_ADDR 0xc8
#define ATSHA204_NAME "atsha204"
#define SHA204_SUCCESS 1
#define SHA204_BAD_CRC -1
#define WRITE_CFG_SUCCESS 8
#define WRITE_OTP_SUCCESS 7
#define WRITE_DATA_SUCCESS 6

/*word address use to different function*/
#define RESET_ADDRESS	0x00
#define SLEEP_ADDRESS	0x01
#define	IDLE_ADDRESS	0x02
#define COMMAND_ADDRESS 0x03


#define ATSHA204_READ_COMMAND		0x02
#define ATSHA204_WRITE_COMMAND		0x12
#define ATSHA204_LOCK_COMMAND		0x17
#define ATSHA204_NONCE_COMMAND		0x16
#define ATSHA204_MAC_COMMAND		0x08
#define ATSHA204_CHECKMAC_COMMAND	0x28

#define SHFR(x, n)    (x >> n)
#define ROTR(x, n)   ((x >> n) | (x << ((sizeof(x) << 3) - n)))
#define ROTL(x, n)   ((x << n) | (x >> ((sizeof(x) << 3) - n)))
#define CH(x, y, z)  ((x & y) ^ (~x & z))
#define MAJ(x, y, z) ((x & y) ^ (x & z) ^ (y & z))

#define SHA256_F1(x) (ROTR(x,  2) ^ ROTR(x, 13) ^ ROTR(x, 22))
#define SHA256_F2(x) (ROTR(x,  6) ^ ROTR(x, 11) ^ ROTR(x, 25))
#define SHA256_F3(x) (ROTR(x,  7) ^ ROTR(x, 18) ^ SHFR(x,  3))
#define SHA256_F4(x) (ROTR(x, 17) ^ ROTR(x, 19) ^ SHFR(x, 10))

#define UNPACK32(x, str)                      \
{                                             \
    *((str) + 3) = (uint8) ((x)      );       \
    *((str) + 2) = (uint8) ((x) >>  8);       \
    *((str) + 1) = (uint8) ((x) >> 16);       \
    *((str) + 0) = (uint8) ((x) >> 24);       \
}

#define PACK32(str, x)                        \
{                                             \
    *(x) =   ((uint32) *((str) + 3)      )    \
           | ((uint32) *((str) + 2) <<  8)    \
           | ((uint32) *((str) + 1) << 16)    \
           | ((uint32) *((str) + 0) << 24);   \
}


/* Macros used for loops unrolling */

#define SHA256_SCR(i)                         \
{                                             \
    w[i] =  SHA256_F4(w[i -  2]) + w[i -  7]  \
          + SHA256_F3(w[i - 15]) + w[i - 16]; \
}

#define SHA256_EXP(a, b, c, d, e, f, g, h, j)               \
{                                                           \
    t1 = wv[h] + SHA256_F2(wv[e]) + CH(wv[e], wv[f], wv[g]) \
         + sha256_k[j] + w[j];                              \
    t2 = SHA256_F1(wv[a]) + MAJ(wv[a], wv[b], wv[c]);       \
    wv[d] += t1;                                            \
    wv[h] = t1 + t2;                                        \
}


typedef enum 
{
	CONFIG_ZONE_BIT=0,//config zone
	OTP_ZONE_BIT,	// otp zone
	DATA_ZONE_BIT,	//data zone
}DIFF_ZONE;

static __u32 twi_id = 1;
static __u32 iic_addr = 0x64; 
static int READ32_MODE = 1;
static int READ4_MODE = 0;

struct atsha204_st {
	struct i2c_client *client;
	u8 device_id;
};

static const struct i2c_device_id atsha204_id[] = {
	{ATSHA204_NAME, 0},
	{}
};
void sleep(int count);

/************************************************************************************/
uint32 sha256_h0[8] =
            {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
             0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};

//flash uint32 sha256_k[64] =
uint32 sha256_k[64] =
            {0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5,
             0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
             0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3,
             0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
             0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc,
             0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
             0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7,
             0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
             0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13,
             0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
             0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
             0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
             0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5,
             0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
             0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208,
             0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

/* SHA-256 functions */

// some helper macros for TPM implementation
//#define zram32  ((UINT32*) zram)
//#define eob32(buf) ((sizeof(buf)/sizeof(UINT32)))

void sha256_transf(sha256_ctx *ctx, const uint8 *message,
                   uint32 block_nb)
{
    uint32 w[64];
    uint32 wv[8];
//	uint32 *wv = &zram32[eob32(zram)-8];
//	uint32 *w = &zram32[eob32(zram)-(8+64)];

    uint32 t1, t2;
    const uint8 *sub_block;
    int i;

#ifndef UNROLL_LOOPS
    int j;
#endif

    for (i = 0; i < (int) block_nb; i++) {
        sub_block = message + (i << 6);

#ifndef UNROLL_LOOPS
        for (j = 0; j < 16; j++) {
            PACK32(&sub_block[j << 2], &w[j]);
        }

        for (j = 16; j < 64; j++) {
            SHA256_SCR(j);
        }

        for (j = 0; j < 8; j++) {
            wv[j] = ctx->h[j];
        }

        for (j = 0; j < 64; j++) {
            t1 = wv[7] + SHA256_F2(wv[4]) + CH(wv[4], wv[5], wv[6])
                + sha256_k[j] + w[j];
            t2 = SHA256_F1(wv[0]) + MAJ(wv[0], wv[1], wv[2]);
            wv[7] = wv[6];
            wv[6] = wv[5];
            wv[5] = wv[4];
            wv[4] = wv[3] + t1;
            wv[3] = wv[2];
            wv[2] = wv[1];
            wv[1] = wv[0];
            wv[0] = t1 + t2;
        }

        for (j = 0; j < 8; j++) {
            ctx->h[j] += wv[j];
        }
#else
        PACK32(&sub_block[ 0], &w[ 0]); PACK32(&sub_block[ 4], &w[ 1]);
        PACK32(&sub_block[ 8], &w[ 2]); PACK32(&sub_block[12], &w[ 3]);
        PACK32(&sub_block[16], &w[ 4]); PACK32(&sub_block[20], &w[ 5]);
        PACK32(&sub_block[24], &w[ 6]); PACK32(&sub_block[28], &w[ 7]);
        PACK32(&sub_block[32], &w[ 8]); PACK32(&sub_block[36], &w[ 9]);
        PACK32(&sub_block[40], &w[10]); PACK32(&sub_block[44], &w[11]);
        PACK32(&sub_block[48], &w[12]); PACK32(&sub_block[52], &w[13]);
        PACK32(&sub_block[56], &w[14]); PACK32(&sub_block[60], &w[15]);

        SHA256_SCR(16); SHA256_SCR(17); SHA256_SCR(18); SHA256_SCR(19);
        SHA256_SCR(20); SHA256_SCR(21); SHA256_SCR(22); SHA256_SCR(23);
        SHA256_SCR(24); SHA256_SCR(25); SHA256_SCR(26); SHA256_SCR(27);
        SHA256_SCR(28); SHA256_SCR(29); SHA256_SCR(30); SHA256_SCR(31);
        SHA256_SCR(32); SHA256_SCR(33); SHA256_SCR(34); SHA256_SCR(35);
        SHA256_SCR(36); SHA256_SCR(37); SHA256_SCR(38); SHA256_SCR(39);
        SHA256_SCR(40); SHA256_SCR(41); SHA256_SCR(42); SHA256_SCR(43);
        SHA256_SCR(44); SHA256_SCR(45); SHA256_SCR(46); SHA256_SCR(47);
        SHA256_SCR(48); SHA256_SCR(49); SHA256_SCR(50); SHA256_SCR(51);
        SHA256_SCR(52); SHA256_SCR(53); SHA256_SCR(54); SHA256_SCR(55);
        SHA256_SCR(56); SHA256_SCR(57); SHA256_SCR(58); SHA256_SCR(59);
        SHA256_SCR(60); SHA256_SCR(61); SHA256_SCR(62); SHA256_SCR(63);

        wv[0] = ctx->h[0]; wv[1] = ctx->h[1];
        wv[2] = ctx->h[2]; wv[3] = ctx->h[3];
        wv[4] = ctx->h[4]; wv[5] = ctx->h[5];
        wv[6] = ctx->h[6]; wv[7] = ctx->h[7];

        SHA256_EXP(0,1,2,3,4,5,6,7, 0); SHA256_EXP(7,0,1,2,3,4,5,6, 1);
        SHA256_EXP(6,7,0,1,2,3,4,5, 2); SHA256_EXP(5,6,7,0,1,2,3,4, 3);
        SHA256_EXP(4,5,6,7,0,1,2,3, 4); SHA256_EXP(3,4,5,6,7,0,1,2, 5);
        SHA256_EXP(2,3,4,5,6,7,0,1, 6); SHA256_EXP(1,2,3,4,5,6,7,0, 7);
        SHA256_EXP(0,1,2,3,4,5,6,7, 8); SHA256_EXP(7,0,1,2,3,4,5,6, 9);
        SHA256_EXP(6,7,0,1,2,3,4,5,10); SHA256_EXP(5,6,7,0,1,2,3,4,11);
        SHA256_EXP(4,5,6,7,0,1,2,3,12); SHA256_EXP(3,4,5,6,7,0,1,2,13);
        SHA256_EXP(2,3,4,5,6,7,0,1,14); SHA256_EXP(1,2,3,4,5,6,7,0,15);
        SHA256_EXP(0,1,2,3,4,5,6,7,16); SHA256_EXP(7,0,1,2,3,4,5,6,17);
        SHA256_EXP(6,7,0,1,2,3,4,5,18); SHA256_EXP(5,6,7,0,1,2,3,4,19);
        SHA256_EXP(4,5,6,7,0,1,2,3,20); SHA256_EXP(3,4,5,6,7,0,1,2,21);
        SHA256_EXP(2,3,4,5,6,7,0,1,22); SHA256_EXP(1,2,3,4,5,6,7,0,23);
        SHA256_EXP(0,1,2,3,4,5,6,7,24); SHA256_EXP(7,0,1,2,3,4,5,6,25);
        SHA256_EXP(6,7,0,1,2,3,4,5,26); SHA256_EXP(5,6,7,0,1,2,3,4,27);
        SHA256_EXP(4,5,6,7,0,1,2,3,28); SHA256_EXP(3,4,5,6,7,0,1,2,29);
        SHA256_EXP(2,3,4,5,6,7,0,1,30); SHA256_EXP(1,2,3,4,5,6,7,0,31);
        SHA256_EXP(0,1,2,3,4,5,6,7,32); SHA256_EXP(7,0,1,2,3,4,5,6,33);
        SHA256_EXP(6,7,0,1,2,3,4,5,34); SHA256_EXP(5,6,7,0,1,2,3,4,35);
        SHA256_EXP(4,5,6,7,0,1,2,3,36); SHA256_EXP(3,4,5,6,7,0,1,2,37);
        SHA256_EXP(2,3,4,5,6,7,0,1,38); SHA256_EXP(1,2,3,4,5,6,7,0,39);
        SHA256_EXP(0,1,2,3,4,5,6,7,40); SHA256_EXP(7,0,1,2,3,4,5,6,41);
        SHA256_EXP(6,7,0,1,2,3,4,5,42); SHA256_EXP(5,6,7,0,1,2,3,4,43);
        SHA256_EXP(4,5,6,7,0,1,2,3,44); SHA256_EXP(3,4,5,6,7,0,1,2,45);
        SHA256_EXP(2,3,4,5,6,7,0,1,46); SHA256_EXP(1,2,3,4,5,6,7,0,47);
        SHA256_EXP(0,1,2,3,4,5,6,7,48); SHA256_EXP(7,0,1,2,3,4,5,6,49);
        SHA256_EXP(6,7,0,1,2,3,4,5,50); SHA256_EXP(5,6,7,0,1,2,3,4,51);
        SHA256_EXP(4,5,6,7,0,1,2,3,52); SHA256_EXP(3,4,5,6,7,0,1,2,53);
        SHA256_EXP(2,3,4,5,6,7,0,1,54); SHA256_EXP(1,2,3,4,5,6,7,0,55);
        SHA256_EXP(0,1,2,3,4,5,6,7,56); SHA256_EXP(7,0,1,2,3,4,5,6,57);
        SHA256_EXP(6,7,0,1,2,3,4,5,58); SHA256_EXP(5,6,7,0,1,2,3,4,59);
        SHA256_EXP(4,5,6,7,0,1,2,3,60); SHA256_EXP(3,4,5,6,7,0,1,2,61);
        SHA256_EXP(2,3,4,5,6,7,0,1,62); SHA256_EXP(1,2,3,4,5,6,7,0,63);

        ctx->h[0] += wv[0]; ctx->h[1] += wv[1];
        ctx->h[2] += wv[2]; ctx->h[3] += wv[3];
        ctx->h[4] += wv[4]; ctx->h[5] += wv[5];
        ctx->h[6] += wv[6]; ctx->h[7] += wv[7];
#endif /* !UNROLL_LOOPS */
    }
}

void sha256_init(sha256_ctx *ctx)
{
#ifndef UNROLL_LOOPS
    int i;
    for (i = 0; i < 8; i++) {
        ctx->h[i] = sha256_h0[i];
    }
#else
    ctx->h[0] = sha256_h0[0]; ctx->h[1] = sha256_h0[1];
    ctx->h[2] = sha256_h0[2]; ctx->h[3] = sha256_h0[3];
    ctx->h[4] = sha256_h0[4]; ctx->h[5] = sha256_h0[5];
    ctx->h[6] = sha256_h0[6]; ctx->h[7] = sha256_h0[7];
#endif /* !UNROLL_LOOPS */

    ctx->len = 0;
    ctx->tot_len = 0;
}

void sha256_update(sha256_ctx *ctx, const uint8 *message,
                   uint32 len)
{
    uint32 block_nb;
    uint32 new_len, rem_len, tmp_len;
    const uint8 *shifted_message;

    tmp_len = SHA256_BLOCK_SIZE - ctx->len;
    rem_len = len < tmp_len ? len : tmp_len;

    memcpy(&ctx->block[ctx->len], message, rem_len);

    if (ctx->len + len < SHA256_BLOCK_SIZE) {
        ctx->len += len;
        return;
    }

    new_len = len - rem_len;
    block_nb = new_len / SHA256_BLOCK_SIZE;

    shifted_message = message + rem_len;

    sha256_transf(ctx, ctx->block, 1);
    sha256_transf(ctx, shifted_message, block_nb);

    rem_len = new_len % SHA256_BLOCK_SIZE;

    memcpy(ctx->block, &shifted_message[block_nb << 6],
           rem_len);

    ctx->len = rem_len;
    ctx->tot_len += (block_nb + 1) << 6;
}

void sha256_final(sha256_ctx *ctx, uint8 *digest)
{
    uint32 block_nb;
    uint32 pm_len;
    uint32 len_b;

#ifndef UNROLL_LOOPS
    int i;
#endif

    block_nb = (1 + ((SHA256_BLOCK_SIZE - 9)
                     < (ctx->len % SHA256_BLOCK_SIZE)));

    len_b = (ctx->tot_len + ctx->len) << 3;
    pm_len = block_nb << 6;

    memset(ctx->block + ctx->len, 0, pm_len - ctx->len);
    ctx->block[ctx->len] = 0x80;
    UNPACK32(len_b, ctx->block + pm_len - 4);

    sha256_transf(ctx, ctx->block, block_nb);

#ifndef UNROLL_LOOPS
    for (i = 0 ; i < 8; i++) {
        UNPACK32(ctx->h[i], &digest[i << 2]);
    }
#else
   UNPACK32(ctx->h[0], &digest[ 0]);
   UNPACK32(ctx->h[1], &digest[ 4]);
   UNPACK32(ctx->h[2], &digest[ 8]);
   UNPACK32(ctx->h[3], &digest[12]);
   UNPACK32(ctx->h[4], &digest[16]);
   UNPACK32(ctx->h[5], &digest[20]);
   UNPACK32(ctx->h[6], &digest[24]);
   UNPACK32(ctx->h[7], &digest[28]);
#endif /* !UNROLL_LOOPS */
}

void sha256(const uint8 *message, uint32 len, uint8 *digest)
{
    sha256_ctx ctx;

    sha256_init(&ctx);
    sha256_update(&ctx, message, len);
    sha256_final(&ctx, digest);
}
/*************************************************************************************/

static void delay_us(int num)
{
	int i;
	while(num>0)
		{
			for(i=0;i<5;i++)
			{
				;
			}
			num--;
		}
}

void sha204c_calculate_crc(uint8_t length, uint8_t *data, uint8_t *crc) {
	uint8_t counter;
	uint16_t crc_register = 0;
	uint16_t polynom = 0x8005;
	uint8_t shift_register;
	uint8_t data_bit, crc_bit;
	for (counter = 0; counter < length; counter++) {
		for (shift_register = 0x01; shift_register > 0x00; shift_register <<= 1) {
			data_bit = (data[counter] & shift_register) ? 1 : 0;
			crc_bit = crc_register >> 15;
			// Shift CRC to the left by 1.
			crc_register <<= 1;
			if ((data_bit ^ crc_bit) != 0)
			crc_register ^= polynom;
		}
	}
	crc[0] = (uint8_t) (crc_register & 0x00FF);
	crc[1] = (uint8_t) (crc_register >> 8);
}


uint8_t sha204c_check_crc(uint8_t *response)
{
	uint8_t crc[2];
	uint8_t count = response[0]; //状态包长度(字节)
	count -= 2;
	sha204c_calculate_crc(count, response, crc);
	return (crc[0] == response[count] && crc[1] == response[count + 1]) ? SHA204_SUCCESS : SHA204_BAD_CRC;
}

static union{
	unsigned short dirty_addr_buf[2];
	const unsigned short normal_i2c[2];
}u_i2c_addr = {{0x00},};

static int fm_fetch_sysconfig_para(void)           //从配置文件获取 设备信息: name ，addr
{
	int ret = -1;
	
	u_i2c_addr.dirty_addr_buf[0] = iic_addr;
	u_i2c_addr.dirty_addr_buf[1] = I2C_CLIENT_END;
	printk("%s: after: client name is atsh204, dirty_addr_buf: 0x%hx. dirty_addr_buf[1]: 0x%hx \n", \
			__func__, u_i2c_addr.dirty_addr_buf[0], u_i2c_addr.dirty_addr_buf[1]);

	twi_id = 1;
	ret = 0;

script_parser_fetch_err:
	atsh204_err("=========script_parser_fetch_err============\n");
	return ret;
}

int atsha204_detect(struct i2c_client *client, struct i2c_board_info *info)
{
	struct i2c_adapter *adapter = client->adapter;
	
	atsh204_dbg(">>>>>> atsh204 enter iic detect,adapter->nr:%d\n",adapter->nr);
	if(twi_id == adapter->nr){
		atsh204_dbg("%s: Detected chip %s at adapter %d, address 0x%02x\n",
			 __func__, ATSHA204_NAME, i2c_adapter_id(adapter), client->addr);

		strlcpy(info->type, ATSHA204_NAME, I2C_NAME_SIZE);  // name 需匹配 i2c_driver ->id_table
		return 0;
	}else{
		atsh204_dbg("atsh204 detect fail...!");
		return -ENODEV;
	}

}

void atsha204_reset_address_count(struct i2c_client *client)
{
	int ret = -99;
	uint8_t txdata[33] = {0};
	txdata[0] = 0x00;
	ret = i2c_master_send(client, txdata, 33);
	
}

static int atsha204_i2c_write_bytes(struct i2c_client *client,uint8_t WordAddress,uint8_t *data,uint8_t len)
{
	int ret = -1,i = 0;
	atsha204_reset_address_count(client);
	uint8_t *buffer = kmalloc(len+1, GFP_KERNEL);
	atsh204_dbg("arno atsha204_i2c_write_bytes size is %d \n",len);
	buffer[0] = WordAddress;
	for(i = 0;i < len;i++){
		buffer[i+1] = data[i];
	}
	ret = i2c_master_send(client, buffer, len+1);
	kfree(buffer);
	return ret;
}

static int atsha204_lock_cfg(struct i2c_client *client)
{
	int ret = -99;
	uint8_t recbuf[4] = {0};
	uint8_t buffer1[7] = {0x07,0x17,0x80,0x00,0x00,0x00,0x00};
	sha204c_calculate_crc(5,buffer1,&buffer1[5]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer1,buffer1[0]);
	msleep(30);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
}

static int atsha204_lock_otp_data(struct i2c_client *client)
{
	int ret = -99;
	uint8_t recbuf[4] = {0};
	uint8_t buffer1[7] = {0x07,0x17,0x81,0x00,0x00,0x00,0x00};
	sha204c_calculate_crc(5,buffer1,&buffer1[5]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer1,buffer1[0]);
	msleep(30);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("atsh204 lock otp success\n");
			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("atsh204 lock otp fail\n");
		return -1;
	}
}

static uint8_t *atsha204_read_command(struct i2c_client *client,int mode,DIFF_ZONE zone,uint8_t wordaddr)
{
	int count=0;
	int i ,ret = -99;
	int len;
	uint8_t txdata[64] = {0};
	uint8_t recbuf[48] = {0};
	if(0 == mode)
	{
		len = 4+3;// read comman lengh is 4 bytes plug 3 bytes (count+CRC)
	}
	else
	{
		len = 32+3;// read comman lengh is 32 bytes plug 3 bytes (count+CRC)
	}
	txdata[count++] = 0x00;	//no used,will replace
	txdata[count++] = ATSHA204_READ_COMMAND;	// read command
	if(mode == 0)
		txdata[count++] = 0x00 | zone;	//different zone and read mode  here should be change by hecong 08/29
	else
		txdata[count++] = 0x80 | zone ;	//different zone and read mode  here should be change by hecong 08/29

	txdata[count++] = wordaddr;	//word address, low byte
	txdata[count++] = 0x00;	//word address ,high byte
	txdata[0]=count+2;	//two bytes CRC
	sha204c_calculate_crc(count,txdata,txdata+count);
	for(i=0;i<txdata[0];i++)
	{
		if(i==1)
			printk("\n");
		else if(i == 17 )
			printk("\n");
		else if( i == 23)
			printk("\n");
		atsh204_dbg("arno write0x%02x ",txdata[i]);
	}
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,txdata,txdata[0]);
	
	msleep(30);	//different commands have different delay times
	i2c_master_recv(client, recbuf, len);
	ret = sha204c_check_crc(recbuf);
	for(i=0;i<len;i++)
	{
		atsh204_dbg("arno read 0x%02x \n",recbuf[i]);
	}
	if(ret == SHA204_SUCCESS){
		atsh204_dbg("read success\n");
		return recbuf;
	}else{
		atsh204_dbg("read fail\n");
		return NULL;
	}

}

static int atsha204_write_cfg(struct i2c_client *client)
{
	int ret = -99;
	uint8_t recbuf[4] = {0};
	
	uint8_t buffer1[11] = {0x0B,0x12,0x00,0x04,0x00,0xC9,0x00,0x55,0x00,0x00,0x00};
	uint8_t buffer2[11] = {0x0B,0x12,0x00,0x05,0x00,0x8F,0x80,0x80,0xA1,0x00,0x00};
	uint8_t buffer3[11] = {0x0B,0x12,0x00,0x06,0x00,0x82,0xE0,0xA3,0x60,0x00,0x00};
	uint8_t buffer4[11] = {0x0B,0x12,0x00,0x07,0x00,0x94,0x40,0xA0,0x85,0x00,0x00};
	uint8_t buffer5[39] = {0x27,0x12,0x80,0x08,0x00,0x86,0x40,0x87,0x07,
													0x0F,0x00,0x89,0xF2,
													0x8A,0x7A,0x0B,0x0B,
													0x0C,0x0C,0xDD,0x4D,
													0xC2,0x42,0xAF,0x8F,
													0x3F,0x00,0x7F,0x00,
													0x7F,0x00,0x07,0x00,
													0x7F,0x00,0x07,0x00,
													0x00,0x00};
	uint8_t buffer6[11] = {0x0B,0x12,0x00,0x10,0x00,0x7F,0x00,0x7F,0x00,0x00,0x00};
	uint8_t buffer7[11] = {0x0B,0x12,0x00,0x11,0x00,0x07,0xFF,0xFF,0xFF,0x00,0x00};
	uint8_t buffer8[11] = {0x0B,0x12,0x00,0x12,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
	uint8_t buffer9[11] = {0x0B,0x12,0x00,0x13,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
	uint8_t buffer10[11] ={0x0B,0x12,0x00,0x14,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
	sha204c_calculate_crc(9,buffer1,&buffer1[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer1,buffer1[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(9,buffer2,&buffer2[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer2,buffer2[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(9,buffer3,&buffer3[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer3,buffer3[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(9,buffer4,&buffer4[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer4,buffer4[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer5,&buffer5[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer5,buffer5[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(9,buffer6,&buffer6[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer6,buffer6[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(9,buffer7,&buffer7[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer7,buffer7[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(9,buffer8,&buffer8[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer8,buffer8[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(9,buffer9,&buffer9[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer9,buffer9[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(9,buffer10,&buffer10[9]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer10,buffer10[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	return WRITE_CFG_SUCCESS;
}

static u32 atsha204_wakeup(struct i2c_client *client, u8 *buf, u32 num)
{
	struct i2c_msg xfer_msg[1];
	uint8_t buffer[4];
	u32 ret = -99;
	
	__u16 addr0 = client->addr;
	client->addr = 0x00;
	xfer_msg[0].addr = 0x00;
	xfer_msg[0].len = num + 1;
	xfer_msg[0].flags = client->flags & I2C_M_TEN;
	xfer_msg[0].buf = buf;
	ret = i2c_transfer(client->adapter, xfer_msg, 1);
	atsh204_dbg("i2c_send_wake ret is %d\n",ret);
	client->addr = addr0;
	msleep(4);
	ret = i2c_master_recv(client, buffer, 2);
	atsh204_dbg("i2c_receive_bytes ret is %d\n",ret);
	if((buffer[0]==0x04)&&(buffer[1]==0x11))
	{
		atsh204_dbg("wakeup is ok\n");
		msleep(4);
		return ret;
	}
	else
	{
		atsh204_dbg("wakeup is fail\n");
		msleep(4);
		return -1;
	}
	
}

static void atsha204_sleep(struct i2c_client *client)
{
	uint8_t data[32] = {0};
	int ret = -99;
	ret = atsha204_i2c_write_bytes(client,SLEEP_ADDRESS,NULL,0);
	atsh204_dbg("atsh204 sleep ret is %d\n",ret);
}

static int watch_dog_feed(struct i2c_client *client)
{
	uint8_t data[1] = {0};
	uint8_t buf[4] = {0};
	int ret = -99;
	ret = atsha204_i2c_write_bytes(client,IDLE_ADDRESS,data,1);
	atsh204_dbg("atsh204 idle ret is %d\n",ret);
	ret = atsha204_wakeup(client, buf, 1);
}

static int atsha204_write_otp(struct i2c_client *client)
{
	int ret = -99;
	uint8_t recbuf[4] = {0};
	
	uint8_t buffer1[39] = {0x27,0x12,0x81,0x00,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
	uint8_t buffer2[39] = {0x27,0x12,0x81,0x08,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
	sha204c_calculate_crc(37,buffer1,&buffer1[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer1,buffer1[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer2,&buffer2[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer2,buffer2[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	

	
	return WRITE_OTP_SUCCESS;
}

static int atsha204_mac(struct i2c_client *client)
{
	uint8_t recbuf[39] = {0};
	int i = 0,ret = -99;
	int len = 0x58; //MsgBuf has 88 byte data
	sha256_ctx ctx;
	uint8_t MsgBuf[128] = {0x11,0x11,0x23,0xB6,0xCC,0x53,0xB7,0xB9,0xE9,0xBB,0x51,0xFD,0x2F,0x74,0xCD,0x0E,
						   0x91,0xD9,0x7F,0xEB,0x84,0x7B,0x98,0x09,0xF4,0xCD,0x93,0x6A,0xB6,0x48,0x11,0x11,
						   0x24,0x66,0x17,0xc8,0xba,0x55,0xf9,0x56,0x71,0x93,0x16,0xdc,0x5d,0xa2,0x76,0x46,
						   0x14,0x85,0x46,0x95,0x31,0xaa,0x82,0x37,0x43,0xa6,0xe7,0xe8,0x39,0xd7,0x23,0x88,
						   0x08,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xee,
						   0x00,0x00,0x00,0x00,0x01,0x23,0x00,0x00};
	uint8_t Digest[32] = {0};
	uint8_t buffer1[39] = {0x27,0x08,0x00,0x01,0x00,0x24,0x66,0x17,0xc8,0xba,0x55,0xf9,0x56,0x71,0x93,0x16,0xdc,0x5d,0xa2,0x76,0x46,0x14,0x85,0x46,0x95,0x31,0xaa,0x82,0x37,0x43,0xa6,0xe7,0xe8,0x39,0xd7,0x23,0x88,0x00,0x00};
	sha204c_calculate_crc(37,buffer1,&buffer1[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer1,buffer1[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 35);
	ret = sha204c_check_crc(recbuf);
	
	sha256_init(&ctx);
	sha256_update(&ctx, MsgBuf, len);
	sha256_final(&ctx, Digest);
	
	atsh204_dbg(">>>>>> atsha204 digest is :\n");
	for(i = 0;i < 32;i++){
		atsh204_dbg("%02X ",Digest[i]);
		if(recbuf[i+1] != Digest[i]){
			atsh204_dbg(">>>>>>> atsha204 check fail : Digest[i] != recbuf[i+1] :%2X <<<<<<<<<<<<\n",Digest[i]);
			break;
		}
		if((i+1) % 8 == 0)
			atsh204_dbg("\n");
		if(i == 31)
			atsh204_dbg(">>>>>>> atsha204 check success <<<<<<<<<<<<\n");

	}
	if(i<30)
	{
		for(i = 0;i < 32;i++){
			atsh204_dbg("%02X ",Digest[i]);
			if((i+1) % 8 == 0)
				atsh204_dbg("\n");
			if(i == 31)
				atsh204_dbg(">>>>>>> atsha204 check success <<<<<<<<<<<<\n");
		}
	}	

	for(i = 0;i < 35;i++){
		atsh204_dbg("%02X ",recbuf[i]);
		if(i % 8 == 0)
			atsh204_dbg("\n");
	}
	return ret;
}

static int atsha204_write_data(struct i2c_client *client)
{
	int ret = -99;
	uint8_t recbuf[4] = {0};
/*	uint8_t buffer1[39] = {0x27,0x12,0x82,0x00,0x00,0xd4,0x84,0x46,0x11,0x46,0xdc,0xa1,0x1a,0x25,0x52,0x41,0x21,0x74,0x76,0x77,0xd7,0x7a,0x13,0xf4,0xc9,0x74,0x78,0x89,0x12,0x34,0x52,0xcd,0xad,0xfc,0x11,0x4a,0xf8,0x00,0x00};
	uint8_t buffer2[39] = {0x27,0x12,0x82,0x08,0x00,0x11,0x11,0x22,0xba,0x46,0x13,0xfc,0xcd,0xdc,0xac,0xca,0x84,0x46,0x15,0x14,0x52,0x63,0x58,0x96,0x56,0x36,0xfa,0xa4,0xa5,0x6a,0x41,0xc5,0x5d,0xf9,0x9a,0x2c,0x99,0x00,0x00};	
	uint8_t buffer3[39] = {0x27,0x12,0x82,0x10,0x00,0xcc,0x12,0x54,0x00,0x01,0x4a,0x32,0xfc,0x44,0x85,0xad,0x25,0xc8,0x23,0x89,0xf5,0x4a,0xc8,0xb5,0x5b,0xb9,0x24,0xab,0xcc,0x23,0x74,0x07,0x80,0x08,0xb4,0xad,0xda,0x00,0x00};
	uint8_t buffer4[39] = {0x27,0x12,0x82,0x18,0x00,0xc1,0x25,0x35,0xd4,0x85,0x74,0x45,0x54,0x64,0x71,0x14,0x19,0xb4,0xa5,0xff,0x54,0x74,0xf9,0x2b,0xb7,0xa4,0x61,0x7f,0xa3,0x32,0x12,0x28,0x76,0xc9,0x84,0x46,0x3f,0x00,0x00};
	uint8_t buffer5[39] = {0x27,0x12,0x82,0x20,0x00,0xa5,0xc8,0x02,0x10,0x54,0x40,0xfb,0x41,0xa6,0x22,0xa7,0x16,0x84,0x76,0x00,0x43,0x31,0xb7,0xaa,0x45,0x73,0x92,0x17,0x93,0x13,0x73,0x18,0x43,0x43,0x18,0x16,0x67,0x00,0x00};
	uint8_t buffer6[39] = {0x27,0x12,0x82,0x28,0x00,0x45,0x87,0x46,0x92,0x20,0xb6,0x46,0x16,0xf5,0x56,0x96,0xfc,0x13,0xa6,0xa6,0x73,0xb9,0x84,0x71,0x43,0x38,0x16,0x56,0xd9,0xe9,0xe4,0xa6,0xff,0x41,0xee,0xa6,0xf8,0x00,0x00};
	uint8_t buffer7[39] = {0x27,0x12,0x82,0x30,0x00,0xd6,0x23,0x45,0x12,0x56,0x89,0x74,0x89,0x46,0x24,0x63,0x71,0x46,0x16,0x75,0x89,0xd5,0x13,0xe9,0x76,0x14,0x53,0x42,0x89,0x82,0x46,0x13,0xd5,0x11,0x25,0x69,0x46,0x00,0x00};
	uint8_t buffer8[39] = {0x27,0x12,0x82,0x38,0x00,0xc3,0xfb,0x39,0x47,0x56,0x84,0x14,0x26,0x84,0x43,0x49,0x11,0x44,0x13,0x34,0x31,0x98,0x35,0x34,0x11,0x47,0x36,0x44,0x36,0x47,0x62,0x34,0x74,0xd5,0xe9,0xac,0xfd,0x00,0x00};
	uint8_t buffer9[39] = {0x27,0x12,0x82,0x40,0x00,0x46,0x48,0x14,0xca,0x15,0x78,0x91,0x68,0xc6,0x4f,0xc7,0xa6,0x14,0x22,0x47,0xdc,0x63,0x34,0x52,0x18,0xf4,0x7d,0x12,0x00,0x1f,0xaa,0x13,0x31,0x25,0x19,0x67,0x94,0x00,0x00};
	uint8_t buffer10[39] ={0x27,0x12,0x82,0x48,0x00,0xce,0xe3,0x24,0xa6,0x18,0xf6,0x66,0x44,0xa5,0xc6,0xd8,0xcc,0x77,0xe9,0xf6,0xd5,0x5f,0x2a,0x36,0x71,0x24,0x46,0xfc,0xaa,0x12,0xc6,0x13,0xa9,0x16,0xff,0x69,0x82,0x00,0x00};
	uint8_t buffer11[39] ={0x27,0x12,0x82,0x50,0x00,0x45,0x95,0xd6,0xf6,0x21,0x71,0x24,0xff,0x22,0x22,0x22,0xa3,0x75,0xf6,0x24,0xd6,0xa3,0xb6,0x84,0x78,0x89,0x34,0xd6,0x54,0x5f,0x2a,0x8d,0xe5,0x6e,0x22,0x33,0x46,0x00,0x00};
	uint8_t buffer12[39] ={0x27,0x12,0x82,0x58,0x00,0x13,0x74,0xef,0xfc,0x1a,0x51,0x61,0x38,0x94,0x2b,0x7b,0x56,0x69,0x43,0x27,0x73,0x81,0xa6,0x12,0xf6,0xd2,0x21,0xcb,0xbf,0xcd,0xcf,0xad,0x00,0xff,0x46,0x33,0x44,0x00,0x00};
	uint8_t buffer13[39] ={0x27,0x12,0x82,0x60,0x00,0x56,0xd4,0x32,0xac,0xcd,0x56,0x77,0xd9,0x46,0x17,0xed,0xd6,0xac,0x13,0x74,0x58,0x13,0xed,0x13,0xd6,0x11,0x78,0x98,0x58,0x68,0x41,0x28,0xdc,0xcd,0xca,0x13,0x12,0x00,0x00};
	uint8_t buffer14[39] ={0x27,0x12,0x82,0x68,0x00,0x46,0x78,0xaa,0x56,0xfc,0xed,0xef,0x2e,0x3e,0x16,0x34,0xaa,0xcd,0xac,0x83,0x17,0x16,0x54,0x83,0x27,0x19,0x94,0x67,0x49,0x35,0x44,0x87,0x53,0x46,0x24,0x35,0x68,0x00,0x00};
	uint8_t buffer15[39] ={0x27,0x12,0x82,0x70,0x00,0xd6,0xa3,0xd2,0xe6,0xa2,0xc6,0x34,0x24,0x86,0x98,0x37,0xdd,0x3a,0x2c,0x14,0xd6,0x2c,0x7f,0xee,0xff,0x3a,0x68,0x10,0x00,0x00,0xad,0x43,0x2e,0xd7,0xd3,0x24,0xa6,0x00,0x00};
	uint8_t buffer16[39] ={0x27,0x12,0x82,0x78,0x00,0x24,0x76,0xa6,0x35,0x62,0x75,0x44,0x66,0xee,0xdf,0x3a,0xd8,0xd9,0x6a,0x27,0x46,0x53,0x56,0x84,0x78,0x89,0x34,0x55,0x39,0x56,0x77,0xff,0x34,0x77,0x3a,0x85,0x64,0x00,0x00};*/
	
	uint8_t buffer1[39] = {0x27,0x12,0x82,0x00,0x00,0x11,0x11,0x23,0xB6,0xCC,0x53,0xB7,0xB9,0xE9,0xBB,0x51,0xFD,0x2F,0x74,0xCD,0x0E,0x91,0xD9,0x7F,0xEB,0x84,0x7B,0x98,0x09,0xF4,0xCD,0x93,0x6A,0xB6,0x48,0x11,0x11,0x00,0x00};
	uint8_t buffer2[39] = {0x27,0x12,0x82,0x08,0x00,0x11,0x11,0x23,0xB6,0xCC,0x53,0xB7,0xB9,0xE9,0xBB,0x51,0xFD,0x2F,0x74,0xCD,0x0E,0x91,0xD9,0x7F,0xEB,0x84,0x7B,0x98,0x09,0xF4,0xCD,0x93,0x6A,0xB6,0x48,0x11,0x11,0x00,0x00};	
	uint8_t buffer3[39] = {0x27,0x12,0x82,0x10,0x00,0x22,0x22,0xC1,0x7C,0x1C,0x4D,0x56,0x89,0xAA,0x00,0x43,0xE3,0x9C,0xFB,0x6B,0x0B,0x68,0x49,0xE3,0x2C,0x24,0xA4,0x1B,0x06,0x34,0x49,0x1E,0x90,0x6B,0x62,0x22,0x22,0x00,0x00};
	uint8_t buffer4[39] = {0x27,0x12,0x82,0x18,0x00,0xc1,0x25,0x35,0xd4,0x85,0x74,0x45,0x54,0x64,0x71,0x14,0x19,0xb4,0xa5,0xff,0x54,0x74,0xf9,0x2b,0xb7,0xa4,0x61,0x7f,0xa3,0x32,0x12,0x28,0x76,0xc9,0x84,0x46,0x3f,0x00,0x00};
	uint8_t buffer5[39] = {0x27,0x12,0x82,0x20,0x00,0xa5,0xc8,0x02,0x10,0x54,0x40,0xfb,0x41,0xa6,0x22,0xa7,0x16,0x84,0x76,0x00,0x43,0x31,0xb7,0xaa,0x45,0x73,0x92,0x17,0x93,0x13,0x73,0x18,0x43,0x43,0x18,0x16,0x67,0x00,0x00};
	uint8_t buffer6[39] = {0x27,0x12,0x82,0x28,0x00,0x45,0x87,0x46,0x92,0x20,0xb6,0x46,0x16,0xf5,0x56,0x96,0xfc,0x13,0xa6,0xa6,0x73,0xb9,0x84,0x71,0x43,0x38,0x16,0x56,0xd9,0xe9,0xe4,0xa6,0xff,0x41,0xee,0xa6,0xf8,0x00,0x00};
	uint8_t buffer7[39] = {0x27,0x12,0x82,0x30,0x00,0xd6,0x23,0x45,0x12,0x56,0x89,0x74,0x89,0x46,0x24,0x63,0x71,0x46,0x16,0x75,0x89,0xd5,0x13,0xe9,0x76,0x14,0x53,0x42,0x89,0x82,0x46,0x13,0xd5,0x11,0x25,0x69,0x46,0x00,0x00};
	uint8_t buffer8[39] = {0x27,0x12,0x82,0x38,0x00,0xc3,0xfb,0x39,0x47,0x56,0x84,0x14,0x26,0x84,0x43,0x49,0x11,0x44,0x13,0x34,0x31,0x98,0x35,0x34,0x11,0x47,0x36,0x44,0x36,0x47,0x62,0x34,0x74,0xd5,0xe9,0xac,0xfd,0x00,0x00};
	uint8_t buffer9[39] = {0x27,0x12,0x82,0x40,0x00,0x46,0x48,0x14,0xca,0x15,0x78,0x91,0x68,0xc6,0x4f,0xc7,0xa6,0x14,0x22,0x47,0xdc,0x63,0x34,0x52,0x18,0xf4,0x7d,0x12,0x00,0x1f,0xaa,0x13,0x31,0x25,0x19,0x67,0x94,0x00,0x00};
	uint8_t buffer10[39] ={0x27,0x12,0x82,0x48,0x00,0xce,0xe3,0x24,0xa6,0x18,0xf6,0x66,0x44,0xa5,0xc6,0xd8,0xcc,0x77,0xe9,0xf6,0xd5,0x5f,0x2a,0x36,0x71,0x24,0x46,0xfc,0xaa,0x12,0xc6,0x13,0xa9,0x16,0xff,0x69,0x82,0x00,0x00};
	uint8_t buffer11[39] ={0x27,0x12,0x82,0x50,0x00,0x45,0x95,0xd6,0xf6,0x21,0x71,0x24,0xff,0x22,0x22,0x22,0xa3,0x75,0xf6,0x24,0xd6,0xa3,0xb6,0x84,0x78,0x89,0x34,0xd6,0x54,0x5f,0x2a,0x8d,0xe5,0x6e,0x22,0x33,0x46,0x00,0x00};
	uint8_t buffer12[39] ={0x27,0x12,0x82,0x58,0x00,0x13,0x74,0xef,0xfc,0x1a,0x51,0x61,0x38,0x94,0x2b,0x7b,0x56,0x69,0x43,0x27,0x73,0x81,0xa6,0x12,0xf6,0xd2,0x21,0xcb,0xbf,0xcd,0xcf,0xad,0x00,0xff,0x46,0x33,0x44,0x00,0x00};
	uint8_t buffer13[39] ={0x27,0x12,0x82,0x60,0x00,0x56,0xd4,0x32,0xac,0xcd,0x56,0x77,0xd9,0x46,0x17,0xed,0xd6,0xac,0x13,0x74,0x58,0x13,0xed,0x13,0xd6,0x11,0x78,0x98,0x58,0x68,0x41,0x28,0xdc,0xcd,0xca,0x13,0x12,0x00,0x00};
	uint8_t buffer14[39] ={0x27,0x12,0x82,0x68,0x00,0x46,0x78,0xaa,0x56,0xfc,0xed,0xef,0x2e,0x3e,0x16,0x34,0xaa,0xcd,0xac,0x83,0x17,0x16,0x54,0x83,0x27,0x19,0x94,0x67,0x49,0x35,0x44,0x87,0x53,0x46,0x24,0x35,0x68,0x00,0x00};
	uint8_t buffer15[39] ={0x27,0x12,0x82,0x70,0x00,0xd6,0xa3,0xd2,0xe6,0xa2,0xc6,0x34,0x24,0x86,0x98,0x37,0xdd,0x3a,0x2c,0x14,0xd6,0x2c,0x7f,0xee,0xff,0x3a,0x68,0x10,0x00,0x00,0xad,0x43,0x2e,0xd7,0xd3,0x24,0xa6,0x00,0x00};
	uint8_t buffer16[39] ={0x27,0x12,0x82,0x78,0x00,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0x00,0x00};
	
	sha204c_calculate_crc(37,buffer1,&buffer1[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer1,buffer1[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer2,&buffer2[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer2,buffer2[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer3,&buffer3[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer3,buffer3[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}
	
	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer4,&buffer4[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer4,buffer4[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer5,&buffer5[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer5,buffer5[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer6,&buffer6[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer6,buffer6[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer7,&buffer7[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer7,buffer7[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer8,&buffer8[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer8,buffer8[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer9,&buffer9[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer9,buffer9[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer10,&buffer10[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer10,buffer10[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer11,&buffer11[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer11,buffer11[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer12,&buffer12[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer12,buffer12[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer13,&buffer13[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer13,buffer13[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer14,&buffer14[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer14,buffer14[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer15,&buffer15[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer15,buffer15[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	memset(recbuf,0,strlen(recbuf));
	sha204c_calculate_crc(37,buffer16,&buffer16[37]);
	atsha204_i2c_write_bytes(client,COMMAND_ADDRESS,buffer16,buffer16[0]);
	msleep(50);	//different commands have different delay times
	i2c_master_recv(client, recbuf, 4);
	ret = sha204c_check_crc(recbuf);
	if(ret == SHA204_SUCCESS){
		if(recbuf[1] == 0x00){
			atsh204_dbg("write success\n");
//			return ret;
		}else{
			return -1;	
		}
	}else{
		atsh204_dbg("write fail\n");
		return -1;
	}

	
	return WRITE_DATA_SUCCESS;
}

void sleep(int m)
{
	int i=0;

	for(i=0; i<m; i++)
		msleep(1000);
}

static int __devinit atsha204_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct atsha204_st *sha204_st;
	unsigned char *value;
	u32 ret = 99;
	int ret2 = -99, ret3 = -99, i = 0;
	
	atsh204_dbg(">>>>>> atsha204 enter prob,match\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		atsh204_err("I2C functionality not supported\n");
		return -ENODEV;
	}

	sha204_st = kzalloc(sizeof(*sha204_st), GFP_KERNEL);
	if (!sha204_st){
		atsh204_err("allocate data fail!\n");
		return -ENOMEM;
	}

	sha204_st->client = client;
	sha204_st->device_id = id->driver_data;
	
	i2c_set_clientdata(sha204_st->client,sha204_st);
	//////////////////////////////////
	uint8_t buf[4] = {0};
	uint8_t recbuf[255] = {0};

	ret = atsha204_wakeup(client, buf, 1);

#if 1
	if(atsha204_write_cfg(client) == 8){
		atsh204_dbg(">>>>>> atsha204 write cfg success\n");
	}else{
		atsh204_dbg(">>>>>> atsha204 write cfg fail\n");
	}
#endif	
	
	atsha204_read_command(client,READ32_MODE,CONFIG_ZONE_BIT,0x00);
	atsha204_read_command(client,READ32_MODE,CONFIG_ZONE_BIT,0x08);
	atsha204_read_command(client,READ4_MODE,CONFIG_ZONE_BIT,0x10);
	atsha204_read_command(client,READ4_MODE,CONFIG_ZONE_BIT,0x11);
	atsha204_read_command(client,READ4_MODE,CONFIG_ZONE_BIT,0x12);
	atsha204_read_command(client,READ4_MODE,CONFIG_ZONE_BIT,0x13);
	atsha204_read_command(client,READ4_MODE,CONFIG_ZONE_BIT,0x14);
	atsha204_read_command(client,READ4_MODE,CONFIG_ZONE_BIT,0x15);
	printk("\!!!!!!!!\\\n");
	atsha204_read_command(client,READ4_MODE,DATA_ZONE_BIT,0x15);
#if 1	
	if(atsha204_lock_cfg(client) != -1){
		atsh204_dbg(">>>>>> atsha204 lock cfg success\n");
	}else{
		atsh204_dbg(">>>>>> atsha204 lock cfg fail\n");
	}
	watch_dog_feed(client);
	if(atsha204_write_otp(client) == 7){
		atsh204_dbg(">>>>>> atsha204 write otp success\n");
	}else{
		atsh204_dbg(">>>>>> atsha204 write otp fail\n");
	}
	watch_dog_feed(client);
	if(atsha204_write_data(client) == 6){
		atsh204_dbg(">>>>>> atsha204 write data success\n");
	}else{
		atsh204_dbg(">>>>>> atsha204 write data fail\n");
	}
	watch_dog_feed(client);
#if 1	
	if(atsha204_lock_otp_data(client) != -1){
		atsh204_dbg(">>>>>> atsha204 lock otp success\n");
	}else{
		atsh204_dbg(">>>>>> atsha204 lock otp fail\n");
	}

	printk("|||||||||||\n");
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x00);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x08);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x10);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x18);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x20);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x28);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x30);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x38);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x40);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x48);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x50);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x58);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x60);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x68);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x70);
	atsha204_read_command(client,READ32_MODE,DATA_ZONE_BIT,0x78);
	printk("xxxx|||||||||||\n");

#endif
	while(1)
	{
		sleep(5);
		atsha204_mac(client);
		sleep(120);
		printk("sleep 120\n");	
		sleep(240);
		atsha204_mac(client);
		printk("sleep 240\n");	
		atsha204_mac(client);
		sleep(480);
		printk("sleep 480\n");	
		atsha204_mac(client);
		sleep(960);
		printk("sleep 960\n");	
		atsha204_mac(client);
		sleep(1800);
		printk("sleep 1800\n");	
		atsha204_mac(client);
	}
	atsha204_sleep(client);
#endif

	////////////////////////////////
	atsh204_dbg(">>>>>> atsha204 end prob\n");
	return 0;
}

static struct i2c_driver atsha204_driver = {
	.driver = {
		.name = ATSHA204_NAME,
		.owner = THIS_MODULE,
	},
	.probe		= atsha204_probe,
	.id_table		= atsha204_id,
		
};



static int __init atsha204_init(void)
{
	int ret = -1;
	ret = i2c_add_driver(&atsha204_driver);
	atsh204_dbg(">>>>>> atsh204 done i2c_add_driver :%d\n",ret);
	return ret;
}


static void __exit atsha204_exit(void)
{
 
	atsh204_dbg(">>>>>> atsh204 done i2c_del_driver\n");
	i2c_del_driver(&atsha204_driver);
 
	return;
}

module_init(atsha204_init);
module_exit(atsha204_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("atsha204  controller driver");
MODULE_AUTHOR("Dennis Meng(dennis.meng@vicast.com.cn");
 
 
