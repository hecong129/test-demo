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
 * @file atsha204a.h
 * 函数:
 * @author 何聪:
 * @version 1.0
 * @date 2016-08-31
 */

#ifndef ATSHA204A_H
#define ATSHA204A_H

#define SHA224_DIGEST_SIZE ( 224 / 8)
#define SHA256_DIGEST_SIZE ( 256 / 8)

#define SHA256_BLOCK_SIZE  ( 512 / 8)
#define SHA224_BLOCK_SIZE  SHA256_BLOCK_SIZE

#ifndef SHA2_TYPES
#define SHA2_TYPES
typedef unsigned char uint8;
typedef unsigned int  uint16;
typedef unsigned long uint32;
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32 tot_len;
    uint32 len;
    uint8 block[2 * SHA256_BLOCK_SIZE];
    uint32 h[8];
} sha256_ctx;

void sha256_init(sha256_ctx * ctx);
void sha256_update(sha256_ctx *ctx, const uint8 *message, uint32 len);
void sha256_final(sha256_ctx *ctx, uint8 *digest);

void sha256_noPad(sha256_ctx *ctx, uint8 *digest);

void sha256(const uint8 *message, uint32 len, uint8 *digest);

#ifdef __cplusplus
}
#endif

#endif 

