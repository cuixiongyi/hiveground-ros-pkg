/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2013, Imai Laboratory, Keio University.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *      * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *      * Neither the name of the Imai Laboratory, nor the name of its
 *      contributors may be used to endorse or promote products derived from
 *      this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Mahisorn Wongphati
 * Notice: Modified & copied from Leptrino CD example source code
 */

// =============================================================================
//	RS232C 用モジュール
//
//					Filename: rs_comm.h
//
// =============================================================================
//		Ver 1.0.0		2012/11/01
// =============================================================================
#include "pCommon.h"

// =============================================================================
//	マクロ定義
// =============================================================================
#ifndef _RSCOMM_H
#define _RSCOMM_H

#define PAR_NON 		0
#define PAR_ODD 		1
#define PAR_EVEN 		2

#define BIT_LEN_7		7
#define BIT_LEN_8		8

#define CHR_STX		0x02
#define CHR_ETX		0x03
#define CHR_EOT		0x04
#define CHR_ENQ		0x05
#define CHR_ACK		0x06
#define CHR_LF		0x0A
#define CHR_CR		0x0D
#define CHR_DLE		0x10
#define CHR_NAK		0x15
#define CHR_SUB		0x1A

int Comm_Open(const char * dev); //デバイスオープン
void Comm_Close(void); //デバイスクローズ
void Comm_Setup(long baud, int parity, int bitlen, int rts, int dtr, char code); //ポート設定
int Comm_SendData(UCHAR *buff, int l);
int Comm_CheckRcv(void); //受信有無確認
int Comm_GetRcvData(UCHAR *buff); //受信データ取得
void Comm_Rcv(void);

#endif
