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

/********************************************************************************/
/* File Name			: Common.h												*/
/* Description			: 共通定義												*/
/********************************************************************************/
#ifndef _LEPTRINO_COMMON_H
#define _LEPTRINO_COMMON_H

/********************************************************************************/
/*	変数型定義 																	*/
/********************************************************************************/
typedef unsigned char UCHAR;
typedef signed char SCHAR;
typedef unsigned short USHORT;
typedef signed short SSHORT;
typedef unsigned long ULONG;
typedef signed long SLONG;

/********************************************************************************/
/*	定数定義 																	*/
/********************************************************************************/
#define OK	1
#define NG	-1

#define SERIAL_SIZE		8			/* 製品シリアルサイズ */
#define P_NAME_SIZE		16			/* 製品型式サイズ */
#define F_VER_SIZE		4			/* ファームバージョンサイズ */
#define FREQ_SIZE		6			/* 出力レート */

#define MSG_SIZE		128			/* 電文サイズ */

/********************************************************************************/
/*	マクロ 																		*/
/********************************************************************************/

/********************************************************************************/
/*	列挙型定義 																	*/
/********************************************************************************/
/* 力要素番号 */
enum ForceNo
{
  FN_Fx = 0,
  FN_Fy,
  FN_Fz,
  FN_Mx,
  FN_My,
  FN_Mz,
  FN_Num
};

#endif
/************************* (C) COPYRIGHT 2010 Leptrino **************************/
