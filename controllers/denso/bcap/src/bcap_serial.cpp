/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2012, Imai Laboratory, Keio University.
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
 * Based on Denso b-Cap example
 */

#include <bcap/bcap_serial.h>

using namespace boost;

BCapSerial::BCapSerial(const std::string& port, unsigned int baud_rate) :
    BCap(true), io_(), serial_port_(io_, port)
{
  serial_port_.set_option(asio::serial_port_base::baud_rate(baud_rate));
}

BCAP_HRESULT BCapSerial::SendBinary(uint8_t *pBuff, uint32_t lSize)
{
  try
  {
    asio::write(serial_port_, asio::buffer(pBuff, lSize));
    if (m_show_debug_packet)
    {
      printf("-------------- TX --------------\n");
      int count = 0;
      for (uint32_t i = 0; i < lSize; i++)
      {

        printf("%02x ", (unsigned char)pBuff[i]);
        count++;
        if (count == 16)
        {
          count = 0;
          printf("\n");
        }

      }
      printf("\n--------------------------------\n");
    }

  }
  catch (boost::system::system_error & error)
  {
    return BCAP_E_FAIL;
  }
  return BCAP_S_OK;
}

uint8_t * BCapSerial::ReceivePacket()
{
  uint8_t *pTop;
  uint8_t *pPacketBuff = NULL;
  uint32_t lRecvSize;
  int lRecLen;

  int count = 0;

  lRecvSize = 0;
  if (m_show_debug_packet)
  {
    printf("-------------- RX --------------\n");
  }

  lRecLen = asio::read(serial_port_, asio::buffer(&(pRcvBuffer[lRecvSize]), BCAP_HEADER_SIZE));

  if (m_show_debug_packet)
  {
    for (int i = 0; i < lRecLen; i++)
    {
      printf("%02x ", (unsigned char)pRcvBuffer[lRecvSize + i]);
      count++;
      if (count == 16)
      {
        count = 0;
        printf("\n");
      }
    }
  }

  lRecvSize += lRecLen; /* add read bytes */

  pTop = (uint8_t *)memchr((const void *)pRcvBuffer, BCAP_SOH, lRecvSize);
  if (pTop == NULL)
  { /* Is there SOH ? */
    lRecvSize = 0; /* If No SOH, then all read data are discarded */
  }
  else
  {
    if (pTop != pRcvBuffer)
    { /* if (pTop == pRcvBuffer) then SOH is already in the top. */
      lRecvSize = lRecvSize - (pTop - pRcvBuffer); /* exclude before SOH  */
      memmove(pRcvBuffer, pTop, lRecvSize);
    }
  }

  uint32_t lPacketSize;
  uint32_t lRemainSize;

  copyValue(&lPacketSize, &(pRcvBuffer[1]), BCAP_SIZE_LEN);
  lRemainSize = lPacketSize - lRecvSize;

  //printf("\nlPacketSize: %d lRecvSize: %d lRemainSize: %d \n", lPacketSize, lRecvSize, lRemainSize);

  lRecLen = asio::read(serial_port_, asio::buffer(&(pRcvBuffer[lRecvSize]), lRemainSize));

  if (m_show_debug_packet)
  {
    for (int i = 0; i < lRecLen; i++)
    {
      printf("%02x ", (unsigned char)pRcvBuffer[lRecvSize + i]);
      count++;
      if (count == 16)
      {
        count = 0;
        printf("\n");
      }
    }
  }

  lRecvSize += lRecLen; /* add read bytes */

  if (pRcvBuffer[lPacketSize - 1] != BCAP_EOT)
  {
    return NULL;
  }

  pPacketBuff = (unsigned char *)bMalloc(lPacketSize);
  if (pPacketBuff != NULL)
  {
    memcpy(pPacketBuff, pRcvBuffer, lRecvSize);
    return pPacketBuff;
  }
  else
  {
    return NULL;
  }
}
