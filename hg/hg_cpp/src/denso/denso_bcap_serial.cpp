#include <hg_cpp/denso/denso_bcap_serial.h>

BCapSerial::BCapSerial()
	: BCapClient(), SerialPort()
{
}

BCapSerial::~BCapSerial()
{
}

BCAP_HRESULT BCapSerial::sendBinary(uint8_t *pBuff, uint32_t lSize)
{
	BCAP_HRESULT hr = BCAP_E_FAIL;
	int iLen;
	int iSent;

	iLen = (int)lSize;
	if (iLen > 0){

		iSent = Out(pBuff, iLen, 100);
		if (iSent == iLen){
			 hr = BCAP_S_OK;
             if(m_show_debug_packet)
             {
                printf("-------------- TX --------------\n");
                int count = 0;
                for(int i = 0; i < lSize; i++)
		        {
			        
			        printf("%02x ", (unsigned char)pBuff[i]);
			        count++;
			        if(count == 16)
			        {
				        count = 0;
				        printf("\n");
			        }

		        }  
                printf("\n--------------------------------\n"); 
             }
        }
	
	}
	return hr;
}

uint8_t *BCapSerial::receivePacket()
{
	
	uint8_t	pRcvBuffer[LOCALRECBUFFER_SZ];
	uint8_t *pTop;
	uint8_t	*pPacketBuff = NULL;
	uint8_t	*pRemainData;

	uint32_t lRecvSize;
	int lRecLen;
	uint32_t lHeaderLen;

	int count = 0;
	
	/* b-CAP header = 15 bytes, this should be recieved at first */	
	lHeaderLen = BCAP_SIZE_SOH + BCAP_SIZE_LEN + 
				BCAP_SIZE_SERIAL + BCAP_SIZE_RESERVE +
				BCAP_SIZE_FUNCID + BCAP_SIZE_ARGNUM;

	/* Receive b-Cap header */
		lRecvSize = 0;
        if(m_show_debug_packet)
        {
            printf("-------------- RX --------------\n");
        }
		while (lRecvSize < lHeaderLen) {
			//lRecLen = recv(iSockFd, (char *)&(pRcvBuffer[lRecvSize]), lHeaderLen - lRecvSize, 0); 
			lRecLen = In(&(pRcvBuffer[lRecvSize]), lHeaderLen - lRecvSize, 100);

            if(m_show_debug_packet)
            {
			    for(int i = 0; i < lRecLen; i++)
			    {
				    printf("%02x ", (unsigned char)pRcvBuffer[lRecvSize+i]); 
				    count++;
				    if(count == 16)
				    {
					    count = 0;
					    printf("\n");
				    }
			    }
            }

			if(lRecLen <= 0) {	/* if sock errer has detected, then exit  */
				goto ExitPoint;
			}
			lRecvSize += lRecLen;			/* add read bytes */

			pTop = (uint8_t *)memchr((const void *)pRcvBuffer, BCAP_SOH, lRecvSize);
			if (pTop == NULL){				/* Is there SOH ? */
				lRecvSize = 0;				/* If No SOH, then all read data are discarded */
			}
			else{
				if (pTop != pRcvBuffer){	/* if (pTop == pRcvBuffer) then SOH is already in the top. */	
					lRecvSize = lRecvSize - (pTop - pRcvBuffer);	/* exclude before SOH  */
					memmove (pRcvBuffer, pTop, lRecvSize);
				}
			}
		}

	/* Receive the left data of this packet */
	{
		uint32_t lPacketSize;
		uint32_t lRemainSize;

		copyValue(&lPacketSize, &(pRcvBuffer[1]), BCAP_SIZE_LEN);
		lRemainSize  = lPacketSize - lRecvSize;

		pPacketBuff = (unsigned char *)bMalloc(lPacketSize);	
		if (pPacketBuff != NULL){
			memcpy(pPacketBuff, pRcvBuffer, lRecvSize);
			pRemainData = pPacketBuff + lRecvSize;
		}
		else{
			goto ExitPoint;	/* out of memory */
		}

		lRecvSize = 0;
		while (lRecvSize < lRemainSize) {
			//lRecLen = recv(iSockFd, (char *)&(pRemainData[lRecvSize]), lRemainSize -lRecvSize , 0); 
			lRecLen = In(&(pRemainData[lRecvSize]), lRemainSize - lRecvSize, 100);

            if(m_show_debug_packet)
            {
			    for(int i = 0; i < lRecLen; i++)
			    {
				    printf("%02x ", (unsigned char)pRemainData[lRecvSize+i]); 
				    count++;
				    if(count == 16)
				    {
					    count = 0;
					    printf("\n");
				    }
			    }
            }

			if(lRecLen <= 0) {	/* if sock errer has detected, then exit  */

				goto ExitPoint;
			}
			lRecvSize += lRecLen;			/* add read bytes */
		}

		/* Check Terminator EOT  */
		if (pPacketBuff[lPacketSize - 1] != BCAP_EOT) {	
			goto ExitPoint;
		}

	}

	count = 0;
	if(m_show_debug_packet)
    {
        printf("\n--------------------------------\n");
    }

	return pPacketBuff;

ExitPoint:
	if (pPacketBuff != NULL) { 
		free(pPacketBuff);
		pPacketBuff = NULL;
	}
	return NULL;



	return 0;
}
