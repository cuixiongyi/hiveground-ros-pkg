#ifndef _b_cap_serial_client_h_
#define _b_cap_serial_client_h_

#include <bcap/bcap.h>
#include <bcap/serial_port.h>

class BCapSerial : public BCap, public SerialPort
{
public:
  BCapSerial();
  ~BCapSerial();
private:
  BCAP_HRESULT Packet_Send(BCAP_PACKET *pPacket);
  BCAP_HRESULT bCapSendAndRec(BCAP_PACKET *pSndPacket, BCAP_PACKET *pRecPacket);
  BCAP_HRESULT sendBinary(uint8_t *pBuff, uint32_t lSize);
  uint8_t *receivePacket();
};

#endif
