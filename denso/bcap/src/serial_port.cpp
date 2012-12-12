#include <bcap/serial_port.h>

/*!
 * \brief report an error
 * \param defaultError error number
 */
int SerialPort::Error(int defaultError)
{
  ROS_INFO("ERROR: %d (errno=%d '%s')", defaultError, errno, strerror(errno));
  return defaultError;
}

/*!
 * \brief open serial comms
 * \param device_name name of the serial device, eg. /dev/ttyUSB0
 * \return 0 on success, or an error value
 */
int SerialPort::Open(std::string device_name)
{
  SerialHandle = open(device_name.c_str(), O_RDWR | O_NOCTTY);
  if (SerialHandle < 0)
  {
    switch (errno)
    {
      default:
        return Error(ErrorInvalidPort);
    }
  }

  return 0;
}

/*!
 * \brief initialise the serial comms parameters
 * \param baud baud rate
 * \param inDataBits number of data bits
 * \param inStopBits number of stop bits
 * \param inParity parity
 * \return zero on success, or an error value
 */
int SerialPort::Initialise(unsigned baud, unsigned inDataBits, unsigned inStopBits, unsigned inParity)
{
  switch (baud)
  {
    case 1200:
      baud = B1200;
      break;
    case 2400:
      baud = B2400;
      break;
    case 4800:
      baud = B4800;
      break;
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    case 230400:
      baud = B230400;
      break;
    default:
      return ErrorInvalidSettings;
  }

  unsigned int parity = 0;
  switch (inParity)
  {
    case PARITY_NONE:
    {
      parity = 0;
      break;
    }
    case PARITY_EVEN:
    {
      parity = PARENB;
      break;
    }
    case PARITY_ODD:
    {
      parity = PARENB | PARODD;
      break;
    }
  }

  unsigned int dataBits = 0;
  switch (inDataBits)
  {
    case 5:
    {
      dataBits = CS5;
      break;
    }
    case 6:
    {
      dataBits = CS6;
      break;
    }
    case 7:
    {
      dataBits = CS7;
      break;
    }
    case 8:
    {
      dataBits = CS8;
      break;
    }
  }

  unsigned int stopBits = 0;
  switch (inStopBits)
  {
    case 1:
    {
      stopBits = 0;
      break;
    }
    case 2:
    {
      stopBits = CSTOPB;
      break;
    }
  }

  struct termios tio;
  memset(&tio, 0, sizeof(tio));
  tio.c_cflag = baud | parity | dataBits | stopBits | CLOCAL | CREAD;
  tio.c_iflag = IGNPAR;
  tio.c_oflag = 0;
  tio.c_lflag = 0; // set input mode (non-canonical, no echo,...)
  tio.c_cc[VTIME] = 0; // inter-character timer unused
  tio.c_cc[VMIN] = 0; // don't block if no chars available

  tcflush(SerialHandle, TCIFLUSH);
  if (tcsetattr(SerialHandle, TCSANOW, &tio) < 0)
    return ErrorInvalidSettings;

  return 0;
}

/*!
 * \brief sends data to the serial device
 * \param data data to be sent
 * \param size number of bytes
 * \param timeout timeout in milliseconds
 * \return number of bytes written
 */
int SerialPort::Out(const uint8_t* data, size_t size, unsigned timeout)
{
  int bytes = 0;
  for (;;)
  {
    // read...
    bytes = write(SerialHandle, data, size);
    if (bytes > 0)
      break; // end if any data was received
    if (bytes < 0)
      return Error(ErrorTransmitError);
    if (timeout == 0)
      break; // end if we have timed out

    // wait for 10 milliseconds...
    unsigned sleep = 10;
    if (sleep > timeout)
      sleep = timeout;
    timeout -= sleep;
    usleep(sleep * 1000);
  }

  return bytes;
}

/*!
 * \brief reads data from the serial device
 * \param data array into which to read the data
 * \param maxSize maximumnumber of bytes to read
 * \param timeout timeout in milliseconds
 * \return number of bytes read
 */
int SerialPort::In(uint8_t* data, size_t maxSize, unsigned timeout)
{
  int bytes = 0;
  for (;;)
  {
    // read...
    bytes = read(SerialHandle, data, maxSize);
    if (bytes > 0)
      break; // end if any data was received
    if (bytes < 0)
      return Error(ErrorReceiveError);
    if (timeout == 0)
      break; // end if we have timed out

    // wait for 10 milliseconds...
    unsigned sleep = 10;
    if (sleep > timeout)
      sleep = timeout;
    timeout -= sleep;
    usleep(sleep * 1000);
  }

  return bytes;
}

/*!
 * \brief close the serial port
 */
void SerialPort::Close()
{
  if (SerialHandle)
  {
    close(SerialHandle);
    SerialHandle = 0;
  }
}

/*!
 * \brief destructor
 */
SerialPort::~SerialPort()
{
  Close();
}

