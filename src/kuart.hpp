#ifndef K_UART
#define K_UART

#include <map>
#include <HardwareSerial.h>

#define K_UART_BUFF_SIZE 256

class Kuart
{
public:
    Kuart(int uart_nr);

    void begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin);
    void write(int len, unsigned char*);

    void on(std::function<void(int len, uint8_t*)>);
    void proceed();

private:

    void _proceedByte(uint8_t byte, bool newFrame);
    uint8_t _proceedCrc(uint8_t crc, uint8_t ch);

    HardwareSerial SerialPort;
    std::function<void(int len, uint8_t*)> m_handler = nullptr;
    int m_packLen = -1;


    const uint8_t m_startByte = 0x1A;
    uint8_t m_recBuffer[K_UART_BUFF_SIZE];
    uint8_t m_sendBuffer[K_UART_BUFF_SIZE];

    uint8_t m_tmp[10];

    bool m_triggerSB = false;
    uint8_t m_frameCrc = 0xFF;
    uint8_t m_receivePos = 0;
    uint16_t m_receivePackLen = 0;
};




#endif /* K_UART */