#ifndef TCP_CLIENT
#define TCP_CLIENT
//#include <functional.h>
#include <WiFiClient.h>
#include <map>

#ifndef CLIENT_AUTO
#   define CLIENT_AUTO
#endif  /*CLIENT_AUTO*/


#ifdef CLIENT_AUTO
#   define CLIENT_TRY_CONNECT 0
#   define CLIENT_CONNECTED 1
#   define CLIENT_OK 2
#   define CLIENT_ERROR_CONNECTION -1

#   define CLIENT_TIMEOUT 500U
#endif /*CLIENT_AUTO*/

#define TCP_CLIENT_BUFF_SIZE 256

class TcpClient 
{
public:
    TcpClient();
    inline bool connected() {return m_client.connected();}
    inline bool connect(const char* host, uint16_t port) { return m_client.connect(host, port); }
    void write(int len, unsigned char*);

    void on(uint8_t cmd, std::function<void(int len, uint8_t*)>);
    void proceed();

    #ifdef CLIENT_AUTO
        int clientAutoProceedNonBlock(unsigned int timeMs, const uint16_t port, const char * host);
    #endif /*CLIENT_AUTO*/

private:
#ifdef CLIENT_AUTO
    int clientAutoState = 1;
    unsigned int clientAutolastTime;
#endif /*CLIENT_AUTO*/

    void _proceedByte(uint8_t byte, bool newFrame);
    uint8_t _proceedCrc(uint8_t crc, uint8_t ch);
    void _proceedPack();

    WiFiClient m_client;
    std::map<uint8_t, std::function<void(int len, uint8_t*)>> m_handlers;
    int m_packLen = -1;
    uint8_t *m_buffer;



    const uint8_t m_startByte = 0x1A;
    uint8_t m_recBuffer[TCP_CLIENT_BUFF_SIZE];
    uint8_t m_sendBuffer[TCP_CLIENT_BUFF_SIZE];

    uint8_t m_tmp[10];

    bool m_triggerSB = false;
    uint8_t m_frameCrc = 0xFF;
    uint8_t m_receivePos = 0;
    uint16_t m_receivePackLen = 0;
};




#endif