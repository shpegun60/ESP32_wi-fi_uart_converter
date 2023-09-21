#include "TcpClient.hpp"


TcpClient::TcpClient()
{
    //m_buffer = new uint8_t[150];
}

void TcpClient::proceed()
{
    int len  = m_client.read(m_tmp, 10);
    
    //return;
    for (int  i = 0; i < len; ++i) {
        auto ch = m_tmp[i];

        if (m_triggerSB) {
            if(ch == m_startByte) { //{SB}{SB} -> {SB}
                _proceedByte(ch, false);
            } else { //{SB}{!SB} -> {SB} and newframe
                _proceedByte(ch, true);
            }
            m_triggerSB = false;
        } else if (ch == m_startByte) { //{!SB}{SB} -> set flag and skip step
            m_triggerSB = true;
        } else { //{!SB}{!SB} -> {!SB}
            _proceedByte(ch, false);
        }
    }
}


#ifdef CLIENT_AUTO
int TcpClient::clientAutoProceedNonBlock(unsigned int timeMs, const uint16_t port, const char * host)
{
    switch (clientAutoState)
    {
        case 0:
            if (connected()) {
                proceed();
                return CLIENT_OK;
            }
            ++clientAutoState;
            return CLIENT_TRY_CONNECT;
            break;

        case 1:
            if (connect(host, port)) {
                clientAutoState = 0;
                return CLIENT_CONNECTED;
            }
            ++clientAutoState;
            clientAutolastTime = timeMs;
            break;

        case 2:
            if((timeMs - clientAutolastTime) > CLIENT_TIMEOUT) {
                clientAutoState = 1;
                clientAutolastTime = timeMs;
                return CLIENT_TRY_CONNECT;
            }
            break;

        default:
            clientAutoState = 1;
            break;
    }

    return CLIENT_ERROR_CONNECTION;
}

#endif /* CLIENT_AUTO */




void TcpClient::_proceedByte(uint8_t ch, bool newFrame)
{
    //Serial.print("PROCEED BYTE: "); Serial.print(ch); Serial.print("  ");Serial.print(newFrame); Serial.println("");
    if (newFrame) {
        m_frameCrc = 0xFF;
        m_receivePos = 0;
    }

    if (m_receivePos == 0) {
        m_receivePackLen = ch;

        if (m_receivePackLen > m_startByte) {
            m_receivePackLen -= 1;
        }
    } else if ((m_receivePos - 1) < m_receivePackLen) {
        m_recBuffer[m_receivePos-1] = ch;
    } else if ((m_receivePos - 1) == m_receivePackLen && m_frameCrc == ch) {
            _proceedPack();
    } else {
        return;
    }

    m_receivePos++;
    m_frameCrc =  _proceedCrc(m_frameCrc, ch);
}

void TcpClient::_proceedPack() 
{
    //Serial.println("PACK received: ");
    
    if (m_receivePackLen > 0) {
        auto s = m_handlers.find(m_recBuffer[0]);
        if (s != m_handlers.end()) {
            s->second(m_receivePackLen - 1, (m_recBuffer+1));
        }
    }
}

void TcpClient::on(uint8_t cmd, std::function<void(int len, uint8_t*)> foo)
{
    m_handlers.insert({cmd, foo});
}

void TcpClient::write(int len, unsigned char *ptr)
{
    char crc = static_cast<char>(0xFF);
    m_sendBuffer[0] = m_startByte;
    m_sendBuffer[1] = len >= m_startByte ? len + 1 : len;

    int pos = 2;
    auto addByte = [&](uint8_t b) {
        crc = _proceedCrc(crc, b);
        m_sendBuffer[pos++] = b;
        if (b == m_startByte) {
            m_sendBuffer[pos++] = b;
        }
    };

    for (uint8_t i = 0; i < len; i++) {
        addByte(ptr[i]);
    }
    addByte(crc);

    m_client.write(m_sendBuffer, pos);
}

uint8_t TcpClient::_proceedCrc(uint8_t crc, uint8_t ch) {
    crc ^= ch;
    for (int i = 0; i < 8; i++)
        crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    return crc;
}


#undef CLIENT_AUTO