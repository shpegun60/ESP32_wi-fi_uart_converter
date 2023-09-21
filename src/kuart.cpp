#include "kuart.hpp"


Kuart::Kuart(int uart_nr) : 
    SerialPort(uart_nr)
{
    
}

void Kuart::begin(unsigned long baud, uint32_t config, int8_t rxPin, int8_t txPin)
{
    SerialPort.begin(baud, config, rxPin, txPin);
}

void Kuart::write(int len, unsigned char *ptr)
{
    uint8_t packLen = static_cast<uint8_t>(len >= m_startByte ? (len + 1) : len);
    uint8_t crc = _proceedCrc(0xFF, packLen);

    m_sendBuffer[0] = m_startByte;
    m_sendBuffer[1] = packLen;

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

    SerialPort.write(m_sendBuffer, pos);
}

void Kuart::on(std::function<void(int len, uint8_t*)> foo)
{
    m_handler = foo;
}

void Kuart::proceed()
{
    int len = SerialPort.read(m_tmp, 10);

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


void Kuart::_proceedByte(uint8_t ch, bool newFrame)
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
        if(m_handler) {
            m_handler(m_receivePackLen, m_recBuffer);
        }
    } else {
        return;
    }

    m_receivePos++;
    m_frameCrc =  _proceedCrc(m_frameCrc, ch);
}


uint8_t Kuart::_proceedCrc(uint8_t crc, uint8_t ch) {
    crc ^= ch;
    for (int i = 0; i < 8; i++)
        crc = crc & 0x80 ? (crc << 1) ^ 0x31 : crc << 1;
    return crc;
}