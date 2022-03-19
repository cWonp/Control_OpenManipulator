#ifndef CTCP_PACKET_H
#define CTCP_PACKET_H

#include <QtNetwork>
#include <QByteArray>
#include <vector>

#define STEP_PC_TCPIP_PORT 2222

#define VISION_PC_IP "192.168.0.101"
//#define STEP_PC_IP "192.168.0.101"
#define STEP_PC_IP "192.168.0.100"

#define RX_BUFFER_SIZE 1024
#define TX_BUFFER_SIZE 1024

/*
enum DATA_PACKET_TYPE{
    CHAR = 0,
    UCHAR16_T,
    BOOL,
    INT8_T,
    UINT8_T,
    INT16_T,
    INT32_T,
    INT64_T,
    INT,
    FLOAT,
    DOUBLE
};
*/

class CTCP_Packet
{
public:
    CTCP_Packet();
    ~CTCP_Packet();

    template <class T>
    uint8_t getDataType(std::vector<T>& vec);
    uint8_t getDataSize(uint8_t dataPacketType);

    /*
    template <class T>
    void sendPacket(uint16_t header, T &vec);
    */

    //    template<class T>
    //    void encode(T &val);


    void setCommandHeader(uint16_t header);

    bool connect();

    void encode(std::vector<char> &vec);
    void encode(std::vector<uchar> &vec);
    void encode(std::vector<char16_t> &vec);
    //    void encode(std::vector<bool> &vec);
    void encode(std::vector<int8_t> &vec);
    void encode(std::vector<int16_t> &vec);
    void encode(std::vector<uint16_t> &vec);
    void encode(std::vector<int> &vec);
    void encode(std::vector<uint32_t> &vec);
    void encode(std::vector<int64_t> &vec);
    void encode(std::vector<uint64_t> &vec);
    void encode(std::vector<float> &vec);
    void encode(std::vector<double> &vec);

    void sendPacket();

    template <class T>
    void encode(T &val)
    {
        memcpy(&txBuffer[encodeIndex], &val, sizeof(val));
        encodeIndex = encodeIndex + sizeof(val);
    }

    template <class T>
    void encode_size(T &val, int size)
    {
        memcpy(&txBuffer[encodeIndex], &val, size);
        encodeIndex = encodeIndex + size;
    }


private:
    QTcpSocket *tcpClient;
    unsigned char txBuffer[TX_BUFFER_SIZE];
    int encodeIndex;
    uint16_t header;

};

#endif // CTCP_PACKET_H
