#include "../include/covid19_communication_client/ctcp_packet.h"

CTCP_Packet::CTCP_Packet()
{
    tcpClient = new QTcpSocket();

//    if(tcpClient->isOpen())
//    {
//        tcpClient->flush();
//        tcpClient->reset();
//        tcpClient->disconnectFromHost();
//        tcpClient->close();
//    }
//    tcpClient->reset();

    // Initialize Variables
    encodeIndex = 0;
    for(int i=0; i<TX_BUFFER_SIZE; i++)
    {
        txBuffer[i] = 0;
    }
}

CTCP_Packet::~CTCP_Packet()
{
    if(tcpClient->isOpen())
    {
        tcpClient->disconnectFromHost();
        tcpClient->close();
        tcpClient->abort();
    }
}


bool CTCP_Packet::connect()
{
    if(tcpClient->isOpen())
    {
        tcpClient->flush();
        tcpClient->reset();
        tcpClient->disconnectFromHost();
        tcpClient->close();
    }
    tcpClient->flush();
    tcpClient->connectToHost(STEP_PC_IP, STEP_PC_TCPIP_PORT);
    //tcpClient->connectToHost(QHostAddress::LocalHost, STEP_PC_TCPIP_PORT);

    if(tcpClient->waitForConnected(10000))
    {
        qDebug() << "Connected!";
        return true;
    }
    else
    {
        qDebug() << "Not Connected!";
        return false;
    }
}

/*
template <typename T>
uint8_t CTCP_Packet::getDataType(std::vector<T>& vec)
{
    uint8_t DataType = -1;

    if(typeid(char) == typeid(vec.at(0))) DataType = CHAR;
    else if(typeid(char16_t) == typeid(vec.at(0))) DataType = UCHAR16_T;
    else if(typeid(bool) == typeid(vec.at(0))) DataType = BOOL;
    else if(typeid(int8_t) == typeid(vec.at(0))) DataType = INT8_T;
    else if(typeid(uint8_t) == typeid(vec.at(0))) DataType = UINT8_T;
    else if(typeid(int16_t) == typeid(vec.at(0))) DataType = INT16_T;
    else if(typeid(int32_t) == typeid(vec.at(0))) DataType = INT32_T;
    else if(typeid(int64_t) == typeid(vec.at(0))) DataType = INT64_T;
    else if(typeid(int) == typeid(vec.at(0))) DataType = INT;
    else if(typeid(float) == typeid(vec.at(0))) DataType = FLOAT;
    else if(typeid(double) == typeid(vec.at(0))) DataType = DOUBLE;

    return DataType;
}

uint8_t CTCP_Packet::getDataSize(uint8_t dataPacketType)
{
    uint8_t dataSize = 0;

    if(dataPacketType == CHAR) dataSize = sizeof(char);
    else if(dataPacketType == UCHAR16_T) dataSize = sizeof(char16_t);
    else if(dataPacketType == BOOL) dataSize = sizeof(bool);
    else if(dataPacketType == INT8_T) dataSize = sizeof(int8_t);
    else if(dataPacketType == UINT8_T) dataSize = sizeof(uint8_t);
    else if(dataPacketType == INT16_T) dataSize = sizeof(int16_t);
    else if(dataPacketType == INT32_T) dataSize = sizeof(int32_t);
    else if(dataPacketType == INT64_T) dataSize = sizeof(int64_t);
    else if(dataPacketType == INT) dataSize = sizeof(int);
    else if(dataPacketType == FLOAT) dataSize = sizeof(float);
    else if(dataPacketType == DOUBLE) dataSize = sizeof(double);

    return dataSize;
}
*/


void CTCP_Packet::setCommandHeader(uint16_t header)
{
    // Initialize Variable
    encodeIndex = 0;

    for(int i=0; i<TX_BUFFER_SIZE; i++)
    {
        txBuffer[i] = 0;
    }

    // Packet Header 1
    txBuffer[encodeIndex] = 13;
    encodeIndex++;

    // Packet Header 2
    txBuffer[encodeIndex] = 10;
    encodeIndex++;

//    // Packet Header 3
//    txBuffer[encodeIndex] = 255;
//    encodeIndex++;

    // Command Header
    memcpy(&txBuffer[encodeIndex], &header, sizeof(header));
    encodeIndex = encodeIndex + sizeof(header);

}

void CTCP_Packet::encode(std::vector<float> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();
}


void CTCP_Packet::encode(std::vector<double> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();
}

void CTCP_Packet::encode(std::vector<char> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();
}
void CTCP_Packet::encode(std::vector<uchar> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();
}
void CTCP_Packet::encode(std::vector<char16_t> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();
}

//void CTCP_Packet::encode(std::vector<bool> &vec)
//{
//    uint16_t DataAmount = vec.size();

//    if(DataAmount <= 0)
//    {
//        encodeIndex = 0;
//        return;
//    }

//    for(int i=0; i<DataAmount; i++)
//    {
//        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
//        encodeIndex = encodeIndex + sizeof(vec.at(i));
//    }

//    vec.clear();
//}

void CTCP_Packet::encode(std::vector<int8_t> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }
    vec.clear();
}

void CTCP_Packet::encode(std::vector<int16_t> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();

}
void CTCP_Packet::encode(std::vector<uint16_t> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();

}
void CTCP_Packet::encode(std::vector<int> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();

}
void CTCP_Packet::encode(std::vector<uint32_t> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();

}
void CTCP_Packet::encode(std::vector<int64_t> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();

}
void CTCP_Packet::encode(std::vector<uint64_t> &vec)
{
    uint16_t DataAmount = vec.size();

    if(DataAmount <= 0)
    {
        encodeIndex = 0;
        return;
    }

    for(int i=0; i<DataAmount; i++)
    {
        memcpy(&txBuffer[encodeIndex], &vec.at(i), sizeof(vec.at(i)));
        encodeIndex = encodeIndex + sizeof(vec.at(i));
    }

    vec.clear();

}


QByteArray IntToArray(qint32 source) // Use qint32 to ensure that the number have 4 bytes
{
   // Avoid use of cast, this is the Qt way to serialize objects
   QByteArray temp;
   QDataStream data(&temp, QIODevice::ReadWrite);
   data << source;
   return temp;
}

void CTCP_Packet::sendPacket()
{
   ///////////////////////////////////////////////////////
   ///////////////////////////////////////////////////////
   // Packet Header 1,
   // Packet Header 2,
   // Command Header,
   // Data Amount,
   // Data Type,
   // Data
   ///////////////////////////////////////////////////////
   ///////////////////////////////////////////////////////

   tcpClient->flush();

   if(tcpClient->isWritable())
   {
       tcpClient->write(IntToArray(encodeIndex));
       tcpClient->write((char*)txBuffer, encodeIndex);
   }
}





