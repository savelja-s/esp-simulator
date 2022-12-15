#ifndef __PACKER_H__
#define  __PACKER_H__

#include <stdint.h>
#include <string>

enum PacketType
{
  PACKET_BUTTON,
  PACKET_SENSOR,
  PACKET_HALL,
  PACKET_ENCODER
};

class Packet {
   private:
    uint8_t         type;
    uint32_t        packet_size;
    std::string     packet_body;
  public:
    Packet() {};
    Packet(uint8_t type, std::string& packet_body);
    void Create(uint8_t type, std::string packet_body);
    std::string Serialize();
    void DeSerialize(std::string packet);
    ~Packet() {};
};

#endif