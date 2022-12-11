#ifndef __PACKER_H__
#define  __PACKER_H__

#include <stdint.h>
#include <string>

class Packet {
   private:
    uint8_t         type;
    uint32_t        packet_size;
    std::string     packet_body;
  public:
    static const uint8_t PACKET_BUTTON = 1;
    static const uint8_t PACKET_SENSOR = 2;
    Packet() {};
    Packet(uint8_t type, std::string& packet_body);
    void Create(uint8_t type, std::string packet_body);
    std::string Serialize();
    void DeSerialize(std::string packet);
    ~Packet() {};
};

#endif