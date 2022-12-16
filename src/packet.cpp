#include "packet.h"
#include <sstream>
#include <iomanip>
#include <Arduino.h>


Packet::Packet(uint8_t type, std::string& packet_body) {

}

void Packet::Create(uint8_t type, std::string packet_body) {
    this->type = type;
    this->packet_size = packet_body.size();
    this->packet_body = packet_body;
}

std::string Packet::Serialize() {
    std::ostringstream stringStream;

    stringStream << (int)type << std::setw(4) << packet_size << packet_body;
    std::string result(stringStream.str());

    // Serial.println(result.c_str());

    return result;
}

void Packet::DeSerialize(std::string packet) {
    std::istringstream(packet) >> this->type >> this->packet_size;
    this->packet_body = packet.substr(5, packet_size);

    Serial.println("=====================");
    Serial.println(this->type);
    Serial.println(this->packet_size);
    Serial.println(this->packet_body.c_str());
    Serial.println("=====================");
}