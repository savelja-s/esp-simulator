#include <stdint.h>
#include <string>
#include "packet.h" 

int main() {
    
    uint8_t button = 23;
    std::string body = "{ \"PressedButtonId\": 1 }";

     Packet packet1;
     packet1.Create(button, body);

     Packet packet2;
     packet2.DeSerialize(packet1.Serialize());


}