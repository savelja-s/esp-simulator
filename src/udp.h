#include "AsyncUDP.h"
#include <ArduinoJson.h>

#define LISTEN_PORT 8888
#define SEND_PORT 49152

// IPAddress addr(192, 168, 0, 29);
IPAddress addr(192, 168, 31, 218);

AsyncUDP udp;

void parsePacket(AsyncUDPPacket packet)
{
    // Выводи в последовательный порт все полученные данные
    Serial.write(packet.data(), packet.length());
    Serial.println();
    DynamicJsonDocument doc(packet.length());
    deserializeJson(doc, packet.data());
    if (doc.containsKey("soundID") && doc.containsKey("status"))
    {
        String status = doc["status"];
        String soundID = doc["soundID"];
        Serial.println("Sound " + status + " with ID " + soundID + ".Need add logic");
    }
    packet.println("OK");
    packet.flush();
}
String create_quanternion_json(float w, float x, float y, float z)
{
    DynamicJsonDocument doc(1024);
    doc["q.w"] = w;
    doc["q.x"] = x;
    doc["q.y"] = y;
    doc["q.z"] = z;
    String output;
    serializeJson(doc, output);
    return output;
}

std::string createJsonButton(int buttonPIN, int status)
{
    DynamicJsonDocument doc(1024);
    doc["buttonId"] = buttonPIN;
    doc["status"] = status;

    std::string output;
    serializeJson(doc, output);
    return output;
}

std::string createJsonEncoder(int encoderPIN, int value)
{
    DynamicJsonDocument doc(1024);
    doc["encoderId"] = encoderPIN;
    doc["value"] = value;

    std::string output;
    serializeJson(doc, output);
    return output;
}
void sendByUDP(uint8_t type, std::string body)
{
    Packet packet;
    packet.Create(type, body);
    udp.connect(addr, SEND_PORT);
    udp.print(packet.Serialize().c_str());
    udp.close();
}

void initUDP()
{
    if (udp.listen(LISTEN_PORT))
    {
        Serial.print("[UDP] Listen IP:");
        Serial.print(WiFi.localIP().toString().c_str());
        Serial.print(" PORT:");
        Serial.println(LISTEN_PORT);
        udp.onPacket(parsePacket);
    }
}