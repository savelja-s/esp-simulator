// #include "AsyncUDP.h"

// const uint16_t port = 49152;
// const uint16_t port2 = 49153;
// IPAddress addr(192, 168, 0, 29);

// AsyncUDP udp;
// AsyncUDP udp2;

// void parsePacket(AsyncUDPPacket packet)
// {
//     // Выводи в последовательный порт все полученные данные
//     Serial.write(packet.data(), packet.length());
//     Serial.println();
// }

// void begin_udp(AsyncUDP udp, uint16_t port)
// {
//     // Если удалось подключится по UDP
//     if (udp.connect(addr, port))
//     {
//         Serial.println(String(port)+"UDP подключён");
//         // вызываем callback функцию при получении пакета
//         udp.onPacket(parsePacket);
//     }
//     // Если подключение не удалось
//     else
//     {
//         Serial.println(String(port)+"UDP не подключён");
//         // Входим в бесконечный цикл
//         while (1)
//         {
//             delay(1000);
//         }
//     }
// }

// void sendByUDP(String msg)
// {
//    udp.print(msg); 
// }