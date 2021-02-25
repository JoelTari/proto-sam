#include "eMQTT5/lib/include/Network/Clients/MQTT.hpp"
#include <iostream>

class ImplMsgReceived : public Network::Client::MessageReceived
{
  void messageReceived(const DynamicStringView &  topic,
                       const DynamicBinDataView & payload,
                       const uint16               packetIdentifier,
                       const PropertiesView &     properties)
  {
    // nada im not subscribing to anything for now??
    std::cout << "Message Received, processing...\n";

    std::cout << "A message has been processed\n";
  }
};

int main()
{
  ImplMsgReceived emptyStuff;

  Network::Client::MQTTv5 mqttobj("cppCode", &emptyStuff);

  std::cout << "Connect to broker :  " << mqttobj.connectTo("localhost", 1883)
            << "\n";

  const char * msg = "Im speaking from C++ to whomever wishes to hear !";
  //json versoin
  /* const char * msg = "{\"presence\": \"Im speaking from C++ to whomever wishes to hear !\"}"; */
  mqttobj.publish("presence", (uint8 *)msg, strlen(msg));
  mqttobj.subscribe("requestChan");

  // event loop
  while(1){
    mqttobj.eventLoop();
    usleep(10000);
  }

  return 0;
}
