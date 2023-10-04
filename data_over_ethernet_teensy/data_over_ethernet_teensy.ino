#include <SPI.h>
#include <Ethernet.h>
// #include <ETH.h>

// MAC address for your Ethernet module
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };

// IP address settings
IPAddress ip(192, 168, 0, 185);      // Static IP address for ESP32 (choose a unique address)
IPAddress gateway(192, 168, 0, 1);   // Gateway IP address (your router)
IPAddress subnet(255, 255, 255, 0);  // Subnet mask (same as your laptop)
IPAddress dns(8, 8, 8, 8);           // DNS server (e.g., Google DNS)


IPAddress serverIp(192, 168, 0, 140);  // IP address of your PV server
int serverPort = 53;                // Port of your PV server

uint8_t count[100];
long timer1, timer2;

EthernetClient client;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Setup started");
  // Start Ethernet connection
  Ethernet.init(10);
  // Start Ethernet connection
  bool i = Ethernet.begin(mac, ip, dns, gateway, subnet);
  Serial.println(i);
  // Wait for Ethernet to initialize
  delay(1000);
  for(int i=0; i<100; i++)
  {
    count[i]=125;
  }
  Serial.println("Setup done");
}

void loop() {

  // Serial.println("loop started");
  //     // client.println("new data incoming!!");
  //     client.write((uint8_t*)&count, sizeof(count));
  //     Serial.println(count);
  // Check if Ethernet is connected and available

  delay(1000);
  Serial.println(Ethernet.linkStatus());
  if (Ethernet.linkStatus() == LinkON) {
    if (!client.connected()) {
      // Connect to the PV server
      if (client.connect(serverIp, serverPort)) {
        Serial.println("Connected to PC server");
        // client.println("Hello, PC server!");  // Send your data here
        // delay(2000);
        // timer1 = micros();
        // for (int i = 0; i < 1000; i++) {
          client.write((uint8_t*)&count, 100);
        // }
        // timer2 = micros();
        // delay(3000);
        // long diff = timer2 - timer1;
        // Serial.println(diff);
        // while (1)
        //   ;
      } else {
        Serial.println("Connection failed");
      }
    }

    // else {
    // }

    // Check for data from the PV server
    // if (client.available()) {
    //   char c = client.read();
    //   Serial.print(c);  // Print received data to Serial Monitor
    // }
    // } else {
    //   Serial.println("Ethernet not connected");
  } else {
    Serial.println("Ethernet not connected");
  }
  // Serial.println("loop end!!");
  // delay(1000);
  // count++;
}



// #include <Ethernet.h>

// byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
// IPAddress ip(192, 168, 1, 100);
// EthernetServer server(80);

// void setup() {
//   Ethernet.begin(mac, ip);
//   server.begin();
// }

// void loop() {
//   EthernetClient client = server.available();

//   if (client) {
//     if (client.connected()) {
//       client.println("HTTP/1.1 200 OK");
//       client.println("Content-Type: text/html");
//       client.println();
//       client.println("<html><body><h1>Hello, World!</h1></body></html>");
//       client.stop();
//     }
//   }
// }
