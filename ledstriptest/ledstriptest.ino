#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <FastLED.h>
#include <Wire.h>

#define PIN        6
#define NUMPIXELS  461

// MAC address for your W5500 (needs to be unique in your network)
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
 
// IP address of your Arduino (adjust as needed)
IPAddress ip(192, 168, 0, 2);
 
// Server IP and port to connect to
IPAddress server(192, 168, 0, 1); // IP of the server (192.168.0.1)
unsigned int port = 50000;        // Port to connect to (50000)
 
// Create a UDP object
EthernetUDP Udp;

CRGB leds[NUMPIXELS];
bool isRed = true;

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
 
  // Start Ethernet
  Ethernet.init(8);
  Ethernet.begin(mac, ip);
 
  // Allow the Ethernet shield some time to initialize
  delay(1000);
  Serial.println("Starting UDP connection...");
 
  // Start the UDP connection on a local port
  Udp.begin(port);
  Serial.println("UDP connection started.");
  // Send an initial message to the server
}

void loop() {

  /*
  if (interuptThrown) {
    Lees de sensor data uit
    interutThrown = false;
    sensordatapresent = true;
  }
  */

  // if (sensordataPresent) {
    fastLedUpdate();
    sendPacket("Hello, server!");
  //}
}

void fastLedUpdate() {
  FastLED.addLeds<NEOPIXEL, PIN>(leds, NUMPIXELS);
  if (isRed) {
    fill_solid(leds, NUMPIXELS, CRGB::Blue);
  } else {
    fill_solid(leds, NUMPIXELS, CRGB::Red);
  }
  FastLED.show();
  isRed = !isRed;
  delay(1000);
}

void sendPacket(const char* message) {
  Udp.beginPacket(server, port);
  Udp.write(message);
  Udp.endPacket();
}
