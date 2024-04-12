/*********************************************************************************
 * ESP-Serial-Bridge
 *
 * Simple WiFi Serial Bridge for Espressif microcontrollers
 *
 * Forked from https://github.com/AlphaLima/ESP32-Serial-Bridge
 *
 * Added compatibility for ESP8266, WiFi reconnect on failure, and mDNS
 *discovery.
 *
 * Note: ESP8266 is limited to 115200 baud and may be somewhat unreliable in
 *       this application.
 *
 *   -- Yuri - Aug 2021
 *
 * Fixed compatibility with Arduino framework 2.0 -- Yuri - Apr 2023
 * Implemented UDP + bug fixes                    -- Yuri - Apr 2023
 *
 * Disclaimer: Don't use for life support systems or any other situation
 * where system failure may affect user or environmental safety.
 *********************************************************************************/

#include <Arduino.h>

#define ESP32

#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"

#include <AsyncUDP.h>

AsyncUDP udp0, udp1, udp2;
AsyncUDP *udp[NUM_COM] = {&udp0, &udp1, &udp2};
uint16_t udp_port[NUM_COM] = {SERIAL0_UDP_PORT, SERIAL1_UDP_PORT,
                              SERIAL2_UDP_PORT};

HardwareSerial Serial_one(1);
HardwareSerial Serial_two(2);
HardwareSerial *COM[NUM_COM] = {&Serial, &Serial_one, &Serial_two};

uint8_t buf[NUM_COM][BUFFERSIZE];

#ifdef MODE_STA
void WiFiStationDisconnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
    debug.print("WiFi disconnected: ");
}

void WiFiStationConnected(WiFiEvent_t event, WiFiEventInfo_t info)
{
    debug.println("WiFi connected.");
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
    debug.print("Got IP address: ");
    debug.println(WiFi.localIP());

    if (!MDNS.begin(HOSTNAME))
    {
        debug.println("Error starting mDNS");
    }
    else
    {
        debug.print("Started mDNS, discoverable as: ");
        debug.println(HOSTNAME);
        // MDNS.addService("_telnet", "_tcp", SERIAL0_TCP_PORT); // We're not running a telnet server.
    }
}
#endif

void setup()
{
#ifdef DEBUG
    debug.begin(115200);
#endif

    WiFi.disconnect(true); // delete old config (attempt true, true)
    delay(500);

    // Set up serial ports
    COM[0]->begin(UART_BAUD0, SERIAL_PARAM0, SERIAL0_RXPIN, SERIAL0_TXPIN);
    COM[1]->begin(UART_BAUD1, SERIAL_PARAM1, SERIAL1_RXPIN, SERIAL1_TXPIN);
    COM[2]->begin(UART_BAUD2, SERIAL_PARAM2, SERIAL2_RXPIN, SERIAL2_TXPIN);


    debug.print("\n\nWiFi serial bridge ");
    debug.println(VERSION);

#ifdef MODE_AP
    debug.println("Open ESP Access Point Mode");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(SSID, PASSWD);                        // configure SSID and password for softAP
    delay(2000);                                      // VERY IMPORTANT
    WiFi.softAPConfig(STATIC_IP, STATIC_IP, NETMASK); // configure ip address for softAP
#endif

#ifdef MODE_STA
    debug.println("Open ESP Station Mode");
    WiFi.mode(WIFI_STA);
    WiFi.onEvent(WiFiGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP);
    WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
    

    WiFi.begin(SSID, PASSWD);
    debug.print("Connecting to: ");
    debug.print(SSID);
    debug.print("..");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        debug.print(".");
    }
    WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
#endif

    for (uint16_t num = 0; num < NUM_COM; num++)
    {
        if (udp[num]->listen(udp_port[num]))
        {
            debug.printf("Listening on UDP port %d\n", udp_port[num]);
            udp[num]->onPacket([num](AsyncUDPPacket packet)
                               { COM[num]->write(packet.data(), packet.length()); });
        }
    }

#ifdef BATTERY_SAVER
    esp_err_t esp_wifi_set_max_tx_power(50); // lower WiFi Power
#endif
}

void loop()
{
    for (int num = 0; num < NUM_COM; num++)
    {
        if (COM[num] != NULL && COM[num]->available())
        {
            int readCount = COM[num]->read(&buf[num][0], BUFFERSIZE); // Read up to BUFFERSIZE bytes from the serial device.
            if (readCount == 0)
                continue;

            udp[num]->broadcastTo(buf[num], readCount, udp_port[num]); // Send out the UDP message.
        }
    }
}
