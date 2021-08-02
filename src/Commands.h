#pragma once
#include <Shell.h>

#ifdef ARDUINO_ARCH_ESP32
#define USE_WIFI
#include <WiFi.h>
#define USE_WIFI
#undef USE_ETHERNET
#else
#define USE_ETHERNET
#endif  // ESP32

#ifdef USE_ETHERNET
#include <Ethernet.h>
#endif

// P1AM
#ifdef ARDUINO_ARCH_SAMD
#define MAC_ADDRESS 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40
#define STATIC_IP   169, 254, 0, 1
#endif

// MEGA2560
#ifdef ARDUINO_ARCH_AVR
#define MAC_ADDRESS 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x41
#define STATIC_IP   192, 168, 1, 161
#endif

// ESP32 Thing Plus
#ifdef ARDUINO_ARCH_ESP32
#define MAC_ADDRESS 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x42
#define STATIC_IP   192, 168, 1, 162
#endif

/**
 * @brief Singleton class representing the exposed Commands (API). The API is nominally
 * exposed with Shell (both Ethernet/Telnet and Serial), but can also be exposed via a keypad or other input in 
 * parallel.
 * 
 * This class leverages the ArduinoShell library. 
 */
class Commands : public Printable {
 private:
  static const uint16_t MAX_CLIENTS = 4;

 public:
  // Initialize the Commands over Serial
  bool begin();

  // Initialize the Commands interface over Ethernet / Wifi (telnet)
  bool beginServer(const char* ssid = nullptr, const char* pwd = nullptr);

  // Call every iteration of loop()
  void handle();

  void logCommand(const __FlashStringHelper* name, int argc, const ShellArguments& argv);

  Shell _serialShell;
  Shell _telnetShell[MAX_CLIENTS];
  char shellPrompt[2] = ">";

  /**
   * @brief `Printable::printTo` -    *
   * @param p
   * @return size_t
   */
  size_t printTo(Print& p) const override {
    int n = p.print(F("IP Address = "));
#ifdef USE_WIFI
    return n += p.print(WiFi.localIP());
#endif
#ifdef USE_ETHERNET
    return n += p.print(Ethernet.localIP());
#endif
  };

 //private:
  // Networking
  byte _macaddress[6];
  IPAddress _ip;
  static const uint16_t _tcpPort = 23;
#ifdef USE_WIFI
  WiFiServer _shellServer;
  WiFiClient* _shellClient[MAX_CLIENTS];
#endif
#ifdef USE_ETHERNET
  EthernetServer _shellServer = _tcpPort;
  EthernetClient* _shellClient[MAX_CLIENTS];
#endif
  bool _isInitialized = false;
#ifdef USE_ETHERNET
  bool _netWasConnected = false;
#endif
#ifdef USE_WIFI
  bool _autoConnectWifi = true;
#endif

  // =======================================================
  // Singleton support
  // https://stackoverflow.com/a/1008289/297526
 public:
  /**
   * @brief Returns the single instance object
   *
   */
  static Commands& getInstance() {
    static Commands instance;  // Guaranteed to be destroyed.
                               // Instantiated on first use.
    return instance;
  }

 private:
  // Prohibiting External Constructions
  Commands() : _macaddress{MAC_ADDRESS}, _ip(STATIC_IP) {}

  // C++ 11
  // =======
  // We can use the better technique of deleting the methods
  // we don't want.
 public:
  // This breaks printTo
  //Commands(Commands const&) = delete;
  void operator=(Commands const&) = delete;
  // =======================================================
};

extern Commands& Cmds;