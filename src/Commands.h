#pragma once
#include <Ethernet.h>
#include <Shell.h>

// P1AM
#ifdef ARDUINO_ARCH_SAMD
#define MAC_ADDRESS 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40
#define STATIC_IP   192, 168, 1, 160
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
class Commands {
 public:
  // Initilaize the REST API
  bool begin(bool raw = false);

  // Call every iteration of loop()
  void handle();

  void logCommand(const __FlashStringHelper* name, int argc, const ShellArguments& argv);

  Shell _serialShell;
  Shell _telnetShell;
  char shellPrompt[2] = ">";

  bool getRaw() { return _raw; }
  void setRaw(bool raw) {
    _raw = raw;
    if (_raw) {
      _telnetShell.setEcho(false);
      _telnetShell.setPrompt(nullptr);
    } else {
      _telnetShell.setEcho(true);
      _telnetShell.setPrompt(shellPrompt);
    }
  }

 private:
  // Networking
  byte _macaddress[6];
  IPAddress _ip;  // static address
  EthernetServer _shellServer;
  EthernetClient _shellClient;
  bool _haveClient = false;
  bool _isInitialized = false;
  bool _raw = false;
  bool _ethernetWasConnected = false;

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
  Commands();

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