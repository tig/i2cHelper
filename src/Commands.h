#pragma once
#include <Ethernet.h>
#include <Shell.h>

#ifdef ARDUINO_SAMD_MKRZERO
#define MAC_ADDRESS 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x40
#define STATIC_IP   192, 168, 1, 160
#else
#define MAC_ADDRESS 0x90, 0xA2, 0xDA, 0x0E, 0xFE, 0x41
#define STATIC_IP   192, 168, 1, 161
#endif
#define TCP_PORT 80

/**
 * @brief Singleton class representing the exposed Commands (API). The API is nominally
 * exposed with Shell (both Ethernet/Telnet and Serial), but can also be exposed via a keypad or other input in 
 * parallel.
 */
class Commands {
 public:
  // Initilaize the REST API
  bool begin();

  // Call every iteration of loop()
  void handle();

  void logCommand(const __FlashStringHelper* name, int argc, const ShellArguments &argv);

  Shell _serialShell;
  Shell _telnetShell;
  char shellPrompt [2] = ">";

 private:

 // Networking
  byte _macaddress[6];
  IPAddress _ip;  // static address
  EthernetServer _shellServer;
  EthernetClient _shellClient;
  bool _haveClient = false;
  bool _isInitialized = false;

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

extern Commands &Cmds;