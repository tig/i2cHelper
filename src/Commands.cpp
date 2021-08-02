#include <Arduino.h>
#include <ArduinoLog.h>
#include <assert.h>
#include <Shell.h>

//#include "main.h"
#include "Commands.h"
#include "i2c.h"
#include "Sensors.h"
#include "Relay.h"

void Commands::logCommand(const __FlashStringHelper *name, int argc, const ShellArguments &argv) {
  Log.trace(F(">>>>> Command: %S"), name);
  for (int i = 1; i <= argc; i++) {
    Log.trace(F(" %s"), argv[i]);
  }
  Log.traceln(" <<<<<");
}

// Defines functions to be exposed to the rest API
bool Commands::begin() {
  if (_isInitialized) return true;

  for (uint16_t client = 0; client < MAX_CLIENTS; client++) {
    _shellClient[client] = nullptr;
  }

  bool success = true;

  _serialShell.setPrompt(shellPrompt);
  _serialShell.begin(Serial, 20, Terminal::Mode::Serial);

  return _isInitialized = success;
}

/**
 * @brief Initializes the Ethernet/Wifi stack and starts the TCP/IP Server
 */
bool Commands::beginServer(const char *ssid, const char *pwd) {
#ifdef USE_WIFI
  if (ssid != nullptr && pwd != nullptr) {
    // Only allow connect via command if we've already tried once
    _autoConnectWifi = false;

    Log.noticeln(F("Setting up TCP/IP Server on Wifi"));
    Log.notice(F("  Trying WiFi '%s', '******' "), ssid);

    WiFi.disconnect(true);
    //delay(1000);
    WiFi.begin(ssid, pwd);

    pinMode(13, OUTPUT);
    int ledState = 0;
    int tries = 15;
    while (tries && WiFi.status() != WL_CONNECTED) {
      // Blink LED while we're connecting:
      digitalWrite(13, ledState);
      ledState = (ledState + 1) % 2;  // Flip ledState
      delay(500);
      Log.notice(F("."));
      tries--;
    }
    digitalWrite(13, 0);

    if (WiFi.status() == WL_CONNECTED) {
      Log.noticeln(F("\n  Connected. IP Address (DHCP): %p"), WiFi.localIP());
    } else {
      Log.noticeln(F("\n  ERROR: WiFi did not connect; Status = %d"), WiFi.status());
    }
  }
#endif

#ifdef USE_ETHERNET
  if (Ethernet.linkStatus() != LinkON) {
    _netWasConnected = false;
    Log.errorln(F("  ERROR: Ethernet is not connected"));

    //success = false;
  } else {
    Log.noticeln(F("Setting up TCP/IP Server on Ethernet"));

    _netWasConnected = true;

    // Start the Ethernet connection and the server
    Ethernet.init(5);  // MKR ETH shield (pin 5)
    if (Ethernet.begin(_macaddress, 2000) == 0) {
      // try to congifure using _ip address instead of DHCP:
      //Log.noticeln(F("  Trying to use static IP: %p"), _ip);
      Ethernet.begin(_macaddress, _ip);
      Log.noticeln(F("  IP Address (STATIC): %p"), Ethernet.localIP());
    } else {
      Log.noticeln(F("  IP Address (DHCP): %p"), Ethernet.localIP());
    }
  }
#endif

#ifdef USE_WIFI
  if (WiFi.status() == WL_CONNECTED) {
    _ip = WiFi.localIP();
    _shellServer.begin(_tcpPort);
#endif
#ifdef USE_ETHERNET
    if (Ethernet.linkStatus() == LinkON) {
      _ip = Ethernet.localIP();
#endif
      Log.noticeln(F("Connect to %p:%d. Type '?` to get a list of commands."), _ip, _tcpPort);
    }
    return true;
  }

  void Commands::handle() {
#ifdef USE_ETHERNET
    if (Ethernet.linkStatus() == LinkON && _netWasConnected == false) {
      Log.noticeln(F("Ethernet cable detected"));
      delay(2000);
      beginServer();
    } else if (Ethernet.linkStatus() == LinkOFF && _netWasConnected == true) {
      Log.errorln(F("ERROR: Ethernet cable has been unplugged"));
      _netWasConnected = false;
    }

    if (Ethernet.linkStatus() == LinkON) {
      // Maintain the DHCP lease over time.
      //Ethernet.maintain();
    }
#endif

#ifdef USE_WIFI
    if (WiFi.status() == WL_CONNECTED) {
      // Maintain the DHCP lease over time.
      //WiFi.maintain();
    } else if (_autoConnectWifi) {
      beginServer();
    }
#endif

    // Handle new/disconnecting clients.
#ifdef USE_WIFI
    WiFiClient newClient = _shellServer.available();
#endif
#ifdef USE_ETHERNET
    EthernetClient newClient = _shellServer.accept();
#endif
    if (newClient.connected()) {
      uint16_t client;
      for (client = 0; client < MAX_CLIENTS; client++) {
        if (_shellClient[client] == nullptr) {

#ifdef USE_WIFI
          _shellClient[client] = new WiFiClient(newClient);
#endif
#ifdef USE_ETHERNET
          _shellClient[client] = new EthernetClient(newClient);
#endif
         
          Log.noticeln(F("Client #%d at %p is attempting to connect to port %d."), client,
              _shellClient[client]->remoteIP(), _shellClient[client]->localPort());

          if (_telnetShell[client].begin(*_shellClient[client], 20, Terminal::Mode::Serial)) {
            Log.noticeln(F("  Client %p connected to port %d."),
                _shellClient[client]->remoteIP(), _shellClient[client]->localPort());
            _telnetShell[client].setEcho(true);
            _telnetShell[client].setPrompt(shellPrompt);
          } else {
            Log.noticeln(F("  Client connect attempt failed. Shell::begin returned false (out of memory?)."));
          }
          break;
        }
      }
      if (client == MAX_CLIENTS) {
        newClient.stop();
      }
    } else {
      for (uint16_t client = 0; client < MAX_CLIENTS; client++) {
        if (_shellClient[client] != nullptr && !_shellClient[client]->connected()) {
          // The client has been disconnected. Shut down the shell.
          // TODO: if this client was logging, set back to Serial
          _telnetShell[client].end();

          Log.noticeln(F("Client %d has been disconnected from port."), client);
          // Now shut down the client
          _shellClient[client]->stop();
          delete _shellClient[client];
          _shellClient[client] = nullptr;
        } else if (_shellClient[client] != nullptr) {
          // Perform periodic shell processing on the active client.
          _telnetShell[client].loop();
        }
      }
    }

    if (Serial.available()) {
      _serialShell.loop();
    }
  }

  Commands &Cmds = Commands::getInstance();