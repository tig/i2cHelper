#include <Arduino.h>
#include <ArduinoLog.h>
#include <assert.h>
#include <Ethernet.h>
#include <Shell.h>
#include <P1AM.h>

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

Commands::Commands() : _macaddress{MAC_ADDRESS}, _ip(STATIC_IP), _shellServer(23) {
}

// Defines functions to be exposed to the rest API
bool Commands::begin() {
  if (_isInitialized) return true;

  bool success = true;

  Log.noticeln(F("Setting up Ethernet and Server."));
  Log.noticeln(F("Use Putty or other telnet app to connect. Type '?` to get a list of commands."));
  // Start the Ethernet connection and the server
  Ethernet.init(5);  // MKR ETH shield (pin 5)
  if (Ethernet.begin(_macaddress, 2000) == 0) {
    // try to congifure using _ip address instead of DHCP:
    Log.noticeln(F("  Trying to use static IP: %p"), _ip);
    Ethernet.begin(_macaddress, _ip);
    Log.noticeln(F("  IP Address (STATIC): %p"), Ethernet.localIP());
  } else {
    Log.noticeln(F("  IP Address (DHCP): %p"), Ethernet.localIP());
  }

  _telnetShell.setPrompt(shellPrompt);

  _serialShell.setPrompt(shellPrompt);
  _serialShell.begin(Serial, 20, Terminal::Mode::Serial);
  _shellServer.begin();

  return _isInitialized = success;
}

void Commands::handle() {
  // Maintain the DHCP lease over time.
  Ethernet.maintain();

  // Handle new/disconnecting clients.
  if (!_haveClient) {
    // Check for new client connections.
    _shellClient = _shellServer.available();
    if (_shellClient) {
      Log.noticeln(F("Client %p is attempting to connect to port %d."), _shellClient.remoteIP(), _shellClient.localPort());
      _haveClient = true;
      if (_telnetShell.begin(_shellClient, 5, Terminal::Mode::Telnet)) {
        Log.noticeln(F("Client %p connected to port %d."), _shellClient.remoteIP(), _shellClient.localPort());
      } else {
        Log.noticeln(F("Client connect attempt failed. Shell::begin returned false (out of memory?)."));
      }
    }
  } else if (!_shellClient.connected()) {
    // The current client has been disconnected.  Shut down the shell.
    _telnetShell.end();
    Log.noticeln(F("Client %p has been disconnected from port %d."), _shellClient.remoteIP(), _shellClient.localPort());
    _shellClient.stop();
    _shellClient = EthernetClient();
    _haveClient = false;
  }

  if (Serial.available()) {
    _serialShell.loop();
  }

  // Perform periodic shell processing on the active client.
  _telnetShell.loop();
}

Commands &Cmds = Commands::getInstance();