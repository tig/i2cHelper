#include <Arduino.h>
#include <ArduinoLog.h>
#include <assert.h>
#include <Ethernet.h>
#include <Shell.h>
#include <P1AM.h>

#include "main.h"
#include "Api.h"
#include "i2c.h"
#include "Sensors.h"

void Api::logCommand(const __FlashStringHelper *name, int argc, const ShellArguments &argv) {
  Log.trace(F(">>>>> Command: %S"), name);
  for (int i = 1; i <= argc; i++) {
    Log.trace(F(" %s"), argv[i]);
  }
  Log.traceln(" <<<<<");
}

Api::Api() : _macaddress{MAC_ADDRESS}, _ip(STATIC_IP), _shellServer(23) {
  cmdStatus = ShellCommandClass(status, "Prints status - [system|door|actuator|slider [fwd|rwd]]", {
    Api::getInstance().logCommand(command->name, argc, argv);

    // char szBuf[50];
    // if (argc == 1 || (argc > 1 && !strcmp_P(argv[1], PSTR("system")))) {
    //   strcpy_P(szBuf, System::getInstance().getCurrentState() ? (const char *)System::getInstance().getCurrentState()->name : PSTR("n/a"));
    //   shell.println(szBuf);
    // }

    // if (argc == 1 || (argc > 1 && !strcmp_P(argv[1], PSTR("door")))) {
    //   strcpy_P(szBuf, System::getInstance().door.getCurrentState() ? (const char *)System::getInstance().door.getCurrentState()->name : PSTR("n/a"));
    //   shell.println(szBuf);
    // }

    // if (argc == 1 || (argc > 1 && !strcmp_P(argv[1], PSTR("actuator")))) {
    //   strcpy_P(szBuf, System::getInstance().door.actuator.getCurrentState() ? (const char *)System::getInstance().door.actuator.getCurrentState()->name : PSTR("n/a"));
    //   shell.println(szBuf);
    // }

    // if (argc == 1 || (argc > 1 && !strcmp_P(argv[1], PSTR("slider")))) {
    //   if (argc == 1){
    //     strcpy_P(szBuf, System::getInstance().slider.getCurrentState() ? (const char *)System::getInstance().slider.getCurrentState()->name : PSTR("n/a"));
    //     shell.println(szBuf);
    //   }

    //   if (argc == 1 || (argc > 2 && !strcmp_P(argv[2], PSTR("fwd")))) {
    //     System::getInstance().slider.probeForwardSensor();
    //     shell.println(System::getInstance().slider.getForwardCachedDistance(), DEC);
    //   }

    //   if (argc == 1|| (argc > 2 && !strcmp_P(argv[2], PSTR("rwd")))) {
    //     System::getInstance().slider.probeRearwardSensor();
    //     shell.println(System::getInstance().slider.getRearwardCachedDistance(), DEC);
    //   }
    //   if (argc == 2) {
    //     strcpy_P(szBuf, System::getInstance().slider.getCurrentState() ? (const char *)System::getInstance().slider.getCurrentState()->name : PSTR("n/a"));
    //     shell.println(szBuf);
    //   }
    //  }
  });

  cmdLog = ShellCommandClass(log, "Sets logging - [serial*|shell] [verbose*|info|silent]", {
    Api::getInstance().logCommand(command->name, argc, argv);

    int level = LOG_LEVEL_VERBOSE;
    if (argc > 2 && !strcmp_P(argv[2], PSTR("verbose"))) {
      level = LOG_LEVEL_VERBOSE;
    } else if (argc > 2 && !strcmp_P(argv[2], PSTR("info"))) {
      level = LOG_LEVEL_INFO;
    } else if (argc > 2 && !strcmp_P(argv[2], PSTR("silent"))) {
      level = LOG_LEVEL_SILENT;
    }

    if (argc > 1 && !strcmp_P(argv[1], PSTR("serial"))) {
      Log.begin(level, &Serial, false);
    } else if (argc > 1 && !strcmp_P(argv[1], PSTR("shell"))) {
      Log.begin(level, &shell, false);
    }
  });

  cmdShell = ShellCommandClass(shell, "Configures shell - [telnet*|raw]", {
    Api::getInstance().logCommand(command->name, argc, argv);

    if (argc > 1 && !strcmp_P(argv[1], PSTR("raw"))) {
      shell.setEcho(false);
      shell.setPrompt(nullptr);
    } else if (argc > 1 && !strcmp_P(argv[1], PSTR("telnet"))) {
      shell.setEcho(true);
      shell.setPrompt(Api::getInstance().shellPrompt);
    }
  });

  cmdReboot = ShellCommandClass(reboot, "Reboots the microcontroller", {
    Api::getInstance().logCommand(command->name, argc, argv);
    Log.errorln("REBOOTING IN 1 second.");
    P1.configWD(1000, TOGGLE);
    while (1) {
      delay(5000);
    }
  });

  cmdInit = ShellCommandClass(init, "Inits and tests - [i2c|mux|fwd|rwd]", {
    Api::getInstance().logCommand(command->name, argc, argv);

    if (argc > 1 && !strcmp_P(argv[1], PSTR("i2c"))) {
      Bus.scanI2C(30);
    }
    if (argc > 1 && !strcmp_P(argv[1], PSTR("mux"))) {
      Bus.testI2CMux(30);
    }
    if (argc > 1 && !strcmp_P(argv[1], PSTR("fwd"))) {
      fwdDistanceSensor.begin();
    }
    if (argc > 1 && !strcmp_P(argv[1], PSTR("rwd"))) {
      rwdDistanceSensor.begin();
    }
  });

  cmdGet = ShellCommandClass(get, "Gets a value - [fwd|rwd]", {
    Api::getInstance().logCommand(command->name, argc, argv);

    if (argc > 1 && !strcmp_P(argv[1], PSTR("fwd"))) {
      fwdDistanceSensor.getDistance();
      Log.noticeln(fwdDistanceSensor);
      shell.println(fwdDistanceSensor._cachedDistance, DEC);
    }
    if (argc > 1 && !strcmp_P(argv[1], PSTR("rwd"))) {
      rwdDistanceSensor.getDistance();
      Log.noticeln(rwdDistanceSensor);
      shell.println(rwdDistanceSensor._cachedDistance, DEC);
    }
  });

  cmdSet = ShellCommandClass(set, "Sets a property - [relay1|relay2|motor|resurrect] ([on|off] | [fwd|rev|off])", {
    Api::getInstance().logCommand(command->name, argc, argv);

    if (argc > 1 && !strcmp_P(argv[1], PSTR("relay1"))) {
    }
    if (argc > 1 && !strcmp_P(argv[1], PSTR("relay2"))) {
    }
  });
}

// Defines functions to be exposed to the rest API
bool Api::begin() {
  if (_isInitialized) return true;

  bool success = true;

  Log.noticeln(F("Setting up Ethernet and Server"));
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

  _shell.setPrompt(shellPrompt);
  _shellServer.begin();

  return _isInitialized = success;
}

void Api::handle() {
  // Maintain the DHCP lease over time.
  Ethernet.maintain();

  // Handle new/disconnecting clients.
  if (!_haveClient) {
    // Check for new client connections.
    _shellClient = _shellServer.available();
    if (_shellClient) {
      Log.noticeln(F("Client %p is attempting to connect to port %d."), _shellClient.remoteIP(), _shellClient.localPort());
      _haveClient = true;
      if (_shell.begin(_shellClient, 5, Terminal::Mode::Telnet)) {
        Log.noticeln(F("Client %p connected to port %d."), _shellClient.remoteIP(), _shellClient.localPort());
      } else {
        Log.noticeln(F("Client connect attempt failed. Shell::begin returned false (out of memory?)."));
      }
    }
  } else if (!_shellClient.connected()) {
    // The current client has been disconnected.  Shut down the shell.
    _shell.end();
    Log.noticeln(F("Client %p has been disconnected from port %d."), _shellClient.remoteIP(), _shellClient.localPort());
    _shellClient.stop();
    _shellClient = EthernetClient();
    _haveClient = false;
  }

  // Perform periodic shell processing on the active client.
  _shell.loop();
}
