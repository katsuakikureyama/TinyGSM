/**************************************************************
 *
 * This sketch connects to a website and downloads a page.
 * It can be used to perform HTTP/RESTful API calls.
 *
 * For this example, you need to install ArduinoHttpClient library:
 *   https://github.com/arduino-libraries/ArduinoHttpClient
 *   or from http://librarymanager/all#ArduinoHttpClient
 *
 * TinyGSM Getting Started guide:
 *   https://tiny.cc/tinygsm-readme
 *
 * For more HTTP API examples, see ArduinoHttpClient library
 *
 * NOTE: This example may NOT work with the XBee because the
 * HttpClient library does not empty to serial buffer fast enough
 * and the buffer overflow causes the HttpClient library to stall.
 * Boards with faster processors may work, 8MHz boards will not.
 **************************************************************/

/**************************************************************
 * nRF9160 Circuit Dojo Feather TinyGSM example
 * The following development boards/parts were used
 * + Circuit Dojo nRF9160 Feather
 *   https://www.jaredwolff.com/store/nrf9160-feather/
 *   
 * + (required) Adafruit Feather M0 Basic
 *   https://www.adafruit.com/product/2772
 *   ARM Cortex M0 core Atmel ATSAMD21G18 MCU at 48MHz
 *   
 * + (alternate) Arduino Zero
 *   has the same ARM Cortex M0 core Atmel ATSAMD21G18 MCU
 *   https://store-usa.arduino.cc/products/arduino-zero?selectedStore=us
 *   
 * + (alternate) Adafruit Grand Central M4 Express
 *   ARM Cortex M4 core Atmel ATSAMD51P20 MCU at 120MHz
 *   https://www.adafruit.com/product/4064
 *   This board is nice to connect with a camera and
 *   upload images to a server
 *   
 * + (optional) (recommended) nRF9160-DK (for JTAG programming)
 *   https://www.digikey.com/en/products/detail/nordic-semiconductor-asa/NRF9160-DK/9740721
 *   
 * + (optional) NRF5340-DK (for JTAG programming)
 *   https://www.digikey.com/en/products/detail/nordic-semiconductor-asa/NRF5340-DK/13544603
 *   
 * + (optional) (recommended) TC2030-CTX-NL (for JTAG programming)
 *   6-Pin “No Legs” Cable with 10-pin micro-connector for Cortex processors
 *   https://www.tag-connect.com/product/tc2030-ctx-nl-6-pin-no-legs-cable-with-10-pin-micro-connector-for-cortex-processors
 *   
 *   
 * Program the nRF9160 Circuit Dojo Feather as shown below
 * The source code is also available in extras
 *   + based on the v1.8.0 SDK serial_lte_modem sample application
 *   + a few additions were made to support binary data transfers
 *   + nrf9160_circuitdojo_feather_1.8.0_serial_lte_modem_115200_src.zip
 *     + The source code is not complete, will need to install
 *       the full toolchain if rebuilding the nRF9160 firmware
 *       is desired, beyond the scope of this information
 * Add the "Adafruit SAMD boards" in the board manager
 * + the following "board manager URL" can be added in preferences
 *   https://adafruit.github.io/arduino-board-index/package_adafruit_index.json
 *   
 * Select the "Adafruit SAMD" "Adafruit Feather M0" board 
 * 
 * Connect the nRF9160 RST pin to Arduino D6 (can be changed)
 * Connect the nRF9160 RX  pin to Arduino TX (D1)
 * Connect the nRF9160 TX  pin to Arduino RX (D0)
 * Power the nRF9160 through it's USB connector
 * nRF9160 UART signal is 115200 BAUD, no flow control (CTS/RTS)
 * nRF9160 UART signal level is 3.3V
 * 
 * Set GPRS credentials
 * + In my experience GPRS credentials were always required
 **************************************************************/

/**************************************************************
 * Programming the nRF9160 Circuit Dojo
 * Modem firmware (baseband)
 * + The nRF9160 chip has ARM Cortex M33 core plus an
 *   independent LTE modem core, which runs its own firmware
 * + Default installed firmware is version 1.2.3 which does not
 *   need to be upgraded (at the current time)
 * + IMPORTANT: currently nRF9160 Feather is shipped with version
 *   1.2.3 of the modem firmware. If you upgrade > v1.3.0 you
 *   cannot downgrade according to Nordic.
 * + The modem baseband firmware should not need to be worried
 *   about and will not be addressed in this information
 * 
 * 
 * There are two methods to programming
 * + Bootloader method
 *   + can be used if the bootloader is present
 *     + comes from the factory with a bootloader installed
 *   + does not require an external programmer
 *   + can be programmed through the USB connector
 * + JTAG programming
 *   + necessary if bootloader is not functioning or present
 *   + requires an external programmer
 *   + requires a programming cable (link below)
 *   + a little more involved
 * 
 * 
 * Bootloader programming method
 * + For this method to work, an bootloader must be present
 * + If there is no bootloader, or it is damaged it can be
 *   repaired using the method below
 *   + Once repair, the bootloader steps below can be used
 *     to program the device
 *   + To place the nRF9160 Feather into programming mode
 *     + While holding the button "BTN" press the "RST"
 *       button and continue holding until the blue LED
 *       comes on steadily
 *   + Download "newtmgr CLI (v1)" from the below page
 *     https://docs.jaredwolff.com/nrf9160-downloads.html
 *   + MacOS programming commands
 *     + First setup the comm port
 *       newtmgr conn add serial type=serial connstring='dev=/dev/tty.SLAB_USBtoUART,baud=1000000'
 *     + Once in bootloader (programming) mode use the
 *       below command to program
 *       newtmgr -c serial image upload nrf9160_circuitdojo_feather_1.8.0_serial_lte_modem_115200_app_update.bin
 *     + Reset the nRF9160 Feather using the "RST" button 
 *     
 *     
 * JTAG programming method
 *   + nRF9160-DK (development board from Nordic)
 *     + personal preference
 *   + nRF53-DK (development board from Nordic)
 *   + Other JTAG programmer such as a Segger JLink
 *   + Also requires a TagConnect cable
 *     + TC2030-CTX-NL 6-Pin “No Legs” Cable with 10-pin micro-connector for Cortex processors
 *     + https://www.tag-connect.com/product/tc2030-ctx-nl-6-pin-no-legs-cable-with-10-pin-micro-connector-for-cortex-processors
 *   + Programming can be accomplished by two software tools
 *     + nrfjprog
 *       + Included in "nRF Command Line Tools" by Nordic
 *       + Download from Nordic's website
 *         https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download#infotabs
 *       + Command line for programming
 *         nrfjprog -f NRF91 --program nrf9160_circuitdojo_feather_1.8.0_serial_lte_modem_115200_merged.hex --sectorerase
 *     + "Programmer" application in the "nRF Connect" application by Nordic
 *       + download for free from Nordic's website
 *       https://www.nordicsemi.com/Products/Development-tools/nrf-connect-for-desktop/download
 *       After installing this above application, install the
 *       "Programmer" application in the "nRF Connect" application
 *   + Verify settings on the nRF9160-DK (if using this development
 *     board to program)
 *     + Set the "SW5" "PROG/DEBUG" switch to the "nRF52" setting
 *       + This is a check to make sure we are programming the
 *         nRF9160 we have connected and not the onboard nRF9160
 *         If the nRF9160 target is not connected correctly it
 *         will attempt to program the nRF52 which will result
 *         in an easy to recognize error message instead of
 *         silently programming the nRF9160 onboard the nRF9160-DK
 *     + Set the "SW11, VDD IO" voltage switch to 3V
 *       + The nRF9160 can operate at 1V8 but the nRF9160 Feather
 *         operates at 3V3 and we need to match this
 *       + By default the nRF9160-DK board comes set at 1V8 from
 *         the factory
 *     + Connect the TagConnect TC2030-CTX-NL cable to the
 *       "Debug out" connector, it only fits one way so it
 *       can not be connected the wrong way by accident
 *     + As a precaution, connect an LTE antenna to the "LTE"
 *       connector on the nRF9160
 *       + Being capable of outputting 250mW of RF power, damage
 *         could occur if an antenna is not connected while the
 *         nRF9160 is transmitting
 *       + The strong RF signal can be reflected back to the
 *         nRF9160 transmitter components and dissipated as heat
 *     + Connect the nRF9160's USB connector to power
 *       + The nRF9160 Feather needs external power from the USB
 *         connector for programming, the programming cable will
 *         not power the target (nRF9160 Feather)
 *     + Find the programming pads on the nRF9160 Feather and
 *       insert the legs of the TagConnect programming cable
 *       into the PCB, it will only fit one way
 *       + The connector's pins are spring loaded and will
 *         need to be held in place until programming is
 *         completed (several seconds)
 *     + While holding the programming connector on the pads
 *       program using one of the methods above
 *     + After programming is completed remove the programming
 *       cable, if using nrfjprog the reset button will need
 *       to be pressed to reset the MCU
 *       + The reset button is labeled "RST" on the nRF9160
 *         Feather
 *       + The nRF Connect Programmer resets the MCU after
 *         programming is completed
 *       
 *   
 * Bootloader programming/repair
 * + The nRF9160 Feather will be loaded with the factory default
 *   application firmware (including the bootloader)
 * + Will require an external JTAG programmer
 *   + nRF9160-DK (development board from Nordic)
 *   + nRF53-DK (development board from Nordic)
 *   + Other JTAG programmer such as a Segger JLink
 * + Following the JTAG programming procedure above program
 *   the following file
 *   extras/nrf9160_circuitdojo_feather_bootloader_v2-010421-1502-merged.hex
 * + Command line for programming
 *   nrfjprog -f NRF91 --program nrf9160_circuitdojo_feather_bootloader_v2-010421-1502-merged.hex --sectorerase
 **************************************************************/
 
// Select your modem:
// #define TINY_GSM_MODEM_SIM800
// #define TINY_GSM_MODEM_SIM808
// #define TINY_GSM_MODEM_SIM868
// #define TINY_GSM_MODEM_SIM900
// #define TINY_GSM_MODEM_SIM7000
// #define TINY_GSM_MODEM_SIM7000SSL
// #define TINY_GSM_MODEM_SIM7080
// #define TINY_GSM_MODEM_SIM5360
// #define TINY_GSM_MODEM_SIM7600
// #define TINY_GSM_MODEM_UBLOX
// #define TINY_GSM_MODEM_SARAR4
// #define TINY_GSM_MODEM_M95
// #define TINY_GSM_MODEM_BG96
// #define TINY_GSM_MODEM_A6
// #define TINY_GSM_MODEM_A7
// #define TINY_GSM_MODEM_M590
// #define TINY_GSM_MODEM_MC60
// #define TINY_GSM_MODEM_MC60E
// #define TINY_GSM_MODEM_ESP8266
// #define TINY_GSM_MODEM_XBEE
// #define TINY_GSM_MODEM_SEQUANS_MONARCH
#define TINY_GSM_MODEM_NRF9160

// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial

// Set serial for AT commands (to the module)
// Use Hardware Serial on Mega, Leonardo, Micro
#ifndef __AVR_ATmega328P__
#define SerialAT Serial1

// or Software Serial on Uno, Nano
#else
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(2, 3);  // RX, TX
#endif

// Increase RX buffer to capture the entire response
// Chips without internal buffering (A6/A7, ESP8266, M590)
// need enough space in the buffer for the entire response
// else data will be lost (and the http library will fail).
#if !defined(TINY_GSM_RX_BUFFER)
#ifdef TINY_GSM_MODEM_NRF9160
#define TINY_GSM_RX_BUFFER 750
#else
#define TINY_GSM_RX_BUFFER 650
#endif
#endif

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

// Define the serial console for debug prints, if needed
#define TINY_GSM_DEBUG SerialMon
// #define LOGGING  // <- Logging is for the HTTP library

// Range to attempt to autobaud
// NOTE:  DO NOT AUTOBAUD in production code.  Once you've established
// communication, set a fixed baud rate using modem.setBaud(#).
#define GSM_AUTOBAUD_MIN 9600
#define GSM_AUTOBAUD_MAX 115200

// Add a reception delay, if needed.
// This may be needed for a fast processor at a slow baud rate.
// #define TINY_GSM_YIELD() { delay(2); }

// Define how you're planning to connect to the internet
// These defines are only for this example; they are not needed in other code.
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false

// set GSM PIN, if any
#define GSM_PIN ""

// Your GPRS credentials, if any
const char apn[]      = "YourAPN";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Your WiFi connection credentials, if applicable
const char wifiSSID[] = "YourSSID";
const char wifiPass[] = "YourWiFiPass";

// Server details
const char server[]   = "vsh.pp.ua";
const char resource[] = "/TinyGSM/logo.txt";
const int  port       = 80;

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

// Just in case someone defined the wrong thing..
#if TINY_GSM_USE_GPRS && not defined TINY_GSM_MODEM_HAS_GPRS
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS false
#define TINY_GSM_USE_WIFI true
#endif
#if TINY_GSM_USE_WIFI && not defined TINY_GSM_MODEM_HAS_WIFI
#undef TINY_GSM_USE_GPRS
#undef TINY_GSM_USE_WIFI
#define TINY_GSM_USE_GPRS true
#define TINY_GSM_USE_WIFI false
#endif

#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm        modem(debugger);
#else
TinyGsm        modem(SerialAT);
#endif

TinyGsmClient client(modem);
HttpClient    http(client, server, port);

void setup() {
  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // !!!!!!!!!!!
  // Set your reset, enable, power pins here
  // !!!!!!!!!!!
  #ifdef TINY_GSM_MODEM_NRF9160
  #define NRF9160_RESET_PIN 6
  pinMode(NRF9160_RESET_PIN, OUTPUT);
  digitalWrite(NRF9160_RESET_PIN, LOW);
  delay(1000);
  #endif

  SerialMon.println("Wait...");

  // Set GSM module baud rate
  //TinyGsmAutoBaud(SerialAT, GSM_AUTOBAUD_MIN, GSM_AUTOBAUD_MAX);
  #ifdef TINY_GSM_MODEM_NRF9160
  digitalWrite(NRF9160_RESET_PIN, HIGH);
  #endif
  SerialAT.begin(115200);
  
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

#if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }
#endif
}

void loop() {
#if TINY_GSM_USE_WIFI
  // Wifi connection parameters must be set before waiting for the network
  SerialMon.print(F("Setting SSID/password..."));
  if (!modem.networkConnect(wifiSSID, wifiPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");
#endif

#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_XBEE
  // The XBee must run the gprsConnect function BEFORE waiting for network!
  modem.gprsConnect(apn, gprsUser, gprsPass);
#endif

#if TINY_GSM_USE_GPRS && defined TINY_GSM_MODEM_NRF9160
  // The nRG9160 must run the gprsConnect function BEFORE waiting for network to set APN information!
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) {
    SerialMon.println("GPRS connected");
  }
#endif

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isNetworkConnected()) { SerialMon.println("Network connected"); }

#if TINY_GSM_USE_GPRS && !defined TINY_GSM_MODEM_NRF9160
  // GPRS connection parameters are usually set after network registration
  SerialMon.print(F("Connecting to "));
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println(" fail");
    delay(10000);
    return;
  }
  SerialMon.println(" success");

  if (modem.isGprsConnected()) { SerialMon.println("GPRS connected"); }
#endif

  #ifdef TINY_GSM_MODEM_NRF9160
  IPAddress local = IPAddress(0, 0, 0, 0);
  while(local == IPAddress(0, 0, 0, 0)) {
    local = modem.localIP();
    SerialMon.println("Waiting for IP address...");
    delay(500);
  }
  SerialMon.print("Local IP: ");
  SerialMon.println(local);
  #endif

  SerialMon.print(F("Performing HTTP GET request... "));
  int err = http.get(resource);
  if (err != 0) {
    SerialMon.println(F("failed to connect"));
    delay(10000);
    return;
  }

  int status = http.responseStatusCode();
  SerialMon.print(F("Response status code: "));
  SerialMon.println(status);
  if (!status) {
    delay(10000);
    return;
  }

  SerialMon.println(F("Response Headers:"));
  while (http.headerAvailable()) {
    String headerName  = http.readHeaderName();
    String headerValue = http.readHeaderValue();
    SerialMon.println("    " + headerName + " : " + headerValue);
  }

  int length = http.contentLength();
  if (length >= 0) {
    SerialMon.print(F("Content length is: "));
    SerialMon.println(length);
  }
  if (http.isResponseChunked()) {
    SerialMon.println(F("The response is chunked"));
  }

  String body = http.responseBody();
  SerialMon.println(F("Response:"));
  SerialMon.println(body);

  SerialMon.print(F("Body length is: "));
  SerialMon.println(body.length());

  // Shutdown

  http.stop();
  SerialMon.println(F("Server disconnected"));

#if TINY_GSM_USE_WIFI
  modem.networkDisconnect();
  SerialMon.println(F("WiFi disconnected"));
#endif
#if TINY_GSM_USE_GPRS
  modem.gprsDisconnect();
  SerialMon.println(F("GPRS disconnected"));
#endif

  // Do nothing forevermore
  while (true) { delay(1000); }
}