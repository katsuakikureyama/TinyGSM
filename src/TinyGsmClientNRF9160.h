/**
 * @file       TinyGsmClientSIM7600.h
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */
#include <deque>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <regex>
#ifndef SRC_TINYGSMCLIENTNRF9160_H_
#define SRC_TINYGSMCLIENTNRF9160_H_

// #define TINY_GSM_DEBUG Serial
// #define TINY_GSM_USE_HEX

#define TINY_GSM_MUX_COUNT 1
#define TINY_GSM_BUFFER_READ_AND_CHECK_SIZE
// #define TINY_GSM_BUFFER_READ_NO_CHECK
#define NRF9160_NET_IPV4_MTU_SIZE_RX 2100
#define NRF9160_TCP_PAYLOAD_SIZE_RX 2048


//#define NRF9160_NET_IPV4_MTU_SIZE_TX 288 // 576 / 2 = 288
#define NRF9160_NET_IPV4_MTU_SIZE_TX 100 // appears to be close to the upper limit
                                         // to not overrun the nRF9160 buffer, 128 is too big
#define NRF9160_RETRY_COUNT 10
#define DATATYPE 0 //HEX datamode
//#define DATATYPE 1 //text datamode

#include "TinyGsmBattery.tpp"
#include "TinyGsmGPRS.tpp"
#include "TinyGsmGPS.tpp"
#include "TinyGsmGSMLocation.tpp"
#include "TinyGsmModem.tpp"
#include "TinyGsmSMS.tpp"
#include "TinyGsmTCP.tpp"
#include "TinyGsmTemperature.tpp"
#include "TinyGsmTime.tpp"

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM    = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;
#if defined       TINY_GSM_DEBUG
static const char GSM_CME_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CME ERROR:";
static const char GSM_CMS_ERROR[] TINY_GSM_PROGMEM = GSM_NL "+CMS ERROR:";
#endif

enum RegStatus {
  REG_NO_RESULT    = -1,
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};

class TinyGsmNrf9160 : public TinyGsmModem<TinyGsmNrf9160>,
                       public TinyGsmGPRS<TinyGsmNrf9160>,
                       public TinyGsmTCP<TinyGsmNrf9160, TINY_GSM_MUX_COUNT>,
                       public TinyGsmSMS<TinyGsmNrf9160>,
                       public TinyGsmGSMLocation<TinyGsmNrf9160>,
                       public TinyGsmGPS<TinyGsmNrf9160>,
                       public TinyGsmTime<TinyGsmNrf9160>,
                       public TinyGsmBattery<TinyGsmNrf9160>,
                       public TinyGsmTemperature<TinyGsmNrf9160> {
  friend class TinyGsmModem<TinyGsmNrf9160>;
  friend class TinyGsmGPRS<TinyGsmNrf9160>;
  friend class TinyGsmTCP<TinyGsmNrf9160, TINY_GSM_MUX_COUNT>;
  friend class TinyGsmSMS<TinyGsmNrf9160>;
  friend class TinyGsmGPS<TinyGsmNrf9160>;
  friend class TinyGsmGSMLocation<TinyGsmNrf9160>;
  friend class TinyGsmTime<TinyGsmNrf9160>;
  friend class TinyGsmBattery<TinyGsmNrf9160>;
  friend class TinyGsmTemperature<TinyGsmNrf9160>;

  /*
   * Inner Client
   */
 public:
  class GsmClientNrf9160 : public GsmClient {
    friend class TinyGsmNrf9160;

   public:
    GsmClientNrf9160() {}

    explicit GsmClientNrf9160(TinyGsmNrf9160& modem, uint8_t mux = 1) {
      init(&modem, mux);
    }

    bool init(TinyGsmNrf9160* modem, uint8_t mux = 1) {
       Serial.println("nrf init");
      this->at       = modem;
      sock_available = 0;
      prev_check     = 0;
      sock_connected = false;
      got_data       = false;

      if (mux < TINY_GSM_MUX_COUNT) {
        this->mux = mux;
      } else {
        this->mux = (mux % TINY_GSM_MUX_COUNT);
      }
      at->sockets[this->mux] = this;

      return true;
    }

   public:
    virtual int connect(const char* host, uint16_t port, int timeout_s) {
       Serial.println("nrf connect");
      stop();
      TINY_GSM_YIELD();
      rx.clear();
      sock_connected = at->modemConnect(host, port, mux, false, timeout_s);
      return sock_connected;
    }
    TINY_GSM_CLIENT_CONNECT_OVERRIDES

    void stop(uint32_t maxWaitMs) {
      int timeout_s = 5;
      uint32_t timeout_ms = ((uint32_t)timeout_s) * 1000;
      dumpModemBuffer(maxWaitMs);
      //at->sendAT(GF("+CIPCLOSE="), mux);
      uint8_t request_op   = 0; // 1 = port open request
      uint8_t request_type = 1; // 1 = SOCK_STREAM for TCP (protocol)
      uint8_t request_role = 0; // 0 = client
      at->sendAT(GF("#XSOCKET="), GF("0,1,0"));
      sock_connected = false;
      
      at->waitResponse(timeout_ms, "OK", "#XSOCKET");
      
      at->streamSkipUntil('\n');

      // wait for OK
      at->waitResponse(2000L);
      //Serial.println("waiting response done");
    }
    void stop() override {
      stop(15000L);
    }

    /*
     * Extended API
     */

    String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  };

  /*
   * Inner Secure Client
   */

  /*TODO(?))
  class GsmClientSecureSIM7600 : public GsmClientNrf9160
  {
  public:
    GsmClientSecure() {}

    GsmClientSecure(TinyGsmNrf9160& modem, uint8_t mux = 0)
     : public GsmClient(modem, mux)
    {}

  public:
    int connect(const char* host, uint16_t port, int timeout_s) override {
      stop();
      TINY_GSM_YIELD();
      rx.clear();
      sock_connected = at->modemConnect(host, port, mux, true, timeout_s);
      return sock_connected;
    }
    TINY_GSM_CLIENT_CONNECT_OVERRIDES
  };
  */

  /*
   * Constructor
   */
 public:
  explicit TinyGsmNrf9160(Stream& stream) : stream(stream) {
    memset(sockets, 0, sizeof(sockets));
  }

  /*
   * Basic functions
   */
 protected:
  bool initImpl(const char* pin = NULL) {
    DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);
    DBG(GF("### TinyGSM Compiled Module:  TinyGsmClientNRF9160"));
      
    //if (!testAT()) { return false; }
    if (!testAT()) {
      DBG(GF("testAT failed"));
    }

    //
    sendAT(GF("+CEREG=1"));  // 1 – Enable unsolicited result codes +CEREG:<stat>
    waitResponse();
    /*
    0 – Not registered. UE is not currently searching for an operator to register to.
    1 – Registered, home network.
    2 – Not registered, but UE is currently trying to attach or searching an operator to register to.
    3 – Registration denied.
    4 – Unknown (e.g. out of E-UTRAN coverage).
    5 – Registered, roaming.
    90 – Not registered due to UICC failure.
    */

    DBG(GF("### Modem:"), getModemName());

    // set modem in online mode (we need this to check the SIM PIN)
    sendAT(GF("+CFUN=1"));
    waitResponse();

    //wait a second so the SIM chip finishes powering up
    delay(1000);

    // after powering up to to offline mode, but leave the SIM powered
    // we need to do this because we can't change APN settings unless we go to offline mode
    // however, leave the SIM powered, if we unlocked it we want it to stay unlocked
    sendAT(GF("+CFUN=44"));
    waitResponse();

    SimStatus ret = getSimStatus();
    // if the sim isn't ready and a pin has been provided, try to unlock the sim
    if (ret != SIM_READY && pin != NULL && strlen(pin) > 0) {
      simUnlock(pin);
      return (getSimStatus() == SIM_READY);
    } else {
      // if the sim is ready, or it's locked but no pin has been provided,
      // return true
      return (ret == SIM_READY || ret == SIM_LOCKED);
    }
  }

  String getModemNameImpl() {
     Serial.println("nrf getModemNameImpl");
    String name = "Nordic nRF9160";

    sendAT(GF("+CGMM"));
    String res2;
    if (waitResponse(1000L, res2) != 1) { return name; }
    res2.replace(GSM_NL "OK" GSM_NL, "");
    res2.replace("_", " ");
    res2.trim();

    name = res2;
    DBG("### Modem:", name);
    return name;
  }

  bool factoryDefaultImpl() {  // these commands aren't supported
    return false;
  }

  /*
   * Power functions
   */
 protected:
  bool restartImpl(const char* pin = NULL) {
     Serial.println("nrf restartImpl");
    if (!testAT()) { return false; }
    sendAT(GF("#XRESET"));
    if (waitResponse(10000L) != 1) { return false; }
    delay(5000L);  // TODO(?):  Test this delay!
    return init();
  }

  bool powerOffImpl() {
    sendAT(GF("+CFUN=0"));
    return waitResponse() == 1;
  }

  bool radioOffImpl() {
    if (!setPhoneFunctionality(4)) { return false; }
    delay(3000);
    return true;
  }

  bool sleepEnableImpl(bool enable = true) {
    sendAT(GF("+CSCLK="), enable);
    return waitResponse() == 1;
  }

  bool setPhoneFunctionalityImpl(uint8_t fun, bool reset = false) {
    sendAT(GF("+CFUN="), fun, reset ? ",1" : "");
    return waitResponse(10000L) == 1;
  }

  /*
   * Generic network functions
   */
 public:
  RegStatus getRegistrationStatus() {
    return (RegStatus)getRegistrationStatusXREG("CEREG");
  }

 protected:
  bool isNetworkConnectedImpl() {
    RegStatus s = getRegistrationStatus();
    return (s == REG_OK_HOME || s == REG_OK_ROAMING);
  }

 public:
  String getNetworkModes() {
    sendAT(GF("+CNMP=?"));
    if (waitResponse(GF(GSM_NL "+CNMP:")) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    return res;
  }

  String setNetworkMode(uint8_t mode) {
    sendAT(GF("+CNMP="), mode);
    if (waitResponse(GF(GSM_NL "+CNMP:")) != 1) { return "OK"; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    return res;
  }

  String getLocalIPImpl() {
    //sendAT(GF("+IPADDR"));  // Inquire Socket PDP address
    //sendAT(GF("+CGPADDR=1"));  // Show PDP address
    sendAT(GF("+CGPADDR=0"));  // Show PDP address
    String res;
    if (waitResponse(10000L, res) != 1) { return ""; }
    res.replace(GSM_NL "OK" GSM_NL, "");
    res.replace(GSM_NL, "");
    res.trim();
    return res;
  }

  /*
   * GPRS functions
   */
 protected:
  bool gprsConnectImpl(const char* apn, const char* user = NULL,
                       const char* pwd = NULL) {
     Serial.println("nrf gprsConnectedImpl");
    gprsDisconnect();  // Make sure we're not connected first

    // Define the PDP context

    // The CGDCONT commands set up the "external" PDP context

    // Define external PDP context 0
    sendAT(GF("+CGDCONT=0,\"IP\",\""), apn, '"', ",\"0.0.0.0\",0,0");
    waitResponse();

    // Set the external authentication
    if (user && strlen(user) > 0) {
      //sendAT(GF("+CGAUTH=1,1,\""), user, GF("\",\""), pwd, '"'); // PAP type
      sendAT(GF("+CGAUTH=0,2,\""), user, GF("\",\""), pwd, '"'); // CHAP type
      waitResponse();
    }

    // set modem in online mode
    sendAT(GF("+CFUN=1"));
    waitResponse();

    return true;
  }

  bool gprsDisconnectImpl() {
     Serial.println("nrf gprsDisconnectedImple");
    // Close all sockets and stop the socket service
    // Note: On the LTE models, this single command closes all sockets and the
    // service

    // set modem in offline mode
    sendAT(GF("+CFUN=4"));
    waitResponse();
    return true;
  }

  bool isGprsConnectedImpl() {
 Serial.println("nrf isGprsConnetedImple");
    return true;
  }

  /*
   * SIM card functions
   */
 protected:
  // Gets the CCID of a sim card via AT+CCID
  String getSimCCIDImpl() {
     Serial.println("nrf getSimCCIDImpl");
    sendAT(GF("+CICCID"));
    if (waitResponse(GF(GSM_NL "+ICCID:")) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  /*
   * Phone Call functions
   */
 protected:
  bool callAnswerImpl() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  bool callNumberImpl(const String& number) TINY_GSM_ATTR_NOT_IMPLEMENTED;
  bool callHangupImpl() TINY_GSM_ATTR_NOT_IMPLEMENTED;
  bool dtmfSendImpl(char cmd,
                    int  duration_ms = 100) TINY_GSM_ATTR_NOT_IMPLEMENTED;

  /*
   * Messaging functions
   */
 protected:
  // Follows all messaging functions per template

  /*
   * GSM Location functions
   */
 protected:
  // Can return a GSM-based location from CLBS as per the template

  /*
   * GPS/GNSS/GLONASS location functions
   */
 protected:
  // enable GPS
  bool enableGPSImpl() {
    sendAT(GF("+CGPS=1"));
    if (waitResponse() != 1) { return false; }
    return true;
  }

  bool disableGPSImpl() {
    sendAT(GF("+CGPS=0"));
    if (waitResponse() != 1) { return false; }
    return true;
  }

  // get the RAW GPS output
  String getGPSrawImpl() {
    sendAT(GF("+CGNSSINFO=32"));
    if (waitResponse(GF(GSM_NL "+CGNSSINFO:")) != 1) { return ""; }
    String res = stream.readStringUntil('\n');
    waitResponse();
    res.trim();
    return res;
  }

  // get GPS informations
  bool getGPSImpl(float* lat, float* lon, float* speed = 0, float* alt = 0,
                  int* vsat = 0, int* usat = 0, float* accuracy = 0,
                  int* year = 0, int* month = 0, int* day = 0, int* hour = 0,
                  int* minute = 0, int* second = 0) {
    sendAT(GF("+CGNSSINFO"));
    if (waitResponse(GF(GSM_NL "+CGNSSINFO:")) != 1) { return false; }

    uint8_t fixMode = streamGetIntBefore(',');  // mode 2=2D Fix or 3=3DFix
                                                // TODO(?) Can 1 be returned
    if (fixMode == 1 || fixMode == 2 || fixMode == 3) {
      // init variables
      float ilat = 0;
      char  north;
      float ilon = 0;
      char  east;
      float ispeed       = 0;
      float ialt         = 0;
      int   ivsat        = 0;
      int   iusat        = 0;
      float iaccuracy    = 0;
      int   iyear        = 0;
      int   imonth       = 0;
      int   iday         = 0;
      int   ihour        = 0;
      int   imin         = 0;
      float secondWithSS = 0;

      streamSkipUntil(',');               // GPS satellite valid numbers
      streamSkipUntil(',');               // GLONASS satellite valid numbers
      streamSkipUntil(',');               // BEIDOU satellite valid numbers
      ilat  = streamGetFloatBefore(',');  // Latitude in ddmm.mmmmmm
      north = stream.read();              // N/S Indicator, N=north or S=south
      streamSkipUntil(',');
      ilon = streamGetFloatBefore(',');  // Longitude in ddmm.mmmmmm
      east = stream.read();              // E/W Indicator, E=east or W=west
      streamSkipUntil(',');

      // Date. Output format is ddmmyy
      iday   = streamGetIntLength(2);    // Two digit day
      imonth = streamGetIntLength(2);    // Two digit month
      iyear  = streamGetIntBefore(',');  // Two digit year

      // UTC Time. Output format is hhmmss.s
      ihour = streamGetIntLength(2);  // Two digit hour
      imin  = streamGetIntLength(2);  // Two digit minute
      secondWithSS =
          streamGetFloatBefore(',');  // 4 digit second with subseconds

      ialt   = streamGetFloatBefore(',');  // MSL Altitude. Unit is meters
      ispeed = streamGetFloatBefore(',');  // Speed Over Ground. Unit is knots.
      streamSkipUntil(',');                // Course Over Ground. Degrees.
      streamSkipUntil(',');  // After set, will report GPS every x seconds
      iaccuracy = streamGetFloatBefore(',');  // Position Dilution Of Precision
      streamSkipUntil(',');   // Horizontal Dilution Of Precision
      streamSkipUntil(',');   // Vertical Dilution Of Precision
      streamSkipUntil('\n');  // TODO(?) is one more field reported??

      // Set pointers
      if (lat != NULL)
        *lat = (floor(ilat / 100) + fmod(ilat, 100.) / 60) *
            (north == 'N' ? 1 : -1);
      if (lon != NULL)
        *lon = (floor(ilon / 100) + fmod(ilon, 100.) / 60) *
            (east == 'E' ? 1 : -1);
      if (speed != NULL) *speed = ispeed;
      if (alt != NULL) *alt = ialt;
      if (vsat != NULL) *vsat = ivsat;
      if (usat != NULL) *usat = iusat;
      if (accuracy != NULL) *accuracy = iaccuracy;
      if (iyear < 2000) iyear += 2000;
      if (year != NULL) *year = iyear;
      if (month != NULL) *month = imonth;
      if (day != NULL) *day = iday;
      if (hour != NULL) *hour = ihour;
      if (minute != NULL) *minute = imin;
      if (second != NULL) *second = static_cast<int>(secondWithSS);

      waitResponse();
      return true;
    }

    waitResponse();
    return false;
  }


  /**
   *  CGNSSMODE: <gnss_mode>,<dpo_mode>
   *  This command is used to configure GPS, GLONASS, BEIDOU and QZSS support
   * mode. 0 : GLONASS 1 : BEIDOU 2 : GALILEO 3 : QZSS dpo_mode: 1 enable , 0
   * disable
   */
  String setGNSSModeImpl(uint8_t mode, bool dpo) {
    String res;
    sendAT(GF("+CGNSSMODE="), mode, ",", dpo);
    if (waitResponse(10000L, res) != 1) { return ""; }
    res.replace(GSM_NL, "");
    res.trim();
    return res;
  }

  uint8_t getGNSSModeImpl() {
    sendAT(GF("+CGNSSMODE?"));
    if (waitResponse(GF(GSM_NL "+CGNSSMODE:")) != 1) { return 0; }
    return stream.readStringUntil(',').toInt();
  }


  /*
   * Time functions
   */
 protected:
  String getGSMDateTimeImpl(TinyGSMDateTimeFormat format) {
    sendAT(GF("+CCLK?"));
    if (waitResponse(2000L, GF("+CCLK: \"")) != 1) { return ""; }

    String res;

    switch (format) {
      case DATE_FULL: res = stream.readStringUntil('"'); break;
      case DATE_TIME:
        streamSkipUntil(',');
        res = stream.readStringUntil('"');
        break;
      case DATE_DATE: res = stream.readStringUntil(','); break;
    }
    waitResponse();  // Ends with OK
    return res;
  }

  // The BG96 returns UTC time instead of local time as other modules do in
  // response to CCLK, so we're using QLTS where we can specifically request
  // local time.
  bool getNetworkTimeImpl(int* year, int* month, int* day, int* hour,
                          int* minute, int* second, float* timezone) {
    sendAT(GF("+CCLK?"));
    if (waitResponse(2000L, GF("+CCLK: \"")) != 1) { return false; }

    int iyear     = 0;
    int imonth    = 0;
    int iday      = 0;
    int ihour     = 0;
    int imin      = 0;
    int isec      = 0;
    int itimezone = 0;

    // Date & Time
    iyear       = streamGetIntBefore('/');
    imonth      = streamGetIntBefore('/');
    iday        = streamGetIntBefore(',');
    ihour       = streamGetIntBefore(':');
    imin        = streamGetIntBefore(':');
    isec        = streamGetIntLength(2);
    char tzSign = stream.read();
    itimezone   = streamGetIntBefore(',');
    if (tzSign == '-') { itimezone = itimezone * -1; }
    streamSkipUntil('\n');  // DST flag

    // Set pointers
    if (iyear < 2000) iyear += 2000;
    if (year != NULL) *year = iyear;
    if (month != NULL) *month = imonth;
    if (day != NULL) *day = iday;
    if (hour != NULL) *hour = ihour;
    if (minute != NULL) *minute = imin;
    if (second != NULL) *second = isec;
    if (timezone != NULL) *timezone = static_cast<float>(itimezone) / 4.0;

    // Final OK
    waitResponse();  // Ends with OK
    return true;
  }

  /*
   * Battery functions
   */
 protected:
  // returns volts, multiply by 1000 to get mV
  uint16_t getBattVoltageImpl() {
    sendAT(GF("+CBC"));
    if (waitResponse(GF(GSM_NL "+CBC:")) != 1) { return 0; }

    // get voltage in VOLTS
    float voltage = streamGetFloatBefore('\n');
    // Wait for final OK
    waitResponse();
    // Return millivolts
    uint16_t res = voltage * 1000;
    return res;
  }

  int8_t getBattPercentImpl() TINY_GSM_ATTR_NOT_AVAILABLE;

  uint8_t getBattChargeStateImpl() TINY_GSM_ATTR_NOT_AVAILABLE;

  bool getBattStatsImpl(uint8_t& chargeState, int8_t& percent,
                        uint16_t& milliVolts) {
    chargeState = 0;
    percent     = 0;
    milliVolts  = getBattVoltage();
    return true;
  }

  /*
   * Temperature functions
   */
 protected:
  // get temperature in degree celsius
  uint16_t getTemperatureImpl() {
    sendAT(GF("+CPMUTEMP"));
    if (waitResponse(GF(GSM_NL "+CPMUTEMP:")) != 1) { return 0; }
    // return temperature in C
    uint16_t res = streamGetIntBefore('\n');
    // Wait for final OK
    waitResponse();
    return res;
  }



uint8_t waitForAvailable ( uint8_t mux){

    uint32_t startMillis = millis();
    while ( (millis() - startMillis) < sockets[mux]->_timeout ){
     if (stream.available() > 0 ) {
            return stream.available();
            
        }
    }
    return 0;

}


  /*
   * Client related functions
   */
 protected:


  bool modemConnect(const char* host, uint16_t port, uint8_t mux,
                    bool ssl = false, int timeout_s = 15) {
    Serial.println("hostname");
    
    Serial.println(host);
    if (ssl) { DBG("SSL not yet supported on this module!"); }
    // Make sure we'll be getting data manually on this connection
    //sendAT(GF("+CIPRXGET=1"));
    //if (waitResponse() != 1) { return false; }

    // #XSOCKET
    // op, type, role
    // op =   1 = port open request
    // type = 1 = SOCK_STREAM for TCP (protocol)
    // role = 0 = client
    uint8_t request_op   = 1; // 1 = port open request
    uint8_t request_type = 1; // 1 = SOCK_STREAM for TCP (protocol)
    uint8_t request_role = 0; // 0 = client
    sendAT(GF("#XSOCKET="), GF("1,1,0"));

    uint32_t timeout_ms = ((uint32_t)timeout_s) * 1000;
    if (waitResponse(timeout_ms, GF("#XSOCKET:")) != 1) { return false; }
    uint8_t result_op       = streamGetIntBefore(',');
    uint8_t result_type     = streamGetIntBefore(',');
    uint8_t result_protocol = streamGetIntBefore('\n'); // 6 = IPPROTO_TCP
    //if (request_op != result_op || request_type != result_type || result_protocol != result_protocol) return false;
    //fix for nRF9160 SDK 1.9.1
    if (request_type != result_type || result_protocol != result_protocol) return false;

    // wait for ok
    if (waitResponse(timeout_ms, "OK") != 1) { return false; }

    //socket options, receive timeout 2 seconds
    sendAT(GF("#XSOCKETOPT="), GF("1,20,2"));
    // wait for ok
    if (waitResponse(timeout_ms, "OK") != 1) { return false; }

    // Establish a connection in multi-socket mode
    // #XCONNECT
    // op  = 1 = open
    // url
    // port
    sendAT(GF("#XCONNECT="), GF("\""), host, GF("\","), port);
    // The reply is OK followed by +CIPOPEN: <link_num>,<err> where <link_num>
    // is the mux number and <err> should be 0 if there's no error
    if (waitResponse(timeout_ms, GF("#XCONNECT:")) != 1) { return false; }
    uint8_t opened_result = streamGetIntBefore('\n');
    if (opened_result != 1) return false;

    // wait for ok
    if (waitResponse(timeout_ms, "OK\r\n") != 1) { return false; }

    sockets[mux]->sock_connected = true;

    // small delay for testing
    delay(500L);  // TODO(?):  Test this delay!

    return true;
  }

std::deque<uint8_t> gBuffer;


// 1バイトの16進文字列をデコードして`uint8_t`に変換する関数
uint8_t hexToByte(const std::string& hexString) {
    return static_cast<uint8_t>(std::stoi(hexString, nullptr, 16));
}

// データを2文字ごとに読み取ってデコードし、gBufferに追加する関数
void fillBuffer(std::deque<uint8_t>& data) {
   
  while (!data.empty()) {
    
      gBuffer.push_back(data.front());
      data.pop_front();
  }
   /*
    std::string hexString;
    
    while (!data.empty()) {
        // `data`から1バイト取得して`char`にキャスト
      
        char byteChar = static_cast<char>(data.front());
        data.pop_front();
        
        // 2文字ごとに連結して`hexString`を作成
        hexString += byteChar;
        if (hexString.size() == 2) {
            // 2文字の`hexString`を`uint8_t`にデコードし、gBufferに追加
            uint8_t decodedByte = hexToByte(hexString);
            gBuffer.push_back(decodedByte);
            hexString.clear(); // 次の2文字のためにクリア
        }
    }
    */
}

bool removeUpToFirstSequence(std::deque<uint8_t>& data) {
    // 探したいシーケンス
    std::deque<uint8_t> sequence = {0x0D, 0x0A};

    // シーケンスの開始位置を探す
    for (size_t i = 0; i <= data.size() - sequence.size(); ++i) {
        // シーケンスと一致するか確認
        if (data[i] == sequence[0] && data[i + 1] == sequence[1]) {
            // シーケンスの直後からデータを残す
            data.erase(data.begin(), data.begin() + i + sequence.size());
            return true; // 削除したことを示す
        }
    }
    return false; // シーケンスが見つからなかった場合
}

bool checkLastSequenceIfError(const std::deque<uint8_t>& data) {
    // "ERROR" シーケンス
    
    std::deque<uint8_t> errorSequence = {0x45, 0x52, 0x52, 0x4F, 0x52, 0x0D, 0x0A};

    // データが "ERROR" シーケンス以上の長さか確認
    if (data.size() >= errorSequence.size()) {
        // 最後のバイトが "ERROR" シーケンスと一致するか確認
        for (size_t i = 0; i < errorSequence.size(); ++i) {
            if (data[data.size() - errorSequence.size() + i] != errorSequence[i]) {
                return false;
            }
        }
        return true; // 最後のシーケンスが "ERROR" と一致
    }
    return false; // データが短すぎる、または一致しない
}
bool removeLastSequenceIfMatches(std::deque<uint8_t>& data) {
    // 確認したいシーケンス
    
    std::deque<uint8_t> sequence = {0x0D, 0x0A, 0x4F, 0x4B, 0x0D, 0x0A};

    // データが6バイト以上あるか確認
    if (data.size() >= sequence.size()) {
        // 最後の6バイトがシーケンスと一致するか確認
        bool matches = true;
        for (size_t i = 0; i < sequence.size(); ++i) {
            if (data[data.size() - sequence.size() + i] != sequence[i]) {
                matches = false;
                break;
            }
        }

        // 一致する場合は最後の6バイトを削除して true を返す
        if (matches) {
            data.erase(data.end() - sequence.size(), data.end());
            return true;
        }
    }
    // 一致しない場合は false を返す
    return false;
}

bool removeUpToAndIncludingConsecutiveHexZeros(std::deque<uint8_t>& data, size_t zeroCountThreshold) {
    size_t zeroCount = 0;
    for (size_t i = 0; i < data.size(); ++i) {
        if (data[i] == 0x00) {
            zeroCount++;
            if (zeroCount >= zeroCountThreshold) {
                // 指定された数以上の0x00が連続している場所を見つけたので、その位置以降のデータを残す
                data.erase(data.begin(), data.begin() + i + 1);
                return true;
            }
        } else {
            zeroCount = 0;  // 0x00以外の値があればカウントリセット
        }
    }
    return false;  // 指定された数以上の0x00が見つからなかった場合
}
void printAsHexString(const std::deque<uint8_t>& buffer) {
    
    for (auto byte : buffer) {
        if (byte < 0x10) {
            Serial.print("0");  // 1桁の場合は先頭に0を追加
        }
        Serial.print(byte, HEX);  // 16進数で出力
    }
    Serial.println();  // 改行
}
bool dataAvailableFlg=false;
  int16_t modemSend(const void* buff, size_t buff_len, uint8_t mux) {
    uint8_t datatype = DATATYPE;
    uint8_t tempchar;
    size_t buff_index;
    size_t result_len = -1;
    size_t len = buff_len;
    dataAvailableFlg=false;
    while (!gBuffer.empty()) {
      
      gBuffer.pop_back();
    }
       while (stream.available()) {
        stream.read();
      }

    
    
    int timeout_s = 1;
    uint32_t timeout_ms = ((uint32_t)timeout_s) * 1000;


      //loop here incase of send error, we want to retry
      uint32_t startMillis = millis();
      // while(1) {

        if ( (millis() - startMillis > sockets[mux]->_timeout) ) {
          Serial.println("Timeout reached.");
            return 0;
          //break; // タイムアウトが経過したらループを終了
        }
        
        sendAT(GF("#XSEND"));

        waitResponse(timeout_ms, "OK");

// printHex(reinterpret_cast<const uint8_t*>(buff), len);
        for(size_t buff_index = 0; buff_index < len; buff_index++) {
          
          tempchar = reinterpret_cast<const uint8_t*>(buff)[buff_index];
          stream.write(tempchar);
          
         }
        
        stream.write("+++");
        stream.flush();

      
      while (stream.available() > 0) {
        stream.read();
      }
    //  }
     
    //sockets[mux]->sock_available=1;
   size_t c = -1;


  waitResponse(timeout_ms, "#XDATAMODE:");
  result_len =streamGetIntBefore('\n');
  if(result_len>4096){  result_len=buff_len; }
  // waitResponse(timeout_ms, "#XDATAMODE: 0\r\n");
   
  while (stream.available()) { stream.read(); }  
    /*
   if ( waitForAvailable(mux) != -1 ) {
        waitResponse(timeout_ms, "#XDATAMODE: 0\n");
    }
    */
//    delay(100L);     
 
    // stream.flush();
    return result_len;
  }

  size_t modemRead(size_t s, uint8_t mux) {
    // uint8_t datatype = DATATYPE;
    if (!sockets[mux]) return 0;

if( s > gBuffer.size() ){
bool cmdSending = false;
//while (true) {

    if (!cmdSending) {
        sendAT(GF("#XRECV=0,256"));
        cmdSending = true;
    }

 
    std::vector<uint8_t> dataBuffer;
    int read = 0;

    while (waitForAvailable(mux) > 0) {
        uint8_t byte = stream.read();  // バイナリとして1バイトずつ読み込み
        dataBuffer.push_back(byte);
    }

    if (dataBuffer.empty()) {
        read = 1;
     //   break;
    } 

    // バイナリデータ内で"ERROR"を探す
    const std::vector<uint8_t> errorPattern = {'E', 'R', 'R', 'O', 'R'};
    auto errorPos = std::search(dataBuffer.begin(), dataBuffer.end(), errorPattern.begin(), errorPattern.end());
    if (errorPos != dataBuffer.end()) {
        read = 8;
        Serial.println("error!");
       // break;
    }

    // "\r\nOK" のシーケンスをバイナリとして探して削除
    const std::vector<uint8_t> okPattern = {'\r', '\n', 'O', 'K'};
    auto posOK = std::search(dataBuffer.begin(), dataBuffer.end(), okPattern.begin(), okPattern.end());
    if (posOK != dataBuffer.end()) {
        dataBuffer.erase(posOK, dataBuffer.end());
    }
    // else{ break; }
    // "\r\n" のシーケンスをバイナリとして探して削除
    const std::vector<uint8_t> newlinePattern = {'\r', '\n'};
    auto pos = std::search(dataBuffer.begin(), dataBuffer.end(), newlinePattern.begin(), newlinePattern.end());
    if (pos != dataBuffer.end()) {
        dataBuffer.erase(dataBuffer.begin(), pos + newlinePattern.size());
        pos = std::search(dataBuffer.begin(), dataBuffer.end(), newlinePattern.begin(), newlinePattern.end());
        if (pos != dataBuffer.end()) {
            dataBuffer.erase(dataBuffer.begin(), pos + newlinePattern.size());
        }
    }

    cmdSending = false;

   // if (dataBuffer.empty()) break;

    size_t si = dataBuffer.size();
    for (size_t i = 0; i < si; ++i) {
        gBuffer.push_back(dataBuffer[i]);  // gBufferに追加
    }

    if (si != NRF9160_TCP_PAYLOAD_SIZE_RX) {
   //     break;
    }
 // }
  
  }
  
    int bytesRead=0; 
     
    for (size_t i = 0; i < s; i++ ) {
        
         if (!gBuffer.empty()) {
         
          uint8_t c = gBuffer.front();
          sockets[mux]->rx.put(c);
          gBuffer.pop_front();
           bytesRead++;
        }
    }

    // sockets[mux]->sock_available = gBuffer.size();

    sockets[mux]->sock_available = bytesRead;
       
    dataAvailableFlg=true;
   // Serial.println("読み込んだ");
    // Serail.println(bytesRead);

    return bytesRead;
  
  }
  
  size_t modemGetAvailable(uint8_t mux) {
    if (!sockets[mux]){

      return 0;}
     // return gBuffer.size();
     
      if(dataAvailableFlg == true) {
        
        sockets[mux]->sock_available =gBuffer.size();
        return gBuffer.size();
      } else  {  
        sockets[mux]->sock_available =0;
        // Serial.println("unavailable");
        return -1;
      
      }
  }

  bool modemGetConnected(uint8_t mux) {
    // Read the status of all sockets at once
    
    sendAT(GF("+XSOCKET?"));
    if (waitResponse(GF("+XSOCKET:")) != 1) {
      // return false;  // TODO:  Why does this not read correctly?
    }
    for (int muxNo = 0; muxNo < TINY_GSM_MUX_COUNT; muxNo++) {
      // +CIPCLOSE:<link0_state>,<link1_state>,...,<link9_state>
      bool muxState = stream.parseInt();
      if (sockets[muxNo]) { sockets[muxNo]->sock_connected = muxState; }
    }
    waitResponse();  // Should be an OK at the end
    if (!sockets[mux]) return false;
    return sockets[mux]->sock_connected;
  }

  /*
   * Utilities
   */
 public:
  // TODO(vshymanskyy): Optimize this!
  int8_t waitResponse(uint32_t timeout_ms, String& data,
                      GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    /*String r1s(r1); r1s.trim();
    String r2s(r2); r2s.trim();
    String r3s(r3); r3s.trim();
    String r4s(r4); r4s.trim();
    String r5s(r5); r5s.trim();
    DBG("### ..:", r1s, ",", r2s, ",", r3s, ",", r4s, ",", r5s);*/
    data.reserve(64);
    uint8_t  index       = 0;
    uint32_t startMillis = millis();
    do {
      TINY_GSM_YIELD();
      while (stream.available() > 0) {
        TINY_GSM_YIELD();
        int8_t a = stream.read();
        if (a <= 0) continue;  // Skip 0x00 bytes, just in case
        data += static_cast<char>(a);
        if (r1 && data.endsWith(r1)) {
          index = 1;
          goto finish;
        } else if (r2 && data.endsWith(r2)) {
          index = 2;
          goto finish;
        } else if (r3 && data.endsWith(r3)) {
#if defined TINY_GSM_DEBUG
          if (r3 == GFP(GSM_CME_ERROR)) {
            streamSkipUntil('\n');  // Read out the error
          }
#endif
          index = 3;
          goto finish;
        } else if (r4 && data.endsWith(r4)) {
          index = 4;
          goto finish;
        } else if (r5 && data.endsWith(r5)) {
          index = 5;
          goto finish;
        } else if (data.endsWith(GF(GSM_NL "+CIPRXGET:"))) {
          int8_t mode = streamGetIntBefore(',');
          if (mode == 1) {
            int8_t mux = streamGetIntBefore('\n');
            if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
              sockets[mux]->got_data = true;
            }
            data = "";
            // DBG("### Got Data:", mux);
          } else {
            data += mode;
          }
        } else if (data.endsWith(GF(GSM_NL "+RECEIVE:"))) {
          int8_t  mux = streamGetIntBefore(',');
          int16_t len = streamGetIntBefore('\n');
          if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
            sockets[mux]->got_data = true;
            if (len >= 0 && len <= 1024) { sockets[mux]->sock_available = len; }
          }
          data = "";
          // DBG("### Got Data:", len, "on", mux);
        } else if (data.endsWith(GF("#XSOCKET:"))) {
          // todo
          // not working,, we never get here...
          // supposed to detect a port closing, but we aren't
          int8_t mux = 1;
          uint8_t result_op = streamGetIntBefore(',');
          streamSkipUntil('\n');
          //if (result_op == 0) {
            sockets[mux]->sock_connected = false;
          //}
          data = "";
          DBG("### Closed: ", mux);
        } else if (data.endsWith(GF("+CIPEVENT:"))) {
          // Need to close all open sockets and release the network library.
          // User will then need to reconnect.
          DBG("### Network error!");
          if (!isGprsConnected()) { gprsDisconnect(); }
          data = "";
        }
      }
    } while (millis() - startMillis < timeout_ms);
  finish:
    if (!index) {
      data.trim();
      //Serial.println("...?");
      if (data.length()) { DBG("### Unhandled:", data); }
      data = "";
    }
    // data.replace(GSM_NL, "/");
    // DBG('<', index, '>', data);
    return index;
  }

  int8_t waitResponse(uint32_t timeout_ms, GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    String data;
    return waitResponse(timeout_ms, data, r1, r2, r3, r4, r5);
  }

  int8_t waitResponse(GsmConstStr r1 = GFP(GSM_OK),
                      GsmConstStr r2 = GFP(GSM_ERROR),
#if defined TINY_GSM_DEBUG
                      GsmConstStr r3 = GFP(GSM_CME_ERROR),
                      GsmConstStr r4 = GFP(GSM_CMS_ERROR),
#else
                      GsmConstStr r3 = NULL, GsmConstStr r4 = NULL,
#endif
                      GsmConstStr r5 = NULL) {
    return waitResponse(1000, r1, r2, r3, r4, r5);
  }

 public:
  Stream& stream;

 protected:
  GsmClientNrf9160* sockets[TINY_GSM_MUX_COUNT];
  const char*       gsmNL = GSM_NL;
};

#endif  // SRC_TINYGSMCLIENTSIM7600_H_