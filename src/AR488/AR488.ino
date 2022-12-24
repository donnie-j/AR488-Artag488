/***** FWVER "AR488 GPIB controller, ver. 0.51.15, 12/10/2022" *****/
/*
  Arduino IEEE-488 implementation by John Chajecki
  Refactor by D. Jeff Dionne

  Inspired by the original work of Emanuele Girlando, licensed under a Creative
  Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
  Any code in common with the original work is reproduced here with the explicit
  permission of Emanuele Girlando, who has kindly reviewed and tested this code.

  Thanks also to Luke Mester for comparison testing against the Prologix interface.
  AR488 is Licenced under the GNU Public licence.

  Thanks to 'maxwell3e10' on the EEVblog forum for suggesting additional auto mode
  settings and the macro feature.

  Thanks to 'artag' on the EEVblog forum for providing code for the 32u4.
*/

#include <avr/wdt.h>
#include "AR488_Config.h"
#include "AR488_GPIBbus.h"
#include "AR488_ComPorts.h"
#include "AR488_Eeprom.h"
#include "AR488_cmd.h"
#include "AR488_help.h"

#define PBSIZE 256

#ifdef USE_MACROS
/***** STARTUP MACRO *****/
const char startup_macro[] PROGMEM = {MACRO_0};

/***** Consts holding USER MACROS 1 - 9 *****/
const char macro_1 [] PROGMEM = {MACRO_1};
const char macro_2 [] PROGMEM = {MACRO_2};
const char macro_3 [] PROGMEM = {MACRO_3};
const char macro_4 [] PROGMEM = {MACRO_4};
const char macro_5 [] PROGMEM = {MACRO_5};
const char macro_6 [] PROGMEM = {MACRO_6};
const char macro_7 [] PROGMEM = {MACRO_7};
const char macro_8 [] PROGMEM = {MACRO_8};
const char macro_9 [] PROGMEM = {MACRO_9};

const char * const macros[] PROGMEM = {
  startup_macro,
  macro_1,
  macro_2,
  macro_3,
  macro_4,
  macro_5,
  macro_6,
  macro_7,
  macro_8,
  macro_9
};
#endif

#define OK 0
#define ERR 1

/***** Control characters *****/
#define ESC  0x1B   // the USB escape char
#define CR   0xD    // Carriage return
#define LF   0xA    // Newline/linefeed
#define PLUS 0x2B   // '+' character

char pBuf[PBSIZE];
uint8_t pbPtr = 0;

GPIBbus gpibBus;

// CR/LF terminated line ready to process

// GPIB data receive flags
bool autoRead = false;              // Auto reading (auto mode 3) GPIB data in progress
bool readWithEoi = false;           // Read eoi requested
bool readWithEndByte = false;       // Read with specified terminator character
bool isQuery = false;               // Direct instrument command is a query
uint8_t tranBrk = 0;                // Transmission break on 1=++, 2=EOI, 3=ATN 4=UNL
uint8_t endByte = 0;                // Termination character
bool dataBufferFull = false;        // Flag when parse buffer is full

bool isEsc = false;           // Charcter escaped
bool isPlusEscaped = false;   // Plus escaped
bool isVerbose = false;       // Verbose mode
uint8_t lnRdy = 0;            // Input line ready to process
bool isRO = false;            // Read only mode flag
uint8_t isTO = 0;             // Talk only mode flag
bool isProm = false;          // Pomiscuous mode
bool isSrqa = false;          // SRQ auto mode
bool sendIdn = false;         // Send response to *idn?

uint8_t runMacro = 0;         // Whether to run Macro 0 (macros must be enabled)

void flushPbuf() {
  memset(pBuf, '\0', PBSIZE);
  pbPtr = 0;
}

void showPrompt() {
  dataPort.print("> ");
}

void sendToInstrument(char *buffr, uint8_t dsize) {
  if (buffr[dsize-1] == '?') isQuery = true;
  if (!gpibBus.haveAddressedDevice()) gpibBus.addressDevice(gpibBus.cfg.paddr, LISTEN);

  gpibBus.sendData(buffr, dsize);

  if (dataBufferFull) dataBufferFull = false;
  else gpibBus.unAddressDevice();

  if (isVerbose) showPrompt();

  flushPbuf();
  lnRdy = 0;
}

uint8_t parseInput(char c) {

  uint8_t r = 0;

  if (pbPtr < PBSIZE) {
    if (isVerbose && c!=LF) dataPort.print(c);  // Humans like to see what they are typing...
    switch (c) {
      case CR:
      case LF:
        if (isEsc) {
          addPbuf(c);
          isEsc = false;
        } else { // Carriage return on blank line?
          if (pbPtr == 0) {
            flushPbuf();
            if (isVerbose) {
              dataPort.println();
              showPrompt();
            }
            return 0;
          } else { // Buffer starts with ++ and contains at least 3 characters - command?
            if (pbPtr>2 && isCmd(pBuf) && !isPlusEscaped) { // Exclamation mark (break read loop command)
              if (pBuf[2]==0x21) {
                r = 3;
                flushPbuf();
              }else{
                r = 1;
              }
            }else if (pbPtr>3 && gpibBus.cfg.idn>0 && isIdnQuery(pBuf)){
              sendIdn = true;
              flushPbuf();
            }else if (pbPtr > 0) {
              r = 2;
            }
            isPlusEscaped = false;
          }
        }
        break;

      case ESC:
        if (isEsc) {
          addPbuf(c);
          isEsc = false;
        } else {
          isEsc  = true;  // Set escape flag
        }
        break;

      case PLUS:
        if (isEsc) {
          isEsc = false;
          if (pbPtr < 2) isPlusEscaped = true;
        }
        addPbuf(c);
        break;

      default:
        addPbuf(c);
        isEsc = false;
    }
  }

  if (pbPtr >= PBSIZE) {
    if (isCmd(pBuf) && !r) {  // Command without terminator and buffer full
      if (isVerbose) {
        dataPort.println(F("ERROR - Command buffer overflow!"));
      }
      flushPbuf();
    } else {
      dataBufferFull = true;
      r = 2;
    }
  }

  return r;
}

uint8_t serialIn_h() {
  uint8_t bufferStatus = 0;

  // Parse while characters available and line is not complete
  while (dataPort.available() && bufferStatus==0) bufferStatus = parseInput(dataPort.read());

  return bufferStatus;
}

void getCmd(char *buffr) {
  char *token;
  char *params;
  int i = 0;

  if (buffr[0] == 0x00 || buffr[0] == CR || buffr[0] == LF) return;

  token = strtok(buffr, " \t");

  for (i=0; cmdHidx[i].token; i++) if (strcasecmp(cmdHidx[i].token, token) == 0) break;
  
  if (cmdHidx[i].token) {
    if (cmdHidx[i].opmode & gpibBus.cfg.cmode) {
      params = token + strlen(token) + 1;
  
      if (strlen(params) > 0) {
        cmdHidx[i].handler(params);
      } else {
        cmdHidx[i].handler(NULL);
      }
    } else {
      errBadCmd();
      if (isVerbose) dataPort.println(F("getCmd: command not available in this mode."));
    }
  } else {
    errBadCmd();
  }
}

char line[PBSIZE];
void execCmd(char *buffr, uint8_t dsize) {
  memcpy(line, buffr+2, dsize-2); // save line stripping off the ++, clear pBuf
  flushPbuf();
  lnRdy = 0;

  line[dsize - 2] = line[dsize - 1] = '\0';

  if (isVerbose) dataPort.println(); // Shift output to next line
  getCmd(line);

  if (isVerbose) showPrompt();
}

void addPbuf(char c) {
  pBuf[pbPtr++] = c;
}

/******  Arduino standard SETUP procedure *****/
void setup() {
  wdt_disable();

  // Initialise parse buffer
  flushPbuf();

  AR_SERIAL_PORT.begin(AR_SERIAL_SPEED);

  if (!isEepromClear()) {
    if (!epReadData(gpibBus.cfg.db, GPIB_CFG_SIZE)) {
      epErase();
      gpibBus.setDefaultCfg();
      epWriteData(gpibBus.cfg.db, GPIB_CFG_SIZE);
    }
  }

  // Start the interface in the configured mode
  gpibBus.begin();

#if defined(USE_MACROS) && defined(RUN_STARTUP)
  execMacro(0);
#endif

  dataPort.flush();
}


/***** Arduino main loop *****/
void loop() {
  bool errFlg = false; 

#ifdef USE_MACROS
  if (runMacro > 0) {
    execMacro(runMacro);
    runMacro = 0;
  }
#endif

  // lnRdy=1: received a command so execute it...
  if (lnRdy == 1) {
    if (autoRead) {
      // Issuing any command stops autoread mode
      autoRead = false;
      gpibBus.unAddressDevice();
    }
    execCmd(pBuf, pbPtr);
  }

  if (gpibBus.isController()) {
    if (lnRdy == 2) { // lnRdy=2: received data - send it to the instrument...
      sendToInstrument(pBuf, pbPtr);
      if (gpibBus.cfg.amode == 1 || (gpibBus.cfg.amode == 2 && isQuery)) {
        errFlg = gpibBus.receiveData(dataPort, gpibBus.cfg.eoi, false, 0);
        isQuery = false;
      }
    }

    if (isSrqa) { // Automatic serial poll (check status of SRQ and SPOLL if asserted)?
      if (gpibBus.isAsserted(SRQ)) spoll_h(NULL);
    }

    if ((gpibBus.cfg.amode==3) && autoRead && !lnRdy) errFlg = gpibBus.receiveData(dataPort, readWithEoi, readWithEndByte, endByte);

    if (errFlg && isVerbose) {
      dataPort.println(F("Error while receiving data."));
      errFlg = false;
    }
  }

  if (sendIdn) { // IDN query
    if (gpibBus.cfg.idn==1) dataPort.println(gpibBus.cfg.sname);
    if (gpibBus.cfg.idn==2) {dataPort.print(gpibBus.cfg.sname);dataPort.print("-");dataPort.println(gpibBus.cfg.serial);}
    sendIdn = false;
  }

  // If charaters waiting in the serial input buffer then call handler
  if (dataPort.available()) lnRdy = serialIn_h();

  delayMicroseconds(5);
}

/***** Initialise controller mode *****/
void initController() {
  gpibBus.stop();
  gpibBus.startControllerMode();
}

bool isCmd(char *buffr) {
  if (buffr[0] == PLUS && buffr[1] == PLUS) return true;
  return false;
}

bool isIdnQuery(char *buffr) {
  if (strncasecmp(buffr, "*idn?", 5)==0) return true;
  return false;
}

bool isRead(char *buffr) {
  if (strncmp(buffr+2, "read", 4) == 0) return true;
  return false;
}

bool notInRange(char *param, uint16_t lowl, uint16_t higl, uint16_t &rval) {
  if (strlen(param) == 0) return true;

  // Convert to integer
  rval = 0;
  rval = atoi(param);

  // Check range
  if (rval < lowl || rval > higl) {
    errBadCmd();
    if (isVerbose) {
      dataPort.print(F("Valid range is between "));
      dataPort.print(lowl);
      dataPort.print(F(" and "));
      dataPort.println(higl);
    }
    return true;
  }
  return false;
}

#ifdef USE_MACROS
void execMacro(uint8_t idx) {
  char c;
  const char * macro = pgm_read_word(macros + idx);
  int ssize = strlen_P(macro);

  // Read characters from macro character array
  for (int i = 0; i < ssize; i++) {
    c = pgm_read_byte_near(macro + i);

    if (c == CR || c == LF || i == (ssize - 1)) {
      if (i == ssize-1) {
        if (pbPtr < (PBSIZE - 1)){
          if ((c != CR) && (c != LF)) addPbuf(c);
        } else {
          flushPbuf(); // Buffer full - clear and exit
          return;
        }
      }

      if (isCmd(pBuf)) execCmd(pBuf, strlen(pBuf));
      else sendToInstrument(pBuf, strlen(pBuf));

      flushPbuf();
    } else {
      if (pbPtr < (PBSIZE - 1)) addPbuf(c);
      else { // Exceeds buffer size - clear buffer and exit
        i = ssize;
        return;
      }
    }
  }

  flushPbuf();
}
#endif

void errBadCmd() {
  dataPort.println(F("Unrecognized command"));
}
void addr_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 1, 30, val)) return;
    if (val == gpibBus.cfg.caddr) {
      errBadCmd();
      if (isVerbose) dataPort.println(F("That is my address! Address of a remote device is required."));
      return;
    }
    gpibBus.cfg.paddr = val;
    if (isVerbose) {
      dataPort.print(F("Set device primary address to: "));
      dataPort.println(val);
    }
  } else {
    dataPort.println(gpibBus.cfg.paddr);
  }
}

void rtmo_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 1, 32000, val)) return;
    gpibBus.cfg.rtmo = val;
    if (isVerbose) {
      dataPort.print(F("Set [read_tmo_ms] to: "));
      dataPort.print(val);
      dataPort.println(F(" milliseconds"));
    }
  } else {
    dataPort.println(gpibBus.cfg.rtmo);
  }
}

void eos_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 3, val)) return;
    gpibBus.cfg.eos = (uint8_t)val;
    if (isVerbose) {
      dataPort.print(F("Set EOS to: "));
      dataPort.println(val);
    };
  } else {
    dataPort.println(gpibBus.cfg.eos);
  }
}

void eoi_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val)) return;
    gpibBus.cfg.eoi = val ? true : false;
    if (isVerbose) {
      dataPort.print(F("Set EOI assertion: "));
      dataPort.println(val ? "ON" : "OFF");
    };
  } else {
    dataPort.println(gpibBus.cfg.eoi);
  }
}

void cmode_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val)) return;
    switch (val) {
      case 0:
        gpibBus.startDeviceMode();
        break;
      case 1:
        gpibBus.startControllerMode();
        break;
    }
    if (isVerbose) {
      dataPort.print(F("Interface mode set to: "));
      dataPort.println(val ? "CONTROLLER" : "DEVICE");
    }
  } else {
    dataPort.println(gpibBus.isController());
  }
}

void eot_en_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val)) return;
    gpibBus.cfg.eot_en = val ? true : false;
    if (isVerbose) {
      dataPort.print(F("Appending of EOT character: "));
      dataPort.println(val ? "ON" : "OFF");
    }
  } else {
    dataPort.println(gpibBus.cfg.eot_en);
  }
}

void eot_char_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 255, val)) return;
    gpibBus.cfg.eot_ch = (uint8_t)val;
    if (isVerbose) {
      dataPort.print(F("EOT set to ASCII character: "));
      dataPort.println(val);
    };
  } else {
    dataPort.println(gpibBus.cfg.eot_ch, DEC);
  }
}

void amode_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 3, val)) return;
    if (val > 0 && isVerbose) {
      dataPort.println(F("WARNING: automode ON can cause some devices to generate"));
      dataPort.println(F("         'addressed to talk but nothing to say' errors"));
    }
    gpibBus.cfg.amode = (uint8_t)val;
    if (gpibBus.cfg.amode < 3) autoRead = false;
    if (isVerbose) {
      dataPort.print(F("Auto mode: "));
      dataPort.println(gpibBus.cfg.amode);
    }
  } else {
    dataPort.println(gpibBus.cfg.amode);
  }
}

void ver_h(char *params) {
  // If "real" requested
  if (params != NULL && strncasecmp(params, "real", 3) == 0) {
    dataPort.println(F(FWVER));
    // Otherwise depends on whether we have a custom string set
  } else {
    if (strlen(gpibBus.cfg.vstr) > 0) {
      dataPort.println(gpibBus.cfg.vstr);
    } else {
      dataPort.println(F(FWVER));
    }
  }
}

void read_h(char *params) {
  // Clear read flags
  readWithEoi = false;
  readWithEndByte = false;
  endByte = 0;
  // Read any parameters
  if (params != NULL) {
    if (strlen(params) > 3) {
      if (isVerbose) dataPort.println(F("Invalid parameter - ignored!"));
    } else if (strncasecmp(params, "eoi", 3) == 0) { // Read with eoi detection
      readWithEoi = true;
    } else { // Assume ASCII character given and convert to an 8 bit byte
      readWithEndByte = true;
      endByte = atoi(params);
    }
  }

  if (gpibBus.cfg.amode == 3) {
    // In auto continuous mode we set this flag to indicate we are ready for continuous read
    autoRead = true;
  } else {
    // If auto mode is disabled we do a single read
    gpibBus.addressDevice(gpibBus.cfg.paddr, TALK);
    gpibBus.receiveData(dataPort, readWithEoi, readWithEndByte, endByte);
  }
}

void clr_h() {
  if (gpibBus.sendSDC())  {
    if (isVerbose) dataPort.println(F("Failed to send SDC"));
    return;
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
}

void llo_h(char *params) {
  if (digitalRead(REN)==LOW) {
    if (params != NULL) {
      if (0 == strncmp(params, "all", 3)) {
        if (gpibBus.sendCmd(GC_LLO)) {
          if (isVerbose) dataPort.println(F("Failed to send universal LLO."));
        }
      }
    } else {
      // Send LLO to currently addressed device
      if (gpibBus.sendLLO()){
        if (isVerbose) dataPort.println(F("Failed to send LLO!"));
      }
    }
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
}

void loc_h(char *params) {
  if (digitalRead(REN)==LOW) {
    if (params != NULL) {
      if (strncmp(params, "all", 3) == 0) {
        // Send request to clear all devices to local
        gpibBus.sendAllClear();
      }
    } else {
      // Send GTL to addressed device
      if (gpibBus.sendGTL()) {
        if (isVerbose) dataPort.println(F("Failed to send LOC!"));
      }
      // Set GPIB controls back to idle state
      gpibBus.setControls(CIDS);
    }
  }
}

void ifc_h() {
  if (gpibBus.cfg.cmode==2) {
    // Assert IFC
    gpibBus.setControlVal(0b00000000, 0b00000001, 0);
    delayMicroseconds(150);
    // De-assert IFC
    gpibBus.setControlVal(0b00000001, 0b00000001, 0);
    if (isVerbose) dataPort.println(F("IFC signal asserted for 150 microseconds"));
  }
}

void trg_h(char *params) {
  char *param;
  uint8_t addrs[15] = {0};
  uint16_t val = 0;
  uint8_t cnt = 0;

  addrs[0] = addrs[0]; // Meaningless as both are zero but defaults compiler warning!

  if (params == NULL) {
    // No parameters - trigger addressed device only
    addrs[0] = gpibBus.cfg.paddr;
    cnt++;
  } else {
    // Read address parameters into array
    while (cnt < 15) {
      if (cnt == 0) {
        param = strtok(params, " \t");
      } else {
        param = strtok(NULL, " \t");
      }
      if (param == NULL) {
        break;  // Stop when there are no more parameters
      }else{    
        if (notInRange(param, 1, 30, val)) return;
        addrs[cnt] = (uint8_t)val;
        cnt++;
      }
    }
  }

  // If we have some addresses to trigger....
  if (cnt > 0) {
    for (int i = 0; i < cnt; i++) {
      // Sent GET to the requested device
      if (gpibBus.sendGET(addrs[i]))  {
        if (isVerbose) dataPort.println(F("Failed to trigger device!"));
        return;
      }
    }

    // Set GPIB controls back to idle state
    gpibBus.setControls(CIDS);

    if (isVerbose) dataPort.println(F("Group trigger completed."));
  }
}

void rst_h() {
#ifdef WDTO_1S
  // Where defined, reset controller using watchdog timeout
  unsigned long tout;
  tout = millis() + 2000;
  wdt_enable(WDTO_1S);
  while (millis() < tout) {};
  // Should never reach here....
  if (isVerbose) {
    dataPort.println(F("Reset FAILED."));
  };
#else
  // Otherwise restart program (soft reset)
  asm volatile ("  jmp 0");
#endif
}

void spoll_h(char *params) {
  char *param;
  uint8_t addrs[15];
  uint8_t sb = 0;
  uint8_t r;
  //  uint8_t i = 0;
  uint8_t j = 0;
  uint16_t addrval = 0;
  bool all = false;
  bool eoiDetected = false;

  // Initialise address array
  for (int i = 0; i < 15; i++) {
    addrs[i] = 0;
  }

  // Read parameters
  if (params == NULL) {
    // No parameters - trigger addressed device only
    addrs[0] = gpibBus.cfg.paddr;
    j = 1;
  } else {
    // Read address parameters into array
    while (j < 15) {
      if (j == 0) {
        param = strtok(params, " \t");
      } else {
        param = strtok(NULL, " \t");
      }
      // No further parameters so exit 
      if (!param) break;
      
      // The 'all' parameter given?
      if (strncmp(param, "all", 3) == 0) {
        all = true;
        j = 30;
        if (isVerbose) dataPort.println(F("Serial poll of all devices requested..."));
        break;
      // Valid GPIB address parameter ?
      } else if (strlen(param) < 3) { // No more than 2 characters
        if (notInRange(param, 1, 30, addrval)) return;
        addrs[j] = (uint8_t)addrval;
        j++;
      // Other condition
      } else {
        errBadCmd();
        if (isVerbose) dataPort.println(F("Invalid parameter"));
        return;
      }

    }
  }

  // Send Unlisten [UNL] to all devices
  if ( gpibBus.sendCmd(GC_UNL) ) return;

  // Controller addresses itself as listner
  if ( gpibBus.sendCmd(GC_LAD + gpibBus.cfg.caddr) ) return;

  // Send Serial Poll Enable [SPE] to all devices
  if ( gpibBus.sendCmd(GC_SPE) ) return;

  // Poll GPIB address or addresses as set by i and j
  for (int i = 0; i < j; i++) {
    if (all) addrval = i;
    else addrval = addrs[i];

    // Don't need to poll own address
    if (addrval != gpibBus.cfg.caddr) {
      if ( gpibBus.sendCmd(GC_TAD + addrval) ) return;
      // Set GPIB control to controller active listner state (ATN unasserted)
      gpibBus.setControls(CLAS);

      // Read the response byte (usually device status) using handshake - suppress EOI detection
      r = gpibBus.readByte(&sb, false, &eoiDetected);

      // If we successfully read a byte
      if (!r) {
        if (j == 30) {
          // If all, return specially formatted response: SRQ:addr,status but only when RQS bit set
          if (sb & 0x40) {
            dataPort.print(F("SRQ:")); dataPort.print(i); dataPort.print(F(",")); dataPort.println(sb, DEC);
            // Exit on first device to respond
            i = j;
          }
        } else {
          // Return decimal number representing status byte
          dataPort.println(sb, DEC);
          if (isVerbose) {
            dataPort.print(F("Received status byte ["));
            dataPort.print(sb);
            dataPort.print(F("] from device at address: "));
            dataPort.println(addrval);
          }
          // Exit on first device to respond
          i = j;
        }
      } else {
        if (isVerbose) dataPort.println(F("Failed to retrieve status byte"));
      }
    }
  }
  if (all) dataPort.println();

  // Send Serial Poll Disable [SPD] to all devices
  if ( gpibBus.sendCmd(GC_SPD) ) return;

  // Send Untalk [UNT] to all devices
  if ( gpibBus.sendCmd(GC_UNT) ) return;

  // Unadress listners [UNL] to all devices
  if ( gpibBus.sendCmd(GC_UNL) ) return;

  // Set GPIB control to controller idle state
  gpibBus.setControls(CIDS);

  if (isVerbose) dataPort.println(F("Serial poll completed."));
}

void srq_h() {
  dataPort.println(gpibBus.isAsserted(SRQ));
}

void stat_h(char *params) {
  uint16_t statusByte = 0;

  if (params != NULL) {
    if (notInRange(params, 0, 255, statusByte)) return;
    gpibBus.setStatus((uint8_t)statusByte);
  } else {
    dataPort.println(gpibBus.cfg.stat);
  }
}

void save_h() {
#ifdef E2END
  epWriteData(gpibBus.cfg.db, GPIB_CFG_SIZE);
  if (isVerbose) dataPort.println(F("Settings saved."));
#else
  dataPort.println(F("EEPROM not supported."));
#endif
}

void lon_h(char *params) {
  uint16_t lval;
  if (params != NULL) {
    if (notInRange(params, 0, 1, lval)) return;
    isRO = lval ? true : false;
    if (isRO) {
      isTO = 0;       // Talk-only mode must be disabled!
      isProm = false; // Promiscuous mode must be disabled!
    }
    if (isVerbose) {
      dataPort.print(F("LON: "));
      dataPort.println(lval ? "ON" : "OFF") ;
    }
  } else {
    dataPort.println(isRO);
  }
}

void help_h(char *params) {
  char c, t;
  char token[20];
  uint8_t i;

  i = 0;
  for (size_t k = 0; k < strlen_P(cmdHelp); k++) {
    c = pgm_read_byte_near(cmdHelp + k);
    if (i < 20) {
      if(c == ':') {
        token[i] = 0;
        if((params == NULL) || (strcmp(token, params) == 0)) {
          dataPort.print(F("++"));
          dataPort.print(token);
          dataPort.print(c);
          k++;
          t = pgm_read_byte_near(cmdHelp + k);
          dataPort.print(F(" ["));
          dataPort.print(t);
          dataPort.print(F("]"));
          i = 255; // means we need to print until \n
        }
        
      } else {
        token[i] = c;
        i++;
      }
    }
    else if (i == 255) {
      dataPort.print(c);
    }
    if (c == '\n') {
      i = 0;
    }
  }
}

void aspoll_h() {
  spoll_h((char*)"all");
}

void dcl_h() {
  if ( gpibBus.sendCmd(GC_DCL) )  {
    if (isVerbose) dataPort.println(F("Sending DCL failed"));
    return;
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
}

void default_h() {
  gpibBus.setDefaultCfg();
}

void eor_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 15, val)) return;
    gpibBus.cfg.eor = (uint8_t)val;
    if (isVerbose) {
      dataPort.print(F("Set EOR to: "));
      dataPort.println(val);
    };
  } else {
    if (gpibBus.cfg.eor>7) gpibBus.cfg.eor = 0;  // Needed to reset FF read from EEPROM after FW upgrade
    dataPort.println(gpibBus.cfg.eor);
  }
}

void ppoll_h() {
  uint8_t sb = 0;

  gpibBus.setControls(CIDS);
  delayMicroseconds(20);
  // Assert ATN and EOI
  gpibBus.setControlVal(0b00000000, 0b10010000, 0);
  //  setGpibState(0b10010000, 0b00000000, 0b10010000);
  delayMicroseconds(20);
  // Read data byte from GPIB bus without handshake
  sb = readGpibDbus();
  // Return to controller idle state (ATN and EOI unasserted)
  gpibBus.setControls(CIDS);

  // Output the response byte
  dataPort.println(sb, DEC);

  if (isVerbose) dataPort.println(F("Parallel poll completed."));
}

void ren_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val)) return;
    digitalWrite(REN, (val ? LOW : HIGH));
    if (isVerbose) {
      dataPort.print(F("REN: "));
      dataPort.println(val ? "REN asserted" : "REN un-asserted") ;
    };
  } else {
    dataPort.println(digitalRead(REN) ? 0 : 1);
  }
}

void verb_h() {
  isVerbose = !isVerbose;
  dataPort.print("Verbose: ");
  dataPort.println(isVerbose ? "ON" : "OFF");
}

void setvstr_h(char *params) {
  uint8_t plen;
  char idparams[64];
  plen = strlen(params);
  memset(idparams, '\0', 64);
  strncpy(idparams, "verstr ", 7);
  if (plen>47) plen = 47; // Ignore anything over 47 characters
  strncat(idparams, params, plen);

  id_h(idparams);
}

void prom_h(char *params) {
  uint16_t pval;
  if (params != NULL) {
    if (notInRange(params, 0, 1, pval)) return;
    isProm = pval ? true : false;
    if (isProm) {
      isTO = 0;     // Talk-only mode must be disabled!
      isRO = false; // Listen-only mode must be disabled!
    }
    if (isVerbose) {
      dataPort.print(F("PROM: "));
      dataPort.println(pval ? "ON" : "OFF") ;
    }
  } else {
    dataPort.println(isProm);
  }
}

void ton_h(char *params) {
  uint16_t toval;
  if (params != NULL) {
    if (notInRange(params, 0, 2, toval)) return;
    isTO = (uint8_t)toval;
    if (isTO>0) {
      isRO = false;   // Read-only mode must be disabled in TO mode!
      isProm = false; // Promiscuous mode must be disabled in TO mode!
    }
  }else{
    if (isVerbose) {
      dataPort.print(F("TON: "));
      switch (isTO) {
        case 1:
          dataPort.println(F("ON unbuffered"));
          break;
        case 2:
          dataPort.println(F("ON buffered"));
          break;
        default:
          dataPort.println(F("OFF"));
      }
    }
    dataPort.println(isTO);
  }
}

void srqa_h(char *params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 1, val)) return;
    switch (val) {
      case 0:
        isSrqa = false;
        break;
      case 1:
        isSrqa = true;
        break;
    }
    if (isVerbose) dataPort.println(isSrqa ? "SRQ auto ON" : "SRQ auto OFF") ;
  } else {
    dataPort.println(isSrqa);
  }
}

void repeat_h(char *params) {

  uint16_t count;
  uint16_t tmdly;
  char *param;

  if (params != NULL) {
    // Count (number of repetitions)
    param = strtok(params, " \t");
    if (strlen(param) > 0) {
      if (notInRange(param, 2, 255, count)) return;
    }
    // Time delay (milliseconds)
    param = strtok(NULL, " \t");
    if (strlen(param) > 0) {
      if (notInRange(param, 0, 30000, tmdly)) return;
    }

    // Pointer to remainder of parameters string
    param = strtok(NULL, "\n\r");
    if (strlen(param) > 0) {
      for (uint16_t i = 0; i < count; i++) {
        // Send string to instrument
        gpibBus.sendData(param, strlen(param));
        delay(tmdly);
        gpibBus.receiveData(dataPort, gpibBus.cfg.eoi, false, 0);
      }
    } else {
      errBadCmd();
      if (isVerbose) dataPort.println(F("Missing parameter"));
      return;
    }
  } else {
    errBadCmd();
    if (isVerbose) dataPort.println(F("Missing parameters"));
  }
}

void macro_h(char *params) {
#ifdef USE_MACROS
  uint16_t val;
  const char * macro;

  if (params != NULL) {
    if (notInRange(params, 0, 9, val)) return;
    runMacro = (uint8_t)val;
  } else {
    for (int i = 0; i < 10; i++) {
      macro = (pgm_read_word(macros + i));
      if (strlen_P(macro) > 0) {
        dataPort.print(i);
        dataPort.print(" ");
      }
    }
    dataPort.println();
  }
#else
  memset(params, '\0', 5);
  dataPort.println(F("Disabled"));
#endif
}

void xdiag_h(char *params){
  char *param;
  uint8_t mode = 0;
  uint8_t byteval = 0;
  
  // Get first parameter (mode = 0 or 1)
  param = strtok(params, " \t");
  if (param != NULL) {
    if (strlen(param)<4){
      mode = atoi(param);
      if (mode>2) {
        dataPort.println(F("Invalid: 0=data bus; 1=control bus"));
        return;
      }
    }
  }
  // Get second parameter (8 bit byte)
  param = strtok(NULL, " \t");
  if (param != NULL) {
    if (strlen(param)<4){
      byteval = atoi(param);
    }

    switch (mode) {
      case 0:
          // Set to required value
          gpibBus.setDataVal(byteval);
          // Reset after 10 seconds
          delay(10000);
          gpibBus.setDataVal(0);
          break;
      case 1:
          // Set to required state
          gpibBus.setControlVal(0xFF, 0xFF, 1);  // Set direction (all pins as outputs)
          gpibBus.setControlVal(~byteval, 0xFF, 0);  // Set state (asserted=LOW so must invert value)
          // Reset after 10 seconds
          delay(10000);
          if (gpibBus.cfg.cmode==2) {
            gpibBus.setControls(CINI);
          }else{
            gpibBus.setControls(DINI);
          }
          break;
    }
  }
}

void id_h(char *params) {
  uint8_t dlen = 0;
  char * keyword; // Pointer to keyword following ++id
  char * datastr; // Pointer to supplied data (remaining characters in buffer)
  char serialStr[10];

  if (params != NULL) {
    keyword = strtok(params, " \t");
    datastr = keyword + strlen(keyword) + 1;
    dlen = strlen(datastr);
    if (dlen) {
      if (strncasecmp(keyword, "verstr", 6)==0) {
        if (dlen>0 && dlen<48) {
          memset(gpibBus.cfg.vstr, '\0', 48);
          strncpy(gpibBus.cfg.vstr, datastr, dlen);
          if (isVerbose) dataPort.print(F("VerStr: "));dataPort.println(gpibBus.cfg.vstr);
        }else{
          if (isVerbose) dataPort.println(F("Length of version string must not exceed 48 characters!"));
          errBadCmd();
        }
        return;
      }
      if (strncasecmp(keyword, "name", 4)==0) {
        if (dlen>0 && dlen<16) {
          memset(gpibBus.cfg.sname, '\0', 16);
          strncpy(gpibBus.cfg.sname, datastr, dlen);
        }else{
          if (isVerbose) dataPort.println(F("Length of name must not exceed 15 characters!"));
          errBadCmd();
        }
        return;
      }
      if (strncasecmp(keyword, "serial", 6)==0) {
        if (dlen < 10) {
          gpibBus.cfg.serial = atol(datastr);
        }else{
          if (isVerbose) dataPort.println(F("Serial number must not exceed 9 characters!"));
          errBadCmd();
        }
        return;
      }
    }else{
      if (strncasecmp(keyword, "verstr", 6)==0) {
        dataPort.println(gpibBus.cfg.vstr);
        return;
      }
      if (strncasecmp(keyword, "fwver", 6)==0) {
        dataPort.println(F(FWVER));
        return;
      }
      if (strncasecmp(keyword, "name", 4)==0) {
        dataPort.println(gpibBus.cfg.sname);
        return;      
      } void addr_h(char *params);
      if (strncasecmp(keyword, "serial", 6)==0) {
        memset(serialStr, '\0', 10);
        snprintf(serialStr, 10, "%09lu", gpibBus.cfg.serial);  // Max str length = 10-1 i.e 9 digits + null terminator 
        dataPort.println(serialStr);
        return;    
      }
    }
  }
  errBadCmd();
}

void idn_h(char * params) {
  uint16_t val;
  if (params != NULL) {
    if (notInRange(params, 0, 2, val)) return;
    gpibBus.cfg.idn = (uint8_t)val;
    if (isVerbose) {
      dataPort.print(F("Sending IDN: "));
      dataPort.print(val ? "Enabled" : "Disabled"); 
      if (val==2) dataPort.print(F(" with serial number"));
      dataPort.println();
    };
  } else {
    dataPort.println(gpibBus.cfg.idn, DEC);
  }  
}

void sendmla_h() {
  if (gpibBus.sendMLA())  {
    if (isVerbose) dataPort.println(F("Failed to send MLA"));
    return;
  }
}

void sendmta_h() {
  if (gpibBus.sendMTA())  {
    if (isVerbose) dataPort.println(F("Failed to send MTA"));
    return;
  }
}

void sendmsa_h(char *params) {
  uint16_t saddr;
  char * param;
  if (params != NULL) {
    // Secondary address
    param = strtok(params, " \t");
    if (strlen(param) > 0) {
      if (notInRange(param, 96, 126, saddr)) return;
      if (gpibBus.sendMSA(saddr)){
        if (isVerbose) dataPort.println(F("Failed to send MSA"));
        return;
      }
    }
    // Secondary address command parameter
    param = strtok(NULL, " \t");
    if (strlen(param)>0) {
      gpibBus.setControls(CTAS);
      gpibBus.sendData(param, strlen(param));
      gpibBus.setControls(CLAS);
    }
  }
}

void unlisten_h() {
  if (gpibBus.sendUNL())  {
    if (isVerbose) dataPort.println(F("Failed to send UNL"));
    return;
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
}

void untalk_h() {
  if (gpibBus.sendUNT())  {
    if (isVerbose) dataPort.println(F("Failed to send UNT"));
    return;
  }
  // Set GPIB controls back to idle state
  gpibBus.setControls(CIDS);
}

void attnRequired() {

  const uint8_t cmdbuflen = 35;
  uint8_t cmdbytes[5] = {0};
  uint8_t db = 0;
  bool eoiDetected = false;
  uint8_t gpibcmd = 0;
  uint8_t bytecnt = 0;
  uint8_t atnstat = 0;
  uint8_t ustat = 0;
  bool addressed = false;

#ifdef EN_STORAGE
  uint8_t saddrcmd = 0;
#endif

  // Set device listner active state (assert NDAC+NRFD (low), DAV=INPUT_PULLUP)
  gpibBus.setControls(DLAS);

  // Read bytes received while ATN is asserted
  while ( (gpibBus.isAsserted(ATN)) && (bytecnt<cmdbuflen) ) {
    if (gpibBus.readByte(&db, false, &eoiDetected) > 0 ) break;
    switch (db) {
      case 0x3F:  
        ustat |= 0x01;
        break;
      case 0x5F:
        ustat |= 0x02;
        break;
      default:
        cmdbytes[bytecnt] = db;
        bytecnt++;
    }
  }

  atnstat |= 0x01;

  if (isProm) {
    device_listen_h();
    gpibBus.setControls(DINI);
    return;
  }

  if (ustat & 0x01) {
    if (!device_unl_h()) ustat &= 0x01; // Clears bit if UNL was not required
  }

  if (ustat & 0x01) {
    if (!device_unt_h()) ustat &= 0x02; // Clears bit if UNT was not required
  }

  if (bytecnt>0) {
    for (uint8_t i=0; i<bytecnt; i++) { 
      if (!cmdbytes[i]) break;  // End loop on zero
      db = cmdbytes[i];

      if (gpibBus.cfg.paddr == (db ^ 0x20)) { // MLA = db^0x20
        atnstat |= 0x02;
        addressed = true;
        gpibBus.setControls(DLAS);
      } else if (gpibBus.cfg.paddr == (db ^ 0x40)) { // MLA = db^0x40
        // Call talk handler to send data
        atnstat |= 0x04;
        addressed = true;
        gpibBus.setControls(DTAS);

#ifdef EN_STORAGE
      } else if (db>0x5F && db<0x80) {
        // Secondary addressing command received
        saddrcmd = db;
        atnstat |= 0x10;
#endif
      }else if (db<0x20){
        // Primary command received
        gpibcmd = db;
        atnstat |= 0x08;
      }
    }
  }

  if (!addressed) {
    gpibBus.setControls(DINI);
    return;
  }

  /***** If addressed, then perform the appropriate actions *****/
  if (addressed) {
    if (gpibcmd) {
      execGpibCmd(gpibcmd);
      gpibcmd = 0;
      atnstat |= 0x20;
      return;      
    }

#ifdef EN_STORAGE
    // If we have a secondary address then perform secondary GPIB command actions *****/
    if (addressed && saddrcmd) {
      // Execute the GPIB secondary addressing command
      storage.storeExecCmd(saddrcmd);
      // Clear secondary address command
      saddrcmd = 0;
      atnstat |= 0x40;
      return;
    }
#endif

    // If no GPIB commands but addressed to listen then just listen
    if (gpibBus.isDeviceAddressedToListen()) {
      device_listen_h();
      atnstat |= 0x80;
      gpibBus.setControls(DIDS);
      return;
    }
  
    // If no GPIB commands but addressed to talk then send data
    if (gpibBus.isDeviceAddressedToTalk()) {
      device_talk_h();
      atnstat |= 0x80;
      gpibBus.setControls(DIDS);
      return;
    }
  }
}

void execGpibCmd(uint8_t gpibcmd){

  // Respond to GPIB command
  switch (gpibcmd) {
    case GC_SPE:
      // Serial Poll enable request
        device_spe_h();
        break;
      case GC_SPD:
        // Serial poll disable request
        device_spd_h();
        break;       
    case GC_UNL:
        // Unlisten
        device_unl_h();
        break;
    case GC_UNT:
        // Untalk
        device_unt_h();
        break;
    case GC_SDC:
        // Device clear (reset)
        device_sdc_h();
        break;
  }
}

void device_listen_h(){
  gpibBus.receiveData(dataPort, false, false, 0x0);
}

void device_talk_h(){
  DB_PRINT("LnRdy: ", lnRdy);
  DB_PRINT("Buffer: ", pBuf);
  if (lnRdy == 2) gpibBus.sendData(pBuf, pbPtr);
  flushPbuf();
  lnRdy = 0;
}

void device_sdc_h() {
  if (isVerbose) dataPort.println(F("Resetting..."));
  rst_h();
  if (isVerbose) dataPort.println(F("Reset failed."));
}

void device_spd_h() {
  gpibBus.setControls(DIDS);
}

void device_spe_h() {
  gpibBus.sendStatus();

  if (gpibBus.cfg.stat & 0x40) {
    gpibBus.setStatus(gpibBus.cfg.stat & ~0x40);
  }
}


/***** Unlisten *****/
bool device_unl_h() {
  readWithEoi = false;
  tranBrk = 3;  // Stop receving transmission

  if (gpibBus.isDeviceAddressedToListen()) {
    gpibBus.setControls(DIDS);
    return true;
  }
  return false;
}

bool device_unt_h() {
  if (gpibBus.isDeviceAddressedToTalk()) {
    gpibBus.setControls(DIDS);
    gpibBus.clearDataBus();
    return true;
  }
  return false;
}

void lonMode() {

  uint8_t db = 0;
  uint8_t r = 0;
  bool eoiDetected = false;

  // Set bus for device read mode
  gpibBus.setControls(DLAS);

  while (isRO) {

    r = gpibBus.readByte(&db, false, &eoiDetected);
    if (r == 0) dataPort.write(db);

    if (dataPort.available()) {

      lnRdy = serialIn_h();

      // We have a command return to main loop and execute it
      if (lnRdy==1) break;

      // Clear the buffer to prevent it getting blocked
      if (lnRdy==2) flushPbuf();
    }
  }
  gpibBus.setControls(DIDS);
}


/***** Talk only mpode *****/
void tonMode() {

  // Set bus for device read mode
  gpibBus.setControls(DTAS);

  while (isTO>0) {

    // Check whether there are charaters waiting in the serial input buffer and call handler
    if (dataPort.available()) {
      if (isTO == 1) {
        // Unbuffered version
        gpibBus.writeByte(dataPort.read(), false);
      }

      if (isTO == 2) {
        // Buffered version
        lnRdy = serialIn_h();

        // We have a command return to main loop and execute it
        if (lnRdy==1) break;

        // Otherwise send the buffered data
        if (lnRdy==2) {
          for (uint8_t i=0; i<pbPtr; i++){
            gpibBus.writeByte(pBuf[i], false);  // False = No EOI
          }
          flushPbuf();
        }
      }
    }
  }
  gpibBus.setControls(DIDS);
}
