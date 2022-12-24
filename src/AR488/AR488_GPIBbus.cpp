#include <Arduino.h>
//#include <SD.h>
#include "AR488_Config.h"
#include "AR488_GPIBbus.h"

#define OK  false
#define ERR true

/***** Control characters *****/
#define ESC  0x1B   // the USB escape char
#define CR   0xD    // Carriage return
#define LF   0xA    // Newline/linefeed
#define PLUS 0x2B   // '+' character

GPIBbus::GPIBbus(){
  setDefaultCfg();
  cstate = 0;
  deviceAddressed = false;
}

void GPIBbus::begin(){
  if (isController()) startControllerMode();
  else startDeviceMode();
}

void GPIBbus::stop(){
  cstate = 0;
  // Input_pullup
  setGpibState(0b00000000, 0b11111111, 1);
  // All lines HIGH
  setGpibState(0b11111111, 0b11111111, 0);
  // Set data bus to default state (all lines input_pullup)
  readyGpibDbus();
}

void GPIBbus::setDefaultCfg(){
  // Set default values ({'\0'} sets version string array to null)
  cfg = {false, false, 2, 0, 1, 0, 0, 0, 0, 1200, 0, {'\0'}, 0, {'\0'}, 0, 0};
}

void GPIBbus::startDeviceMode(){
  // Stop current mode
  stop();
  delayMicroseconds(200); // Allow settling time
  // Start device mode
  cfg.cmode = 1;
  // Set GPIB control bus to device idle mode
  setControls(DINI);
  // Initialise GPIB data lines (sets to INPUT_PULLUP)
  readyGpibDbus();
}

void GPIBbus::startControllerMode(){
  // Send request to clear all devices on bus to local mode
  sendAllClear();
  // Stop current mode
  stop();
  delayMicroseconds(200); // Allow settling time
  // Start controller mode
  cfg.cmode = 2;
  // Set GPIB control bus to controller idle mode
  setControls(CINI);
  // Initialise GPIB data lines (sets to INPUT_PULLUP)
  readyGpibDbus();
  // Assert IFC to signal controller in charge (CIC)
  sendIFC();
  // Attempt to address device to listen
  if (cfg.paddr > 1) addressDevice(cfg.paddr, 0);
}

bool GPIBbus::isController(){
  if (cfg.cmode == 2) return true;
  return false;
}

bool GPIBbus::isAsserted(uint8_t gpibsig){
  // Use digitalRead function to get current Arduino pin state
  return (digitalRead(gpibsig) == LOW) ? true : false;
}

void GPIBbus::sendStatus() {
  // Have been addressed and polled so send the status byte
  if (!(cstate==DTAS)) setControls(DTAS);
  writeByte(cfg.stat, NO_EOI);
  setControls(DIDS);
  // Clear the SRQ bit
  cfg.stat = cfg.stat & ~0x40;
  // De-assert the SRQ signal
  clrSrqSig();
}

void GPIBbus::setStatus(uint8_t statusByte){
  cfg.stat = statusByte;
  if (statusByte & 0x40) {
    // If SRQ bit is set then assert the SRQ signal
    setSrqSig();
  } else {
    // If SRQ bit is NOT set then de-assert the SRQ signal
    clrSrqSig();
  }
}

void GPIBbus::sendIFC(){
  // Assert IFC
  setGpibState(0b00000000, 0b00000001, 0);
  delayMicroseconds(150);
  // De-assert IFC
  setGpibState(0b00000001, 0b00000001, 0);
}

bool GPIBbus::sendSDC(){
  if (addressDevice(cfg.paddr, 0) || sendCmd(GC_SDC)) return ERR;
  if (unAddressDevice()) return ERR;
  return OK;
}

bool GPIBbus::sendLLO(){
  if (addressDevice(cfg.paddr, 0) || sendCmd(GC_LLO)) return ERR;  
  if (unAddressDevice()) return ERR;
  return OK;
}

bool GPIBbus::sendGTL(){
  if (addressDevice(cfg.paddr, 0) || sendCmd(GC_GTL)) return ERR;
  if (unAddressDevice()) return ERR;  
  return OK;
}

bool GPIBbus::sendGET(uint8_t addr){
  if (addressDevice(addr, 0) || sendCmd(GC_GET)) return ERR;
  if (unAddressDevice()) return ERR;  
  return OK;
}

void GPIBbus::sendAllClear(){
  // Un-assert REN
  setControlVal(0b00100000, 0b00100000, 0);
  delay(40);
  // Simultaneously assert ATN and REN
  setControlVal(0b00000000, 0b10100000, 0);
  delay(40);
  // Unassert ATN
  setControlVal(0b10000000, 0b10000000, 0);
}

bool GPIBbus::sendMTA(){
  if (cstate!=CCMS) setControls(CCMS);
  if (addressDevice(cfg.paddr, 1)) return ERR;
  return OK;
}

bool GPIBbus::sendMLA(){
  if (cstate!=CCMS) setControls(CCMS);
  if (addressDevice(cfg.paddr, 0)) return ERR;
  return OK;
}

bool GPIBbus::sendMSA(uint8_t addr) {
  if (sendCmd(addr)) return ERR;
  // Unassert ATN
  setControlVal(0b10000000, 0b10000000, 0);
  return OK;
}

bool GPIBbus::sendUNT(){
  if (sendCmd(GC_UNT)) return ERR;
  setControls(CIDS);
  deviceAddressed = false;
  return OK;
}

bool GPIBbus::sendUNL(){
  if (sendCmd(GC_UNL)) return ERR;
  setControls(CIDS);
  deviceAddressed = false;
  return OK;
}

bool GPIBbus::sendCmd(uint8_t cmdByte){
  bool stat = false;
  // Set lines for command and assert ATN
  if (cstate!=CCMS) setControls(CCMS);
  // Send the command
  stat = writeByte(cmdByte, NO_EOI);
  return stat ? ERR : OK;
}

bool GPIBbus::receiveData(Stream& dataStream, bool detectEoi, bool detectEndByte, uint8_t endByte) {

  uint8_t r = 0; //, db;
  uint8_t bytes[3] = {0};
  uint8_t eor = cfg.eor&7;
  int x = 0;
  bool readWithEoi = false;
  bool eoiDetected = false;

  endByte = endByte;  // meaningless but defeats vcompiler warning!

  // Reset transmission break flag
  txBreak = 0;

  // EOI detection required ?
  if (cfg.eoi || detectEoi || (cfg.eor==7)) readWithEoi = true;    // Use EOI as terminator

  // Set up for reading in Controller mode
  if (cfg.cmode == 2) {   // Controler mode
    
    addressDevice(cfg.paddr, 1);
    // Wait for instrument ready
    // Set GPIB control lines to controller read mode
    setControls(CLAS);
    
  // Set up for reading in Device mode
  } else {  // Device mode
    // Set GPIB controls to device read mode
    setControls(DLAS);
    readWithEoi = true;  // In device mode we read with EOI by default
  }
  readyGpibDbus();

  // Perform read of data (r=0: data read OK; r>0: GPIB read error);
  while (r == 0) {

    // Tranbreak > 0 indicates break condition
    if (txBreak) break;

    // ATN asserted
    if (isAsserted(ATN)) break;

    // Read the next character on the GPIB bus
    r = readByte(&bytes[0], readWithEoi, &eoiDetected);

    if (isAsserted(ATN)) r = 2;

    // If IFC or ATN asserted then break here
    if ( (r==1) || (r==2) ) break;

    // If successfully received character
    if (r==0) {
      dataStream.print((char)bytes[0]);
      x++;

      // EOI detection enabled and EOI detected?
      if (readWithEoi) {
        if (eoiDetected) break;
      }else{
        // Has a termination sequence been found ?
        if (detectEndByte) {
          if (r == endByte) break;
        }else{
          if (isTerminatorDetected(bytes, eor)) break;
        }
      }

      // Shift last three bytes in memory
      bytes[2] = bytes[1];
      bytes[1] = bytes[0];
    }else{
      // Stop (error or timeout)
      break;
    }
  }

  // Detected that EOI has been asserted
  if (eoiDetected && cfg.eot_en) dataStream.print(cfg.eot_ch);

  // Return controller to idle state
  if (cfg.cmode == 2) {
    // Untalk bus and unlisten controller
    unAddressDevice();
    // Set controller back to idle state
    setControls(CIDS);
  } else {
    // Set device back to idle state
    setControls(DIDS);
  }

  // Reset break flag
  if (txBreak) txBreak = false;
  if (r > 0) return ERR;

  return OK;
}

void GPIBbus::sendData(char *data, uint8_t dsize) {

  bool err = false;
  // Controler can unlisten bus and address devices
  if (cfg.cmode == 2) {
    // Set control lines to write data (ATN unasserted)
    setControls(CTAS);
  } else {
    setControls(DTAS);
  }

  // Write the data string
  for (int i = 0; i < dsize; i++) {
    // If EOI asserting is on
    if (cfg.eoi) {
      // Send all characters
      err = writeByte(data[i], NO_EOI);
    } else {
      // Otherwise ignore non-escaped CR, LF and ESC
      if ((data[i] != CR) || (data[i] != LF) || (data[i] != ESC)) err = writeByte(data[i], NO_EOI);
    }
    if (err) break;
  }

  if (!err) {
    // Write terminators according to EOS setting
    // Do we need to write a CR?
    if ((cfg.eos & 0x2) == 0) {
      writeByte(CR, NO_EOI);
    }
    // Do we need to write an LF?
    if ((cfg.eos & 0x1) == 0) {
      writeByte(LF, NO_EOI);
    }
  }

  // If EOI enabled and no more data to follow then assert EOI
  if (cfg.eoi) {
    setGpibState(0b00010000, 0b00010000, 1);
    setGpibState(0b00000000, 0b00010000, 0);
    delayMicroseconds(40);
    setGpibState(0b00010000, 0b00010000, 0);
  }

  if (cfg.cmode == 2) {   // Controller mode
    // Controller - set lines to idle?
    setControls(CIDS);

  } else {    // Device mode
    // Set control lines to idle
    setControls(DIDS);
  }
}

void GPIBbus::signalBreak(){
  txBreak = true;
}


/***** Control the GPIB bus - set various GPIB states *****/
/*
 * state is a predefined state (CINI, CIDS, CCMS, CLAS, CTAS, DINI, DIDS, DLAS, DTAS);
 * Bits control lines as follows: 8-ATN, 7-SRQ, 6-REN, 5-EOI, 4-DAV, 3-NRFD, 2-NDAC, 1-IFC
 * setGpibState byte1 (databits) : State - 0=LOW, 1=HIGH/INPUT_PULLUP; Direction - 0=input, 1=output;
 * setGpibState byte2 (mask)     : 0=unaffected, 1=enabled
 * setGpibState byte3 (mode)     : 0=set pin state, 1=set pin direction
 */
void GPIBbus::setControls(uint8_t state) {

  // Switch state
  switch (state) {

    // Controller states
    case CINI:  // Initialisation
      // Set pin direction
      setGpibState(0b10111000, 0b11111111, 1);
      // Set pin state
      setGpibState(0b11011111, 0b11111111, 0);
      break;

    case CIDS:  // Controller idle state
      setGpibState(0b10111000, 0b10011110, 1);
      setGpibState(0b11011111, 0b10011110, 0);
      break;

    case CCMS:  // Controller active - send commands
      setGpibState(0b10111001, 0b10011111, 1);
      setGpibState(0b01011111, 0b10011111, 0);
      break;

    case CLAS:  // Controller - read data bus
      // Set state for receiving data
      setGpibState(0b10100110, 0b10011110, 1);
      setGpibState(0b11011000, 0b10011110, 0);
      break;

    case CTAS:  // Controller - write data bus
      setGpibState(0b10111001, 0b10011110, 1);
      setGpibState(0b11011111, 0b10011110, 0);
      break;

    /* Bits control lines as follows: 8-ATN, 7-SRQ, 6-REN, 5-EOI, 4-DAV, 3-NRFD, 2-NDAC, 1-IFC */

    // Listener states
    case DINI:  // Listner initialisation
      setGpibState(0b00000000, 0b11111111, 1);
      setGpibState(0b11111111, 0b11111111, 0);
      // Set data bus to idle state
      readyGpibDbus();
      break;

    case DIDS:  // Device idle state
      setGpibState(0b00000000, 0b00001110, 1);
      setGpibState(0b11111111, 0b00001110, 0);
      // Set data bus to idle state
      readyGpibDbus();
      break;

    case DLAS:  // Device listner active (actively listening - can handshake)
      setGpibState(0b00000110, 0b00011110, 1);
      setGpibState(0b11111001, 0b00011110, 0);
      break;

    case DTAS:  // Device talker active (sending data)
      setGpibState(0b00011000, 0b00011110, 1);
      setGpibState(0b11111001, 0b00011110, 0);
      break;
    default:
      setGpibState(0b00000110, 0b10111001, 0b11111111);
  }
  cstate = state;
}

void GPIBbus::setControlVal(uint8_t value, uint8_t mask, uint8_t mode){
  setGpibState(value, mask, mode);
}

void GPIBbus::setDataVal(uint8_t value){
  setGpibDbus(value);
}

bool GPIBbus::unAddressDevice() {
  // De-bounce
  delayMicroseconds(30);
  // Utalk/unlisten
  if (sendCmd(GC_UNL)) return ERR;
  if (sendCmd(GC_UNT)) return ERR;
  // Clear flag
  deviceAddressed = false;
  return OK;
}

bool GPIBbus::addressDevice(uint8_t addr, bool talk) {
  if (sendCmd(GC_UNL)) return ERR;
  if (talk) {
    // Device to talk, controller to listen
    if (sendCmd(GC_TAD + addr)) return ERR;
  } else {
    // Device to listen, controller to talk
    if (sendCmd(GC_LAD + addr)) return ERR;
  }

  // Set flag
  deviceAddressed = true;
  return OK;
}

bool GPIBbus::haveAddressedDevice(){
  return deviceAddressed;
}

bool GPIBbus::isDeviceAddressedToListen(){
  if (cstate == DLAS) return true;
  return false;
}

bool GPIBbus::isDeviceAddressedToTalk(){
  if (cstate == DTAS) return true;
  return false;
}

bool GPIBbus::isDeviceInIdleState(){
  if (cstate == DIDS) return true;
  return false;
}

void GPIBbus::clearDataBus(){
  readyGpibDbus();
}

uint8_t GPIBbus::readByte(uint8_t *db, bool readWithEoi, bool *eoi) {

  unsigned long startMillis = millis();
  unsigned long currentMillis = startMillis + 1;
  const unsigned long timeval = cfg.rtmo;
  uint8_t stage = 4;

  bool atnStat = isAsserted(ATN); // Capture state of ATN
  *eoi = false;

  // Wait for interval to expire
  while ( (unsigned long)(currentMillis - startMillis) < timeval ) {

    if (cfg.cmode == 1) {
      // If IFC has been asserted then abort
      if (isAsserted(IFC)) {
        stage = 1;
        break;    
      }

      // ATN unasserted during handshake - not ready yet so abort (and exit ATN loop)
      if ( atnStat && !isAsserted(ATN) ){
        stage = 2;
        break;
      }
    }

    if (stage == 4) {
      // Unassert NRFD (we are ready for more data)
      setGpibState(0b00000100, 0b00000100, 0);
      stage = 6;
    }

    if (stage == 6) {
      // Wait for DAV to go LOW indicating talker has finished setting data lines..
      if (getGpibPinState(DAV) == LOW) {
        // Assert NRFD (Busy reading data)
        setGpibState(0b00000000, 0b00000100, 0);
        stage = 7;
      }
    }

    if (stage == 7) {
      // Check for EOI signal
      if (readWithEoi && isAsserted(EOI)) *eoi = true;
      // read from DIO
      *db = readGpibDbus();
      // Unassert NDAC signalling data accepted
      setGpibState(0b00000010, 0b00000010, 0);
      stage = 8;
    }

    if (stage == 8) {
      // Wait for DAV to go HIGH indicating data no longer valid (i.e. transfer complete)
      if (getGpibPinState(DAV) == HIGH) {
        // Re-assert NDAC - handshake complete, ready to accept data again
        setGpibState(0b00000000, 0b00000010, 0);
        stage = 9;
        break;     
      }
    }

    // Increment time
    currentMillis = millis();
  }

  // Completed, or return where we got to
  if (stage == 9) return 0;
  return stage;
}

uint8_t GPIBbus::writeByte(uint8_t db, bool isLastByte) {
  unsigned long startMillis = millis();
  unsigned long currentMillis = startMillis + 1;
  const unsigned long timeval = cfg.rtmo;
  uint8_t stage = 4;

  // Wait for interval to expire
  while ( (unsigned long)(currentMillis - startMillis) < timeval ) {

    if (cfg.cmode == 1) {
      // If IFC has been asserted then abort
      if (isAsserted(IFC)) {
        setControls(DLAS);       
        stage = 1;
        break;    
      }

      // If ATN has been asserted we need to abort and listen
      if (isAsserted(ATN)) {
        setControls(DLAS);       
        stage = 2;
        break;
      }
    }

    // Wait for NDAC to go LOW (indicating that devices (stage==4) || (stage==8) ) are at attention)
    if (stage == 4) {
      if (getGpibPinState(NDAC) == LOW) stage = 5;
    }

    // Wait for NRFD to go HIGH (indicating that receiver is ready)
    if (stage == 5) {
      if (getGpibPinState(NRFD) == HIGH) stage = 6;
    }

    if (stage == 6){
      // Place data on the bus
      setGpibDbus(db);
      if (cfg.eoi && isLastByte) {
        // If EOI enabled and this is the last byte then assert DAV and EOI
        setGpibState(0b00000000, 0b00011000, 0);
      }else{
        // Assert DAV (data is valid - ready to collect)
        setGpibState(0b00000000, 0b00001000, 0);
      }
      stage = 7;
    }

    if (stage == 7) {
      // Wait for NRFD to go LOW (receiver accepting data)
      if (getGpibPinState(NRFD) == LOW) stage = 8;
    }

    if (stage == 8) {
      // Wait for NDAC to go HIGH (data accepted)
      if (getGpibPinState(NDAC) == HIGH) {
        stage = 9;
        break;
      }
    }

    // Increment time
    currentMillis = millis();
  }

  // Handshake complete
  if (stage == 9) {
    if (cfg.eoi && isLastByte) {
      // If EOI enabled and this is the last byte then un-assert both DAV and EOI
      if (cfg.eoi && isLastByte) setGpibState(0b00011000, 0b00011000, 0);
    }else{
      // Unassert DAV
      setGpibState(0b00001000, 0b00001000, 0);
    }
    // Reset the data bus
    setGpibDbus(0);
    return 0;
  }

  // Otherwise timeout or ATN/IFC return stage at which it ocurred
  return stage;
}

bool GPIBbus::isTerminatorDetected(uint8_t bytes[3], uint8_t eorSequence){
  // Look for specified terminator (CR+LF by default)
  switch (eorSequence) {
    case 0:
        // CR+LF terminator
        if (bytes[0]==LF && bytes[1]==CR) return true;
        break;
    case 1:
        // CR only as terminator
        if (bytes[0]==CR) return true;
        break;
    case 2:
        // LF only as terminator
        if (bytes[0]==LF) return true;
        break;
    case 3:
        // No terminator (will rely on timeout)
        break;
    case 4:
        // Keithley can use LF+CR instead of CR+LF
        if (bytes[0]==CR && bytes[1]==LF) return true;
        break;
    case 5:
        // Solarton (possibly others) can also use ETX (0x03)
        if (bytes[0]==0x03) return true;
        break;
    case 6:
        // Solarton (possibly others) can also use CR+LF+ETX (0x03)
        if (bytes[0]==0x03 && bytes[1]==LF && bytes[2]==CR) return true;
        break;
    default:
        // Use CR+LF terminator by default
        if (bytes[0]==LF && bytes[1]==CR) return true;
        break;
  }
  return false;
}

void GPIBbus::setSrqSig() {
  // Set SRQ line to OUTPUT HIGH (asserted)
  setGpibState(0b01000000, 0b01000000, 1);
  setGpibState(0b00000000, 0b01000000, 0);
}

void GPIBbus::clrSrqSig() {
  // Set SRQ line to INPUT_PULLUP (un-asserted)
  setGpibState(0b00000000, 0b01000000, 1);
  setGpibState(0b01000000, 0b01000000, 0);
}
