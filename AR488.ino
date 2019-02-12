/*
Arduino IEEE-488 implementation by John Chajecki

Inspired by the original work of Emanuele Girlando, licensed under a Creative
Commons Attribution-NonCommercial-NoDerivatives 4.0 International License.
Any code in common with the original work is reproduced here with the explicit
permission of Emanuele Girlando, who has kindly reviewed and tested this code.

Thanks also to Luke Mester for comparison testing against the Prologix interface.
AR488 is Licenced under the GNU Public licence.
*/

// Includes
#include <EEPROM.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>

// Firmware version
#define FWVER "AR488 GPIB controller, version 0.45.11, 12/02/2019"


// Debug options
//#define DEBUG1  // getCmd                                                                    
//#define DEBUG2  // setGpibControls
//#define DEBUG3  // gpibSendData
//#define DEBUG4  // spoll_h
//#define DEBUG5  // attnRequired
//#define DEBUG6  // EEPROM
//#define DEBUG7  // gpibReceiveData
//#define DEBUG8  // ppoll_h


/*
 * Implements most of the CONTROLLER functions;
 * Substantially compatible with 'standard' Prologix "++" commands 
 * (see +savecfg command in the manual for differences)
 *
 * Principle of operation:
 * - Commands received from USB are buffered and whole terminated lines processed 
 * - Interface commands prefixed with "++" are passed to the command handler
 * - Instrument commands and data not prefixed with '++' are sent directly to the GPIB bus.
 * - To receive from the instrument, issue a ++read command or put the controller in auto mode (++auto 1)
 * - Characters received over the GPIB bus are unbuffered and sent directly to USB
 * NOTES: 
 * - GPIB line in a HIGH state is un-asserted
 * - GPIB line in a LOW state is asserted 
 * - The ATMega processor control pins have a high impedance when set as inputs
 * - When set to INPUT_PULLUP, a 10k pull-up (to VCC) resistor is applied to the input
 */

/*
 * Standard commands
 *
 * ++addr         - display/set device address
 * ++auto         - automatically request talk and read response
 * ++clr          - send Selected Device Clear to current GPIB address
 * ++eoi          - enable/disable assertion of EOI signal
 * ++eos          - specify GPIB termination character
 * ++eot_enable   - enable/disable appending user specified character to USB output on EOI detection
 * ++eot_char     - set character to append to USB output when EOT enabled
 * ++ifc          - assert IFC signal for 150 miscoseconds - make AR488 controller in charge
 * ++llo          - local lockout - disable front panel operation on instrument
 * ++loc          - enable front panel operation on instrument
 * ++mode         - set the interface mode (0=controller/1=device)
 * ++read         - read data from instrument
 * ++read_tmo_ms  - read timeout specified between 1 - 3000 milliseconds
 * ++rst          - reset the controller
 * ++savecfg      - save configration
 * ++spoll        - serial poll the addressed host or all instruments
 * ++srq          - return status of srq signal (1-srq asserted/0-srq not asserted)
 * ++status       - set the status byte to be returned on being polled (bit 6 = RQS, i.e SRQ asserted)
 * ++trg          - send trigger to selected devices (up to 15 addresses) 
 * ++ver          - display firmware version
 */
 
/* 
 *  Proprietry commands:
 *  
 * ++aspoll       - serial poll all instruments (alias: ++spoll all)
 * ++default      - set configuration to controller default settings
 * ++dcl          - send unaddressed (all) device clear  [power on reset] (is the rst?)
 * //++ren          - assert or unassert the REN signal
 * ++ppoll        - conduct a parallel poll
 * ++setvstr      - set custom version string (to identify controller, e.g. "GPIB-USB"). Max 47 chars, excess truncated.
 * ++verbose      - verbose (human readable) mode
 * 
 */

/*
 * NOT YET IMPLEMENTED
 * 
 * ++help     - show summary of commands
 * ++debug    - debug mode (0=off, 1=basic, 2=verbose) [maybe best left in ifdef statements?]
 * ++lon      - put controller in listen-only mode (listen to all traffic)
 * ++myaddr   - aset the controller address
 * 
 */


/*
 * Mapping between the Arduino pins and the GPIB connector.
 * NOTE:
 * GPIB pins 10 and 18-24 are connected to GND
 * GPIB pin 12 should be connected to the cable shield (might be n/c)
 * Pin mapping follows the layout originally used by Emanuelle Girlando, but adds
 * the SRQ line (GPIB 10) on pin 2 and the REN line (GPIB 17) on pin 13. The program
 * should therefore be compatible with the original interface design but for full 
 * functionality will need the remaining two pins to be connected.
 * For further information about the AR488 see:
 * 
 * For information regarding the GPIB firmware by Emanualle Girlando see:
 * http://egirland.blogspot.com/2014/03/arduino-uno-as-usb-to-gpib-controller.html
 */

// Internal LED
const int LED = 13;

// NOTE: Pinout last updated 09/01/2019
#define DIO1  A0  /* GPIB 1  : PORTC bit 0 */
#define DIO2  A1  /* GPIB 2  : PORTC bit 1 */
#define DIO3  A2  /* GPIB 3  : PORTC bit 2 */
#define DIO4  A3  /* GPIB 4  : PORTC bit 3 */
#define DIO5  A4  /* GPIB 13 : PORTC bit 4 */
#define DIO6  A5  /* GPIB 14 : PORTC bit 5 */
#define DIO7  4   /* GPIB 15 : PORTD bit 4 */
#define DIO8  5   /* GPIB 16 : PORTD bit 5 */

#define IFC   8   /* GPIB 9  : PORTB bit 0 */
#define NDAC  9   /* GPIB 8  : PORTB bit 1 */
#define NRFD  10  /* GPIB 7  : PORTB bit 2 */
#define DAV   11  /* GPIB 6  : PORTB bit 3 */
#define EOI   12  /* GPIB 5  : PORTB bit 4 */

#define SRQ   2   /* GPIB 10 : PORTD bit 2 */
#define REN   3   /* GPIB 17 : PORTD bit 3 */
#define ATN   7   /* GPIB 11 : PORTD bit 7 */


/*
 * PIN interrupts
 */
#define ATNint 0b10000000
#define SRQint 0b00000100


/*
 * GPIB BUS commands
 */
// Universal Multiline commands (apply to all devices)
#define LLO 0x11
#define DCL 0x14
#define PPU 0x15
#define SPE 0x18
#define SPD 0x19
#define UNL 0x3F
#define TAD 0x40
#define PPE 0x60
#define PPD 0x70
#define UNT 0x5F
// Address commands
#define LAD 0x20
// Addressed commands
#define GTL 0x01
#define SDC 0x04
#define PPC 0x05
#define GET 0x08


/*
 * GPIB control states
 */
// Controller mode
#define CINI 0x01 // Controller idle state
#define CIDS 0x02 // Controller idle state
#define CCMS 0x03 // Controller command state
#define CTAS 0x04 // Controller talker active state
#define CLAS 0x05 // Controller listner active state
// Listner/device mode
#define DINI 0x06 // Device initialise state
#define DIDS 0x07 // Device idle state
#define DLAS 0x08 // Device listener active (listening/receiving)
#define DTAS 0x09 // Device talker active (sending) state


/*
 * Process status values
 */
#define OK 0
#define ERR 1


/*
 * Control characters
 */
#define ESC  0x1B   // the USB escape char
#define CR   0xD    // Carriage return
#define LF   0xA    // Newline/linefeed
#define PLUS 0x2B   // '+' character


/* 
 * PARSE BUFFER 
 */
// Serial input parsing buffer
static const uint8_t PBSIZE = 128;
char pBuf[PBSIZE];
uint8_t pbPtr = 0;


/*
 * Controller configuration
 * Default values set for controller mode
 */     
struct AR488conf {
  uint8_t ew;     // EEPROM write indicator byte
  bool eot_en;    // Enable/disable append EOT char to string received from GPIB bus before sending to USB
  bool eoi;       // Assert EOI on last data char written to GPIB - 0-disable, 1-enable      
  uint8_t cmode;  // Controller mode - 0=unset, 1=device, 2=controller
  uint8_t caddr;  // Controller address
  uint8_t paddr;  // Primary device address
  uint8_t saddr;  // Secondary device address
  uint8_t eos;    // EOS (end of send - to GPIB) character flag [0=CRLF, 1=CR, 2=LF, 3=None]
  uint8_t stat;   // Status byte to return in response to a poll
  int rtmo;       // Read timout (read_tmo_ms) in milliseconds - 0-3000 - value depends on instrument
  char eot_ch;    // EOT character to append to USB output when EOI signal detected
  char vstr[48];  // Custom version string
};

struct AR488conf AR488;


/*
 * Global variables with volatile values related to controller state
 */

// GPIB control state
uint8_t cstate = 0;

// Verbose mode
bool isVerb = false;

// We have a line
//bool crFl = false;          // Carriage return flag
uint8_t lnRdy = 0;      // Line ready to process

// GPIB data receive flags
bool isAuto = false;    // Auto read mode?
bool isReading = false; // Is a GPIB read in progress?
bool aRead = false;     // GPIB data read in progress
bool rEoi = false;      // read eoi requested
bool rEbt = false;      // read with specified terminator character
uint8_t tranBrk = 0;    // transmission break on 1=++, 2=EOI, 3=ATN 4=UNL
uint8_t eByte = 0;      // termination character

// Device mode - send data
bool snd = false;

// Escaped character flag
bool isEsc = false;   // Charcter escaped
bool isPle = false;   // Plus escaped
 
// Received serial poll request?
bool isSprq = false;

// Read only mode flag
bool isRO = false;

// GPIB command parser
bool aTt = false;
bool aTl = false;

// Interrupts
volatile uint8_t pindMem = PIND;
volatile bool isATN = false;  // has ATN been asserted?
volatile bool isSRQ = false;  // has SRQ been asserted?

// SRQ auto mode
bool isSrqa = false;

volatile bool isBAD = false;




/*
 *****  Arduino SETUP procedure *****
 */
void setup() {

  // Turn off internal LED (set OUPTUT/LOW)
  DDRB |= 0b00100000;
  PORTB &= 0b11011111;
  
  // Turn on interrupts
  cli();
//  PCICR |= 0b00000001;  // PORTB
  PCICR |= 0b00000100;  // PORTD
  sei();

  // Initialise parse buffer
  flushPbuf();

  // Start the serial port
  Serial.begin(115200);
  
  // Initialise
  initAR488();

  // Read data from non-volatile memory
  //(will only read if previous config has already been saved)
  epGetCfg();

  // Print version string
//  if (strlen(AR488.vstr)>0) {
//    Serial.println(AR488.vstr);
//  }else{
//    Serial.println(FWVER);
//  }

  // Initialize the interface in device mode
  if (AR488.cmode==1) initDevice();

  // Initialize the interface in controller mode
  if (AR488.cmode==2) initController(); 

  isATN = false;
  isSRQ = false;

  // Save state of the PORTD pins
  pindMem = PIND;

}


/* 
 *****   MAIN LOOP   *****
 */
void loop() {

  int bytes = 0;

  // NOTE: serialEvent() handles serial interrupt
  // Each received char is passed through parser until an un-escaped CR
  // is encountered. If we have a command then parse and execute.
  // If the line is data (inclding direct instrument commands) then
  // send it to the instrument.

  // NOTE: parseInput() ser lnRdy in serialEvent or readBreak
  // lnRdy=1: process command;
  // lnRdy=2: send data to Gpib

  // lnRdy=1: received a command so execute it...
  if (lnRdy==1) {
    processLine(pBuf, pbPtr, 1);
  }

  // Controller mode:
  if (AR488.cmode==2) {
    // lnRdy=2: received data - send it to the instrument...
    if (lnRdy==2) {
      processLine(pBuf,pbPtr, 2);
    }

    // Check status of SRQ and SPOLL if asserted
    if (isSRQ) {
      spoll_h(NULL);
      isSRQ = false;
    }
    
    // Auto-receive data from GPIB bus
    if (isAuto && aRead) gpibReceiveData();
  }

  // Device mode:
  // Check whether ATN asserted, get command then send or receive
  if (AR488.cmode==1) {
    if (isATN) {  // ATN asserted
      attnRequired();
    }else{
      // Not allowed to send data in listen only (++lon) mode so clear the buffer and ATT flag
      if (isRO && lnRdy==2) {
        flushPbuf();
        aTt = false;
      }
      // Addressed to talk
      if (aTt) {
        if (isSprq) {
          gpibSendStatus();
        }else{
          if (lnRdy==2) {
            processLine(pBuf, pbPtr, 2);
          }
        }
        aTt = false;  // Clear talk flag
      }
      if (aTl){ // Addressed to listen
        gpibReceiveData();
        aTl = false;  // Clear listen flag
      }
    }
  }

  delayMicroseconds(5);
}
/***** END MAIN LOOP *****/


/*
 * Initialise the interface
 */
void initAR488() {
  // Set default values
  AR488 = {0xCC,false,false,2,0,1,0,0,0,1200,0,'\0'};

  // Clear version string variable
  memset(AR488.vstr, '\0', 48);
}


/*
 * Initialise device mode
 */
void initDevice() {
  // Set GPIB control bus to device idle mode
  setGpibControls(DINI);
  // Disable SRQ and enable ATN interrupt
  cli();
//  PCMSK2 &= SRQint;
  PCMSK2 |= ATNint;
  sei();
  // Initialise GPIB data lines (sets to INPUT_PULLUP)
  readGpibDbus();
}


/*
 * Initialise controller mode
 */
void initController() {
  // Set GPIB control bus to controller idle mode
  setGpibControls(CINI);  // Controller initialise state
  // Disable ATN and enable SRQ interrupt
  cli();
  PCMSK2 &= ATNint;
//  PCMSK2 |= SRQint;
  sei();
  // Initialise GPIB data lines (sets to INPUT_PULLUP)
  readGpibDbus();
  // Assert IFC to signal controller in charge (CIC)
  ifc_h();
}


/*
 * Serial event interrupt handler
 */
void serialEvent(){
  lnRdy = parseInput(Serial.read());
}


/*
 * Interrupt data transfer when escape pressed
 */
void readBreak(){
  // Check serial input to see if we need to break on ++ character
  if (Serial.available()) {   // Only need to parse if a character is available
    lnRdy = parseInput(Serial.read());
    if (lnRdy==1) tranBrk = 7;
  }

  // Check whether EOI is asserted
  if (digitalRead(EOI)==LOW) tranBrk = 5;
}


/******************************/
/***** Interrupt handlers *****/
/******************************/

// Catches mis-spelled ISR vectors
#pragma GCC diagnostic error "-Wmisspelled-isr"       

// Interrupt for ATN pin
ISR(PCINT2_vect){

  // Has PCINT23 fired (ATN asserted)?
  if (AR488.cmode==1) { // Only in device mode
    if ((PIND^pindMem) & ATNint) {
      if (PIND & ATNint) {
        // isATN is false when ATN is unasserted (HIGH)
        isATN = false;  
      }else{
        // isATN is true when ATN is asserted (LOW)
        isATN = true;
      }
    }
  }

  // Has PCINT19 fired (SRQ asserted)?
  if (AR488.cmode==2) { // Only in controller mode
    if ((PIND^pindMem) & SRQint) {
      if (PIND & SRQint) {
        // isSRQ is false when SRQ is unasserted (HIGH)
        isSRQ = false;  
     }else{
        // isSRQ is true when SRQ is asserted (LOW)
        isSRQ = true;
      }
    }

  }
  
  // Save current state of PORTD register
  pindMem = PIND;

}


// Catchall interrupt vector
/*
ISR(BADISR_vect) {
  // ISR to catch ISR firing without handler
  isBAD = true;
}
*/


/*************************************/
/***** Device operation routines *****/
/*************************************/


/*
 * Unrecognized command
 */
void errBadCmd(){
  Serial.println(F("Unrecognized command"));
}


/*
 * Read configuration from EEPROM
 */
void epGetCfg() {
  int ew = 0x00;
  int epaddr = 0;
  int val;
  val = EEPROM.read(0);
  if (val==0xCC){
    EEPROM.get(epaddr, AR488);
  }
}


/*
 * Save configuraton to EEPROM
 * 4 x 128byte 'pages'
 */
//uint8_t epSaveCfg(uint8_t page){
uint8_t epSaveCfg(){

  long int sz;
  int epaddr=0;

//  if (page<0 || page>4) return ERR;

  sz = sizeof(AR488);
#ifdef debug5
  Serial.print(F("Size of structure: "));
  Serial.println(sz);
#endif
//  epaddr = 128 * (page-1);
  EEPROM.put(epaddr, AR488);
  if (isVerb) Serial.print(F("Settings saved."));
//  if (isVerb) { Serial.print(F("Settings saved to configuration profile ")); Serial.println(page); };
}


/*
 * Add character to the buffer and parse
 */
uint8_t parseInput(char c) {

  uint8_t r = 0;

  // Read until buffer full (buffer-size - 2 characters)
  if (pbPtr<PBSIZE-2){
    // Actions on specific characters
    switch (c) {
      // Carriage return or newline? Then process the line
      case CR:
      case LF:
          // Character must not be escaped
          if (isEsc) {
            addPbuf(c);
            isEsc = false;
          }else{
            // Carriage return on blank line?
            if (pbPtr==0) {
              flushPbuf();
              if (isVerb) {
                Serial.println();
                Serial.print("> ");
              }
              return 0;
            }else{
#ifdef DEBUG1
              Serial.print(F("parseInput: Received ")); Serial.println(pBuf);
#endif
              // Buffer starts with ++ and contains at least 3 characters - command?
              if (pbPtr>2 && isCmd(pBuf) && !isPle) {
//                if (isReading) tranBrk = 7;
                r = 1;
              // Buffer has at least 1 character
              }else if (pbPtr>0){ // Its other data (or instrument commands from user) so pass characters to GPIB bus
                r = 2;
              }
              isPle= false;
              return r;
            }
          }
          break;
      case ESC:
          // Handle the escape character         
          if (isEsc) {
            // Add character to buffer and cancel escape
            addPbuf(c);
            isEsc = false;
          }else{
            // Set escape flag
            isEsc  = true;  // Set escape flag
          }
          break;
      case PLUS:
          if (isEsc) {
            isEsc = false;
            if (pbPtr<2) isPle = true;
          }
          addPbuf(c);
          if (isVerb) Serial.print(c);

          // Break on '++'?
//          if (pbPtr==2 && isCmd(pBuf)) {
//            if (isReading){
//              tranBrk = 1;
//              isAuto = false;
//            }            
//          }
          break;
      // Something else?
      default: // any char other than defined above
          if (isVerb) Serial.print(c);  // Humans like to see what they are typing...
          // Buffer contains '++' (start of command). Stop sending data to serial port by halting GPIB receive.
          addPbuf(c);
          isEsc = false;
      }
  }else{
    // Buffer full - cannot be a command so treat as data and pass to GPIB bus
    r=2;
  }
  return r;
}


/*
 * Is this a command?
 */
bool isCmd(char *buffr){
  if (buffr[0]==PLUS && buffr[1]==PLUS) {
#ifdef DEBUG1
    Serial.println(F("isCmd: Command detected."));
#endif
    return true;
  }
  return false;
}


/*
 * ++read command detected?
 */
bool isRead(char *buffr){
  char cmd[4];
  // Copy 2nd to 5th character
  for (int i=2; i<6; i++){
    cmd[i-2] = buffr[i];
  }
  // Compare with 'read'
  if (strncmp(cmd, "read", 4) == 0) return true;
  return false;
}


/*
 * Add character to the buffer
 */
void addPbuf(char c){
  pBuf[pbPtr] = c;
  pbPtr++;
}


/*
 * Clear the parse buffer 
 */
void flushPbuf() {
  memset(pBuf, '\0', PBSIZE);
  pbPtr = 0;
}


/*
 * Command handler record structure
 * Record: command-token, command-handler, modes-available 
 * NOTE: modes: 1=device, 2=controller, 3=both
 */
struct cmdRec { 
  char* token; 
  void (*handler)(char *params);
  int modes;
};


/*
 * Array containing index of accepted ++ commands
 */
static cmdRec cmdHidx [] = { 
  { "addr",        addr_h,      3 }, 
  { "allspoll",    aspoll_h,    2 },
  { "auto",        amode_h,     2 },
  { "clr",         clr_h,       2 },
  { "dcl",         dcl_h,       2 },
  { "default",     default_h,   3 },
  { "eoi",         eoi_h,       3 },
  { "eos",         eos_h,       3 },
  { "eot_char",    eot_char_h,  3 },
  { "eot_enable",  eot_en_h,    3 },
  { "ifc",         ifc_h,       2 },
  { "llo",         llo_h,       2 },
  { "loc",         loc_h,       2 },
//  { "lon",         lon_h,       1 },
  { "mode" ,       cmode_h,     3 },
  { "ppoll",       ppoll_h,     2 },
  { "read",        read_h,      2 },
  { "read_tmo_ms", rtmo_h,      2 },
  { "ren",         ren_h,       2 },
  { "rst",         rst_h,       3 },
  { "trg",         trg_h,       2 },
  { "savecfg",     save_h,      3 },
  { "setvstr",     setvstr_h,   3 },
  { "spoll",       spoll_h,     2 },
  { "srq",         srq_h,       2 },
  { "srqauto",     srqa_h,      2 },
  { "status",      stat_h,      1 },
  { "ver",         ver_h,       3 },
  { "verbose" ,    verb_h,      3 }
};


/*
 * Execute a ++command or send characters to instrument
 * mode: 1=command; 2=data;
 */
void processLine(char *buffr, uint8_t dsize, uint8_t mode) {
  char line[PBSIZE];

  // Copy collected chars to line buffer
  memcpy(line, buffr, dsize);

  // Flush the parse buffer
  flushPbuf();
  lnRdy = 0;
  
#ifdef DEBUG1
  Serial.print(F("processLine: Received: ")); printHex(line, dsize);
#endif   

  // Line contains a valid command after the ++
  if (mode==1) {
    // Its a ++command so shift everything two bytes left (ignore ++) and parse
    for (int i=0; i<dsize-2; i++){
      line[i] = line[i+2];
    }
    // Replace last two bytes with a null (\0) character
    line[dsize-2] = '\0';
    line[dsize-1] = '\0';
#ifdef DEBUG1
    Serial.print(F("processLine: Sent to the command processor: ")); printHex(line, dsize-2);
#endif   
    // Execute the command
    if (isVerb) Serial.println(); // Shift output to next line
    getCmd(line);
  }

  // This line is not a ++command, so if in controller mode, pass characters to GPIB
//  if (mode==2 && AR488.cmode==2){
  if (mode==2){
#ifdef DEBUG1    
    Serial.print(F("processLine: Sent to the instrument: ")); printHex(line, dsize);
#endif    
    gpibSendData(line, dsize);
  }

  // Show a prompt on completion?
  if (isVerb) {
    // Print prompt
    Serial.println();
    Serial.print("> ");
  }  

}


/*
 *   Extract command and pass to handler
 */
void getCmd(char *buffr) {

  char *token;  // pointer to command token
  char params[64]; // pointer to command parameters
  int casize = sizeof(cmdHidx)/sizeof(cmdHidx[0]);
  int i=0, j=0;

  memset(params, '\0', 64);
  
#ifdef DEBUG1
  Serial.print("getCmd: ");
  Serial.print(buffr); Serial.print(F(" - length:"));Serial.println(strlen(buffr));
#endif

  if (*buffr == (NULL || CR || LF) ) return; // empty line: nothing to parse.
  token=strtok(buffr, " \t");

#ifdef DEBUG1
  Serial.print("getCmd: process token: "); Serial.println(token);
#endif

  // Look for a valid command token
  i=0;
  do {
    if (strcmp(cmdHidx[i].token, token) == 0) break;
    i++;
  } while (i<casize);
  if (i<casize) {
    // We have found a valid command and handler
#ifdef DEBUG1
    Serial.print("getCmd: "); 
    Serial.print("found handler for: "); Serial.println(cmdHidx[i].token);
#endif
    // If command is relevant to controller mode then execute it
    if (cmdHidx[i].modes&AR488.cmode) {
      // Copy command parameters to params and call handler with parameters
      do {
        j++;
        token = strtok(NULL, " \t");
        if (strlen(token)>0) {
          if (j>1) {strcat(params, " ");};
          strcat(params, token);
        }
      } while (token != NULL);
      if (strlen(params)>0) {
#ifdef DEBUG1
        Serial.print(F("Calling handler with parameters: ")); Serial.println(params);
#endif
        cmdHidx[i].handler(params);
      }else{
        // Call handler without parameters
        cmdHidx[i].handler(NULL);
      }
    }else{
      errBadCmd();
      if (isVerb) Serial.println(F("Command not available in this mode."));
    }
  }else{
  // No valid command found
  errBadCmd();
  }
}


void printHex(char *buffr, int dsize){
  for (int i=0; i<dsize; i++){
    Serial.print(buffr[i], HEX); Serial.print(" "); 
  }
  Serial.println();
}


/*************************************/
/***** STANDARD COMMAND HANDLERS *****/
/*************************************/

/*
 * Show or change device address
 */
void addr_h(char *params) { 
//  char *param, *stat; 
  char *param;
  int val;
  if (params!=NULL) {
    // Primary address
    param = strtok(params, " \t");
    if (strlen(param)>0) {
      val = atoi(param);
      if (val<1 || val>30) {
        errBadCmd();
        if (isVerb) Serial.println(F("Invalid: GPIB primary address is in the range 1 - 30")); return;
      }
      if (val == AR488.caddr) {
        errBadCmd();
        if (isVerb) Serial.println(F("That is my address! Address of a remote device is required."));
        return;
      }
      AR488.paddr=val;
      AR488.saddr=0;
      if (isVerb) {Serial.print(F("Set device primary address to: ")); Serial.println(val);}
    }
    // Secondary address
    val = 0;
    param = strtok(NULL, " \t");
    val = atoi(param);
    if (val>0) {
      if (val<96 || val>126) {
        errBadCmd();
        if (isVerb) Serial.println(F("Invalid: GPIB secondary address is in the range 96 - 126"));
        return;
      }
      AR488.saddr=val;
      if (isVerb) {Serial.print("Set device secondary address to: "); Serial.println(val);};
    }
  }else{
    Serial.print(AR488.paddr);
    if (AR488.saddr>0) {Serial.print(F(" ")); Serial.print(AR488.saddr);}
    Serial.println();
  }
}


/*
 * Show or set read timout
 */
void rtmo_h(char *params) { 
  int val;
  if (params!=NULL) {
    val = atoi(params);
    if (val<1||val>3000) {
      errBadCmd();
      if (isVerb) Serial.println(F("Invalid: valid timout range is 1 - 3000 ms."));
      return;
    }
    AR488.rtmo=val;
    if (isVerb) {Serial.print(F("Set [read_tmo_ms] to: ")); Serial.print(val); Serial.println(F(" milliseconds"));}
  }else{
    Serial.println(AR488.rtmo); 
  }
}


/*
 * Show or set end of send character
 */
void eos_h(char *params) {
  int val;
  if (params!=NULL) {
    val = atoi(params);
    if (val<0||val>3) {
      errBadCmd();
      if (isVerb) Serial.println(F("Invalid: expected EOS value of 0 - 3"));
      return;
    }
    AR488.eos=val; if (isVerb) {Serial.print(F("Set EOS to: ")); Serial.println(val);};
  }else{
    Serial.println(AR488.eos);
  }
}


/*
 * Show or set EOI assertion on/off
 */
void eoi_h(char *params) { 
  int val;
  if (params!=NULL) {
    val = atoi(params);
    if (val<0||val>1) {
      errBadCmd();
      if (isVerb) Serial.println(F("Invalid: expected EOI value of 0 or 1"));
      return;
    }
    AR488.eoi = val ? true : false; if (isVerb) {Serial.print(F("Set EOI assertion: ")); Serial.println(val ? "ON" : "OFF");};
  }else{
    Serial.println(AR488.eoi);
  }
}


/*
 * Show or set interface to controller/device mode
 */
void cmode_h(char *params) {
  int val;
  if (params!=NULL) {
    val = atoi(params);
    if (val<0||val>1) {
      errBadCmd();
      if (isVerb) Serial.println(F("Invalid: mode must be 0 or 1"));
      return;
    }
    switch (val) {
      case 0:
          AR488.cmode = 1;
          initDevice();
          break;
      case 1:
          AR488.cmode = 2;
          initController();
          break;
    }
    if (isVerb) {Serial.print(F("Interface mode set to: ")); Serial.println(val ? "CONTROLLER" : "DEVICE");}
  }else{
    Serial.println(AR488.cmode-1);
  }
}


/*
 * Show or enable/disable sending of end of transmission character
 */
void eot_en_h(char *params) { 
  int val;
  if (params!=NULL) {
    val = atoi(params);
    if (val<0||val>1) {
      errBadCmd();
      if (isVerb) Serial.println(F("Invalid: expected EOT value of 0 or 1"));
      return;
    }
    AR488.eot_en = val ? true : false;
    if (isVerb) {Serial.print(F("Set append of EOT character: ")); Serial.println(val ? "ON" : "OFF");}
  }else{
    Serial.println(AR488.eot_en);
  }
}


/*
 * Show or set end of transmission character
 */
void eot_char_h(char *params) {
  int val;
  if (params!=NULL) {
    val = atoi(params); 
    if (val<0||val>256) {
      errBadCmd();
      if (isVerb) Serial.println(F("Invalid: expected EOT character ASCII value in the range 0 - 255"));
      return;
    }
    AR488.eot_ch = val;
    if (isVerb) {Serial.print(F("EOT set to ASCII character: ")); Serial.println(val);};
  }else{
    Serial.println(AR488.eot_ch, DEC);
  }
}


/*
 * Show or enable/disable auto mode
 */
void amode_h(char *params) {
  int val;
  if (params!=NULL) { 
    val = atoi(params);
    if (val<0||val>1) {
      errBadCmd();
      if (isVerb) Serial.println(F("Automode: valid range is [0-disable|1-enable]."));
      return;
    }
    if (val>0 && isVerb) {
      Serial.println(F("WARNING: automode ON can cause some devices to generate"));
      Serial.println(F("         'addressed to talk but nothing to say' errors"));
    }
    isAuto = val ? true : false; if (isVerb) {Serial.print(F("Auto mode: ")); Serial.println(val ? "ON" : "OFF") ;}
  }else{
    Serial.println(isAuto);
  }
}


/*
 * Display the controller version string
 */
void ver_h(char *params) {
  // If "real" requested
  if (params!=NULL && strncmp(params, "real", 3)==0) { 
    Serial.println(F(FWVER));
  // Otherwise depends on whether we have a custom string set
  }else{
    if (strlen(AR488.vstr)>0) {
      Serial.println(AR488.vstr);
    }else{
      Serial.println(F(FWVER));
    }
  }
}


/*
 * Address device to talk and read the sent data
 */
void read_h(char *params) {
  // Clear read flags
  rEoi = false;
  rEbt = false;
  // Read any parameters
  if (params!=NULL) {
    if (strlen(params) > 3){
      if (isVerb) Serial.println(F("Invalid termination character - ignored."));
    }else if (strncmp(params, "eoi", 3)==0){  // Read with eoi detection
      rEoi = true;
    }else{  // Assume ASCII character given and convert to an 8 bit byte
      rEbt = true;
      eByte = atoi(params);
    }
  }
  if (isAuto){
    // in auto mode we set this flag to indicate we are ready for auto read
    aRead = true;
  }else{
    // If auto mode is disabled we do a single read
    gpibReceiveData();
  }
}


/*
 * Send device clear (usually resets the device to power on state)
 */
void clr_h() {
  if (addrDev(AR488.paddr, 0)) {
    if (isVerb) Serial.println(F("Failed to address device")); 
    return; 
  }
  if (gpibSendCmd(SDC))  { 
    if (isVerb) Serial.println(F("Failed to send SDC")); 
    return; 
  }
  if (uaddrDev()) {
    if (isVerb) Serial.println(F("Failed to untalk GPIB bus")); 
    return; 
  }
  // Set GPIB controls back to idle state
  setGpibControls(CIDS);
}


/*
 * Send local lockout command
 */
void llo_h(char *params) {
  // NOTE: REN *MUST* be asserted (LOW)
  bool ren = digitalRead(REN);
  if (!ren) {
    // For 'all' send LLO to the bus without addressing any device - device will show REM when addressed 
    if (params!=NULL) { 
      if (0 == strncmp(params, "all", 3)) {
        if (gpibSendCmd(LLO)){
          if (isVerb) Serial.println(F("Failed to send universal LLO."));
        }
      }
    }else{
      // Address device
      if (addrDev(AR488.paddr, 0)) {
        if (isVerb) Serial.println(F("Failed to address the device."));
        return;
      }
      // Send LLO to currently addressed device
      if (gpibSendCmd(LLO)){
        if (isVerb) Serial.println(F("Failed to send LLO to device"));
        return;
      }
      // Unlisten bus
      if (uaddrDev()){
        if (isVerb) Serial.println(F("Failed to unlisten the GPIB bus"));
        return;
      }
    }
  }
  // Set GPIB controls back to idle state
  setGpibControls(CIDS);
}


/*
 * Send Go To Local (GTL) command
 */
void loc_h(char *params) {
  // REN *MUST* be asserted (LOW)
  bool ren = digitalRead(REN);
  if (!ren) {
    if (params!=NULL) { 
      if (strncmp(params, "all", 3)==0) {
        // Unassert REN
        setGpibState(0b00100000, 0b00100000, 0b00100000);        
        delay(40);
        // Simultaneously assert ATN and REN
        setGpibState(0b10100000, 0b00000000, 0b10100000);
        delay(40);
        // Unassert ATN
        setGpibState(0b10000000, 0b10000000, 0b10000000);
        // Return REN to previous state
        digitalWrite(REN, ren);        
      }
    }else{ 
      // De-assert REN
      setGpibState(0b00100000, 0b00100000, 0b00100000);
      // Address device to listen
      if (addrDev(AR488.paddr, 0)) {
        if (isVerb) Serial.println(F("Failed to address device."));
        return;
      }
      // Send GTL      
      if (gpibSendCmd(GTL)) {
        if (isVerb) Serial.println(F("Failed sending LOC."));
        return;
      }
      // Unlisten bus
      if (uaddrDev()){
        if (isVerb) Serial.println(F("Failed to unlisten GPIB bus."));
        return;
      }
      // Re-assert REN
      setGpibState(0b00100000, 0b00000000, 0b00100000);        

      // Set GPIB controls back to idle state
      setGpibControls(CIDS);
    }
  }
}


 /* 
  * Assert IFC for 150 microseconds making the AR488 the Controller-in-Charge
  * All interfaces return to their idle state
  */
void ifc_h() {
  if (AR488.cmode) {
    setGpibState(0b00000001, 0b00000000, 0b00000001);        
    delayMicroseconds(150);
    setGpibState(0b00000001, 0b00000001, 0b00000001);        
    if (isVerb) Serial.println(F("IFC signal asserted for 150 microseconds"));
  }
}


/*
 * Send a trigger command
 */
void trg_h(char *params) {
  char *param;
  uint8_t addrs[15];
  uint8_t val = 0;
  uint8_t cnt = 0;

  // Initialise address array
  for (int i; i<15; i++){
    addrs[i] = 0;
  }

  // Read parameters
  if (params==NULL){
    // No parameters - trigger addressed device only
    addrs[0] = AR488.paddr;
    cnt++;
  }else{
    // Read address parameters into array
    while (cnt<15) {
      if (cnt==0) {
        param = strtok(params, " \t");
      }else{
        param = strtok(NULL, " \t");
      }
      val = atoi(param);
      if (!val) break;
      if (val<1 || val>30) {
        errBadCmd();
        if (isVerb) Serial.println(F("Invalid: GPIB primary address is in the range 1 - 30"));
        return;
      }
      addrs[cnt] = val;
      cnt++;
    }
  }    

  // If we have some addresses to trigger....
  if (cnt>0){
    for (int i; i<cnt; i++){
      // Address the device
      if (addrDev(addrs[i],0)) {
        if (isVerb) Serial.println(F("Failed to address device")); 
        return;
      }
      // Send GTL
      if (gpibSendCmd(GET))  { 
        if (isVerb) Serial.println(F("Failed to trigger device")); 
        return; 
      }
      // Unaddress device
      if (uaddrDev()) {
        if (isVerb) Serial.println(F("Failed to unlisten GPIB bus")); 
        return;
      }
    }

    // Set GPIB controls back to idle state
    setGpibControls(CIDS);
    
    if (isVerb) Serial.println(F("Group trigger completed."));
  }
}


/*
 * Reset the controller
 */
void rst_h() {
  // Reset controller using watchdog timeout
  unsigned long tout;
  tout = millis()+3000;
  wdt_enable(WDTO_60MS);
  while(millis() < tout) {};
  // Should never reach here....
  if (isVerb) {Serial.println(F("Reset FAILED."));};
}


/*
 * Serial Poll Handler
 */
void spoll_h(char *params) {
  char *param;
  uint8_t addrs[15];
  uint8_t sb = 0;
  uint8_t r;
  uint8_t i = 0;
  uint8_t j = 0;
  uint8_t val = 0;
  bool all = false;

  // Initialise address array
  for (int i; i<15; i++){
    addrs[i] = 0;
  }

  // Read parameters
  if (params==NULL){
    // No parameters - trigger addressed device only
    addrs[0] = AR488.paddr;
    j = 1;
  }else{
    // Read address parameters into array
    while (j<15) {
      if (j==0) {
        param = strtok(params, " \t");
      }else{
        param = strtok(NULL, " \t");
      }
      // The 'all' parameter given?
      if (strncmp(param, "all", 3)==0){
        all=true;
        j = 30;
        if (isVerb) Serial.println(F("Serial poll of all devices requested..."));
        break;
      // Read all address parameters
      }else if (strlen(params)<3) { // No more than 2 characters
        val = atoi(param);
        if (!val) break;
        if (val<1 || val>30) {
          errBadCmd();
          if (isVerb) Serial.println(F("Invalid: GPIB addresses are in the range 1-30"));
          return;
        }
        addrs[j] = val;
        j++;
      }else{
        errBadCmd();
        if (isVerb) Serial.println(F("Invalid parameter"));
        return;      
      }
    }
  }

  // Send Unlisten [UNL] to all devices
  if ( gpibSendCmd(UNL) )  { 
#ifdef DEBUG4
    Serial.println(F("spoll_h: failed to send UNL"));
#endif
    return; 
  }

  // Controller addresses itself as listner
  if ( gpibSendCmd(LAD+AR488.caddr) )  {
#ifdef DEBUG4
    Serial.println(F("spoll_h: failed to send LAD")); 
#endif
    return; 
  }

  // Send Serial Poll Enable [SPE] to all devices
  if ( gpibSendCmd(SPE) )  {
#ifdef DEBUG4
    Serial.println(F("spoll_h: failed to send SPE")); 
#endif
    return; 
  }

  // Poll GPIB address or addresses as set by i and j
  for (i; i<j; i++){

    // Set GPIB address in val
    if (all) {
      val = i;
    }else{
      val = addrs[i];
    }

    // Don't need to poll own address
    if (val != AR488.caddr) {

      // Address a device to talk
      if ( gpibSendCmd(TAD+val) )  {

#ifdef DEBUG4   
        Serial.println(F("spoll_h: failed to send TAD")); 
#endif
        return; 
      }

      // Set GPIB control to controller active listner state (ATN unasserted)
      setGpibControls(CLAS);

      // Read the response byte (usually device status) using handshake
      r = gpibReadByte(&sb);

      // If we successfully read a byte
      if (!r) {
        if (j>1) {
          // If all, return specially formatted response: SRQ:addr,status
          // but only when RQS bit set
          if (sb&0x40) {
            Serial.print(F("SRQ:"));Serial.print(i);Serial.print(F(","));Serial.println(sb,DEC);
            i = j;
          }
        }else{
          // Return decimal number representing status byte
          Serial.println(sb, DEC);
          if (isVerb) {Serial.print(F("Received status byte [")); Serial.print(sb); Serial.print(F("] from device at address: ")); Serial.println(val);}
          i = j;
        }
      }else{
        if (isVerb) Serial.println(F("Failed to retrieve status byte"));
      }
    }
  }
  if (all) Serial.println();

  // Send Serial Poll Disable [SPD] to all devices
  if ( gpibSendCmd(SPD) )  {
#ifdef DEBUG4
    Serial.println(F("spoll_h: failed to send SPD")); 
#endif
    return; 
  }

  // Send Untalk [UNT] to all devices
  if ( gpibSendCmd(UNT) )  {
#ifdef DEBUG4  
    Serial.println(F("spoll_h: failed to send UNT")); 
#endif
    return; 
  }

  // Unadress listners [UNL] to all devices
  if ( gpibSendCmd(UNL) )  {
#ifdef DEBUG4    
    Serial.println(F("spoll_h: failed to send UNL")); 
#endif
    return; 
  }

  // Set GPIB control to controller idle state
  setGpibControls(CIDS);

  // Set SRQ to status of SRQ line. Should now be unasserted but, if it is
  // still asserted, then another device may be requesting service so another
  // serial poll will be called from the main loop
  if (digitalRead(SRQ) == LOW) {
    isSRQ = true;
  }else{
    isSRQ = false;
  }
  if (isVerb) Serial.println(F("Serial poll completed."));

}


/*
 * SRQ command handler
 * Return status of SRQ line
 */
void srq_h(){
  //NOTE: LOW=asserted, HIGH=unasserted
  Serial.println(!digitalRead(SRQ));
}


/*
 * Set the status byte
 */
void stat_h(char *params){
  long int val = 0;
  // A parameter given?
  if (params!=NULL) {
    // Byte value given?
    val = atoi(params);
    if (val<0||val>256) {
      errBadCmd();
      if (isVerb) Serial.println(F("Invalid: expected byte value in the range 0 - 255"));
      return;
    }
    AR488.stat=val;
    if (val&0x40){
      // If bit 6 is set need to assert the SRQ line?
      setGpibState(0b01000000, 0b00000000, 0b01000000);
      if (isVerb) Serial.println(F("SRQ asserted."));
    }else{
      // Otherwise set SRQ line to INPUT_PULLUP
      setGpibState(0b00000000, 0b01000000, 0b01000000);
    }
  }else{
    // Return the currently set status byte
    Serial.println(AR488.stat);      
  }
}


/*
 * Save controller configuration
 */
void save_h(){

  epSaveCfg();
//  epSaveCfg(1);


/*
  sz = sizeof(AR488);
#ifdef DEBUG6
  Serial.print(F("Size of structure: "));
  Serial.println(sz);
#endif
  EEPROM.put(epaddr, AR488);
  if (isVerb) Serial.println(F("Settings saved."));
*/
  
}


/*
 * Show state or enable/disable listen only mode
 */
void lon_h(char *params) {
  int val;
  if (params!=NULL) { 
    val = atoi(params);
    if (val<0||val>1) {
      errBadCmd();
      if (isVerb) Serial.println(F("LON: valid range is [0-disable|1-enable]."));
      return;
    }
    isRO = val ? true : false; if (isVerb) {Serial.print(F("LON: ")); Serial.println(val ? "ON" : "OFF") ;}
  }else{
    Serial.println(isRO);
  }
}


/***********************************/
/***** CUSTOM COMMAND HANDLERS *****/
/***********************************/

/*
 * All serial poll - polls all devices, not just the currently addressed instrument
 * Alias wrapper for ++spoll all
 */
void aspoll_h() {
  spoll_h("all");
}


/* 
 * Send Universal Device Clear
 * The universal Device Clear (DCL) is unaddressed and affects all devices on the Gpib bus.
 */
void dcl_h() {
  if ( gpibSendCmd(DCL) )  { 
    if (isVerb) Serial.println(F("Sending DCL failed")); 
    return; 
  }
  // Set GPIB controls back to idle state
  setGpibControls(CIDS);
}


/*
 * Re-load default configuration
 */
void default_h() {
  initAR488();
}


/*
 * Parallel Poll Handler
 * Device must be set to respond on DIO line 1 - 8
 */
void ppoll_h() {
  uint8_t sb = 0;

  // Poll devices
  // Start in controller idle state
  setGpibControls(CIDS);
  delayMicroseconds(20);
  // Assert ATN and EOI
  setGpibState(0b10010000, 0b00000000, 0b10010000);
  delayMicroseconds(20);
  // Read data byte from GPIB bus without handshake
  sb = readGpibDbus();
  // Return to controller idle state (ATN and EOI unasserted)
  setGpibControls(CIDS);

  // Output the response byte
  Serial.println(sb, DEC);
  
  if (isVerb) Serial.println(F("Parallel poll completed."));
}


/*
 * Assert or de-assert REN 0=de-assert; 1=assert
 */
void ren_h(char *params) {
 char *stat; 
  int val;
  if (params!=NULL) {
    val = atoi(params);
    if (val<0||val>1) { if (isVerb) Serial.println(F("Invalid: expected 0=de-assert, 1=assert")); return;}
    digitalWrite(REN, !val); if (isVerb) {Serial.print(F("REN: ")); Serial.println(val ? "REN asserted" : "REN un-asserted") ;};
  }else{
    Serial.println(digitalRead(REN) ? 0 : 1);
  }
}


/*
 * Enable verbose mode 0=OFF; 1=ON
 */
void verb_h() { 
    isVerb = !isVerb;
    Serial.print("Verbose: ");
    Serial.println(isVerb ? "ON" : "OFF");
}


/*
 * Set version string
 * Replace the standard AR488 version string with something else
 * NOTE: some instrument software requires a sepcific version string to ID the interface
 */
void setvstr_h(char *params) {
  int len;
  if (params!=NULL) {
    len = strlen(params);
    memset(AR488.vstr, '\0', 48);
    if (len<48) {
      strncpy(AR488.vstr, params, len);
    }else{
      strncpy(AR488.vstr, params, 47);
    }
    if (isVerb) {Serial.print(F("Changed version string to: ")); Serial.println(params);};
  }
}


/*
 * SRQ auto - show or enable/disable SRQ interrupt
 * When the SRQ interrupt is enabled, a serial poll is conducted
 * automatically whenever SRQ is asserted and the status byte is  
 * returned for the instrument requiring service
 */
void srqa_h(char *params) {
  int val;
  if (params!=NULL) {
    val = atoi(params);
    if (val<0||val>1) {
      errBadCmd();
      if (isVerb) Serial.println(F("Invalid: expected 0=disable or 1=enable"));
      return;
    }
    switch (val) {
      case 0:
          cli();
          PCMSK2 &= ~SRQint;
          sei();
          isSrqa = false;
          break;
      case 1:
          cli();
          PCMSK2 |= SRQint;
          sei();
          isSrqa = true;
          break;
    }
    if (isVerb) Serial.println(isSrqa ? "SRQ auto ON" : "SRQ auto OFF") ;
  }else{
    Serial.println(isSrqa);
  }
}


/******************************************************/
/***** Device mode GPIB command handling routines *****/
/******************************************************/

struct gpibOpcode { 
  uint8_t opcode;
  void (*handler)();
//  void (*handler)(char *params);
};


/*
 * Gpib Opcode handling routine index
 * Used prmarily in device mode for handling
 * commands received over the GPIB bus
 */
static gpibOpcode goHidx[] = {
  { SDC,    sdc_h },
  { SPD,    spd_h },
  { SPE,    spe_h },
  { UNL,    unl_h }, 
  { UNT,    unt_h }
};


void attnRequired() {

  uint8_t db=0;
  uint8_t stat=0;
  uint8_t addr=0;
  int oasize = sizeof(goHidx)/sizeof(goHidx[0]);
  int i=0;

  int x=0;

  // Set device listner active state (assert NDAC+NRFD, DAV=INPUT_PULLUP)
  setGpibControls(DLAS);

  // Clear listen/talk flags
  aTl = false;
  aTt = false;

#ifdef DEBUG5
  Serial.println(F("ATN asserted - checking whether this is for me...."));
#endif

  // Read bytes
  while (isATN) {
    stat = gpibReadByte(&db);
    if (!stat) {
      
#ifdef DEBUG5
      Serial.println(db, HEX);
#endif

      // MLA: am I being addressed to listen?
      if (AR488.paddr==(db^0x20)) { // MLA = db^0x20
#ifdef DEBUG5        
        Serial.println(F("attnRequired: Controller wants me to data accept data <<<"));
#endif        
        // Set listen flag and listen parameters
        aTl=true;
        // Receive data until EOI or ATN asserted
        // EOI if controller is sending it
        // ATN means next bytes will contain a command
        rEoi=true;
        break;
      // MTA: am I being addressed to talk?
      }else if (AR488.paddr==(db^0x40)) {  // MTA = db^0x40;
#ifdef DEBUG5
        Serial.println(F("attnRequired: Controller wants me to send >>>"));
#endif
        if (!isRO) {  // Cannot talk in read-only (++lon) mode       
            aTt = true;
            break;
#ifdef DEBUG5
        }else{
          Serial.println(F("attnRequired: cannot respond in listen-only [++lon 1] mode"));  
#endif          
        }
      // Some other talker address
      }else if (db>0x3F && db<0x5F){
        if (!isSprq) {
          if (isRO) { // Read-only (++lon) mode
            aTl = true;
            rEoi = true;
          }
        }
      // Some other listener address
      }else if (db>0x1F && db<0x3F){
// Do we need these? - Disabled because it causes serial poll response to fail
// when the AR488 is not the first address in the polled devices address list
// NOTE: sets aTl to true when controller address (20) is received
//          aTl = true;
//          rEoi = true;

        // Ignore
      // Some other byte: lookup opcode handler 
      }else{
#ifdef DEBUG5
        Serial.println(F("Looking up GPIB opcode..."));
#endif
        i = 0;
        do {
          if (goHidx[i].opcode == db) break;
          i++;    // Contains index
        } while (i<oasize);
        // If we have a handler then execute it
        if (i<oasize) {
#ifdef DEBUG5         
          Serial.print(F("Executing GPIB command: ")); Serial.println(db, HEX);
#endif
          goHidx[i].handler();
        }
      }
    }else{
      // Should never get here - so something has gone wrong
      if (isVerb) Serial.println(F("attnRequired: failed to read command byte"));
      break;
    }
  }

/*
Serial.println(F("Debug info:"));
Serial.print(F("x      : ")); Serial.println(x);
Serial.print(F("db     : ")); Serial.println(db, HEX);
Serial.print(F("lfd    : ")); Serial.println(lfd);
Serial.print(F("rEoi   : ")); Serial.println(rEoi);
Serial.print(F("snd    : ")); Serial.println(snd);
Serial.print(F("i      : ")); Serial.println(oasize);
Serial.print(F("oasize : ")); Serial.println(oasize);
Serial.println();
*/

  // Set back to idle state
  if (!aTl && !aTt ) setGpibControls(DIDS);

//Serial.println(F("END attnReceived."));

}


void sdc_h() {
  // If being addressed then reset
  if (isVerb) Serial.println(F("Resetting..."));
#ifdef DEBUG5  
  Serial.print(F("Reset adressed to me: ")); Serial.println(aTl);
#endif
  if (aTl) rst_h();
  if (isVerb) Serial.println(F("Reset failed."));
}


void spd_h() {
  if (isVerb) Serial.println(F("<- serial poll request ended."));
  setGpibDbus(0);
  setGpibControls(DIDS);
  isSprq = false;
}


/*
 * Serial poll enable
 */
void spe_h() {
  if (isVerb) Serial.println(F("Serial poll request received from controller ->"));
  isSprq = true;
}


void unl_h() {
  // Stop receiving and go to idle
#ifdef DEBUG5  
  Serial.println(F("Unlisten received."));
#endif
  aTl = false;
  rEoi = false;     
}


void unt_h() {
  // Stop sending data and go to idle
#ifdef DEBUG5  
  Serial.println(F("Untalk received."));
#endif
  aTt = false;
}


/***************************************/
/***** GPIB DATA HANDLING ROUTINES *****/
/***************************************/

/*
 * Send a single byte GPIB command 
 */
 bool gpibSendCmd(uint8_t cmdByte){

  bool stat=false;

  // Set lines for command and assert ATN
  setGpibControls(CCMS);

  // Send the command
  stat = gpibWriteByte(cmdByte);
  if (stat && isVerb) {Serial.print(F("gpibSendCmd: failed to send command ")); Serial.print(cmdByte, HEX); Serial.println(F(" to device"));}

  // Return to controller idle state
//  setGpibControls(CIDS);
// NOTE: this breaks serial poll

  return stat ? ERR : OK;
}


/*
 * Send the status byte
 */
bool gpibSendStatus(){
  // Have been addressed and polled so send the status byte
  if (isVerb) { Serial.print(F("Sending status byte: ")); Serial.println(AR488.stat); };    
  setGpibControls(DTAS);
  gpibWriteByte(AR488.stat);
  setGpibControls(DIDS);
}


/*
 * Send a series of characters as data to the GPIB bus
 */
void gpibSendData(char *data, uint8_t dsize){

  // If lon is turned on we cannot send data so exit
  if (isRO) return;

  // Controler can unlisten bus and address devices
  if (AR488.cmode==2) {

    // Address device to listen
    if (addrDev(AR488.paddr, 0)) {
      if (isVerb) {
        Serial.print(F("gpibReceiveData: failed to address device "));
        Serial.print(AR488.paddr);
        Serial.println(F(" to listen"));
      }
    }

#ifdef DEBUG3
    Serial.println(F("Device addressed."));
#endif

    // Set control lines to write data (ATN unasserted)
    setGpibControls(CTAS);
    
  }else{
    setGpibControls(DTAS);
  }
#ifdef DEBUG3
  Serial.println(F("Set write data mode."));
#endif

  // Write the data string
  for (int i=0; i<dsize; i++){
    // If EOI asserting is on
    if (AR488.eoi) {
      // Send all characters
      gpibWriteByte(data[i]);
    }else{
      // Otherwise ignore non-escaped CR, LF and ESC
      if ((data[i]!=CR)||(data[i]!=LF)||(data[i]!=ESC)) gpibWriteByte(data[i]);
    }
  }

#ifdef DEBUG3
  Serial.print(F("Sent string [")); Serial.print(data); Serial.println(F("]"));
#endif

  // Write terminators according to EOS setting
  // Do we need to write a CR?
  if ((AR488.eos&0x2) == 0){
    gpibWriteByte(CR);
#ifdef DEBUG3
    Serial.println(F("Appended CR"));
#endif
  }
  // Do we need to write an LF?
  if ((AR488.eos&0x1) == 0){
    gpibWriteByte(LF);
#ifdef DEBUG3
    Serial.println(F("Appended LF"));
#endif
  }

  // If EOI enabled then assert EOI
  if (AR488.eoi) {
    setGpibState(0b00010000, 0b00000000, 0b00010000);
    delayMicroseconds(40);
    setGpibState(0b00010000, 0b00010000, 0b00010000);
#ifdef DEBUG3
    Serial.println(F("Asserted EOI"));
#endif
  }

  if (AR488.cmode==2){

    // Untalk controller and unlisten bus
    if (uaddrDev()) {
      if (isVerb) Serial.println(F("gpibSendData: Failed to unlisten bus"));
    }

#ifdef DEBUG3
    Serial.println(F("Unlisten done"));
#endif

    // Controller - set lines to idle?
    setGpibControls(CIDS);

  }else{
    // Device mode - set control lines to idle
    if (AR488.cmode==1) setGpibControls(DIDS);
  }
}


/*
 * Receive data from the GPIB bus
 */
bool gpibReceiveData(){

  char ch;
  uint8_t r=0,db;

  int x=0;
  int s=0;


  // Set flags
  isReading = true;
  tranBrk = 0;

  // If we are a controller
  if (AR488.cmode == 2) {

    // Address device to talk
    if (addrDev(AR488.paddr, 1)) {
      if (isVerb) {
        Serial.print(F("Failed to address the device"));
        Serial.print(AR488.paddr);
        Serial.println(F(" to talk"));
      }
    }

    // Set GPIB control lines to controller read mode
    setGpibControls(CLAS);

  }else{
    // Set GPIB controls to device read mode
    setGpibControls(DLAS);

#ifdef DEBUG7
    Serial.println(F("Start listen ->"));
    Serial.println(tranBrk);
    Serial.println(isATN);
#endif
  
  }

// Perform read of data (r: 0=data; 1=cmd; >1=error;
  while ( !tranBrk && !isATN && !(r=gpibReadByte(&db)) ) {

    // When reading with EOI=1 or AUTO=1 Check for break condition
    if (rEoi || isAuto) readBreak();
    
    // If attention required by new command then break here 
    if (tranBrk == 7 || isATN) break;

    // Output the character to the serial port
    Serial.print((char)db);

    // EOI detected - print last character and then break on EOI
    if (tranBrk==5) break;

    // Stop if byte = specified EOT character  
    if (db==eByte && rEbt) break;
    // Stop on LF unless expecting EOI
    if (db==LF && !rEoi) break;
    // Stop on timeout
    if (r>0) break;
    
    // Byte counter
    x++;
    
    // Check for interrupt condition (++, EOI or ATN)
//    readBreak();
  }

#ifdef DEBUG7
    Serial.println(F("After loop:"));
    Serial.println(tranBrk);
    Serial.println(isATN);
    Serial.println(r);
#endif

  // End of data - if verbose, report how many bytes read
  if (isVerb) {Serial.print(F("Bytes read: ")); Serial.println(x);}

  // Detected that EOI has been asserted
  if (tranBrk==5) {
    if (isVerb) Serial.println(F("EOI detected!"));      
    // If eot_enabled then add EOT character
    if (AR488.eot_en) Serial.print(AR488.eot_ch);
  }

  // Timeout error?
  if (r>0) {
    if (isVerb && r==1) Serial.println(F("gpibReceiveData: timeout waiting for talker"));
    if (isVerb && r==2) Serial.println(F("gpibReceiveData: timeout waiting for transfer to complete"));
//      return ERR;
  }

  if (AR488.cmode==2){

    // Untalk bus and unlisten controller
    if (uaddrDev()) {
      if (isVerb) Serial.print(F("gpibSendData: Failed to untalk bus"));
    }

    // Set controller back to idle state
    if (AR488.cmode==2) setGpibControls(CIDS);

  }else{
    // Set device back to idle state
    setGpibControls(DIDS);

#ifdef DEBUG7
    Serial.println(F("<- End listen."));
#endif  

  }

  // Reset flags
  isReading = false;
  if (tranBrk>0) tranBrk = 0;

  if (r>0) return ERR;

}


/*
 * Read a SINGLE BYTE of data from the GPIB bus using 3-way handshake
 * (- this function is called in a loop to read data    )
 * (- the GPIB bus must already be configured to listen )
 */
uint8_t gpibReadByte(uint8_t *db){

  // Unassert NRFD (we are ready for more data)
  setGpibState(0b00000100,0b00000100,0b00000100);

  // Wait for DAV to go LOW indicating talker has finished setting data lines..
  if (Wait_on_pin_state(LOW,DAV,AR488.rtmo))  { 
    if (isVerb) Serial.println(F("gpibReadByte: timeout waiting for DAV to go LOW"));
    // No more data for you?
    return 1;
  } 

  // Assert NRFD (NOT ready - busy reading data) 
  setGpibState(0b00000100,0b00000000,0b00000100);

  // read from DIO
  *db = readGpibDbus();

  // Unassert NDAC signalling data accepted
  setGpibState(0b00000010,0b00000010,0b00000010);

  // Wait for DAV to go HIGH indicating data no longer valid (i.e. transfer complete)
  if (Wait_on_pin_state(HIGH,DAV,AR488.rtmo))  { 
    if (isVerb) Serial.println(F("gpibReadByte: timeout waiting DAV to go HIGH")); 
    return 2; 
  }

  // Re-assert NDAC - handshake complete, ready to accept data again
  setGpibState(0b00000010,0b00000000,0b00000010);

  return 0;

}


/*
 * Write a SINGLE BYTE onto the GPIB bus using 3-way handshake
 * (- this function is called in a loop to send data )
 */
bool gpibWriteByte(uint8_t db) {

    // Wait for NDAC to go LOW indicating that devices are at attention
    if (Wait_on_pin_state(LOW,NDAC,AR488.rtmo)) { 
      if (isVerb) Serial.println(F("gpibWriteByte: timeout waiting for receiver attention [NDAC asserted]")); 
      return true; 
    }
    /*
    if (Wait_on_pin_state(LOW,NRFD,AR488.rtmo)) { 
      if (isVerb) Serial.println(F("gpibWriteByte: timeout waiting for receiver attention [NDAC+NRFD asserted]")); 
      return true; 
    }
    */

  // Wait for NRFD to go HIGH (receiver is ready)
  if (Wait_on_pin_state(HIGH,NRFD,AR488.rtmo))  { 
    if (isVerb) Serial.println(F("gpibWriteByte: timeout waiting for receiver ready - [NRFD unasserted]")); 
    return true; 
  }

  // Place data on the bus
  setGpibDbus(db);
// Serial.println(isATN);
  
//  delayMicroseconds(20);

  // Assert DAV (data is valid - ready to collect)
  setGpibState(0b00001000, 0b00000000, 0b00001000);

  // Wait for NRFD to go LOW (receiving)
  if (Wait_on_pin_state(LOW,NRFD,AR488.rtmo))  { 
    if (isVerb) Serial.println(F("gpibWriteByte: timeout receiving - [NRFD asserted]")); 
    return true; 
  }

  // Wait for NDAC to go HIGH (data accepted)
  if (Wait_on_pin_state(HIGH,NDAC,AR488.rtmo))  { 
    if (isVerb) Serial.println(F("gpibWriteByte: timeout waiting for data accepted [NDAC unasserted]")); 
    return true; 
  }
  
  // Unassert DAV
  setGpibState(0b00001000, 0b00001000, 0b00001000);

  // Reset the data bus
  setGpibDbus(0);

  // Exit successfully
  return false;
}


/*
 * Untalk bus then address a device
 * dir: 0=listen; 1=talk;
 */
bool addrDev(uint8_t addr, bool dir){
  if (gpibSendCmd(UNL)) return ERR;
  if (dir) {
    if (gpibSendCmd(LAD+AR488.caddr)) return ERR;
    if (gpibSendCmd(TAD+addr)) return ERR;
  }else{
    if (gpibSendCmd(TAD+AR488.caddr)) return ERR;
    if (gpibSendCmd(LAD+addr)) return ERR;
  }
  return OK;
}


/*
 * Unaddress a device (untalk bus)
 */
bool uaddrDev(){
  // De-bounce
  delayMicroseconds(30);
  // Utalk/unlisten
  if (gpibSendCmd(UNL)) return ERR;
  if (gpibSendCmd(UNT)) return ERR;
  return OK;
}


/**********************************/ 
/*****  GPIB CONTROL ROUTINES *****/
/**********************************/


/*
 * Wait for "pin" to reach a specific state
 * Returns false on success, true on timeout.
 * Pin MUST be set as INPUT_PULLUP otherwise it will not change and simply time out!
 * 
*/
boolean Wait_on_pin_state(uint8_t state, uint8_t pin, int interval) {

  unsigned long timeout = millis()+interval;

  while (digitalRead(pin) != state) {
    if (millis()>=timeout) return true;
    if (digitalRead(EOI)==LOW) tranBrk = 2;
  }
  return false;        // = no timeout therefore succeeded!
}


/*
 * Read the status of the GPIB data bus wires and collect the byte of data
 */
uint8_t readGpibDbus() {
  // Set data pins to input
//  DDRD = DDRD & 0b11001111 ; 
//  DDRC = DDRC & 0b11000000 ; 
  DDRD &= 0b11001111 ; 
  DDRC &= 0b11000000 ; 
//  PORTD = PORTD | 0b00110000; // PORTD bits 5,4 input_pullup
//  PORTC = PORTC | 0b00111111; // PORTC bits 5,4,3,2,1,0 input_pullup
  PORTD |= 0b00110000; // PORTD bits 5,4 input_pullup
  PORTC |= 0b00111111; // PORTC bits 5,4,3,2,1,0 input_pullup

  // Read the byte of data on the bus
  return ~((PIND<<2 & 0b11000000)+(PINC & 0b00111111));  
}


/*
 * Set the status of the GPIB data bus wires with a byte of datacd ~/test
 * 
 */
void setGpibDbus(uint8_t db) {
  // Set data pins as outputs
  DDRD |= 0b00110000;
  DDRC |= 0b00111111;

  // GPIB states are inverted
  db = ~db;

  // Set data bus
  PORTC = (PORTC & ~0b00111111) | (db & 0b00111111);
  PORTD = (PORTD & ~0b00110000) | ((db & 0b11000000) >> 2);
  
  // Set pins with data byte
//  PORTD = ( (PORTD&0b11001111) | (~db>>2 & 0b00110000) ) ;
//  PORTC = ( (PORTC&0b11000000) | (~db    & 0b00111111) ) ;

  delayMicroseconds(2);
}


/*
 * Set the state of the GPIB control lines
 * Bits control lines as follows: 7-ATN, 6-SRQ, 5-REN, 4-EOI, 3-DAV, 2-NRFD, 1-NDAC, 0-IFC
 * pdir:  0=input, 1=output;
 * pstat: 0=LOW, 1=HIGH/INPUT_PULLUP
 * Arduino pin to Port/bit to direction/state byte map:
 * IFC   8   PORTB bit 0 byte bit 0
 * NDAC  9   PORTB bit 1 byte bit 1
 * NRFD  10  PORTB bit 2 byte bit 2
 * DAV   11  PORTB bit 3 byte bit 3
 * EOI   12  PORTB bit 4 byte bit 4
 //* REN   13  PORTB bit 5 byte bit 5
 * SRQ   2   PORTD bit 2 byte bit 6
 * REN   3   PORTD bit 3 byte bit 5 
 * ATN   7   PORTD bit 8 byte bit 7
 */
void setGpibState(uint8_t pdir, uint8_t pstat, uint8_t mask){
  // PORTB - use only the first (right-most) 6 bits
  uint8_t portBd = pdir&0x1F;
  uint8_t portBs = pstat&0x1F;
  uint8_t portBm = mask&0x1F;
  // PORT D - keep bit 7, rotate bit 6 right 4 positions to set bit 2 on register
  uint8_t portDd = (pdir&0x80) + ((pdir&0x40)>>4) + ((pdir&0x20)>>2);
  uint8_t portDs = (pstat&0x80) + ((pstat&0x40)>>4) + ((pstat&0x20)>>2);
  uint8_t portDm = (mask&0x80) + ((mask&0x40)>>4) + ((mask&0x20)>>2);
  // Set registers: register = (register & ~bitmask) | (value & bitmask)
  // Mask: 0=unaffected; 1=to be changed
  // Set data direction registers
  DDRB = ( (DDRB&~portBm) | (portBd&portBm) );
  DDRD = ( (DDRD&~portDm) | (portDd&portDm) );
  // Set data registers
  PORTB = ( (PORTB&~portBm) | (portBs&portBm) );
  PORTD = ( (PORTD&~portDm) | (portDs&portDm) );
}


/*
 * Control the GPIB bus - set various GPIB states
 * state is a predefined state (CINI, CIDS, CTAS, CLAS, DINI, DIDS, DACS);
 * setGpibState (uint8_t direction, uint8_t state[low/high]) 
 * Bits control lines as follows: 8-ATN, 7-SRQ, 6-REN, 5-EOI, 4-DAV, 3-NRFD, 2-NDAC, 1-IFC     
 * setGpibState byte1:  0=input, 1=output;
 * setGpibState byte2: 0=LOW, 1=HIGH/INPUT_PULLUP
 */
bool setGpibControls(uint8_t state){

  // Switch state
  switch (state) {
    // Controller states
    case CINI:  // Initialisation
      setGpibState(0b10111000, 0b11011111, 0b11111111);
#ifdef DEBUG2
      Serial.println(F("Initialised GPIB control mode"));
#endif
      break;
    case CIDS:  // Controller idle state
      setGpibState(0b10111000, 0b11011111, 0b10011110);
#ifdef DEBUG2      
      Serial.println(F("Set GPIB lines to idle state"));
#endif
      break;
    case CCMS:  // Controller active - send commands
      setGpibState(0b10111001, 0b01011111, 0b10011111);
#ifdef DEBUG2      
      Serial.println(F("Set GPIB lines for sending a command"));
#endif
      break;
    case CLAS:  // Controller - read data bus
      setGpibState(0b10100110, 0b11011001, 0b10011110);
#ifdef DEBUG2      
      Serial.println(F("Set GPIB lines for reading data"));
#endif
      break;
    case CTAS:  // Controller - write data bus
      setGpibState(0b10111001, 0b11011111, 0b10011110);
#ifdef DEBUG2      
      Serial.println(F("Set GPIB lines for writing data"));
#endif
      break;     

/* Bits control lines as follows: 8-ATN, 7-SRQ, 6-REN, 5-EOI, 4-DAV, 3-NRFD, 2-NDAC, 1-IFC */  

    // Listener states
    case DINI:  // Listner initialisation
//      setGpibState(0b00000110, 0b10111001, 0b11111111);
      setGpibState(0b00000000, 0b11111111, 0b11111111);
#ifdef DEBUG2    
      Serial.println(F("Initialised GPIB listener mode"));
#endif
      break;
    case DIDS:  // Device idle state
      setGpibState(0b00000000, 0b11111111, 0b00001110);
#ifdef DEBUG2      
      Serial.println(F("Set GPIB lines to idle state"));
#endif
      break;
    case DLAS:  // Device listner active (actively listening - can handshake)
      setGpibState(0b00000110, 0b11111001, 0b00001110);
#ifdef DEBUG2      
      Serial.println(F("Set GPIB lines to idle state"));
#endif
      break;
    case DTAS:  // Device talker active (sending data)
      setGpibState(0b00001000, 0b11111001, 0b00001110);
#ifdef DEBUG2      
      Serial.println(F("Set GPIB lines for listening as addresed device"));
#endif
      break;
#ifdef DEBUG2      
    default:
      // Should never get here!
//      setGpibState(0b00000110, 0b10111001, 0b11111111);
      Serial.println(F("Unknown GPIB state requested!"));
#endif
  }
  
  // Save state and delay to settle
  cstate = state;
  delayMicroseconds(10); 

}

/********** END OF SKETCH **********/
