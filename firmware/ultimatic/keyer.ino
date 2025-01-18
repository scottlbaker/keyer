
// ============================================================================
// keyer.ino :: An Iambic Keyer and Decoder
//
// (c) Scott Baker KJ7NLA
//
//  Arduino IDE settings:
//  board: Arduino UNO
//  bootloader: no bootloader
//  programmer: AVRISP mkII
//
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <EEPROMex.h>

// pin definitions
const uint8_t pinDit  =  2;  // dit key input
const uint8_t pinDah  =  3;  // dah key input
const uint8_t pinKey  =  4;  // keyer output
const uint8_t pinSw1  =  6;  // push-button switch
const uint8_t pinSw2  =  7;  // push-button switch
const uint8_t pinBuzz =  8;  // buzzer/speaker
const uint8_t pinLED  = 10;  // push-button switch

#define VERSION   "1.00d"
#define DATE      "MAY 11,2023"

#include "./include/font.h"
#include "./include/lookup.h"

// generic
#define OFF    0
#define ON     1
#define NO     0
#define YES    1
#define FALSE  0
#define TRUE   1

// prototypes
uint8_t read_switches();
void wait_switches();
void reset_xtimer();
void gpio_init();
void timer1_init();
void timer2_init();
inline void timer1_start();
inline void timer1_stop();
inline void timer2_start();
inline void timer2_stop();
char lookup_cw(uint8_t addr);
void print_cw();
void print_line(uint8_t row, char *str);
void print_tmp(uint8_t row);
void printchar(char ch);
void printstr(char* s);
void show_editstr(uint8_t hdr, uint8_t edit);
void insertchar();
void deletechar();
void spinchar();
void maddr_cmd(uint8_t cmd);
void read_paddles();
void iambic_keyer();
void straight_key();
void send_cwchr(char ch);
void send_cwmsg(char *str, uint8_t prn);
void change_wpm(uint8_t wpm);
void change_volume(uint8_t vol);
void change_tone(uint8_t tone);
void back2run();
void ditcalc();
void menu_wpm();
void menu_mode();
void menu_volume();
void menu_tone();
void menu_swap();
void menu_led();
void menu_save();
void menu_msg();
void menu_edit();
void menu_record();
void save_eeprom();
void read_eeprom();
void factory_reset();
uint8_t len(char *str);
void cpy(char *dst, char *src);
uint8_t cat(char *dst, char *src);
void catc(char *dst, char c);
void lower(uint8_t i);
void wait_ms(uint16_t dly);
void beep(uint8_t onoff);
void doError();
void(*resetFunc) (void) = 0;

// menu IDs
#define BACK2RUN   0
#define INITMENU   1
#define MENUWPM    1
#define MENUMODE   2
#define MENUVOL    3
#define MENUTONE   4
#define MENUSWAP   5
#define MENULED    6
#define MENUSAVE   7
#define MENUMSG    8
#define MENUEDIT   9
#define MENUREC    10

#define TONE700    2   // 700 Hz sidetone
#define TONE600    1   // 600 Hz sidetone

#define INITWPM   25   // initial keyer speed
#define INITVOL    4   // initial volume

uint8_t menuID   = INITMENU;  // menu ID
uint8_t keyerwpm = INITWPM;   // keyer speed
uint8_t volume   = INITVOL;   // sidetone volume
uint8_t cwtone   = TONE700;   // 700 Hz sidetone
uint8_t msgindx  = 1;         // message index
int8_t  incval   = 0;
char    editchar = ' ';
uint8_t spun     = NO;
uint8_t useLED   = NO;

// timer1 interrupt handler
ISR(TIMER1_COMPA_vect) {
  digitalWrite(pinBuzz, HIGH);
}

// timer1 interrupt handler
ISR(TIMER1_COMPB_vect) {
  digitalWrite(pinBuzz, LOW);
}

// timer2 interrupt handler
ISR(TIMER2_COMPA_vect) {
  timer1_start();
}

// timer2 interrupt handler
ISR(TIMER2_COMPB_vect) {
  timer1_stop();
}

#define T1_STOP 0x08
#define T1_RUN  0x09

#define T1_PWM0 0x00
#define T1_PWM1 0x01
#define T1_PWM2 0x03
#define T1_PWM3 0x07
#define T1_PWM4 0x1f
#define T1_PWM5 0xff

// timer 1 init
void timer1_init() {
  TIMSK1 = 0x00;         // disable interrupt
  TCCR1A = 0x00;         // CTC mode 4
  TCCR1B = T1_STOP;      // stop clock
  OCR1AH = 0x03;         // PWM value MSB
  OCR1AL = 0xff;         // PWM value LSB
  OCR1BH = 0x00;         // PWM value MSB
  OCR1BL = 0x1f;         // PWM value LSB
}

// timer 1 start
inline void timer1_start() {
  TCCR1B = T1_RUN;       // turn on clock
  TIMSK1 = 0x06;         // enable COMP interrupts
}

// timer 1 stop
inline void timer1_stop() {
  TIMSK1 = 0x00;         // disable interrupts
  TCCR1B = T1_STOP;      // stop clock
  digitalWrite(pinBuzz, LOW);
}

#define T600A     0xd0   // 600Hz sidetone
#define T600B     0x68   // 600Hz sidetone
#define T700A     0xae   // 700Hz sidetone
#define T700B     0x57   // 700Hz sidetone

// timer 2 init
void timer2_init() {
  TIMSK2 = 0x00;         // disable interrupts
  ASSR   = 0x00;         // use I/O clock
  TCCR2A = 0x02;         // CTC clear on match
  TCCR2B = 0x05;         // div-by-128 prescaler
  OCR2A  = T600A;        // rising edge
  OCR2B  = T600B;        // falling edge
  TIMSK2 = 0x00;         // disable interrupts
}

// timer 2 start
inline void timer2_start() {
  TIMSK2 = 0x06;         // enable interrupts
}

// timer 2 stop
inline void timer2_stop() {
  TIMSK2 = 0x00;         // disable interrupts
  timer1_stop();
}

// delay times
#define ONE_SECOND      1000
#define TWO_SECONDS     2000
#define THREE_SECONDS   3000

// used by setCursor
#define NOSCALE  0
#define SCALE    1

// for display blank/timeout
uint8_t display  =  ON;
uint8_t xtimeout = 120;   // blank after 2 minutes
uint8_t xtimer   =   0;
uint32_t xdt;

// user interface
#define NBP  0  // no-button-pushed
#define BSC  1  // button-single-click
#define BPL  2  // button-push-long

uint8_t event = NBP;

#define LONGPRESS  400   // long button press
#define DITCONST  1200   // dit time constant

// 4x16 OLED
#define COLSIZE   16
#define ROWSIZE    4
#define MAXCOL    COLSIZE-1  // max display column
#define MAXLINE   ROWSIZE-1  // max display row

#define BAUDRATE 115200

// SSD1306 OLED initialization sequence
const uint8_t oled_init_sequence [] = {
  0xD5, 0x80,   // set display clock divide ratio
  0xA8, 0x3F,   // Set multiplex ratio to 1:64
  0xD3, 0x00,   // set display offset = 0
  0x40,         // set display start line address
  0x8D, 0x14,   // set charge pump, internal VCC
  0x20, 0x02,   // set page mode memory addressing
  0xA4,         // output RAM to display
  0xA1,         // set segment re-map
  0xC8,         // set COM output scan direction
  0xDA, 0x12,   // Set com pins hardware configuration
  0x81, 0x80,   // set contrast control register
  0xDB, 0x40,   // set vcomh
  0xD9, 0xF1,   // 0xF1=brighter
  0xB0,         // set page address (0-7)
  0xA6,         // set display mode to normal
  0xAF,         // display ON
};

// SSD1306 OLED routines
class OLEDDevice {
public:
  #define OLED_ADDR    0x3C
  #define OLED_COMMAND 0x00
  #define OLED_DATA    0x40
  #define OLED_PAGE    0xB0
  #define NODISPLAY    0xAE
  #define ONDISPLAY    0xAF

  uint8_t oledX;
  uint8_t oledY;

  uint8_t m_row;
  uint8_t m_col;

  // send a command
  void sendcmd(uint8_t cmd) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(OLED_COMMAND);
    Wire.write(cmd);
    Wire.endTransmission();
  }

  // send data
  void senddata(uint8_t data) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(OLED_DATA);
    Wire.write(data);
    Wire.endTransmission();
  }

  // turn off the display
  void noDisplay() { sendcmd(NODISPLAY); }

  // turn on the display
  void onDisplay() { sendcmd(ONDISPLAY); }

  // set cursor to home
  void home() { setCursor(0,0); }

  // set cursor column and row
  void setCursor(uint8_t col, uint8_t row, uint8_t x=1) {
    if (x) {
      m_row = row;
      m_col = col;
      oledX = col*FONT_W;
      oledY = ((row*FONT_H) & 0x06) | 0x01;
    } else {
      oledX = col;
      oledY &= 0x06;
    }
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(OLED_COMMAND);
    Wire.write(OLED_PAGE | oledY);
    Wire.write(0x10 | ((oledX & 0xf0) >> 4));
    Wire.write(oledX & 0x0f);
    Wire.endTransmission();
  }

  // clear to end of line
  void clr2eol() {
    for (uint8_t x=oledX; x<128; x++) senddata(0);
    setCursor(oledX, oledY+1, 0);
    for (uint8_t x=oledX; x<128; x++) senddata(0);
  }

  // clear a line
  void clrLine(uint8_t row) {
    setCursor(0, row);
    for (uint8_t x=0; x<128; x++) senddata(0);
    setCursor(0, oledY, 0);
    for (uint8_t x=0; x<128; x++) senddata(0);
    setCursor(0, row);
  }

  // clear the screen
  void clrScreen() {
    for (uint8_t row=0; row<4; row++) clrLine(row);
  }

  uint8_t fx1[10] = {0,0,0,0,0,0,0,0,0,0};
  uint8_t fx0[10] = {0,0,0,0,0,0,0,0,0,0};

  // lookup the char in the font table and stretch it
  void lookup(uint8_t ch) {
    uint8_t mk1;
    uint8_t mk2;
    uint8_t dat;
    uint8_t dax;
    for (uint8_t i=0; i<FONT_W; i++) {
      // read the font data
      dat = pgm_read_byte(&(font[((ch-32)*FONT_W)+i]));
      mk1 = 0x01;
      mk2 = 0x03;
      dax = 0;
      // stretch the data vertically
      for (uint8_t j=0; j<4; j++) {
        if (dat & mk1) dax |= mk2;
        mk1 = mk1 <<1;
        mk2 = mk2 <<2;
      }
      fx0[i]= dax;
      mk1 = 0x10;
      mk2 = 0x03;
      dax = 0;
      // stretch the data vertically
      for (uint8_t j=0; j<4; j++) {
        if (dat & mk1) dax |= mk2;
        mk1 = mk1 <<1;
        mk2 = mk2 <<2;
      }
      fx1[i]= dax;
    }
  }

  // print a char
  void putch(uint8_t ch) {
    uint8_t i;
    if ((ch == '\n') || (oledX > (128 - FONT_W))) return;
    if (ch < 32 || ch > 137) ch = 32;
    lookup(ch);
    for (i=0; i<FONT_W; i++) senddata(fx1[i]);
    setCursor(oledX, oledY, 0);
    for (i=0; i<FONT_W; i++) senddata(fx0[i]);
    m_col++;
    setCursor(m_col, m_row);
  }

  // print a string
  void putstr(const char *str) {
    while(*str) putch(*str++);
  }

  // print an 8-bit integer value
  void print8(uint8_t val) {
    char tmp[4] = "  0";
    // convert to string
    for (uint8_t i=2; val; i--) {
      tmp[i] = "0123456789"[val % 10];
      val /= 10;
    }
    // left justify
    while (tmp[0] == ' ') {
      tmp[0] = tmp[1];
      tmp[1] = tmp[2];
      tmp[2] = tmp[3];
    }
    putstr(tmp);
  }

  // print an 16-bit integer value
  void print16(uint16_t val) {
    char tmp[6] = "    0";
    // convert to string
    for (uint8_t i=4; val; i--) {
      tmp[i] = "0123456789"[val % 10];
      val /= 10;
    }
    // left justify
    while (tmp[0] == ' ') {
      tmp[0] = tmp[1];
      tmp[1] = tmp[2];
      tmp[2] = tmp[3];
      tmp[3] = tmp[4];
      tmp[4] = tmp[5];
    }
    putstr(tmp);
  }

  // initialization
  void begin(uint8_t cols, uint8_t rows) {
    Wire.beginTransmission(OLED_ADDR);
    Wire.write(OLED_COMMAND);
    for (uint8_t i=0; i < sizeof(oled_init_sequence); i++) {
      Wire.write(oled_init_sequence[i]);
    }
    Wire.endTransmission();
    delayMicroseconds(100);
    clrScreen();
  }
};

OLEDDevice oled;

uint8_t sw1Pushed = 0;    // button #1 pushed
uint8_t sw2Pushed = 0;    // button #2 pushed
uint8_t recordMsg = OFF;  // record message

// read the UI switches
uint8_t read_switches() {
  sw1Pushed = !digitalRead(pinSw1);
  sw2Pushed = !digitalRead(pinSw2);
  if (sw2Pushed) sw1Pushed = 1;
  return (sw1Pushed || sw2Pushed);
}

// wait for UI switches released
void wait_switches() {
  uint8_t wait1 = 1;
  while (wait1) {
    wait1  = !digitalRead(pinSw1);
    wait1 |= !digitalRead(pinSw2);
    delay(10); // debounce
  }
}

// reset display timeout
void reset_xtimer() {
  xtimer = 0;
  xdt = millis();
  if (display == OFF) {
    display = ON;
    oled.onDisplay();
  }
}

// I/O pin setup
void gpio_init() {
  pinMode(pinDit,  INPUT);
  pinMode(pinDah,  INPUT);
  pinMode(pinSw1,  INPUT);
  pinMode(pinSw2,  INPUT);
  pinMode(pinKey,  OUTPUT);
  pinMode(pinBuzz, OUTPUT);
  pinMode(pinLED,  OUTPUT);
  digitalWrite(pinKey,  LOW);
  digitalWrite(pinBuzz, LOW);
  digitalWrite(pinLED,  LOW);
}

uint8_t maddr = 1;
uint8_t myrow = 0;
uint8_t mycol = 0;

#define MAXLEN  50
char tmpstr[MAXLEN];

// table lookup for CW decoder
char lookup_cw(uint8_t addr) {
  char ch = '*';
  if (addr < 128) ch = pgm_read_byte(m2a + addr);
  return ch;
}

// print a line
void print_line(uint8_t row, char *str) {
  oled.setCursor(0,row);
  oled.putstr(str);
  oled.clr2eol();
}

void print_tmp(uint8_t row) {
  uint8_t x;
  uint8_t x1;
  uint8_t y=0;
  char str[20];
  switch (row) {
    case 2:
      x1 = 16;
      break;
    case 3:
      x1 = 32;
      break;
    default:
      x1 = 0;
      break;
  }
  x=x1;
  while (y < 16) {
    str[y++] = tmpstr[x++];
    if (tmpstr[x] == '\0') break;
  }
  str[y] = '\0';
  oled.setCursor(0,row);
  oled.putstr(str);
  oled.clr2eol();
}

// print the ascii char
void printchar(char ch) {
  if ((myrow <= MAXLINE) & (mycol <= MAXCOL)) {
    oled.setCursor(mycol,myrow);
    if (!maddr) {
      // clear screen if 8-dit code is received
      oled.clrScreen();
      myrow = 0;
      mycol = 0;
    } else {
      oled.putch(ch);
      mycol++;
    }
  }
  if (mycol > MAXCOL) {
    mycol = 0;
    myrow++;
    if (myrow > MAXLINE) myrow = 0;
    oled.clrLine(myrow);
  }
}

// print a string
void printstr(char *s) {
  while ((*s) != 0) {		
    printchar(*s);
    s++;
  }
}

#define MAXMSG  8   // max message index
#define MINMSG  1   // min message index

int8_t  editpos  = 0;
int8_t  spinpos  = 0;
char   *editstr;

char cwmsg[MAXMSG][MAXLEN];

// display the edit string
void show_editstr(uint8_t hdr, uint8_t edit) {
  oled.home();
  switch (hdr) {
    case 1:
      oled.putstr("SEND MESSAGE ");
      break;
    case 2:
      oled.putstr("EDIT MESSAGE ");
      break;
    case 3:
      oled.putstr("REC MESSAGE ");
      break;
    default:
      break;
  }
  if (hdr) {
    oled.print8(msgindx);
    oled.clr2eol();
    editstr = cwmsg[msgindx-1];
  }
  uint8_t strlen = len(editstr);
  cpy (tmpstr, editstr);
  // make lowercase char at edit position
  if (edit) lower(editpos);
  if (strlen <= 16) {
    print_line(1, tmpstr);
    oled.clrLine(2);
    oled.clrLine(3);
  } else if (strlen <= 32) {
    print_tmp(1);
    print_tmp(2);
    oled.clrLine(3);
  } else if (strlen <= 48) {
    print_tmp(1);
    print_tmp(2);
    print_tmp(3);
  }
}

// insert a space into the edit string
void insertchar() {
  uint8_t strlen = len(editstr);
  // check limits
  if (strlen > (MAXLEN-1)) return;
  // check for insert char ^
  for (uint8_t i=0; i<strlen; i++) {
    if (editstr[i] == '^') {
      // insert after current edit position
      for (uint8_t j=strlen; j>editpos; j--) {
        editstr[j] = editstr[j-1];
      }
      editstr[i] = editchar;
      editstr[i+1] = ' ';
      editstr[strlen+1] = '\0';
      show_editstr(0,1);
      break;
    }
  }
}

// delete # char from the edit string
void deletechar() {
  uint8_t strlen = len(editstr);
  // check for end of string %
  for (uint8_t i=0; i<strlen; i++) {
    if (editstr[i] == '%') {
      editstr[i] = '\0';
      break;
    }
  }
  strlen = len(editstr);
  // check for removal char #
  for (uint8_t i=0; i<strlen; i++) {
    if (editstr[i] == '#') {
      // shift and adjust string
      for (uint8_t j=i; j<strlen; j++) {
        editstr[j] = editstr[j+1];
      }
      strlen--;
    }
  }
}

#define MAXSPIN 39

// use encoder for editing a string
void spinchar() {
  const char spinstr[41] =
  "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 #^%";
  if (!spun) {
    editchar = editstr[editpos];
    spun = YES;
  }
  spinpos += incval;
  if (spinpos > MAXSPIN) spinpos = 0;
  if (spinpos <  0) spinpos = MAXSPIN;
  editstr[editpos] = spinstr[spinpos];
  incval = 0;
  show_editstr(0,1);
}

// get spin position
void get_spinpos(char ch) {
  // ascii to index calculations
  if ((ch >= 'A') && (ch <= 'Z')) spinpos = ch-65;
  else if ((ch >= '0') && (ch <= '9')) spinpos = ch-22;
  else if (ch == ' ') spinpos = 36;
  else if (ch == '#') spinpos = 37;
  else if (ch == '^') spinpos = 38;
  else if (ch == '%') spinpos = 39;
}

// convert morse to ascii and print
void print_cw() {
  char ch = lookup_cw(maddr);
  switch (maddr) {
    case 0xc5:
      printstr("<BK>");
      break;
    case 0x45:
      printstr("<SK>");
      break;
    default:
      printchar(ch);
      if (recordMsg) catc(tmpstr, ch);
      break;
  }
}

// update the morse code table address
void maddr_cmd(uint8_t cmd) {
  if (cmd == 2) {
    // print the tranlated ascii
    // and reset the table address
    print_cw();
    maddr = 1;
  }
  else {
    // update the table address
    beep(ON);
    maddr = maddr<<1;
    maddr |= cmd;
  }
}

// An Iambic (mode A/B) morse code keyer
// Copyright 2021 Scott Baker KJ7NLA

uint8_t  keyermode  = 0;   // keyer mode
uint8_t  keyswap    = 0;   // key swap

// keyerinfo bit definitions
#define DIT_REG     0x01     // dit pressed
#define DAH_REG     0x02     // dah pressed
#define KEY_REG     0x03     // dit or dah pressed
#define WAS_DIT     0x04     // last key was dit
#define SQUEEZE     0x08     // both dit and dah pressed

// keyermode bit definitions
#define IAMBICA   0        // iambic A keyer mode
#define IAMBICB   1        // iambic B keyer mode
#define ULTIMAA   2        // ultimatic A mode

// keyerstate machine states
#define KEY_IDLE    0
#define CHK_KEY     1
#define KEY_WAIT    2
#define IDD_WAIT    3
#define LTR_GAP     4
#define WORD_GAP    5

// more key info
#define GOTKEY  (keyerinfo & KEY_REG)
#define NOKEY   !GOTKEY
#define GOTBOTH  GOTKEY == KEY_REG

uint8_t  keyerstate = KEY_IDLE;
uint8_t  keyerinfo  = 0;

uint16_t dittime;        // dit time
uint16_t dahtime;        // dah time
uint16_t lettergap1;     // letter space for decode
uint16_t lettergap2;     // letter space for send
uint16_t wordgap1;       // word space for decode
uint16_t wordgap2;       // word space for send

// read and debounce paddles
void read_paddles() {
  uint8_t ditv1 = !digitalRead(pinDit);
  uint8_t dahv1 = !digitalRead(pinDah);
  uint8_t ditv2 = !digitalRead(pinDit);
  uint8_t dahv2 = !digitalRead(pinDah);
  if (ditv1 && ditv2) {
    if (keyswap) keyerinfo |= DAH_REG;
    else keyerinfo |= DIT_REG;
  }
  if (dahv1 && dahv2) {
    if (keyswap) keyerinfo |= DIT_REG;
    else keyerinfo |= DAH_REG;
  }
  if (GOTBOTH) keyerinfo |= SQUEEZE;
  else keyerinfo &= ~SQUEEZE;
  if (NOKEY) {
    // check for display timeout
    if ((xtimeout > 0) && (display == ON) && ((millis() - xdt) > ONE_SECOND)) {
      xdt = millis();
      xtimer++;
      if (xtimer > xtimeout) {
        display = OFF;
        oled.noDisplay();
      }
    }
  } else {
    reset_xtimer();  // reset display timeout
  }
}

// iambic keyer state machine
void iambic_keyer() {
  static uint32_t ktimer;
  uint8_t send_dit = NO;
  uint8_t send_dah = NO;
  switch (keyerstate) {
    case KEY_IDLE:
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
      } else {
        keyerinfo = 0;
      }
      break;
    case CHK_KEY:
      if (keyerinfo & SQUEEZE) {
        if (keyermode == ULTIMAA) {
          if (keyerinfo & WAS_DIT) send_dah = YES;
          else send_dit = YES;
        } else {
          // Iambic A or B
          if (keyerinfo & WAS_DIT) {
            keyerinfo &= ~WAS_DIT;
            send_dah = YES;
          } else {
            keyerinfo |= WAS_DIT;
            send_dit = YES;
          }
        }
      } else {
        if (keyerinfo & DIT_REG) {
          keyerinfo |= WAS_DIT;
          send_dit = YES;
        }
        if (keyerinfo & DAH_REG) {
          keyerinfo &= ~WAS_DIT;
          send_dah = YES;
        }
      }
      if (send_dit) {
        ktimer = millis() + dittime;
        maddr_cmd(0);
        keyerstate = KEY_WAIT;
      }
      else if (send_dah) {
        ktimer = millis() + dahtime;
        maddr_cmd(1);
        keyerstate = KEY_WAIT;
      }
      else keyerstate = KEY_IDLE;
      break;
    case KEY_WAIT:
      // wait dit/dah duration
      if (millis() > ktimer) {
        // done sending dit/dah
        beep(OFF);
        // inter-symbol time is 1 dit
        ktimer = millis() + dittime;
        keyerstate = IDD_WAIT;
      }
      break;
    case IDD_WAIT:
      // wait time between dit/dah
      if (millis() > ktimer) {
        // wait done
        keyerinfo &= ~KEY_REG;
        if ((keyermode == IAMBICA) || (keyermode == ULTIMAA)) {
          // Iambic A or Ultimatic
          // check for letter space
          ktimer = millis() + lettergap1;
          keyerstate = LTR_GAP;
        } else {
          // Iambic B
          read_paddles();
          if (NOKEY && (keyerinfo & SQUEEZE)) {
            // send opposite of last paddle sent
            if (keyerinfo & WAS_DIT) {
              // send a dah
              ktimer = millis() + dahtime;
              maddr_cmd(1);
            }
            else {
              // send a dit
              ktimer = millis() + dittime;
              maddr_cmd(0);
            }
            keyerinfo = 0;
            keyerstate = KEY_WAIT;
          } else {
            // check for letter space
            ktimer = millis() + lettergap1;
            keyerstate = LTR_GAP;
          }
        }
      }
      break;
    case LTR_GAP:
      if (millis() > ktimer) {
        // letter space found so print char
        maddr_cmd(2);
        // check for word space
        ktimer = millis() + wordgap1;
        keyerstate = WORD_GAP;
      }
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
      } else {
        keyerinfo = 0;
      }
      break;
    case WORD_GAP:
      if (millis() > ktimer) {
        // word gap found so print a space
        maddr = 1;
        print_cw();
        keyerstate = KEY_IDLE;
      }
      read_paddles();
      if (GOTKEY) {
        keyerstate = CHK_KEY;
      } else {
        keyerinfo = 0;
      }
      break;
    default:
      break;
  }
}

// handle straight key mode
void straight_key() {
  uint8_t pin = keyswap ? pinDah : pinDit;
  if (!digitalRead(pin)) {
    beep(ON);
    while(!digitalRead(pin)) wait_ms(1);
    beep(OFF);
  }
}

// convert ascii to morse
void send_cwchr(char ch) {
  uint8_t mcode;
  // remove the ascii code offset (0x20) to
  // create a lookup table address
  uint8_t addr = (uint8_t)ch - 0x20;
  // then lookup the Morse code from the a2m table
  // note: any unknown unknown ascii char is
  // translated to a '?' (mcode 0x4c)
  if (addr < 64) mcode = pgm_read_byte(a2m + addr);
  else mcode = 0x4c;
  // if space (mcode 0x01) is found
  // then wait for one word space
  if (mcode == 0x01) delay(wordgap2);
  else {
    uint8_t mask = 0x80;
    // use a bit mask to find the leftmost 1
    // this marks the start of the Morse code bits
    while (!(mask & mcode)) mask = mask >> 1;
    while (mask != 1) {
      mask = mask >> 1;
      // use the bit mask to select a bit from
      // the Morse code starting from the left
      beep(ON);
      // turn the sidetone on for dit or dah length
      // if the mcode bit is a 1 then send a dah
      // if the mcode bit is a 0 then send a dit
      wait_ms((mcode & mask) ? dahtime : dittime);
      beep(OFF);
      // turn the sidetone off for a symbol space
      delay(dittime);
    }
    delay(lettergap2);  // add letter space
  }
}

// transmit a CW message
void send_cwmsg(char *str, uint8_t prn) {
  if (prn) {
    myrow = 0;
    mycol = 0;
    oled.clrScreen();
  }
  for (uint8_t i=0; str[i]; i++) {
    send_cwchr(str[i]);
    if (prn) {
      printchar(str[i]);
      // stop if button pressed
      if (!digitalRead(pinSw1)) break;
    }
  }
}

// change the keyer speed
void change_wpm(uint8_t wpm) {
  keyerwpm = wpm;
  ditcalc();
}

// change the volume
void change_volume(uint8_t vol) {
  switch (vol) {
    case 0:
      OCR1BL = T1_PWM0;
      break;
    case 1:
      OCR1BL = T1_PWM1;
      break;
    case 2:
      OCR1BL = T1_PWM2;
      break;
    case 3:
      OCR1BL = T1_PWM3;
      break;
    case 4:
      OCR1BL = T1_PWM4;
      break;
    case 5:
      OCR1BL = T1_PWM5;
      break;
    default:
      break;
  }
}

// change the sidetone frequency
void change_tone(uint8_t tone) {
  switch (tone) {
    case 1:
      OCR2A = T600A;
      OCR2B = T600B;
      break;
    case 2:
      OCR2A = T700A;
      OCR2B = T700B;
      break;
    default:
      break;
  }
}

// back to run mode
void back2run() {
  menuID = INITMENU;
  if (!recordMsg) {
    oled.clrScreen();
    print_line(0, "READY >>");
  }
  delay(700);
  oled.clrScreen();
  myrow = 0;
  mycol = 0;
}

// initial keyer speed
void ditcalc() {
  dittime    = DITCONST/keyerwpm;
  dahtime    = (DITCONST * 3)/keyerwpm;
  lettergap1 = (DITCONST * 2.5)/keyerwpm;
  lettergap2 = (DITCONST * 3)/keyerwpm;
  wordgap1   = (DITCONST * 3)/keyerwpm;
  wordgap2   = (DITCONST * 7)/keyerwpm;
}

#define MAXWPM  40   // max keyer speed
#define MINWPM  10   // min keyer speed

// keyer speed control
void menu_wpm() {
  char tmp[12];
  uint8_t prev_wpm = keyerwpm;
  oled.clrScreen();
  print_line(0, "SPEED IS");
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & DAH_REG) keyerwpm++;
    if (keyerinfo & DIT_REG) keyerwpm--;
    // check limits
    if (keyerwpm < MINWPM) keyerwpm = MINWPM;
    if (keyerwpm > MAXWPM) keyerwpm = MAXWPM;
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    oled.setCursor(0,1);
    oled.print8(keyerwpm);
    oled.putstr(" WPM");
  }
  wait_switches();
  if (sw2Pushed) menuID--;
  else if (sw1Pushed) {
    // if wpm changed the recalculate the
    // dit timing and and send an OK message
    if (prev_wpm != keyerwpm) {
      ditcalc();
      send_cwmsg("OK", 0);
      menuID = BACK2RUN;
    } else {
      menuID++;
    }
  }
}

// select keyer mode
void menu_mode() {
  uint8_t prev_mode = keyermode;
  oled.clrScreen();
  print_line(0, "KEYER IS");
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & KEY_REG) keyermode++;
    if (keyermode > ULTIMAA) keyermode = IAMBICA;
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    switch (keyermode) {
      case IAMBICA:
        print_line(1, "IAMBIC A");
        break;
      case IAMBICB:
        print_line(1, "IAMBIC B");
        break;
      case ULTIMAA:
        print_line(1, "ULTIMATIC");
        break;
      default:
        break;
    }
  }
  wait_switches();
  if (sw2Pushed) menuID--;
  else if (sw1Pushed) {
    if (prev_mode != keyermode) {
      send_cwmsg("OK", 0);
      menuID = BACK2RUN;
    } else {
      menuID++;
    }
  }
}

const char *vol_label[6] = {"OFF","1","2","3","4","5"};

#define MAXVOL  5   // max volume
#define MINVOL  0   // min volume

// volume control
void menu_volume() {
  char tmp[12];
  uint8_t prev_vol = volume;
  oled.clrScreen();
  print_line(0, "VOLUME IS");
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & DAH_REG) volume++;
    if (volume && (keyerinfo & DIT_REG)) volume--;
    // check limits
    if (volume > MAXVOL) { volume = MAXVOL; }
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    print_line(1, vol_label[volume]);
  }
  wait_switches();
  if (sw2Pushed) menuID--;
  else if (sw1Pushed) {
    if (prev_vol != volume) {
      change_volume(volume);
      send_cwmsg("OK", 0);
      menuID = BACK2RUN;
    } else {
      menuID++;
    }
  }
}

const char *tone_label[3] = {"OFF","600","700"};

// sidetone control
void menu_tone() {
  uint8_t prev_tone = cwtone;
  oled.clrScreen();
  print_line(0, "SIDETONE IS");
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & DAH_REG) cwtone++;
    if (keyerinfo & DIT_REG) cwtone--;
    // check limits
    if (cwtone < TONE600) cwtone = TONE700;
    if (cwtone > TONE700) cwtone = TONE600;
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    print_line(1, tone_label[cwtone]);
  }
  wait_switches();
  if (sw2Pushed) menuID--;
  else if (sw1Pushed) {
    if (prev_tone != cwtone) {
      change_tone(cwtone);
      send_cwmsg("OK", 0);
      menuID = BACK2RUN;
    } else {
      menuID++;
    }
  }
}

const char *swap_label[2] = {"OFF","ON"};

// dit-dah swap control
void menu_swap() {
  if (keyswap > 1) keyswap = OFF;
  uint8_t prev_swap = keyswap;
  oled.clrScreen();
  print_line(0, "KEYSWAP IS");
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & KEY_REG) keyswap = !keyswap;
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    print_line(1, swap_label[keyswap]);
  }
  wait_switches();
  if (sw2Pushed) menuID--;
  else if (sw1Pushed) {
    if (prev_swap != keyswap) {
      send_cwmsg("OK", 0);
      menuID = BACK2RUN;
    } else {
      menuID++;
    }
  }
}

const char *led_label[2] = {"OFF","ACTIVE"};

// LED on/off
void menu_led() {
  uint8_t prev_led = useLED;
  oled.clrScreen();
  print_line(0, "LED IS");
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & KEY_REG) useLED = !useLED;
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    print_line(1, led_label[useLED]);
  }
  wait_switches();
  if (sw2Pushed) menuID--;
  else if (sw1Pushed) {
    if (prev_led != useLED) {
      send_cwmsg("OK", 0);
      menuID = BACK2RUN;
    } else {
      menuID++;
    }
  }
}

// eeprom save menu
void menu_save() {
  uint8_t saved2eeprom = NO;
  oled.clrScreen();
  print_line(0, "PRESS PADDLE");
  print_line(1, "TO SAVE CONFIG");
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & KEY_REG) {
      save_eeprom();
      saved2eeprom = YES;
      oled.clrScreen();
      print_line(0, "CONFIGURATION");
      print_line(1, "SAVED");
      delay(700);
      keyerinfo = 0;
      sw1Pushed = 1;
      break;
    }
  }
  wait_switches();
  if (sw2Pushed) menuID--;
  else if (sw1Pushed) {
    if (saved2eeprom) {
      send_cwmsg("OK", 0);
      menuID = BACK2RUN;
    } else {
      menuID++;
    }
  }
}

// send a CW message
void menu_msg() {
  uint32_t t0;
  uint8_t tmp;
  oled.clrScreen();
  show_editstr(1,0);
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & DAH_REG) msgindx++;
    if (keyerinfo & DIT_REG) msgindx--;
    // check limits
    if (msgindx < MINMSG) { msgindx = MAXMSG; }
    if (msgindx > MAXMSG) { msgindx = MINMSG; }
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    show_editstr(1,0);
  }
  if (sw1Pushed) {
    event = BSC;
    t0 = millis();
    tmp = sw1Pushed;
    // check for long button press
    while (tmp && (event != BPL)) {
      if (millis() > (t0 + LONGPRESS)) event = BPL;
      tmp = !digitalRead(pinSw1);
      delay(10);  // debounce
    }
  }
  if (event == BPL) {
    event = NBP;
    send_cwmsg(cwmsg[msgindx-1], 1);
    wait_switches();
    sw1Pushed = 1;
    sw2Pushed = 0;
  } else {
    wait_switches();
    if (sw2Pushed) menuID--;
    else if (sw1Pushed) menuID++;
  }
}

// edit a CW message
void menu_edit() {
  uint32_t t0;
  uint8_t tmp;
  oled.clrScreen();
  show_editstr(2,0);
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & DAH_REG) msgindx++;
    if (keyerinfo & DIT_REG) msgindx--;
    // check limits
    if (msgindx < MINMSG) { msgindx = MAXMSG; }
    if (msgindx > MAXMSG) { msgindx = MINMSG; }
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    show_editstr(2,0);
  }
  if (sw1Pushed) {
    event = BSC;
    t0 = millis();
    tmp = sw1Pushed;
    // check for long button press
    while (tmp && (event != BPL)) {
      if (millis() > (t0 + LONGPRESS)) event = BPL;
      tmp = !digitalRead(pinSw1);
      delay(10);  // debounce
    }
  }
  if (event == BPL) {
    event = NBP;
    sw1Pushed = 1;
    sw2Pushed = 0;
    show_editstr(0,1);
    wait_switches();
    edit_submenu();
  } else {
    wait_switches();
    if (sw2Pushed) menuID--;
    else if (sw1Pushed) menuID++;
  }
}

void edit_submenu() {
  uint32_t t0;
  uint8_t tmp;
  // loop until done
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & DAH_REG) incval =  1;
    if (keyerinfo & DIT_REG) incval = -1;
    if (incval) {
      get_spinpos(editstr[editpos]);
      spinchar();
    }
    keyerinfo = 0;
    // check switches
    sw1Pushed = !digitalRead(pinSw1);
    sw2Pushed = !digitalRead(pinSw2);
    if (sw1Pushed) {
      event = BSC;
      t0 = millis();
      tmp = sw1Pushed;
      // check for long button press
      while (tmp && (event != BPL)) {
        if (millis() > (t0 + LONGPRESS)) event = BPL;
        tmp = !digitalRead(pinSw1);
        delay(10);  // debounce
      }
    }
    // if long press then return
    if (event == BPL) {
      event = NBP;
      sw1Pushed = 1;
      sw2Pushed = 0;
      show_editstr(0,0);
      wait_switches();
      break;
    }
    // update edit position
    if (sw1Pushed || sw2Pushed) {
      if (sw1Pushed) editpos++;
      else editpos--;
      // limit checks
      tmp = len(tmpstr);
      if (editpos >= tmp) editpos = 0;
      else if (editpos < 0) editpos = tmp-1;
      // edit string cleanup
      insertchar();
      deletechar();
      show_editstr(0,1);
      spun = NO;
    }
  }
  return;
}

// record a CW message
void menu_record() {
  uint32_t t0;
  uint8_t tmp;
  oled.clrScreen();
  show_editstr(3,0);
  // loop until button is pressed
  while (TRUE) {
    reset_xtimer();  // reset display timeout
    if (read_switches()) break;
    keyerinfo = 0;
    read_paddles();
    if (keyerinfo & DAH_REG) msgindx++;
    if (keyerinfo & DIT_REG) msgindx--;
    // check limits
    if (msgindx < MINMSG) { msgindx = MAXMSG; }
    if (msgindx > MAXMSG) { msgindx = MINMSG; }
    while (GOTKEY) {
      keyerinfo = 0;
      read_paddles();
      delay(10);
    }
    keyerinfo = 0;
    show_editstr(3,0);
  }
  if (sw1Pushed) {
    event = BSC;
    t0 = millis();
    tmp = sw1Pushed;
    // check for long button press
    while (tmp && (event != BPL)) {
      if (millis() > (t0 + LONGPRESS)) event = BPL;
      tmp = !digitalRead(pinSw1);
      delay(10);  // debounce
    }
  }
  if (event == BPL) {
    event = NBP;
    sw1Pushed = 0;
    sw2Pushed = 0;
    recordMsg = ON;
    menuID = BACK2RUN;
    tmpstr[0] = '\0';
    wait_switches();
  } else {
    wait_switches();
    if (sw2Pushed) menuID--;
    else if (sw1Pushed) menuID = BACK2RUN;
  }
}

#define EEPROM_OFFSET 0x00

// write to the eeprom
void save_eeprom() {
  uint16_t addr = EEPROM_OFFSET;
  EEPROM.updateByte(addr, keyerwpm);  addr++;
  EEPROM.updateByte(addr, keyermode); addr++;
  EEPROM.updateByte(addr, volume);    addr++;
  EEPROM.updateByte(addr, cwtone);    addr++;
  EEPROM.updateByte(addr, keyswap);   addr++;
  EEPROM.writeBlock<char>(addr, cwmsg[0], MAXLEN*MAXMSG);
}

// read the eeprom
void read_eeprom() {
  uint16_t addr = EEPROM_OFFSET;
  keyerwpm  =  EEPROM.readByte(addr); addr++;
  keyermode =  EEPROM.readByte(addr); addr++;
  volume    =  EEPROM.readByte(addr); addr++;
  cwtone    =  EEPROM.readByte(addr); addr++;
  keyswap   =  EEPROM.readByte(addr); addr++;
  EEPROM.readBlock<char>(addr, cwmsg[0], MAXLEN*MAXMSG);
}

void factory_reset() {
  oled.clrScreen();
  print_line(0, "FACTORY RESET");
  print_line(1, VERSION);
  print_line(2, DATE);
  cpy(cwmsg[0],"MSG1 0123456789 ABCD");
  cpy(cwmsg[1],"MSG2 0123456789 ABCD");
  cpy(cwmsg[2],"MSG3 0123456789 ABCD");
  cpy(cwmsg[3],"MSG4 0123456789 ABCD");
  cpy(cwmsg[4],"MSG5 0123456789 ABCD");
  cpy(cwmsg[5],"MSG6 0123456789 ABCD");
  cpy(cwmsg[6],"MSG7 0123456789 ABCD");
  cpy(cwmsg[7],"MSG8 0123456789 ABCD");
  keyerwpm  = INITWPM;
  keyermode = IAMBICA;
  volume    = INITVOL;
  cwtone    = TONE700;
  keyswap   = OFF;
  save_eeprom();
  ditcalc();
  change_volume(volume);
  change_tone(cwtone);
  delay(500);
  back2run();
}

// return the length of string
uint8_t len(char *str) {
  uint8_t i=0;
  while (str[i++]);
  return i-1;
}

// copy a string
void cpy(char *dst, char *src) {
  uint8_t i=0;
  while (src[i]) dst[i++] = src[i];
  dst[i] = '\0';
}

// concatenate a string and return length
uint8_t cat(char *dst, char *src) {
  uint8_t i=0;
  uint8_t strlen= len(dst);
  while (src[i]) dst[strlen++] = src[i++];
  dst[strlen] = '\0';
  return i;
}

// concatenate a character to a string
void catc(char *dst, char ch) {
  uint8_t strlen= len(dst);
  dst[strlen++] = ch;
  dst[strlen] = '\0';
}

// convert to lowercase
void lower(uint8_t i) {
  char ch;
  ch = tmpstr[i];
  if ((ch >= 'A') && (ch <= 'Z')) {
    ch += 32;
    tmpstr[i] = ch;
  }
  if ((ch >= '0') && (ch <= '9')) {
    ch += 79;
    tmpstr[i] = ch;
  }
  if (ch == ' ') {
    ch = '_';
    tmpstr[i] = ch;
  }
}

// interruptable delay
void wait_ms(uint16_t dly) {
  uint32_t curTime = millis();
  uint32_t endTime = curTime + dly;
  while(curTime < endTime){
    curTime = millis();
  }
}

// turn beep on/off
void beep(uint8_t onoff) {
  if (onoff == ON) {
    digitalWrite(pinKey, HIGH);
    if (useLED) digitalWrite(pinLED, HIGH);
    if (volume) timer2_start();
  } else {
    digitalWrite(pinKey, LOW);
    if (useLED) digitalWrite(pinLED, LOW);
    if (volume) timer2_stop();
  }
}

// reset the Arduino
void doError() {
  resetFunc();
}

// program setup
void setup() {
  gpio_init();
  timer1_init();
  timer2_init();
  Wire.begin();
  Wire.setClock(600000);
  Serial.begin(BAUDRATE);
  read_eeprom();
  change_wpm(INITWPM);
  change_volume(volume);
  change_tone(cwtone);
  oled.begin(16, 4);
  print_line(0, "FIRMWARE REV");
  print_line(1, VERSION);
  print_line(2, DATE);
  xdt = millis();
  delay(THREE_SECONDS);
  oled.clrScreen();
}

// main loop
void loop() {
  uint32_t t0;
  sw1Pushed |= !digitalRead(pinSw1);
  if (sw1Pushed) {
    // reset display timeout
    xtimer = 0;
    xdt = millis();
    if (display == OFF) {
      display = ON;
      oled.onDisplay();
    }
  }
  if (menuID == BACK2RUN) {
    sw1Pushed = 0;
    sw2Pushed = 0;
    back2run();
  }
  if (recordMsg) {
    // check switches
    sw1Pushed = !digitalRead(pinSw1);
    sw2Pushed = !digitalRead(pinSw2);
    if (sw1Pushed || sw2Pushed) {
      recordMsg = OFF;
      sw1Pushed = 0;
      sw2Pushed = 0;
      event = NBP;
      cpy(cwmsg[msgindx-1],tmpstr);
      wait_switches();
      menuID = MENUEDIT;
      menu_edit();
    }
  }
  if (sw1Pushed) {
    event = BSC;
    t0 = millis();
    // check for long button press
    while (sw1Pushed && (event != BPL)) {
      if (millis() > (t0 + LONGPRESS)) event = BPL;
      sw1Pushed = !digitalRead(pinSw1);
      delay(10);  // debounce
    }
    sw1Pushed = 0;
    sw2Pushed = 0;
    // button single click
    if (event == BSC) {
      switch (menuID) {
        case MENUWPM:
          menu_wpm();
          break;
        case MENUMODE:
          menu_mode();
          break;
        case MENUVOL:
          menu_volume();
          break;
        case MENUTONE:
          menu_tone();
          break;
        case MENUSWAP:
          menu_swap();
          break;
        case MENULED:
          menu_led();
          break;
        case MENUSAVE:
          menu_save();
          break;
        case MENUMSG:
          menu_msg();
          break;
        case MENUEDIT:
          menu_edit();
          break;
        case MENUREC:
          menu_record();
          break;
        default:
          back2run();
          break;
      }
      event = NBP;
    }
    // long button press
    else if (event == BPL) {
      factory_reset();
      event = NBP;
      wait_switches();
    }
  }
  // menu actions complete
  iambic_keyer();
}
