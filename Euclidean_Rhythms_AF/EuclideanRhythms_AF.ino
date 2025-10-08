/*
  Code for a modified version of Hagiwo's excellent Euclidean Rhythm generator
  (https://note.com/solder_state/n/n433b32ea6dbc)
  
  Hardware modifications:
  - SPI rather than I2C OLED display for speed, using Adafruit_SSD1306 hardware SPI
    (Note: as of 16/09/2025, the Adafruit library is ~2x faster than either u8glib or u8g2)
  - buffered outputs plus LEDs
  - random mode probability control via potentiometer and external CV
  - gate input to zero all step counters using voltage control

  Software modifications:
  - code restructured for efficiency
  - uses direct port manipulation to speed up and synchronise trigger output
  - fixed problems with 1-hit mode (line rotated wrong way on offset, consequently
    not matching step/hit markers)
  - menus wrap around, encoder button autorepeats
  - resets current step to 0 when the number of steps rolls around to zero
  - doesn't display step circle if the number of hits or number of steps is zero
  - renamed menu items L(imit) to S(teps) and R(eset) to Z(ero)
  - pattern settings can be stored for retrieval in a subsequent session using P(ut)
    into and G(et) from EEPROM, in both the channel and random menus
  - runs rotary encoder in single-interrupt mode using pins 3 (interrupt) and 4
  - handles incoming triggers on pin 2 using an interrupt service routine, allowing
    screen refresh immediately on menu changes without compromising timing
  - screen saver blanks display after specified period, wakes on rotary encoder
    rotation or button, can be disabled if desired
  - option for zeroing all channels simultaneously by gate input
*/

#include <Encoder.h>
#include <EEPROM.h>

#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 12

// Screen Saver timeout in seconds (comment out next line to disable screen saver)
#define SS_TIMEOUT 30

// Uncomment next line to enable code for a 'zero all steps' gate input
//#define ZRO_PIN 6

// Pin definitions. CLK_PIN MUST be an interrupt-enabled pin (2 or 3 on an Arduino Pro mini)
#define BTN_PIN 5
#define CLK_PIN 2

// OLED connections using hardware SPI
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, 10, 12, -1); // SCK = 13, COPI = 11, D/C = 10, RST = 12, CS not used

// rotary encoder in single-interrupt mode (pin 3)
Encoder encoder(3, 4);
int oldEnc = -999; 
int newEnc = -999;
unsigned long encMillis; // for screen saver

// screen saver
bool screenSave = 0;
#ifdef SS_TIMEOUT
const unsigned long ssTimeout = SS_TIMEOUT * 1000;
#else
const unsigned long ssTimeout = 0;
#endif

#ifdef ZRO_PIN
// zero pin
bool zero = 0;
bool oldZero = 0;
#endif

// encoder button
unsigned long encBtn_millis = 0; // for debounce / autorepeat

// flag for trigger processed in ISR
volatile bool triggered = 0;

// Initial parameters for each channel
// The startup configuration can be changed by altering these values
uint8_t numHits[6]  = { 4, 4, 5, 3, 2, 16 };      // number of hits
uint8_t offset[6]   = { 0, 2, 0, 8, 3, 9 };       // hit offsets
bool    mute[6]     = { 0, 0, 0, 0, 0, 0 };       // 0 = unmuted, 1 = muted
uint8_t numSteps[6] = { 16, 16, 16, 16, 16, 16 }; // enabled steps

// Probabilities of change in random mode
//   With the probability control set to the middle of its range
//   these are the % probabilities that randomisation will occur
// Adjust these values to modify the probabilities of change
const uint8_t hitProb[6]    = {  0, 10, 20, 40, 40, 20 };
const uint8_t offsetProb[6] = {  0, 20, 20, 20, 40, 20 };
const uint8_t muteProb[6]   = { 20, 20, 20, 20, 20, 20 };
const uint8_t hitRngMax[6]  = {  4,  6, 16,  8,  9, 16 };
const uint8_t hitRngMin[6]  = {  4,  1,  6,  1,  5, 10 };

// The 17 Euclidian rhythm patterns of 16 steps
const static uint8_t Euclid[17][16] PROGMEM = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0 },
  { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0 },
  { 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0 },
  { 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 1, 0, 0, 0 },
  { 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0 },
  { 1, 0, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 1, 0, 1, 0 },
  { 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0 },
  { 1, 0, 1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0 },
  { 1, 0, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
  { 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1 },
  { 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
  { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 1 },
  { 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }
};
bool pattern[6][16]; // offset buffer, Stores the offset result

volatile uint8_t playingStep[6] = { 0, 0, 0, 0, 0, 0 }; // playing step number for channels 1 -> 6
volatile unsigned long trgMillis = 0; // for output trigger length and display blanking
uint8_t trgLen = 4; // trigger length in milliseconds - sets to 10ms when screensave active

// Display parameters
int8_t  selectMenu = 0; // 0 = channel/random, 1 = hits/numBars, 2 = offset/get, 3 = numSteps/put, 4 = mute, 5 = zero, 6 = get, 7 = put
uint8_t selectChnl = 0; // 0 -> 5 = channels 1 -> 6, 6 = random mode
bool   dispRefresh = 1; // 1 = refresh display this pass, 0 = no refresh

// Offsets for the channels' Euclidean displays
const uint8_t positionX[6] = { 0, 40, 80, 13, 53, 93 };
const uint8_t positionY[6] = { 0,  0,  0, 33, 33, 33 };

// Vertex coordinates
const uint8_t vertexX[16] = { 15, 21, 26, 29, 30, 29, 26, 21, 15,  9,  4,  1,  0,  1,  4,  9 };
const uint8_t vertexY[16] = {  0,  1,  4,  9, 15, 21, 26, 29, 30, 29, 26, 21, 15,  9,  4,  1 };

uint8_t stepCnt = 0; // step count
uint8_t barNow = 1; // current bar; increases by one after 16 steps
const uint8_t barMax[4] = { 2, 4, 8, 16 }; // mumber of bars to stay fixed in random mode
uint8_t barSelect = 1; // selected initial barMax value

void setup() {

  display.begin(SSD1306_SWITCHCAPVCC, 0, 1);

  // Pin modes
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(CLK_PIN, INPUT);
  pinMode(14, OUTPUT); // A0: channel 1
  pinMode(15, OUTPUT); // A1: channel 2
  pinMode(16, OUTPUT); // A2: channel 3
  pinMode(17, OUTPUT); // A3: channel 4
  pinMode(18, OUTPUT); // A4: channel 5
  pinMode(19, OUTPUT); // A5: channel 6

  attachInterrupt(digitalPinToInterrupt(CLK_PIN), handleTrigger, RISING);

  // Set up the OLED display
  display.setTextSize(0); // Output character size
  display.setTextColor(WHITE); // Output text color
  display.clearDisplay();
  display.display();

  encMillis = millis();
}

void loop() {
#ifdef ZRO_PIN
  //----------------Gate on Zero pin---------------------
  zero = digitalRead(ZRO_PIN);
  if (zero && !oldZero) {
    oldZero = 1;
    for (uint8_t k = 0; k < 6; k++) {
      playingStep[k] = 0;
    }
  } else if (!zero && oldZero) {
    oldZero = 0;
  }
#endif

  //-----------------Rotary encoder----------------------
  newEnc = encoder.read();
  if (newEnc < oldEnc - 3) { // turn left by one detent
    oldEnc = newEnc;
    encMillis = millis();
    if (screenSave) {
      screenSave = 0;
      trgLen = 4;
    } else {
      --selectMenu;
    }

  } else if (newEnc > oldEnc + 3) { // turn right by one detent
    oldEnc = newEnc;
    encMillis = millis();
    if (screenSave) {
      screenSave = 0;
      trgLen = 4;
    } else {
      ++selectMenu;
    }
  }

  if (!screenSave) {
    // Update the current menu item
    if (6 == selectChnl) { // random mode
      if (selectMenu > 3) selectMenu = 0;
      else if (selectMenu < 0) selectMenu = 1;
    } else { // channel mode
      if (selectMenu > 7) selectMenu = 0;
      else if (selectMenu < 0) selectMenu = 7;
    }
  }
  
  //-----------------Encoder button----------------------
  if (!digitalRead(BTN_PIN) && (millis() - encBtn_millis >= 400)) { // button pressed outside debounce/autorepeat period
    encBtn_millis = millis();
    encMillis = millis();

    if (screenSave) {
      screenSave = 0;
      trgLen = 4;

    } else {
      switch (selectMenu) {
        case 0: // select channel
          ++selectChnl;
          selectChnl %= 7;
          break;

        case 1: // increment number of hits (channel mode) or number of bars (random mode)
          if (selectChnl != 6) { // channel mode
            ++numHits[selectChnl];
            numHits[selectChnl] %= 17;

          } else { // random mode
            ++barSelect;
            barSelect %= 4;
          }
          break;

        case 2: // increment offset (channel mode) or get from EEPROM (random mode)
          if (selectChnl != 6) { // channel mode
            ++offset[selectChnl];
            offset[selectChnl] %= 16;

          } else { // random mode
            getFromEEPROM();
          }
          break;

        case 3: // increment number of steps (channel mode) or put into EEPROM (random mode)
          if (selectChnl != 6) { // channel mode
            ++numSteps[selectChnl];
            numSteps[selectChnl] %= 17;
            if (playingStep[selectChnl] > numSteps[selectChnl]) playingStep[selectChnl] = 0;

          } else { // random mode
            putIntoEEPROM();
          }
          break;

        case 4: // toggle mute
          mute[selectChnl] = !mute[selectChnl] ;
          break;

        case 5: // reset to step 0 on all channels
          for (uint8_t k = 0; k < 6; k++) {
            playingStep[k] = 0;
          }
          break;

        case 6: // Get stored settings from EEPROM
          getFromEEPROM();
          break;

        case 7: // Put current settings into EEPROM
          putIntoEEPROM();
          break;
      }
    }
  }

  //------------update steps and bars after trigger processing-------------
  if (triggered) {
    triggered = 0;

    if (6 == selectChnl) { // random mode setting
      ++stepCnt;
      if (stepCnt > 15) {
        ++barNow;
        stepCnt = 0;
        if (barNow > barMax[barSelect]) {
          barNow = 1;
          randomChange();
        }
      }
    }
    dispRefresh = 1;
  }

  //-----------------set channel patterns----------------------
  for (uint8_t k = 0; k < 6; k++) { // k = channels 1 -> 6
    for (uint8_t j = offset[k]; j < 16; j++) {
      pattern[k][j - offset[k]] = pgm_read_byte(&Euclid[numHits[k]][j]);
    }

    for (uint8_t j = 0; j < offset[k]; j++) {
      pattern[k][16 - offset[k] + j] = pgm_read_byte(&Euclid[numHits[k]][j]);
    }
  }
  //dispRefresh = 1;

  if (millis() - trgMillis > trgLen) { // turn off triggers after 5ms
    PORTC = 0;
  }

  if (dispRefresh || (millis() - trgMillis > 500)) {
    OLED_display();
    dispRefresh = 0;
  }
}

void randomChange() {
  // Called in random mode when barNow is reset

  // Configured so a 0V probability input will fix the patterns,
  // 50% gives about the same probability as the original Hagiwo code
  // and 100% gives about double the chance of change
  unsigned long mult = 1L + (unsigned long) analogRead(A6);
  for (uint8_t k = 0; k < 6; k++) {
    if ((mult * (unsigned long) hitProb[k]) >> 9 >= random(1, 100)) { // random change of numHits
      numHits[k] = random(hitRngMin[k], hitRngMax[k] + 1);
    }

    if ((mult * (unsigned long) offsetProb[k]) >> 9 >= random(1, 100)) { // random change of offset
      offset[k] = random(0, 16);
    }

    mute[k] = ((mult * (unsigned long) muteProb[k]) >>9 >= random(1, 100)); // random change of mute
  }
}

void OLED_display() {
  display.clearDisplay();

#ifdef SS_TIMEOUT
  if (!screenSave && millis() - encMillis > ssTimeout) {
    // Blank screen the specified period after the last encoder operation
    screenSave = 1;
    trgLen = 10;
    display.display();
  }
  if (screenSave) return;
#endif

  if (selectChnl != 6) { // channel mode menu
    display.setCursor(120, 0);
    display.print(selectChnl + 1);
    display.setCursor(120, 8);
    display.print("H");
    display.setCursor(120, 16);
    display.print("O");
    display.setCursor(120, 24);
    display.print("S");
    display.setCursor(0, 33);
    display.print("M");
    display.setCursor(0, 41);
    display.print("Z");
    display.setCursor(0, 49);
    display.print("G");
    display.setCursor(0, 57);
    display.print("P");

  } else { // random mode menu
    display.setCursor(120, 0);
    display.print("R");
    display.setCursor(120, 8);
    display.print("L");
    display.setCursor(120, 16);
    display.print("G");
    display.setCursor(120, 24);
    display.print("P");
    display.drawRect(0, 62 - barMax[barSelect] * 2, 6, barMax[barSelect] * 2 + 1, WHITE);
    display.fillRect(0, 63 - barNow * 2 , 6, barMax[barSelect] * 2 + 1, WHITE);
  }

  // draw menu select triangle
  switch (selectMenu) {
    case 0:
      display.fillTriangle(113, 0, 113, 6, 118, 3, WHITE);
      break;
    case 1:
      display.fillTriangle(113, 8, 113, 14, 118, 11, WHITE);
      break;
    case 2:
      display.fillTriangle(113, 16, 113, 22, 118, 19, WHITE);
      break;
    case 3:
      display.fillTriangle(113, 24, 113, 30, 118, 27, WHITE);
  }
  if (selectChnl != 6) { // channel mode
    switch (selectMenu) {
      case 4:
        display.fillTriangle(12, 34, 12, 40, 7, 37, WHITE);
        break;
      case 5:
        display.fillTriangle(12, 42, 12, 48, 7, 45, WHITE);
        break;
      case 6:
        display.fillTriangle(12, 50, 12, 56, 7, 53, WHITE);
        break;
      case 7:
        display.fillTriangle(12, 58, 12, 64, 7, 61, WHITE);
    }
  }

  // draw step dots
  for (uint8_t k = 0; k < 6; k++) {
    for (uint8_t j = 0; j <= numSteps[k] - 1; j++) { // j = steps
      display.drawPixel(vertexX[j] + positionX[k], vertexY[j] + positionY[k], WHITE);
    }
  }

  // draw hits lines
  for (uint8_t k = 0; k < 6; k++) {
    uint8_t lineXbuf[17]; // Buffers for drawing lines
    uint8_t lineYbuf[17];
    uint8_t vertexCount = 0;
    uint8_t j;
    for (j = 0; j < 16; j++) {
      if (pattern[k][j]) {
        // calculate screen coordinates of the current pattern's vertices
        lineXbuf[vertexCount] = vertexX[j] + positionX[k];
        lineYbuf[vertexCount] = vertexY[j] + positionY[k];
        ++vertexCount;
      }
    }

    if (numHits[k] > 1) { // 2 -> 16 hits - draw n-gon
      for (j = 0; j < vertexCount - 1; j++) {
        display.drawLine(lineXbuf[j], lineYbuf[j], lineXbuf[j + 1], lineYbuf[j + 1], WHITE);
      }
      display.drawLine(lineXbuf[0], lineYbuf[0], lineXbuf[j], lineYbuf[j], WHITE);

    } else if (1 == numHits[k]) { // 1 hit - draw line from centre
      display.drawLine(15 + positionX[k], 15 + positionY[k], lineXbuf[0], lineYbuf[0], WHITE);
    }

  // draw play step circle unless muted or numSteps==0
    if (!mute[k] && numSteps[k]) {
      if (pattern[k][playingStep[k]]) {
        display.fillCircle(vertexX[playingStep[k]] + positionX[k], vertexY[playingStep[k]] + positionY[k], 2, WHITE);
      } else {
        display.drawCircle(vertexX[playingStep[k]] + positionX[k], vertexY[playingStep[k]] + positionY[k], 2, WHITE);
      }
    }
  }
  display.display();
}

void handleTrigger(void) {
  trgMillis = millis();
  uint8_t port = 0;
  for (uint8_t j = 0; j < 6; j++) {
    ++playingStep[j]; // increment the step count on an incoming trigger
    playingStep[j] %= numSteps[j];
    if ((1 == pattern[j][playingStep[j]]) && !mute[j]) {
      port |= 64;
    }
    port >>= 1;
  }
  PORTC = port;

  triggered = 1;
}

void getFromEEPROM(void) {
  // values are interleaved for efficiency in reading and writing
  if (EEPROM.read(0) != 255) { // EEPROM contains data
    noInterrupts();
    uint16_t address = 0;
    for (uint8_t k = 0; k < 6; k++) {
      for (uint8_t j = 0; j < 16; j++) {
        pattern[k][j] = EEPROM.read(address++);
      }
      numHits[k] = EEPROM.read(address++);
      offset[k] = EEPROM.read(address++);
      mute[k] = EEPROM.read(address++);
      numSteps[k] = EEPROM.read(address++);
      playingStep[k] = EEPROM.read(address++);
    }
    selectChnl = EEPROM.read(address++);
    stepCnt = EEPROM.read(address++);
    barNow = EEPROM.read(address++);
    barSelect = EEPROM.read(address++);
    interrupts();
  }
}

void putIntoEEPROM(void) {
  // values are interleaved for efficiency in reading and writing
  noInterrupts();
  uint16_t address = 0;
  for (uint8_t k = 0; k < 6; k++) {
    for (uint8_t j = 0; j < 16; j++) {
      EEPROM.update(address++, pattern[k][j]);
    }
    EEPROM.update(address++, numHits[k]);
    EEPROM.update(address++, offset[k]);
    EEPROM.update(address++, mute[k]);
    EEPROM.update(address++, numSteps[k]);
    EEPROM.update(address++, playingStep[k]);
   }
  EEPROM.update(address++, selectChnl);
  EEPROM.update(address++, stepCnt);
  EEPROM.update(address++, barNow);
  EEPROM.update(address++, barSelect);
  interrupts();
}
