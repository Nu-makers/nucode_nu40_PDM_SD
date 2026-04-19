//------------------------------------------------------------------------
//Board Library : NUBoards nRF52 1.0.2
//Board Select  : NU40DK nRF52840
// Modified: Hardware I2S + Dynamic Vol + Dynamic Gain + Smart Wrap OLED Terminal
//------------------------------------------------------------------------
#include <Adafruit_TinyUSB.h>
#include <SPI.h>
#include <SD.h>
#include <PDM.h>
#include <RingBuf.h>    
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define FIFO_SIZE   32767
#define BUFF_SIZE   32767

// --- Custom Pin Map Definition ---
#define SD_SCK      27  
#define SD_MISO     4   
#define SD_MOSI     26  
#define SD_CS       23  
#define SD_DEC      47  

#define MY_LED_RED   13  
#define MY_LED_GREEN 14  
#define MY_LED_BLUE  15  

#define D0 35
#define D1 36

#define I2S_LRC_PIN  40  
#define I2S_BCLK_PIN 41  
#define I2S_DIN_PIN  19  
#define I2S_SD_MODE  42  

// Button pins (4)
#define BTN_REC      31  // P0.31 (BTN1)
#define BTN_PLAY     37  // P1.05 (BTN4)
#define BTN_PREV     39  // P1.07 (BTN2)
#define BTN_NEXT     38  // P1.06 (BTN3)

// OLED pin
#define OLED_SDA     22  // P0.22
#define OLED_SCL     21  // P0.21
// ------------------------

// --- OLED Terminal Display Settings ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET    -1 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define MAX_OLED_LINES 4
#define MAX_CHARS_PER_LINE 21 // 128px / 6px = Max 21 characters
String oledLines[MAX_OLED_LINES] = {"", "", "", ""};

// Internal use: A function that shifts rows of an array up and inserts a new row at the bottom
void pushOledLine(String line) {
  for (int i = 0; i < MAX_OLED_LINES - 1; i++) {
    oledLines[i] = oledLines[i + 1];
  }
  oledLines[MAX_OLED_LINES - 1] = line;
}

// Output function with smart line breaks applied
void oledPrint(String msg) {
  Serial.println(msg);
  
  int len = msg.length();
  if (len == 0) {
    pushOledLine("");
  } else {
    for (int i = 0; i < len; i += MAX_CHARS_PER_LINE) {
      pushOledLine(msg.substring(i, i + MAX_CHARS_PER_LINE));
    }
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  for (int i = 0; i < MAX_OLED_LINES; i++) {
    display.println(oledLines[i]);
  }
  display.display();
}
// ---------------------------------

char logFileName[20];
File logFile;
File playFile;
RingBuf<short, FIFO_SIZE> fifo;

short sampleBuffer[BUFF_SIZE];  
volatile int samplesRead;       

volatile bool isRecording = false;
bool isPlaying = false;

// Variables for post-recording (Collecting remaining buffer)
bool isPostRecording = false;
unsigned long postRecordUntil = 0;

int maxFileIndex = 0;      
int currentPlayIndex = 0;  
uint32_t wavDataSize = 0;

// Volume control variables
int currentVolume = 5;       
const int BASE_VOLUME = 5;   
const int MAX_VOLUME = 40;
bool btn4UsedAsModifier = false; 

// [Modified] Mic Gain control variables (0 ~ 250 in steps of 25)
int currentGain = 50;       // Default gain 50
const int MAX_GAIN = 250;
bool btn1UsedAsModifier = false; // Flag to prevent recording trigger when used for gain adj

// Multi-button debounce
#define NUM_BTNS 4
const int btnPins[NUM_BTNS] = {BTN_REC, BTN_PLAY, BTN_PREV, BTN_NEXT};
int btnLastState[NUM_BTNS] = {HIGH, HIGH, HIGH, HIGH};
int btnState[NUM_BTNS] = {HIGH, HIGH, HIGH, HIGH};
unsigned long btnLastDebounce[NUM_BTNS] = {0, 0, 0, 0};
bool btnPressed[NUM_BTNS] = {false, false, false, false};
bool btnReleased[NUM_BTNS] = {false, false, false, false}; 
const unsigned long debounceDelay = 30;

// --- I2S Playback Buffer ---
#define PLAY_CHUNK_SIZE 256
int16_t monoPlayBuf[PLAY_CHUNK_SIZE];
int16_t stereoPlayBuf[2][PLAY_CHUNK_SIZE * 2]; 
int playBufIdx = 0;
bool isI2SStarted = false;

// Function declarations
void writeWavHeader(File& file, uint32_t dataSize);
void errorBlink(uint8_t err);
void startPlayback();
void stopPlayback();
void initNativeI2S();

// *************************************************************** setup()
void setup() {
  Serial.begin(115200);

  Wire.setPins(OLED_SDA, OLED_SCL);
  Wire.begin();
  
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 Init Failed!");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setTextWrap(false);
    display.setCursor(0, 0);
    display.display();
  }
  
  oledPrint("System Booting...");

  pinMode(MY_LED_RED, OUTPUT);     
  pinMode(MY_LED_GREEN, OUTPUT);
  pinMode(MY_LED_BLUE, OUTPUT);
  pinMode(D0, OUTPUT);          
  pinMode(D1, OUTPUT);
  
  digitalWrite(MY_LED_RED, HIGH);
  digitalWrite(MY_LED_GREEN, HIGH);
  digitalWrite(MY_LED_BLUE, HIGH);
  digitalWrite(D0, LOW);
  digitalWrite(D1, LOW);

  pinMode(I2S_SD_MODE, OUTPUT);
  digitalWrite(I2S_SD_MODE, LOW);

  for(int i=0; i<NUM_BTNS; i++) {
    pinMode(btnPins[i], INPUT_PULLUP);
  }

  SPI.setPins(SD_MISO, SD_SCK, SD_MOSI);
  SPI.begin();

  if(!SD.begin(32000000UL, SD_CS)) {            
    oledPrint("SD Init Failed!");
    errorBlink(2);
  } 

  maxFileIndex = 0;
  while (true) {
    char tempName[20];
    sprintf(tempName, "REC_%03d.WAV", maxFileIndex + 1);
    if (SD.exists(tempName)) {
      maxFileIndex++;
    } else {
      break;
    }
  }
  currentPlayIndex = maxFileIndex > 0 ? maxFileIndex : 0; 
  oledPrint("Found " + String(maxFileIndex) + " Files");

  initNativeI2S();

  PDM.setPins(7, 5, -1);
  PDM.onReceive(onPDMdata);       
  PDM.setBufferSize(BUFF_SIZE);   
  
  // [Modified] Apply default gain
  PDM.setGain(currentGain);                 

  if (!PDM.begin(1, 16000)) {     
    oledPrint("PDM Init Failed!");
    errorBlink(4);
  }

  oledPrint("System Ready!");
}

// ************************************************************** loop()
void loop() 
{
  // --- 1. Button state update ---
  for(int i=0; i<NUM_BTNS; i++) {
    btnPressed[i] = false; 
    btnReleased[i] = false;
    
    int reading = digitalRead(btnPins[i]);
    if (reading != btnLastState[i]) {
      btnLastDebounce[i] = millis();
    }
    if ((millis() - btnLastDebounce[i]) > debounceDelay) {
      if (reading != btnState[i]) {
        btnState[i] = reading;
        if (btnState[i] == LOW) btnPressed[i] = true; 
        else btnReleased[i] = true; 
      }
    }
    btnLastState[i] = reading; 
  }

  // [ BTN1: Record / Stop (Release Trigger) ]
  if (btnReleased[0]) {
    // Check if BTN1 was used as a modifier key for Gain adj
    if (!btn1UsedAsModifier) {
      if (isPlaying) stopPlayback(); 
      if (!isRecording) { 
        isRecording = true; 
        isPostRecording = false;
        maxFileIndex++; 
        currentPlayIndex = maxFileIndex;
        sprintf(logFileName, "REC_%03d.WAV", maxFileIndex);
        oledPrint("REC START: " + String(logFileName));
        logFile = SD.open(logFileName, FILE_WRITE);
        wavDataSize = 0;
        writeWavHeader(logFile, 0); 
        short temp;
        while(fifo.lockedPop(temp)) { } 
      } 
      else if (!isPostRecording) { 
        oledPrint("Finishing REC...");  
        isPostRecording = true;
        postRecordUntil = millis() + 1000; 
      }
    }
    btn1UsedAsModifier = false; // Reset modifier flag on release
  }

  // Handle actual recording stop after 1.0s buffer collection
  if (isPostRecording && millis() > postRecordUntil) {
    isRecording = false;
    isPostRecording = false;
    short temp;
    while(fifo.lockedPop(temp)) {
      logFile.write(byte(temp & 0x00FF)); 
      logFile.write(byte((temp >> 8) & 0xFF));     
      wavDataSize += 2;
    }
    logFile.flush(); 
    logFile.close(); 
    logFile = SD.open(logFileName, O_RDWR); 
    if (logFile) {
      logFile.seek(0); 
      writeWavHeader(logFile, wavDataSize); 
      logFile.close();
      oledPrint("REC STOP: Saved");
    }
  }

  // [ BTN2: Prev / Vol Down / Gain Down ]
  if (btnPressed[2]) {
    if (btnState[1] == LOW) { // BTN4 is pressed (Volume Down)
      if (currentVolume > 5) currentVolume -= 5;
      else if (currentVolume > 0) currentVolume -= 1;
      if (currentVolume < 0) currentVolume = 0; 
      oledPrint("Vol DOWN: " + String(currentVolume));
      btn4UsedAsModifier = true; 
    } 
    else if (btnState[0] == LOW) { // [Added] BTN1 is pressed (Gain Down)
      if (currentGain >= 25) currentGain -= 25;
      else currentGain = 0;
      PDM.setGain(currentGain); // Apply gain immediately
      oledPrint("Gain DOWN: " + String(currentGain));
      btn1UsedAsModifier = true; // Mark as modifier usage
    }
    else { // Single press (Previous track)
      if (currentPlayIndex > 1) { 
        currentPlayIndex--;
        oledPrint("PREV: REC_" + String(currentPlayIndex));
        if (isPlaying) startPlayback(); 
      } else oledPrint("First file!");
    }
  }

  // [ BTN3: Next / Vol Up / Gain Up ]
  if (btnPressed[3]) {
    if (btnState[1] == LOW) { // BTN4 is pressed (Volume Up)
      if (currentVolume < 5) currentVolume += 1;
      else if (currentVolume < MAX_VOLUME) currentVolume += 5;
      if (currentVolume > MAX_VOLUME) currentVolume = MAX_VOLUME; 
      oledPrint("Vol UP: " + String(currentVolume));
      btn4UsedAsModifier = true; 
    } 
    else if (btnState[0] == LOW) { // [Added] BTN1 is pressed (Gain Up)
      if (currentGain <= 225) currentGain += 25;
      else currentGain = 250;
      PDM.setGain(currentGain); // Apply gain immediately
      oledPrint("Gain UP: " + String(currentGain));
      btn1UsedAsModifier = true; // Mark as modifier usage
    }
    else { // Single press (Next track)
      if (currentPlayIndex < maxFileIndex) { 
        currentPlayIndex++;
        oledPrint("NEXT: REC_" + String(currentPlayIndex));
        if (isPlaying) startPlayback(); 
      } else oledPrint("Last file!");
    }
  }

  // [ BTN4: Play / Stop (Release Trigger) ]
  if (btnReleased[1]) {
    if (!btn4UsedAsModifier) { 
      if (!isRecording) { 
        if (isPlaying) stopPlayback();
        else if (maxFileIndex > 0) startPlayback(); 
      }
    }
    btn4UsedAsModifier = false; 
  }

  // --- 2. Audio streaming processing (Recording) ---
  if (isRecording && samplesRead && logFile) {
    digitalWrite(D0, HIGH);
    digitalWrite(MY_LED_GREEN, LOW); 
    short temp;
    while(fifo.lockedPop(temp)) {               
      logFile.write(byte(temp & 0x00FF)); 
      logFile.write(byte((temp >> 8) & 0xFF));     
      wavDataSize += 2; 
    }
    logFile.flush();             
    digitalWrite(MY_LED_GREEN, HIGH);
    digitalWrite(D0, LOW);       
    samplesRead = 0; 
  }

  // --- 3. Audio streaming processing (Playback) ---
  if (isPlaying && !isRecording && playFile) {
    int bytesRead = playFile.read((uint8_t*)monoPlayBuf, sizeof(monoPlayBuf));
    if (bytesRead > 0) {
      int numSamples = bytesRead / 2; 
      for (int i = 0; i < numSamples; i++) {
        int32_t scaledSample = ((int32_t)monoPlayBuf[i] * currentVolume) / BASE_VOLUME;
        if (scaledSample > 32767) scaledSample = 32767;
        else if (scaledSample < -32768) scaledSample = -32768;
        stereoPlayBuf[playBufIdx][i*2]     = (int16_t)scaledSample; 
        stereoPlayBuf[playBufIdx][i*2 + 1] = (int16_t)scaledSample; 
      }
      NRF_I2S->TXD.PTR = (uint32_t)stereoPlayBuf[playBufIdx];
      NRF_I2S->RXTXD.MAXCNT = numSamples; 
      if (!isI2SStarted) {
        NRF_I2S->EVENTS_TXPTRUPD = 0;
        NRF_I2S->TASKS_START = 1;
        isI2SStarted = true;
      }
      while (NRF_I2S->EVENTS_TXPTRUPD == 0) { }
      NRF_I2S->EVENTS_TXPTRUPD = 0; 
      playBufIdx = 1 - playBufIdx; 
    } else stopPlayback();
  }
}

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ Helper Functions

void initNativeI2S() {
  NRF_I2S->PSEL.MCK   = 0xFFFFFFFF; 
  NRF_I2S->PSEL.SCK   = I2S_BCLK_PIN;
  NRF_I2S->PSEL.LRCK  = I2S_LRC_PIN;
  NRF_I2S->PSEL.SDOUT = I2S_DIN_PIN;
  NRF_I2S->PSEL.SDIN  = 0xFFFFFFFF; 
  NRF_I2S->CONFIG.MODE     = I2S_CONFIG_MODE_MODE_Master;
  NRF_I2S->CONFIG.FORMAT   = I2S_CONFIG_FORMAT_FORMAT_I2S;
  NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Stereo;
  NRF_I2S->CONFIG.SWIDTH   = I2S_CONFIG_SWIDTH_SWIDTH_16Bit;
  NRF_I2S->CONFIG.MCKFREQ  = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV63; 
  NRF_I2S->CONFIG.RATIO    = I2S_CONFIG_RATIO_RATIO_32X;
  NRF_I2S->CONFIG.TXEN     = 1;
  NRF_I2S->CONFIG.RXEN     = 0;
}

void startPlayback() {
  if (playFile) playFile.close(); 
  char filename[20];
  sprintf(filename, "REC_%03d.WAV", currentPlayIndex);
  playFile = SD.open(filename, FILE_READ);
  if (playFile) {
    playFile.seek(44); 
    isPlaying = true;
    isI2SStarted = false;
    playBufIdx = 0;
    NRF_I2S->ENABLE = 1; 
    digitalWrite(I2S_SD_MODE, HIGH); 
    oledPrint("PLAY START: " + String(filename));
  } else {
    oledPrint("Play File Error!");
    isPlaying = false;
  }
}

void stopPlayback() {
  isPlaying = false;
  if (playFile) playFile.close();
  if (isI2SStarted) {
    memset(stereoPlayBuf[0], 0, sizeof(stereoPlayBuf[0]));
    memset(stereoPlayBuf[1], 0, sizeof(stereoPlayBuf[1]));
    for (int i = 0; i < 30; i++) {
      NRF_I2S->TXD.PTR = (uint32_t)stereoPlayBuf[playBufIdx];
      NRF_I2S->RXTXD.MAXCNT = PLAY_CHUNK_SIZE;
      while (NRF_I2S->EVENTS_TXPTRUPD == 0) {} 
      NRF_I2S->EVENTS_TXPTRUPD = 0;
      playBufIdx = 1 - playBufIdx;
    }
    digitalWrite(I2S_SD_MODE, LOW);
    delay(50); 
    NRF_I2S->EVENTS_STOPPED = 0;
    NRF_I2S->TASKS_STOP = 1;
    while(NRF_I2S->EVENTS_STOPPED == 0) {}
    isI2SStarted = false;
  }
  NRF_I2S->ENABLE = 0;
  digitalWrite(I2S_SD_MODE, LOW); 
  oledPrint("PLAY STOP");
}

void onPDMdata() {
  digitalWrite(D1, HIGH);
  digitalWrite(MY_LED_BLUE, LOW);
  int bytesAvailable = PDM.available();     
  PDM.read(sampleBuffer, bytesAvailable);   
  samplesRead = bytesAvailable / 2;         
  if (isRecording) { 
    for(int i = 0; i < samplesRead; i++) {    
      if(!fifo.lockedPush(sampleBuffer[i])) {
        errorBlink(5);
      }
    }
  }
  digitalWrite(MY_LED_BLUE, HIGH);
  digitalWrite(D1, LOW);                    
}

void writeWavHeader(File& file, uint32_t dataSize) {
  byte header[44];
  uint32_t fileSize = dataSize + 36;
  uint32_t sampleRate = 16000;
  uint16_t numChannels = 1;      
  uint16_t bitsPerSample = 16;
  uint32_t byteRate = sampleRate * numChannels * (bitsPerSample / 8);
  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  header[4] = (byte)(fileSize & 0xFF); header[5] = (byte)((fileSize >> 8) & 0xFF);
  header[6] = (byte)((fileSize >> 16) & 0xFF); header[7] = (byte)((fileSize >> 24) & 0xFF);
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0; 
  header[20] = 1; header[21] = 0; 
  header[22] = (byte)numChannels; header[23] = 0;
  header[24] = (byte)(sampleRate & 0xFF); header[25] = (byte)((sampleRate >> 8) & 0xFF);
  header[26] = (byte)((sampleRate >> 16) & 0xFF); header[27] = (byte)((sampleRate >> 24) & 0xFF);
  header[28] = (byte)(byteRate & 0xFF); header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF); header[31] = (byte)((byteRate >> 24) & 0xFF);
  header[32] = (byte)(numChannels * bitsPerSample / 8); header[33] = 0; 
  header[34] = (byte)bitsPerSample; header[35] = 0;
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  header[40] = (byte)(dataSize & 0xFF); header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF); header[43] = (byte)((dataSize >> 24) & 0xFF);
  file.write(header, 44);
}

void errorBlink(uint8_t err) {
  Serial.println(err);
  while(true) {      
    digitalWrite(MY_LED_GREEN, HIGH);
    digitalWrite(MY_LED_BLUE, HIGH);
    for(int i = 0; i < err; i++) {
      digitalWrite(MY_LED_RED, LOW); delay(5);
      digitalWrite(MY_LED_RED, HIGH); delay(200);
    }
    delay(500);
  }
}