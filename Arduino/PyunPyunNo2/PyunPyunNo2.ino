/******************************************************
 * Arduino Pyun Pyun Machine Rev.2
 * 
 * CS   10
 * MOSI 11
 * SCK  13
 * 
 * Pot Freq      A0
 * Pot LFO Freq  A1
 * Pot LFO Depth A2
 * Button Wave Form A3 (D17)
 * Button LFO Wave  A4 (D18)
 * 
 * LCD RS pin to digital pin 7
 * LCD Enable pin to digital pin 6
 * LCD D4 pin to digital pin 2
 * LCD D5 pin to digital pin 3
 * LCD D6 pin to digital pin 4
 * LCD D7 pin to digital pin 5
 * LCD R/W pin to ground
 * 10K resistor:
 *   ends to +5V and ground
 *   wiper to LCD VO pin (pin 3)
 * 
 * 2014.01.15 by gizmo
 * 2014.01.15 DAC出力
 * 2014.01.27 WavForm選択をボタンに
 * 2014.01.27 Debounce
 * 2014.02.01 LFO表示を3桁に
 * 2014.08.25 Rev.2
 * 
 ********************************************************/

#include <stdio.h>
#include <stdint.h>

#include <SPI.h>
#include <MCPDAC.h>
#include <LiquidCrystal.h>
#include <Bounce2.h>
#include <avr/pgmspace.h>

#include "WaveTable.h"

#define SERIAL_SPEED 57600
#define DEBOUNCE_INTERVAL 100

#define DAC_CS_PIN		10
#define WAVE_FORM_PIN		17
#define LFO_FORM_PIN		18

#define WAVE_FREQUENCY_PIN	0	// A0
#define LFO_FREQUENCY_PIN	1	// A1
#define LFO_AMOUNT_PIN		2	// A2

#define WAVEFORM_NUM 5
#define DEBOUNCE_INTERVAL 10

#define SAMPLE_CLOCK   15625.0
#define LFO_CLOCK      3125.0
#define WAVETABLE_SIZE 1024

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

const char *waveFormStr[] = { 
  "SIN", "TRI", "SW1", "SW2", "SQR" };
uint16_t *waveForms[WAVEFORM_NUM];

// parameters
double waveFrequency = 1000.0;
double lfoFrequency  = 5.0;
uint8_t lfoAmount    = 0; 

int waveForm = 0;
int lfoForm  = 0;

LiquidCrystal lcd(7, 6, 2, 3, 4, 5);
Bounce bouncerWaveForm = Bounce();
Bounce bouncerLfoForm = Bounce();

volatile uint16_t phaseRegister;
volatile uint16_t tuningWord;

volatile uint16_t lfoPhaseRegister;
volatile uint16_t lfoTuningWord;

volatile uint16_t tick = 0;

void setup()
{
  // PINアサインの初期化
  pinMode(WAVE_FORM_PIN, INPUT_PULLUP);
  pinMode(LFO_FORM_PIN, INPUT_PULLUP);

  // Classの初期化
  Serial.begin(SERIAL_SPEED);
  
  lcd.begin(16, 2);

  MCPDAC.begin(DAC_CS_PIN);
  MCPDAC.setGain(CHANNEL_A,GAIN_LOW);
  MCPDAC.shutdown(CHANNEL_A,false);
  MCPDAC.shutdown(CHANNEL_B,true);

  bouncerWaveForm.attach(WAVE_FORM_PIN);
  bouncerWaveForm.interval(DEBOUNCE_INTERVAL);
  bouncerLfoForm.attach(LFO_FORM_PIN);
  bouncerLfoForm.interval(DEBOUNCE_INTERVAL);

  // Timerの初期化
  SetupTimer2();
  sbi(TIMSK2, TOIE2);

  // 変数の初期化
  lfoTuningWord = lfoFrequency * pow(2.0, 16) / LFO_CLOCK;
  tuningWord = waveFrequency * pow(2.0, 16) / SAMPLE_CLOCK;
 
  phaseRegister = 0;
  lfoPhaseRegister = 0;

  // WaveTableの設定
  waveForms[0] = waveTableSine;
  waveForms[1] = waveTableTriangle;
  waveForms[2] = waveTableSawtoothUp;
  waveForms[3] = waveTableSawtoothDown;
  waveForms[4] = waveTableSqure;

  sei();
}

void loop()
{
  char buff[20];

  // デバイスの読み取り
  waveFrequency = (double)analogRead(WAVE_FREQUENCY_PIN);		// 0.0..1023.0Hz
  lfoFrequency  = (double)analogRead(LFO_FREQUENCY_PIN) / 64 + 0.1;	// 0.1..16.0Hz
  lfoAmount     = map(analogRead(LFO_AMOUNT_PIN), 0, 1023, 0, 255);

  if (bouncerWaveForm.update()) {
    waveForm += !bouncerWaveForm.read();
    if (waveForm >= WAVEFORM_NUM)
      waveForm = 0;
  }
  if (bouncerLfoForm.update()) {
    lfoForm += !bouncerLfoForm.read();
    if (lfoForm >= WAVEFORM_NUM)
      lfoForm = 0;
  }

  // 変数の更新
  tuningWord = waveFrequency * pow(2.0, 16) / SAMPLE_CLOCK;
  lfoTuningWord = lfoFrequency * pow(2.0, 16) / LFO_CLOCK;

  // LCDの表示の初期化
  sprintf(buff, "FREQ LFO DPT %s",  waveFormStr[waveForm]);
  lcd.setCursor(0, 0);
  lcd.print(buff);
  sprintf(buff, "%4d %3d %3d %s", (int)waveFrequency, (int)(lfoFrequency * 10), lfoAmount, waveFormStr[lfoForm]);
  lcd.setCursor(0, 1);
  lcd.print(buff); 
}

//******************************************************************
// timer2 setup
// set prscaler to 8, PWM mode to phase correct PWM,  16000000 / 8 / 128 = 15625 Hz clock
void SetupTimer2()
{
  // Timer2 PWM Mode set to Phase Correct PWM
  /*
	cbi (TCCR2A, COM2A0);  
   	cbi (TCCR2A, COM2A1);
   	*/

  sbi (TCCR2A, WGM20);  // Mode 7 / Fast PWM
  sbi (TCCR2A, WGM21);
  sbi (TCCR2B, WGM22);

  OCR2A = 127;

  // Timer2 Clock Prescaler to : 8
  cbi (TCCR2B, CS20);
  sbi (TCCR2B, CS21);
  cbi (TCCR2B, CS22);
}

//******************************************************************
// Timer2 Interrupt Service at 15,625Hz = 64uSec
ISR(TIMER2_OVF_vect) {
  uint16_t lfoIndex;
  int16_t lfoValue;
  uint16_t index;
  uint16_t waveValue;

  tick++;

  // Caluclate LFO Value

  if (tick > SAMPLE_CLOCK / LFO_CLOCK) {
    // ↑なぜか右辺が浮動し小数点型でないと動かない
    
    tick = 0;

    lfoPhaseRegister += lfoTuningWord;

    // 16bitのlfoPhaseRegisterをテーブルの10bit(1024個)に丸める
    lfoIndex = lfoPhaseRegister >> 6;

    // lookupTable(12bit) * lfoAmount(8bit) : 20bit -> 16bit
    lfoValue = (((int32_t)pgm_read_word(waveForms[lfoForm] + lfoIndex)) - 2048) * lfoAmount >> 4;

    /* 
     * シフト可能なビット数は
     * tuningWordの最大値とのからみかも？
     */
    lfoValue = ((int32_t)tuningWord * lfoValue) >> 12;
  }

  // Caluclate Wave Value
  phaseRegister += tuningWord + lfoValue;

  // 16bitのphaseRegisterをテーブルの10bit(1024個)に丸める
  index = phaseRegister >> 6;

  waveValue = pgm_read_word(waveForms[waveForm] + index);

  //Serial.println(waveValue);
  // DACに出力
  MCPDAC.setVoltage(CHANNEL_A, waveValue);
}

