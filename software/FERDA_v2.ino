/*

  This Jindras' code
  
  Distributed under CC BY-NC-SA 4.0 license
  Attribution-NonCommercial-ShareAlike 4.0 International
  License details:
  https://creativecommons.org/licenses/by-nc-sa/4.0/legalcode.en


*/

#include "ch32v20x_opa.h"
#include <EEPROM.h>

#ifdef  TIM_MODULE_ENABLED
#include <HardwareTimer.h>                                                    // Include HardwareTimer for compatibility
HardwareTimer myTimer(TIM2);                                                  // define object, to present we occupy Timer 2
#else
 error, not defined yet
#endif

#define NMRA_ID 13
#define LN_DEVELOPER_ID 27

#define LN_Rx_PIN PA3
#define LN_Tx_PIN PA8

#define LED_RED PB5
#define LED_GRE_L PB4
#define LED_GRE_R PB3


// --- EEPROM Section
#define  EEPROM_ID1   0   // LocoNet Throttle ID - bit 7 mean, it is invalid and must be configured
#define  EEPROM_ID2   1
#define  EEPROM_CFG   2   // Throttle configuration - FF = not configured; run self test
#define  EEPROM_ADR1  3   // loco address high byte. Address 00:00 mean no address, bit 7 mean no address
#define  EEPROM_ADR2  4   // loco address low byte. Address 00:00 mean no address, bit 7 mean no address
#define  EEPROM_SS    5   // Loco speed steps - low 3 bits from SLOT STATUS1; bit 7 mean no address
#define  EEPROM_UID0  6   // processor UID[0] - for validation
#define  EEPROM_UID1  7   // processor UID[1] - for validation
#define  EEPROM_UID2  8   // processor UID[2] - for validation
#define  EEPROM_UID3  9   // processor UID[3] - for validation

/* EEPROM_SS bits
3 BITS for Decoder TYPE encoding for this SLOT
011=send 128 speed mode packets
010=14 step MODE
001=28 step. Generate Trinary packets for this Mobile ADR
000=28 step/ 3 BYTE PKT regular mode
111=128 Step decoder, Allow Advanced DCC consisting
100=28 Step decoder ,Allow Advanced DCC consisting
*/

/* EEPROM_CFG bits
 - FF not configured = do self test
 - 5x self test passed
 BIT0+BIT1
 - x0 4:1 encoder (24/24 steps for example) default
 - x1 2:1 encoder (15/30 steps for example)
 - x3 1:1 encoder (10/40 steps)
 BIT2
 - x4 Encoder can reverse (not implemented yet)
 BIT3
 - x8 1=analog, 0=incremental */

/*Keyboard / pin mapping
PA0 = power sense (wkup)
PA1 = Potentiometer (analog)
PA2 = F1/F9
PA3 = LN-Rx
PA4 = F0
PA5 = F2/F10
PA6 = LN-Vref
PA7 = F3/F11
PA8 = LN-Tx
PA9 = F4/F12
PA10 = F8/F16
PA11 = F7/F15
PA12 = F6/F14
PA13 = SWDIO
PA14 = SWDCLK
PA15 = F5/F13
PB0 = LN-in
PB1 = Shift
PB3 = LED green
PB4 = LED green
PB5 = LED red
PB6 = Dir sw
PB7 = ESTOP
*/

#define CDBackoff 20+6+1			// backoff delay consist of CD backoff + priority delay

uint8_t LNTxBuffer[16];  // maximum to transmitt is 16 characters
uint8_t LNRxBuffer[32];  // maximum to receive is 32 byte long packet
volatile uint8_t LNTxBufferL;     // length of buffer, also used as rediness indicator
uint8_t LNSTATE;         // software serial state machine
uint8_t LNCOUNT;         // Counter for software serial
uint8_t LNBITCNT;        // bit counter in byte
uint8_t LNBYTE;          // actually transmitted byte
uint8_t RXOPCODE;        // Last received opcode
uint8_t RX_XOR;          // Calculation of XOR during receiving
uint8_t RX_PTR;
uint8_t RXBYTECNT;       // length of packet to be received
bool LNRxCheck;          // Flag for checking received byte
bool LN_RCVING;          // Flag, that receiver is active and receiving this packet
bool NewPacket;          // Flag, that new packet is succesfully received

uint8_t MainState;       // main state machine. States: 0=Self test; 1=Wait address; 2=Normal run; 3=Sleep;
uint8_t SubMenu;         // Sub menu in Main state
// 0 = send loco address request -> waiting slot (fast rotate leds)
// 1 = slot data received -> validate IDs in slot (00 = goto 2.2, same = goto 2.3, different = goto 2.E)
// 2 = update slot IDs
// 3 = check slot is active - if not, then submit null move
// 4 = check speed steps (same = skip, differ update slot)
// 5 = request F9-F16
// 6 = operation
// E = Warning state F0 = goto 2.2, Stop = 1.0
// F = Error state - Shift+Stop=goto 0.0


unsigned long LastBlinkMillis;   // last timestamp for blinking
uint8_t BlinkState;      // blinking state machine
uint16_t BlinkSpeed;     // blinking speed - in millis per change
uint32_t LongRefreshMillis;  // timer for long refresh
uint32_t TimeOutMillis;  // timer for timeout commands

uint32_t LastKeyPotMillis;  // keyboard refresh timer
uint8_t PotDelay = 0;       // delay after pot read to next

uint32_t LastGPIO;       // last GPIO state GPIOA->INDR
int32_t LastPot;        // last potentiometer value
uint8_t WaitKeyRelease;     // some key pressed, waiting for release (values <= 16 mean key number)
uint16_t SelfTestMask = 0;  // mask for self test - each bit is one peripheral 

bool WaitSlot;            // Indicate I'm waiting for some slot data
bool LNvU;                // Command station supports Uhlenbrock commands

uint8_t LOC_SLOT;         // SLOT#
//uint8_t LOC_AL;           // Address Low
//uint8_t LOC_AH;           // Address High
uint8_t LOC_SPEED;        // Actual speed
uint8_t LOC_DIRF;         // Dir & F0 to F4
uint8_t LOC_SND;          // F5 to F8
uint8_t LOC_F9F16;        // F9 to F16
uint8_t LOC_STAT;         // slot status
uint8_t TRK;              // command station status
uint8_t LOC_ID1;          // ID of received slot
uint8_t LOC_ID2;          // ID of received slot


uint8_t MY_AL;           // Address Low
uint8_t MY_AH;           // Address High
uint8_t MY_SS;           // Speed Steps
uint8_t MY_ID1;          // Device ID
uint8_t MY_ID2;          // Device ID

void OPA1_Init( void )
{
  // PA6 - OPA1_CH1N
  // PB0 - OPA1_CH1P
  // PA3 - OPA1_OUT0 (USART2_RX)
  OPA_InitTypeDef  OPA_InitStructure = {0};

  pinMode(PB0, INPUT_ANALOG);
  pinMode(PA6, INPUT_ANALOG);
  pin_function(PA3, CH_PIN_DATA(CH_MODE_OUTPUT_50MHz, CH_CNF_OUTPUT_AFPP, NOPULL, AFIO_NONE));

  OPA_InitStructure.OPA_NUM = OPA1;
  OPA_InitStructure.PSEL = CHP1;
  OPA_InitStructure.NSEL = CHN1;
  OPA_InitStructure.Mode = OUT_IO_OUT0;
  OPA_Init( &OPA_InitStructure );
  OPA_Cmd( OPA1, ENABLE );

}

void TIM2_Init( void )
{
    myTimer.setOverflow(60, MICROSEC_FORMAT); // 60 microsecond reset rate used for software serial Tx time ticks
    myTimer.attachInterrupt(LNTxHandler);     //Interrupt 
    TIM_Cmd( TIM2, ENABLE );

    TIM_ClearITPendingBit( TIM2, TIM_IT_Update );   // clear potential interrupt flag from the past

    NVIC_EnableIRQ(TIM2_IRQn);                   // enable Timer 1 update unterrupt on controller

    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);      // enable timer updating event in timer config
}

void UART_Init( void )
{
    USART_InitTypeDef USART_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    USART_InitStructure.USART_BaudRate = 16666;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx;

    USART_Init(USART2, &USART_InitStructure);
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    NVIC_EnableIRQ(USART2_IRQn);                   // enable Uart 2 update interrupt on controller
    USART_Cmd(USART2, ENABLE);

}


#ifdef __cplusplus
extern "C" {
#endif

void USART2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void USART2_IRQHandler(void)
{
  uint8_t Received; // received data
  if (USART2->STATR & USART_STATR_FE) { // freme error -> break
    Received = USART2->DATAR;
    if (LNSTATE < 5) {             // is it my break??
      digitalWrite(LN_Tx_PIN,LOW);
      LNSTATE=0;                    // no -> reset transmitter
      LNCOUNT = CDBackoff;
    }
  } else {
    if (USART2->STATR & USART_STATR_RXNE) { // read data register not empty = have data
      if (LNRxCheck) { // compare received
        LNRxCheck = 0;
        LN_RCVING = 0; // not receiving this packet (when sending...)
        Received = USART2->DATAR;     // take data
        if (LNBYTE != Received) {     // are they same ??
          LNCOUNT = 15;               // no, send break
          LNSTATE = 5;
          digitalWrite(LN_Tx_PIN,HIGH);
        }
      } else {      // normal receive
        Received = USART2->DATAR;     // take data
        LNSTATE = 0; // received byte = reset state
        LNCOUNT = CDBackoff;
        if (Received & 0x80) {        // is it opcode??
          // 100F DCBA		Message is 2 bytes, including Checksum
          // 101F DCBA		Message is 4 bytes, including Checksum
          // 110F DCBA		Message is 6 bytes, including Checksum
          // 111F DCBA		Message is N bytes, next bite is length
          LN_RCVING = 1;  // receiving this byte
          RXOPCODE = Received;  // store it as opcode
          RX_XOR = Received;    // and use for XOR
          RX_PTR = Received >> 5;   // use RX_PTR for calculation
          RX_PTR &= 0x03;
          switch (RX_PTR) {
            case 0:             // 100F DCBA		Message is 2 bytes, including Checksum
              RXBYTECNT = 1;    // 2 byte packet, -1 as opcode is already received
              break;
            case 1:             // 101F DCBA		Message is 4 bytes, including Checksum
              RXBYTECNT = 3;    // 4 byte packet, -1 as opcode is already received
              break;
            case 2:             // 110F DCBA		Message is 6 bytes, including Checksum
              RXBYTECNT = 5;    // 6 byte packet, -1 as opcode is already received
              break;
            case 3:             // 111F DCBA		Message is N bytes, next bite is length
              RXBYTECNT = 0xFF; // 4 byte packet, -1 as opcode is already received
              break;
          }
          RX_PTR = 0;           // pointer to buffer
        } else {                // no opcode
          if (LN_RCVING) {      // Am I receiving this packet?
            LNRxBuffer[RX_PTR++] = Received;    // store to buffer

            RX_XOR ^= Received; // add to XOR
            if (RXBYTECNT & 0x80) { // is it N for packet length?
              RXBYTECNT = Received & 0x1F; // understand only up to 14 bytes packet
              if (RXBYTECNT < 4) {LN_RCVING = 0; RXBYTECNT=3;} // minimum 4 bytes, othervise not understand
              RXBYTECNT--;   // minus opcode
            }
            if (!(--RXBYTECNT)) {  // last byte?
              if (RX_XOR == 0xFF) {NewPacket=1;}   // Valid XOR, valid packet
              LN_RCVING = 0;    // not receiving anymore
            }
          }
        }
      }
    }
  }
}

#ifdef __cplusplus
}
#endif


void LNTxHandler(void)
{
  TIM_ClearITPendingBit( TIM2, TIM_IT_Update );   // reset interrupt flag
  switch (LNSTATE) {
    case 0: // LNCDbackoff	; 0
      // wait for end of CD back off
      if (!(--LNCOUNT)) {LNSTATE++;}
      break;
    case 1: // LNWaitData		; 1
      // wait for data in buffer to send
      if (LNTxBufferL) {  // any data to send?
        LNCOUNT = 0; // point to first byte in message
        LNSTATE++;
      } else {break;}
      // break; // immediate continue to next state
    case 2: // LNStartBit		; 2
      if ((USART1->STATR & 0x0400) || (digitalRead(LN_Rx_PIN) == LOW)) { // Receiver busy??
        LNSTATE = 0; // reset state
        LNCOUNT = CDBackoff;
      } else {
        digitalWrite(LN_Tx_PIN,HIGH); // startbit is here
        LNBITCNT=8; // 8 bits per byte
        LNBYTE=LNTxBuffer[LNCOUNT];
        LNSTATE++;
      }
      break;
    case 3: // IntTxBit		; 3     Transmitt one bit from byte
      ((LNBYTE & 1) ? digitalWrite(LN_Tx_PIN,LOW) : digitalWrite(LN_Tx_PIN,HIGH));  // set the bit
      if (LNBYTE & 1) {
        delayMicroseconds(16);
        if (digitalRead(LN_Rx_PIN) == LOW) {
          digitalWrite(LN_Tx_PIN,HIGH); // start with BREAK signal
		      LNCOUNT=15; // for 15 ticks
		      LNSTATE=5;  // in state 5
        }
      }
      LNBYTE = LNBYTE >> 1;
      if (!(--LNBITCNT)) {    // last bit??
        LNBYTE=LNTxBuffer[LNCOUNT];   // restore byte we expect
        LNRxCheck = 1;                // set flag, we are waiting
        LNSTATE++;
      }
      break;
    case 4: // LNCheckRx		; 4 // wait for received byte (include stop bit)(goto 2 or 0)
      digitalWrite(LN_Tx_PIN,LOW);
	    if (!(LNRxCheck)) {
        LNSTATE=2;              // next byte
        LNCOUNT++;
        if (LNCOUNT == LNTxBufferL) { // is it last byte?
          LNTxBufferL=0;              // indicate empty buffer
          LNSTATE=0;                    // and reset transmitter
          LNCOUNT = CDBackoff;

        }
      }
      break;
    case 5: // LNBreak		; 5
      LNCOUNT--;
      if (LNCOUNT == 0) {
        digitalWrite(LN_Tx_PIN,LOW);
        LNSTATE = 0; // reset state
        LNCOUNT = CDBackoff;
		    TIM2->CNT = random(TIM2->ATRLR);
      }
      break;
  }
}

void Keyboard_Init() {
  // PA0 = power sense (wkup)
  pinMode(PA2, INPUT_ANALOG); // PA1 = Potentiometer (analog)
  pinMode(PA2, INPUT_PULLUP); // PA2 = F1/F9
  // PA3 = LN-Rx
  pinMode(PA4, INPUT_PULLUP); // PA4 = F0
  pinMode(PA5, INPUT_PULLUP); // PA5 = F2/F10
  // PA6 = LN-Vref
  pinMode(PA7, INPUT_PULLUP); // PA7 = F3/F11
  // PA8 = LN-Tx
  pinMode(PA9, INPUT_PULLUP); // PA9 = F4/F12
  pinMode(PA10, INPUT_PULLUP); // PA10 = F8/F16
  pinMode(PA11, INPUT_PULLUP); // PA11 = F7/F15
  pinMode(PA12, INPUT_PULLUP); // PA12 = F6/F14
  // PA13 = SWDIO
  // PA14 = SWDCLK
  pinMode(PA15, INPUT_PULLUP); // PA15 = F5/F13
  //PB0 = LN-in
  pinMode(PB1, INPUT_PULLUP); // PB1 = Shift
  // PB2
  // PB3 = LED green
  // PB4 = LED green
  // PB5 = LED red
  pinMode(PB6, INPUT_PULLUP); // PB6 = Dir sw
  pinMode(PB7, INPUT_PULLUP); // PB7 = ESTOP

  LastGPIO = ((GPIOB->INDR) << 16) | GPIOA->INDR;
  LastGPIO = 0xFFFFFFFF ^ LastGPIO;
  LastGPIO = LastGPIO & 0xC29EB4;         // mask for buttons and dir switch
  if (LastGPIO & 0x829EB4) {WaitKeyRelease = 0xFF;} // mask for buttons not dir switch

}

// the setup function runs once when you press reset or power the board
void setup() {
  unsigned long *UID1=(unsigned long *)0x1FFFF7E8;
  uint8_t *UIDb=(uint8_t *)0x1FFFF7E8;
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GRE_L, OUTPUT);
  pinMode(LED_GRE_R, OUTPUT);
  pinMode(LN_Tx_PIN, OUTPUT);

  LNSTATE = 0; // reset state
  LNCOUNT = CDBackoff * 4;            // longer time to stabilize
  LNTxBufferL = 0;                    // nothing to send
  //delay(1000);                      // wait for a second
  LOC_SLOT = 0xFF;                    // no slot assigned

  UART_Init();
  OPA1_Init();
  EEPROM.begin();
  Keyboard_Init();
  randomSeed(UID1[0]);  // start random numbers 
  // Serial.begin(230400);
  // Check eeprom validity...
  if ((EEPROM.read(EEPROM_UID0) != UIDb[0]) || (EEPROM.read(EEPROM_UID1) != UIDb[1]) || (EEPROM.read(EEPROM_UID2) != UIDb[2]) || (EEPROM.read(EEPROM_UID3) != UIDb[3])) {

    EEPROM.write(EEPROM_ID1,0x12);
    EEPROM.write(EEPROM_ID2,0x34);
    EEPROM.write(EEPROM_CFG,0xF0);   // Throttle configuration - FF = not configured; run self test
    EEPROM.write(EEPROM_ADR1,00);    // loco address high byte. Address 00:00 mean no address, bit 7 mean no address
    EEPROM.write(EEPROM_ADR2,00);    // loco address low byte. Address 00:00 mean no address, bit 7 mean no address
    EEPROM.write(EEPROM_SS,0xFF);    // Loco speed steps - low 3 bits from SLOT STATUS1; bit 7 mean no address

    EEPROM.write(EEPROM_UID0,UIDb[0]);
    EEPROM.write(EEPROM_UID1,UIDb[1]);
    EEPROM.write(EEPROM_UID2,UIDb[2]);
    EEPROM.write(EEPROM_UID3,UIDb[3]);
    EEPROM.commit();
  }

  MY_ID1=EEPROM.read(EEPROM_ID1);
  MY_ID2=EEPROM.read(EEPROM_ID2);
  //EEPROM.read(EEPROM_CFG);   // Throttle configuration - FF = not configured; run self test
  MY_AH=EEPROM.read(EEPROM_ADR1);    // loco address high byte. Address 00:00 mean no address, bit 7 mean no address
  MY_AL=EEPROM.read(EEPROM_ADR2);    // loco address low byte. Address 00:00 mean no address, bit 7 mean no address
  MY_SS=EEPROM.read(EEPROM_SS);    // Loco speed steps - low 3 bits from SLOT STATUS1; bit 7 mean no address

  MainState=2;       // main state machine. States: 0=Self test; 1=Wait address; 2=Normal run; 3=Sleep;
  if (((MY_AH == 00) && (MY_AL == 00)) || (MY_AH > 127) || (MY_AL > 127)) {MainState=1;} // no address found, wait for new
  if ((EEPROM.read(EEPROM_CFG) & 0xF0) != 0x50) {MainState=0;} // not configured yet - go to self test

  if (digitalRead(PB1) == LOW) {MainState=0;} // someone holding shift - go to self test

  digitalWrite(LED_RED, HIGH);    // all LEDs off
  digitalWrite(LED_GRE_L, HIGH);
  digitalWrite(LED_GRE_R, HIGH);

  while (digitalRead(LN_Rx_PIN) == LOW) {random(5);}  // wait for LocoNet pin activated and mix random numbers for fun
  TIM2_Init();

  if (MainState == 0) {
    LNTxBuffer[0] = 0xA8;       // OPC_FRED_SELFTEST - for self test => send self test start
    LNTxBuffer[1] = 0x2A;       // 2A Self test begin
    LNTxBuffer[2] = 0x16;       // 16 Self test begin
    LNTxBuffer[3] = 0xA8 ^ 0x2A ^ 0x16 ^ 0xFF;       // Checksum 
    LNTxBufferL = 4;            // 4 bytes to send
    BlinkSpeed = 120;           // fast blink
  } else {
    LNTxBuffer[0] = 0xBB;       // OPC_RQ_SL_DATA - for others => send read master slot
    LNTxBuffer[1] = 0x00;       // <SLOT> = 0 (Master slot)
    LNTxBuffer[2] = 0x00;       // <SLOTH> = 0 (0 requested for P1, same as for P2)
    LNTxBuffer[3] = 0xBB ^ 0xFF;       // Checksum 
    LNTxBufferL = 4;            // 4 bytes to send
  }
  if (MainState == 1) {
    digitalWrite(LED_RED, LOW);  // RED on when waiting address
  }
  LastBlinkMillis = millis();
  LastKeyPotMillis = millis();
  LongRefreshMillis = millis();
  
  pinMode(PA2, INPUT);

}

// the loop function runs over and over again forever
void loop() {
  switch (MainState) {
    case 0: // 0=Self test
      SelfTestLoop();
      break;
    case 1: // 1=Wait address
      WaitAddressLoop();
      break;
    case 2: // 2=Normal run
      NormalRunLoop();
      break;
    case 3: // 3=Sleep
      SleepLoop();
      break;
  }
}

void PinUp() {
  //digitalWrite(LED_RED, HIGH);
}

void SleepLoop() {
  // enable intrrupt for RA0 first !!

  TIM_Cmd( TIM2, DISABLE );       // Disable hardware not used in sleep mode -> Timer 2
  OPA_Cmd( OPA1, DISABLE );       // Disable hardware not used in sleep mode -> Comparator
  USART_Cmd(USART2, DISABLE);     // Disable hardware not used in sleep mode -> Usart

  attachInterrupt(PA0, GPIO_Mode_IN_FLOATING, PinUp, EXTI_Mode_Interrupt, EXTI_Trigger_Rising);   // attach interrupt for wake up pin (Arduino does not support events)

  PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);   // This is main sleep routine. It will STOP processor until wake up interrupt will appear

  detachInterrupt(PA0);           // welcome back. Interrupt is no necessary anymore
  SystemInit();                   // WARNING - part of sleep is change clock from PLL to HSI -> 8MHz. Restore clock back!!

  OPA_Cmd( OPA1, ENABLE );        // Restore hardware disabled in sleep mode -> Comparator
  USART_Cmd(USART2, ENABLE);      // Restore hardware disabled in sleep mode -> Usart
  TIM_Cmd( TIM2, ENABLE );        // Restore hardware disabled in sleep mode -> Timer 2


  // When restored, and SubMenu=6, return, otherwise restart
  if (digitalRead(PA0) == HIGH) {
    delay(500); // small delay for stabilization
    if (digitalRead(PA0) == HIGH) {
      if ((SubMenu == 6) && (digitalRead(PB1) == HIGH) && (digitalRead(PB7) == HIGH)) { // Shift = go to self test, EStop = restart at any time
          MainState = 2;
          LNSTATE = 0; // reset state; reset all prepared to sent....
          LNCOUNT = CDBackoff;
          LNTxBufferL = 0;                    // nothing to send
        } else {   
        LNSTATE = 0; // reset state
        LNCOUNT = CDBackoff;
        LNTxBufferL = 0;                    // nothing to send
        SubMenu = 0;
        MainState=2;       // main state machine. States: 0=Self test; 1=Wait address; 2=Normal run; 3=Sleep;
        if (((MY_AH == 00) && (MY_AL == 00)) || (MY_AH > 127) || (MY_AL > 127)) {MainState=1;} // no address found, wait for new
        if ((EEPROM.read(EEPROM_CFG) & 0xF0) != 0x50) {MainState=0;} // not configured yet - go to self test

        if (digitalRead(PB1) == LOW) {MainState=0;} // someone holding shift - go to self test

        digitalWrite(LED_RED, HIGH);    // all LEDs off
        digitalWrite(LED_GRE_L, HIGH);
        digitalWrite(LED_GRE_R, HIGH);

        if (MainState == 0) {
          LNTxBuffer[0] = 0xA8;       // OPC_FRED_SELFTEST - for self test => send self test start
          LNTxBuffer[1] = 0x2A;       // 2A Self test begin
          LNTxBuffer[2] = 0x16;       // 16 Self test begin
          LNTxBuffer[3] = 0xA8 ^ 0x2A ^ 0x16 ^ 0xFF;       // Checksum 
          LNTxBufferL = 4;            // 4 bytes to send
          BlinkSpeed = 120;           // fast blink
        } else {
          LNTxBuffer[0] = 0xBB;       // OPC_RQ_SL_DATA - for others => send read master slot
          LNTxBuffer[1] = 0x00;       // <SLOT> = 0 (Master slot)
          LNTxBuffer[2] = 0x00;       // <SLOTH> = 0 (0 requested for P1, same as for P2)
          LNTxBuffer[3] = 0xBB ^ 0xFF;       // Checksum 
          LNTxBufferL = 4;            // 4 bytes to send
        }
        if (MainState == 1) {
          digitalWrite(LED_RED, LOW);  // RED on when waiting address
        }
        LastBlinkMillis = millis();
        LastKeyPotMillis = millis();
        LongRefreshMillis = millis();
      }
    }
  }
}

void NormalRunLoop() {
// 0 = send loco address request -> waiting slot (fast rotate leds)
// 1 = wait slot data received -> validate IDs in slot (00 = goto 2.2, same = goto 2.3, different = goto 2.E)
// 2 = update slot IDs
// 3 = check slot is active - if not, then submit null move
// 4 = check speed steps (same = skip, differ update slot)
// 5 = request F9-F16
// 6 = operation
// E = Warning state F0 = goto 2.2, Stop = 1.0
// F = Error state - Shift+Stop=goto 0.0

  // --------------- Imediate function (for example send command, set led rotation etc) ---------
  switch (SubMenu) {
    case 0: // 0 = send loco address request -> waiting slot (fast rotate leds)
      BlinkSpeed = 120;   // fast blink
      if (!(LNTxBufferL)) {
        LNTxBuffer[0] = 0xBF;       // OPC_LOCO_ADR 0xBF
        LNTxBuffer[1] = MY_AH;      // Loco address high
        LNTxBuffer[2] = MY_AL;      // Loco address low
        LNTxBuffer[3] = 0xBF ^ 0xFF ^ LNTxBuffer[1] ^ LNTxBuffer[2];       // Checksum 
        LNTxBufferL = 4;            // 4 bytes to send
        TimeOutMillis = millis();   // prepare for timeout
        SubMenu++;
      }
      break;
    case 1: // 1 = wait slot data received
      if (millis() - TimeOutMillis >= 2000) {SubMenu = 0;} // 10 sec timeout for response
      BlinkSpeed = 120;   // fast blink
      break;
    case 2: // 2 = update slot IDs
      if ((LOC_ID1 == MY_ID1) && (LOC_ID2 == MY_ID2)) {
        SubMenu++;
      } else if (!(LNTxBufferL)) {
        LOC_STAT &= 0x78;           // wash speed steps out
        LOC_STAT |= (MY_SS & 0x07);  // add requested speed steps
        Submit_LN_UpdateMySlot();
        SubMenu++;
      }
      BlinkSpeed = 120;   // fast blink
      break;
    case 3: // 3 = check slot is active - if not, then submit null move
      if ((LOC_STAT & 0x30) == 0x30) {
        SubMenu++;
      } else if (!(LNTxBufferL)) {
        Submit_LN_NullMove();
        SubMenu++;
      }
      BlinkSpeed = 120;   // fast blink
      break;
    case 4: // 4 = check speed steps (same = skip, differ update slot)
      if ((LOC_STAT & 0x07) == (MY_SS & 0x07)) {
        SubMenu++;
      } else if (!(LNTxBufferL)) {
        LOC_STAT &= 0x78;           // wash speed steps out
        LOC_STAT |= (MY_SS & 0x07);  // add requested speed steps
        Submit_LN_UpdateMySlot();
        SubMenu++;
      }
      BlinkSpeed = 120;   // fast blink
      break;
    case 5: // 5 = request F9-F16
      if ((!LNvU) && (!(TRK & 0x40))) {   // not Uhlenbrock, not LN P2 command station
        LOC_F9F16 = 0;            // empty functions, have no info
        SubMenu++;                // and go next
      }
      if ((LNvU) && (!(LNTxBufferL))) {   // send uhlenbrock version of P2 slot request
        LNTxBuffer[0] = 0xBB;       // OPC_RQ_SL_DATA
        LNTxBuffer[1] = LOC_SLOT;       // <SLOT>
        LNTxBuffer[2] = 0x08;       // <SLOTB> in Uhlenbrock bit 3 mean LocoSlotDataP2 response is requested
        LNTxBuffer[3] = 0xBB ^ 0x08 ^ 0xFF ^ LNTxBuffer[1];       // Checksum 
        LNTxBufferL = 4;            // 4 bytes to send
        LOC_F9F16 = 0;            // empty functions, have no info yet
        SubMenu++;                // and go next
      }
      if ((TRK & 0x40) && (!(LNTxBufferL))) {   // send digitrax version of P2 slot request
        LNTxBuffer[0] = 0xBB;       // OPC_RQ_SL_DATA
        LNTxBuffer[1] = LOC_SLOT;       // <SLOT>
        LNTxBuffer[2] = 0x40;       // <SLOTB> in digitrax bit 6 mean LocoSlotDataP2 response is requested
        LNTxBuffer[3] = 0xBB ^ 0x40 ^ 0xFF ^ LNTxBuffer[1];       // Checksum 
        LNTxBufferL = 4;            // 4 bytes to send
        LOC_F9F16 = 0;            // empty functions, have no info yet
        SubMenu++;                // and go next
      }
      break;
    // 6 = operation
    case 0x0E:       // E = Warning state F0 = goto 2.2, Stop = 1.0
      BlinkSpeed = 120;   // fast blink
      break;
    // F = Error state - Shift+Stop=goto 0.0
  }
  ///////////// LED section //////////
  if ((BlinkSpeed) && (millis() - LastBlinkMillis >= BlinkSpeed)) {
    LastBlinkMillis += BlinkSpeed;
    BlinkState++;
    if (SubMenu <= 5 ) {    // states up to 5 requesting loco information, blink left right
        digitalWrite(LED_RED, HIGH);
        ((BlinkState & 0x01) ? digitalWrite(LED_GRE_L, HIGH) : digitalWrite(LED_GRE_L, LOW));
        ((BlinkState & 0x01) ? digitalWrite(LED_GRE_R, LOW) : digitalWrite(LED_GRE_R, HIGH));
    }
    if (SubMenu == 6 ) {    // state 6 is normal run, green indicate direction; green blinks, when wrong direction
      digitalWrite(LED_RED, HIGH);
      if (((!(LastGPIO & 0x400000)) != (!(LOC_DIRF & 0x20))) || (LOC_SPEED == 1)) {
        if (LOC_DIRF & 0x20) {((BlinkState & 0x01) ? digitalWrite(LED_GRE_L, HIGH) : digitalWrite(LED_GRE_L, LOW)); digitalWrite(LED_GRE_R, HIGH);}
        else {digitalWrite(LED_GRE_L, HIGH); ((BlinkState & 0x01) ? digitalWrite(LED_GRE_R, LOW) : digitalWrite(LED_GRE_R, HIGH));}
      } else {
        if (LOC_DIRF & 0x20) {digitalWrite(LED_GRE_L, LOW); digitalWrite(LED_GRE_R, HIGH);}
        else {digitalWrite(LED_GRE_L, HIGH); digitalWrite(LED_GRE_R, LOW);}
      }
    }
    if (SubMenu == 0x0E ) {    // state E is warning, blink red
        ((BlinkState & 0x01) ? digitalWrite(LED_RED, HIGH) : digitalWrite(LED_RED, LOW));
        digitalWrite(LED_GRE_L, HIGH);
        digitalWrite(LED_GRE_R, HIGH);
    }
    if (SubMenu == 0x0F ) {    // state F is error, blink red
        ((BlinkState & 0x03) ? digitalWrite(LED_RED, LOW) : digitalWrite(LED_RED, HIGH));
        digitalWrite(LED_GRE_L, HIGH);
        digitalWrite(LED_GRE_R, HIGH);
    }
  }
  if (millis() - LongRefreshMillis >= 59900) {  // forced refresh every 59 sec
    LongRefreshMillis = millis();
    LastPot = 0xFFFF;           // unreal value to force refresh
  }
  if (digitalRead(PA0) == LOW) {
    MainState=3;
  }     // power down
  if ((millis() - LastKeyPotMillis >= 19) && (MainState == 2)) {
    LastKeyPotMillis = millis();
    ///////////// Potentiometer section //////////
    if (SubMenu == 6 ) {    // potetntiometer is used only in state 6
      if ((PotDelay) || (LNTxBufferL)) {
        if (PotDelay) {PotDelay--;}
      } else {
        int32_t PotRead = analogRead(PA1);
        if (abs(PotRead - LastPot) > 30) {
          LongRefreshMillis = millis();
          LastPot = PotRead;
          if ((LOC_SPEED != 1) || (LastPot < 60)) {
            LOC_SPEED = LastPot >> 5;
            if (LOC_SPEED == 1) {LOC_SPEED = 0;}        // skip speed step 1 -> it is estop
            if (LOC_SPEED > 127) {LOC_SPEED = 127;}     // maximum speed is 127
            LNSendSPEED();
            PotDelay = 25;
          }
        }
      }
    }
    ///////////// Keyboard section //////////
    uint32_t ReadGPIO = ((GPIOB->INDR) << 16) | GPIOA->INDR;
    ReadGPIO = 0xFFFFFFFF ^ ReadGPIO;
    ReadGPIO = ReadGPIO & 0xC29EB4;         // mask for buttons and dir switch
    // 0x829EB4 // mask for buttons exclude dir switch
    if ((WaitKeyRelease) && !(ReadGPIO & 0x809EB4)) { // mask for buttons exclude dir switch, exclude shift
      WaitKeyRelease=0;
    }
    if (LastGPIO ^ ReadGPIO) {              // some key pressed
      LastGPIO = ReadGPIO;
      // PA0 = power sense (wkup)
      // PA1 = Potentiometer (analog)
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000004)) {RunKeyF1F9();} // PA2 = F1/F9
      // PA3 = LN-Rx
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000010)) {RunKeyF0();} // PA4 = F0
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000020)) {RunKeyF2F10();} // PA5 = F2/F10
      // PA6 = LN-Vref
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000080)) {RunKeyF3F11();} // PA7 = F3/F11
      // PA8 = LN-Tx
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000200)) {RunKeyF4F12();} // PA9 = F4/F12
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000400)) {RunKeyF8F16();} // PA10 = F8/F16
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000800)) {RunKeyF7F15();} // PA11 = F7/F15
      if (!(WaitKeyRelease) && (ReadGPIO & 0x001000)) {RunKeyF6F14();} // PA12 = F6/F14
      // PA13 = SWDIO
      // PA14 = SWDCLK
      if (!(WaitKeyRelease) && (ReadGPIO & 0x008000)) {RunKeyF5F13();} // PA15 = F5/F13
      //PB0 = LN-in
      if (!(WaitKeyRelease) && (ReadGPIO & 0x020000)) {RunKeyShift();} // PB1 = Shift
      // PB2
      // PB3 = LED green
      // PB4 = LED green
      // PB5 = LED red
      //if (!(WaitKeyRelease) && (ReadGPIO & 0x400000)) {SelfTestKeyDir(0x0D);} // PB6 = Dir sw
      if (!(WaitKeyRelease) && (ReadGPIO & 0x800000)) {RunKeyESTOP();} // PB7 = ESTOP
    }
    if ((!(LastGPIO & 0x400000)) != (!(LOC_DIRF & 0x20))) {
      if (LOC_SPEED > 1) {
        BlinkSpeed=250;   // slow blink left right
      } else {
        LOC_DIRF ^= 0x20;
        LNSendDIRF();
      }
    }
  }
    ///////////// LocoNet section //////////
  if (NewPacket) {
    decodeNormalRun();
  }
}

void decodeNormalRun() {
  NewPacket = 0;
      // packets we are interested:
				// OPC_SL_RD_DATA 0xE7 ;SLOT DATA return
        // OPC_SL_RD_DATA_P2 0xE6 ;SLOT DATA in protocol 2 version
				// OPC_LONG_ACK 0xB4 ;Long acknowledge  (for selected cases only)
				// OPC_IDLE 0x85
				// OPC_GPON 0x83
				// OPC_GPOFF 0x82
				// OPC_LOCO_SND 0xA2
				// OPC_LOCO_DIRF 0xA1
				// OPC_LOCO_SPD 0xA0

  switch (RXOPCODE) {
    case 0xA0:   // OPC_LOCO_SPD   --- when seen, it indicate, someone control my slot
    case 0xA1:   // OPC_LOCO_DIRF   /
    case 0xA2:   // OPC_LOCO_SND   /
      if (LNRxBuffer[0]==LOC_SLOT) {  // is it about my slot?
        SubMenu=0x0E;                 // yes, then go to error state
      }
      break;
    case 0x82:   // OPC_GPOFF
      TRK &= 0xFE;    // set local state
      break;
    case 0x83:   // OPC_GPON
      TRK |= 0x03;    // set local state
      break;
    case 0x85:   // OPC_IDLE
      TRK &= 0xFD;    // set local state
      break;
    case 0xB4:   // OPC_LONG_ACK
      if ((SubMenu == 1) && (LNRxBuffer[0] == 0x3F)) {  // waiting for slot and received LACK for OPC_LOCO_ADR 0xBF ? 
        SubMenu=0x0F;                 // yes, then go to error state
      }
      break;
    case 0xE7:   // OPC_SL_RD_DATA
      if (SubMenu == 1) {  // waiting for address slot
        if ((LNRxBuffer[0] ==  0x0E) && (LNRxBuffer[3] ==  MY_AL) && (LNRxBuffer[8] ==  MY_AH)) {   // slot with my address
          LOC_SLOT = LNRxBuffer[1];             // SLOT#
          // LNRxBuffer[3];               // Address Low
          // LNRxBuffer[8];               // Address High
          LOC_DIRF = LNRxBuffer[5];             // Dir & F0 to F4
          LOC_SND = LNRxBuffer[9];              // F5 to F8
          //F9F16;            // F9 to F16
          LOC_STAT = LNRxBuffer[2];             // slot status
          TRK = LNRxBuffer[6];              // command station status

          if ((LNRxBuffer[10] == 0) && (LNRxBuffer[11] == 0)) {
            SubMenu = 2;        // ID 00:00 -> update my ID
          } else if ((LNRxBuffer[10] == MY_ID1) && (LNRxBuffer[11] == MY_ID2)) {
            SubMenu = 3;        // ID = MY_ID -> check status
          } else {
            SubMenu = 0x0E;        // ID != MY_ID -> blink for update
          }
        }
      }
      if ((LNRxBuffer[0] ==  0x0E) && (LNRxBuffer[1] ==  LOC_SLOT) && (LNRxBuffer[3] ==  MY_AL) && (LNRxBuffer[8] ==  MY_AH)) {   // my slot with my address 
        //LOC_SLOT = LNRxBuffer[1];             // SLOT#
        // LNRxBuffer[3];               // Address Low
        // LNRxBuffer[8];               // Address High
        LOC_DIRF = LNRxBuffer[5];             // Dir & F0 to F4
        LOC_SND = LNRxBuffer[9];              // F5 to F8
        //F9F16;            // F9 to F16
        LOC_STAT = LNRxBuffer[2];             // slot status
        TRK = LNRxBuffer[6];              // command station status
      }
      if ((LNRxBuffer[0] == 0x0E) && (LNRxBuffer[1] == 0)) {   // Slot #0 = accept command station type
        TRK = LNRxBuffer[6];              // command station status
        LNvU = ((LNRxBuffer[10] == 0x49) && (LNRxBuffer[11] == 0x42));       // Uhlenbrock type command stations identify themselves as "IB"
      }
      break;
    case 0xE6:    // OPC_SL_RD_DATA_P2
      if ((LNRxBuffer[0] == 0x15) && (LNRxBuffer[1] == 0x00) && (LNRxBuffer[2] == LOC_SLOT) ) {   // accept my slot only
        TRK = LNRxBuffer[6];              // command station status
        LOC_F9F16 = (LNRxBuffer[10] >> 4) & 0x07;       // F9, F10, F11
        if (LNRxBuffer[8] & 0x10) {LOC_F9F16 |= 0x08;}  // F12
        LOC_DIRF = LNRxBuffer[9];                       // DIRF
        LOC_SND = LNRxBuffer[10] & 0x0F;                // SND
        LOC_F9F16 |= (LNRxBuffer[11] << 4) & 0xF0;       // F13, F14, F15, F16
      }
    break;
  }
}

void RunKeyESTOP() {
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    EEPROM.write(EEPROM_ADR1,0);
    EEPROM.write(EEPROM_ADR2,0);
    EEPROM.commit();
    if ((SubMenu>=2) && (SubMenu<=6)) {Submit_LN_ReleaseSlot();}  // submenus, where my IDs can be populated
    SubMenu = 0;                // release loco and go to wait for address
    MainState = 1;
  }
  if (SubMenu == 0x0E ) {       // in submenu E it mean reject
    SubMenu = 0;                // ipdate slot to my IDs
    MainState = 1;
  } else if (SubMenu == 6) {
    if (LOC_SPEED > 1) {
      LOC_SPEED = 1;
      LNSendSPEED();
    }
  }
}

void RunKeyShift() {
  // TBD
}

void RunKeyF0() {
  WaitKeyRelease = 1;
  if (SubMenu == 0x0E ) {       // in submenu E it mean accept
    SubMenu = 2;                // ipdate slot to my IDs
  } else {
    LOC_DIRF ^= 0x10;           // change state of F0
    LNSendDIRF();
  }
}

void RunKeyF1F9() {
  WaitKeyRelease = 1;
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    LOC_F9F16 ^= 0x01;          // change state of F9
    LNSendF9F11();
  } else {
    LOC_DIRF ^= 0x01;           // change state of F1
    LNSendDIRF();
  }
}

void RunKeyF2F10() {
  WaitKeyRelease = 1;
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    LOC_F9F16 ^= 0x02;          // change state of F10
    LNSendF9F11();
  } else {
    LOC_DIRF ^= 0x02;           // change state of F2
    LNSendDIRF();
  }
}

void RunKeyF3F11() {
  WaitKeyRelease = 1;
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    LOC_F9F16 ^= 0x04;          // change state of F11
    LNSendF9F11();
  } else {
    LOC_DIRF ^= 0x04;           // change state of F3
    LNSendDIRF();
  }
}

void RunKeyF4F12() {
  WaitKeyRelease = 1;
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    LOC_F9F16 ^= 0x08;          // change state of F12
    LNSendF12();
  } else {
    LOC_DIRF ^= 0x08;           // change state of F4
    LNSendDIRF();
  }
}

void RunKeyF5F13() {
  WaitKeyRelease = 1;
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    LOC_F9F16 ^= 0x10;          // change state of F13
    LNSendF13();
  } else {
    LOC_SND ^= 0x01;           // change state of F5
    LNSendSND();
  }
}

void RunKeyF6F14() {
  WaitKeyRelease = 1;
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    LOC_F9F16 ^= 0x20;          // change state of F14
    LNSendF14F16();
  } else {
    LOC_SND ^= 0x02;           // change state of F6
    LNSendSND();
  }
}

void RunKeyF7F15() {
  WaitKeyRelease = 1;
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    LOC_F9F16 ^= 0x40;          // change state of F15
    LNSendF14F16();
  } else {
    LOC_SND ^= 0x04;           // change state of F7
    LNSendSND();
  }
}

void RunKeyF8F16() {
  WaitKeyRelease = 1;
  if (LastGPIO & 0x020000) {    // Is Shift pressed?
    LOC_F9F16 ^= 0x80;          // change state of F16
    LNSendF14F16();
  } else {
    LOC_SND ^= 0x08;           // change state of F8
    LNSendSND();
  }
}

void LNSendSND() {
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    LNTxBuffer[0] = 0xA2;		      // OPC_LOCO_DIRF
    LNTxBuffer[1] = LOC_SLOT;     // <SLOT>
    LNTxBuffer[2] = LOC_SND;      // <SPEED> 
    LNTxBuffer[3] = 0xFF ^ 0xA2 ^ LNTxBuffer[1] ^ LNTxBuffer[2];         // <CHK>
    LNTxBufferL = 4;              // 4 bytes to send
  }
}

void LNSendF9F11() {
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    if (LNvU) {             // Uhlenbrock variant
      LNSend_F5F11_UhB();
    } else if (TRK & 0x40) { // Digitrax P2 variant
      LNSend_F7F13_P2();
    } else {
      LNSend_F9F12_IMM();
    }
  }
}

void LNSendF12() {
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    if (LNvU) {             // Uhlenbrock variant
      LNSend_F12_UhB();
    } else if (TRK & 0x40) { // Digitrax P2 variant
      LNSend_F7F13_P2();
    } else {
      LNSend_F9F12_IMM();
    }
  }
}

void LNSendF13() {
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    if (LNvU) {             // Uhlenbrock variant
      LNSend_F13F19_UhB();
    } else if (TRK & 0x40) { // Digitrax P2 variant
      LNSend_F7F13_P2();
    } else {
      LNSend_F13F16_IMM();
    }
  }
}

void LNSendF14F16() {
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    if (LNvU) {             // Uhlenbrock variant
      LNSend_F13F19_UhB();
    } else if (TRK & 0x40) { // Digitrax P2 variant
      LNSend_F14F20_P2();
    } else {
      LNSend_F13F16_IMM();
    }
  }
}

void LNSend_F9F12_IMM() {                 // IM1                        IM2                          IM3                         IM4
  if (MY_AH) { // long version   address is: 1 1 H6 H5 | H4 H3 H2 H1 || H0 L6 L5 L4 | L3 L2 L1 L0 || 1 0 1 0 | F12 F11 F10 F9 || x x x x | x x x x
    LNTxBuffer[0] = 0xED;		      // OPC IMM PACKET
    LNTxBuffer[1] = 0x0B;         // <LEN> 11 bytes
    LNTxBuffer[2] = 0x7F;         // <7F> sign
    LNTxBuffer[3] = 0x43;         // <REP> 4 bytes, repeat 3 times
    if (MY_AH & 0x01) {LNTxBuffer[4] = 0x25;} else {LNTxBuffer[4] = 0x2F;} // <DHI> IM1 and IM3 have top bits 1; IM2 and IM4 depend on H0
    LNTxBuffer[5] = ((MY_AH >> 1) & 0x3F) | 0x40;      // 0 1 H6 H5 H4 H3 H2 H1
    LNTxBuffer[6] = MY_AL;        // 0 L6 L5 L4 L3 L2 L1 L0
    LNTxBuffer[7] = (LOC_F9F16 & 0x0F) | 0x20;   // 0 0 1 0 F12 F11 F10 F9
    LNTxBuffer[8] = LNTxBuffer[5] ^ LNTxBuffer[6] ^ LNTxBuffer[7];   // XOR
    LNTxBuffer[9] = 0;
    LNTxBuffer[10] = 0xFF ^ 0xED ^ 0x0B ^ 0x7F ^ 0x43 ^ LNTxBuffer[4] ^ LNTxBuffer[5] ^ LNTxBuffer[6] ^ LNTxBuffer[7] ^ LNTxBuffer[8];         // <CHK>
    LNTxBufferL = 11;              // 11 bytes to send
  } else {                    // IM1                         IM2                         IM3
  // short version   address is: 0 L6 L5 L4 | L3 L2 L1 L0 || 1 0 1 0 | F12 F11 F10 F9 || x x x x | x x x x
    LNTxBuffer[0] = 0xED;		      // OPC IMM PACKET
    LNTxBuffer[1] = 0x0B;         // <LEN> 11 bytes
    LNTxBuffer[2] = 0x7F;         // <7F> sign
    LNTxBuffer[3] = 0x33;         // <REP> 3 bytes, repeat 3 times
    LNTxBuffer[4] = 0x26;         // <DHI> IM2 and IM3 have top bits 1
    LNTxBuffer[5] = MY_AL;        // 0 L6 L5 L4 L3 L2 L1 L0
    LNTxBuffer[6] = (LOC_F9F16 & 0x0F) | 0x20;   // 0 0 1 0 F12 F11 F10 F9
    LNTxBuffer[7] = LNTxBuffer[5] ^ LNTxBuffer[6];   // XOR
    LNTxBuffer[8] = 0;
    LNTxBuffer[9] = 0;
    LNTxBuffer[10] = 0xFF ^ 0xED ^ 0x0B ^ 0x7F ^ 0x33 ^ 0x26 ^ LNTxBuffer[5] ^ LNTxBuffer[6] ^ LNTxBuffer[7];         // <CHK>
    LNTxBufferL = 11;              // 11 bytes to send
  }
}

void LNSend_F13F16_IMM() {                 // IM1                        IM2                          IM3                  IM4                         IM5
  if (MY_AH) { // long version   address is: 1 1 H6 H5 | H4 H3 H2 H1 || H0 L6 L5 L4 | L3 L2 L1 L0 || 1 1 0 1 | 1 1 1 0 || 0 0 0 0 | F16 F15 F14 F13 || H0 x x x | x x x x
    LNTxBuffer[0] = 0xED;		      // OPC IMM PACKET
    LNTxBuffer[1] = 0x0B;         // <LEN> 11 bytes
    LNTxBuffer[2] = 0x7F;         // <7F> sign
    LNTxBuffer[3] = 0x53;         // <REP> 4 bytes, repeat 3 times
    if (MY_AH & 0x01) {LNTxBuffer[4] = 0x25;} else {LNTxBuffer[4] = 0x37;} // <DHI> IM1 and IM3 have top bits 1; IM2 and IM5 depend on H0
    LNTxBuffer[5] = ((MY_AH >> 1) & 0x3F) | 0x40;      // <IM1> 0 1 H6 H5 H4 H3 H2 H1
    LNTxBuffer[6] = MY_AL;        // <IM2> 0 L6 L5 L4 L3 L2 L1 L0
    LNTxBuffer[7] = 0x5E;         // <IM3> 0xDE - MSB is in <DHI>
    LNTxBuffer[8] = (LOC_F9F16 >> 4) | 0x0F;   // <IM4> 0 0 0 0 F16 F15 F14 F13
    LNTxBuffer[9] = LNTxBuffer[5] ^ LNTxBuffer[6] ^ 0x5E ^ LNTxBuffer[8];   // <IM5> XOR
    LNTxBuffer[10] = 0xFF ^ 0xED ^ 0x0B ^ 0x7F ^ 0x53 ^ 0x5E ^ LNTxBuffer[4] ^ LNTxBuffer[5] ^ LNTxBuffer[6] ^ LNTxBuffer[8] ^ LNTxBuffer[9];         // <CHK>
    LNTxBufferL = 11;              // 11 bytes to send
  } else {                    // IM1                         IM2                  IM3                          IM4
  // short version   address is: 0 L6 L5 L4 | L3 L2 L1 L0 || 1 1 0 1 | 1 1 1 0 || 0 0 0 0 | F16 F15 F14 F13 || 1 x x x | x x x x
    LNTxBuffer[0] = 0xED;		      // OPC IMM PACKET
    LNTxBuffer[1] = 0x0B;         // <LEN> 11 bytes
    LNTxBuffer[2] = 0x7F;         // <7F> sign
    LNTxBuffer[3] = 0x43;         // <REP> 4 bytes, repeat 3 times
    LNTxBuffer[4] = 0x2A;         // <DHI> IM2 and IM4 have top bits 1
    LNTxBuffer[5] = MY_AL;        // <IM1> 0 L6 L5 L4 L3 L2 L1 L0
    LNTxBuffer[6] = 0x5E;         // <IM2> 0xDE - MSB is in <DHI>
    LNTxBuffer[7] = ((LOC_F9F16 >> 4) & 0x0F);   // <IM3> 0 0 0 0 F16 F15 F14 F13
    LNTxBuffer[8] = LNTxBuffer[5] ^ 0x5E ^ LNTxBuffer[7];   // <IM4> XOR
    LNTxBuffer[9] = 0;            // <IM5>
    LNTxBuffer[10] = 0xFF ^ 0xED ^ 0x0B ^ 0x7F ^ 0x43 ^ 0x2A ^ 0x5E ^ LNTxBuffer[5] ^ LNTxBuffer[7] ^ LNTxBuffer[8];         // <CHK>
    LNTxBufferL = 11;              // 11 bytes to send
  }
}

void LNSend_F5F11_UhB() {
  LNTxBuffer[0] = 0xD4;		      // OPC_EXP_CMD
  LNTxBuffer[1] = 0x20;         // slot data + SLOTB
  LNTxBuffer[2] = LOC_SLOT;     // <SLOT>
  LNTxBuffer[3] = 0x07;         // <SUBCODE> functions F5 to F11
  LNTxBuffer[4] = LOC_SND & 0x0F;      // 0 0 0 0 F8 F7 F6 F5
  LNTxBuffer[4] |= ((LOC_F9F16 << 4) & 0x70);   // 0 F11 F10 F9 F8 F7 F6 F5
  LNTxBuffer[5] = 0xFF ^ 0xD4 ^ 0x20 ^ 0x07 ^ LNTxBuffer[2] ^ LNTxBuffer[4];         // <CHK>
  LNTxBufferL = 6;              // 6 bytes to send
}

void LNSend_F12_UhB() {
  LNTxBuffer[0] = 0xD4;		      // OPC_EXP_CMD
  LNTxBuffer[1] = 0x20;         // slot data + SLOTB
  LNTxBuffer[2] = LOC_SLOT;     // <SLOT>
  LNTxBuffer[3] = 0x05;         // <SUBCODE> function F28,F20,F12
  LNTxBuffer[4] = ((LOC_F9F16 << 1) & 0x10);   // 0 0 0 F12 0 0 0 0 -> (0 F28 F20 F12 0 0 0 0)
  LNTxBuffer[5] = 0xFF ^ 0xD4 ^ 0x20 ^ 0x05 ^ LNTxBuffer[2] ^ LNTxBuffer[4];         // <CHK>
  LNTxBufferL = 6;              // 6 bytes to send
}

void LNSend_F13F19_UhB() {
  LNTxBuffer[0] = 0xD4;		      // OPC_EXP_CMD
  LNTxBuffer[1] = 0x20;         // slot data + SLOTB
  LNTxBuffer[2] = LOC_SLOT;     // <SLOT>
  LNTxBuffer[3] = 0x08;         // <SUBCODE> functions F13 to F19
  LNTxBuffer[4] = ((LOC_F9F16 >> 4) & 0x0F);   // 0 0 0 0 F16 F15 F14 F13
  LNTxBuffer[5] = 0xFF ^ 0xD4 ^ 0x20 ^ 0x08 ^ LNTxBuffer[2] ^ LNTxBuffer[4];         // <CHK>
  LNTxBufferL = 6;              // 6 bytes to send
}

void LNSend_F7F13_P2() {
  LNTxBuffer[0] = 0xD5;		      // OPC_D5_GROUP
  LNTxBuffer[1] = 0x18;         // slot data + SLOTB
  LNTxBuffer[2] = LOC_SLOT;     // <SLOT>
  LNTxBuffer[3] = MY_ID1;         // functions F5 to F11
  LNTxBuffer[4] = (LOC_SND >> 2) & 0x03;      // 0 0 0 0 0 0 F8 F7
  LNTxBuffer[4] |= ((LOC_F9F16 << 2) & 0x7C);   // 0 F13 F12 F11 F10 F9 F8 F7
  LNTxBuffer[5] = 0xFF ^ 0xD5 ^ 0x18 ^ LNTxBuffer[2] ^ LNTxBuffer[3] ^ LNTxBuffer[4];         // <CHK>
  LNTxBufferL = 6;              // 6 bytes to send
}

void LNSend_F14F20_P2() {
  LNTxBuffer[0] = 0xD5;		      // OPC_D5_GROUP
  LNTxBuffer[1] = 0x20;         // slot data + SLOTB
  LNTxBuffer[2] = LOC_SLOT;     // <SLOT>
  LNTxBuffer[3] = MY_ID1;         // functions F5 to F11
  LNTxBuffer[4] = (LOC_F9F16 >> 5) & 0x07;      // 0 0 0 0 0 F16 F15 F14
  LNTxBuffer[5] = 0xFF ^ 0xD5 ^ 0x20 ^ LNTxBuffer[2] ^ LNTxBuffer[3] ^ LNTxBuffer[4];         // <CHK>
  LNTxBufferL = 6;              // 6 bytes to send
}

void LNSendDIRF() {
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    LNTxBuffer[0] = 0xA1;		      // OPC_LOCO_DIRF
    LNTxBuffer[1] = LOC_SLOT;     // <SLOT>
    LNTxBuffer[2] = LOC_DIRF;     // <SPEED> 
    LNTxBuffer[3] = 0xFF ^ 0xA1 ^ LNTxBuffer[1] ^ LNTxBuffer[2];         // <CHK>
    LNTxBufferL = 4;              // 4 bytes to send
  }
}

void LNSendSPEED()  {
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    LNTxBuffer[0] = 0xA0;		      // OPC_LOCO_SPD
    LNTxBuffer[1] = LOC_SLOT;     // <SLOT>
    LNTxBuffer[2] = LOC_SPEED;    // <SPEED> 
    LNTxBuffer[3] = 0xFF ^ 0xA0 ^ LNTxBuffer[1] ^ LNTxBuffer[2];         // <CHK>
    LNTxBufferL = 4;              // 4 bytes to send
  }
}

void Submit_LN_NullMove() {
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    LNTxBuffer[0] = 0xBA;		      // OPC_MOVE_SLOTS
    LNTxBuffer[1] = LOC_SLOT;     // <SLOT>
    LNTxBuffer[2] = LOC_SLOT;     // <SLOT> again
    LNTxBuffer[3] = 0x45;         // <CHK>
    LNTxBufferL = 4;              // 4 bytes to send
    LOC_STAT |= 0x30;             // add to local
  }
}

void Submit_LN_UpdateMySlot() {
	// OPC_WR_SL_DATA 0xEF ;WRITE SLOT DATA, 10 bytes
	//<0xEF>,<0E>,<SLOT#>,<STAT>,<ADR>,<SPD>,<DIRF>,<TRK>
	//<SS2>,<ADR2>,<SND>,<ID1>,<ID2>,<CHK>
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    LNTxBuffer[0] = 0xEF;		      // OPC_WR_SL_DATA 0xEF ;WRITE SLOT DATA
    LNTxBuffer[1] = 0x0E;		      // 0E = length 14 bytes
    LNTxBuffer[2] = LOC_SLOT;     // <SLOT>
    LNTxBuffer[3] = LOC_STAT;     // <STAT>
    LNTxBuffer[4] = MY_AL;           // <ADR>
    LNTxBuffer[5] = LOC_SPEED;      // <SPD>
    LNTxBuffer[6] = LOC_DIRF;         // <DIRF>
    LNTxBuffer[7] = TRK;          // <TRK>
    LNTxBuffer[8] = 0;            // <SS2>
    LNTxBuffer[9] = MY_AH;           // <ADR2>
    LNTxBuffer[10] = LOC_SND;         // <SND>
    LNTxBuffer[11] = MY_ID1;      // <ID1>
    LNTxBuffer[12] = MY_ID2;      // <ID2>
    LNTxBuffer[13] = 0xFF;        // prepare Checksum 
    for (int i = 0; i < 13; i++) {LNTxBuffer[13] ^= LNTxBuffer[i];}
    LNTxBufferL = 14;             // 14 bytes to send
  }
}

void Submit_LN_ReleaseSlot() {
	// OPC_WR_SL_DATA 0xEF ;WRITE SLOT DATA, 10 bytes
	//<0xEF>,<0E>,<SLOT#>,<STAT>,<ADR>,<SPD>,<DIRF>,<TRK>
	//<SS2>,<ADR2>,<SND>,<ID1>,<ID2>,<CHK>
  if (LOC_SLOT <= 127) {  // not possible for special slots
    while (LNTxBufferL) {}        // wait for empty Tx
    LNTxBuffer[0] = 0xEF;		      // OPC_WR_SL_DATA 0xEF ;WRITE SLOT DATA
    LNTxBuffer[1] = 0x0E;		      // 0E = length 14 bytes
    LNTxBuffer[2] = LOC_SLOT;     // <SLOT>
    LNTxBuffer[3] = LOC_STAT;     // <STAT>
    LNTxBuffer[4] = MY_AL;           // <ADR>
    LNTxBuffer[5] = 0;            // <SPD>
    LNTxBuffer[6] = LOC_DIRF;         // <DIRF>
    LNTxBuffer[7] = TRK;          // <TRK>
    LNTxBuffer[8] = 0;            // <SS2>
    LNTxBuffer[9] = MY_AH;           // <ADR2>
    LNTxBuffer[10] = LOC_SND;         // <SND>
    LNTxBuffer[11] = 0;           // <ID1>
    LNTxBuffer[12] = 0;           // <ID2>
    LNTxBuffer[13] = 0xFF;        // prepare Checksum 
    for (int i = 0; i < 13; i++) {LNTxBuffer[13] ^= LNTxBuffer[i];}
    LNTxBufferL = 14;             // 14 bytes to send
  }
}


void decodeSelfTest() {
  NewPacket = 0;
  // E5 10 01 07 02 10 00 00 00 00 10 00 00 00 00 0E
  // E5 10 - peer packet for LNSV 
  //       01 - source (01 to 0f typically PC)
  //          07 - command (07 discover)
  //             02 - SV type (only 02 supported)
  //                10 - SVX1 - top bits for SV_ADRH, SV_ADRL, DST_H, DST_L
  //                   00 00 - DST_L DST_H
  //                         00 00 - SV_ADR_L SV_ADRH
  //                               10 - SVX2 - top bits for D1 to D4
  //                                  00 00 00 00 - D1 to D4

  if ((RXOPCODE == 0xE5) && (LNRxBuffer[0] == 0x10) && (LNRxBuffer[3] == 0x02) && ((LNRxBuffer[4] & 0xF0) == 0x10) && ((LNRxBuffer[9] & 0xF0) == 0x10)) {
    if (LNRxBuffer[4] & 0x08) {LNRxBuffer[8] |= 0x80;}
    if (LNRxBuffer[4] & 0x04) {LNRxBuffer[7] |= 0x80;}
    if (LNRxBuffer[4] & 0x02) {LNRxBuffer[6] |= 0x80;}
    if (LNRxBuffer[4] & 0x01) {LNRxBuffer[5] |= 0x80;}
    if (LNRxBuffer[9] & 0x08) {LNRxBuffer[13] |= 0x80;}
    if (LNRxBuffer[9] & 0x04) {LNRxBuffer[12] |= 0x80;}
    if (LNRxBuffer[9] & 0x02) {LNRxBuffer[11] |= 0x80;}
    if (LNRxBuffer[9] & 0x01) {LNRxBuffer[10] |= 0x80;}

    switch (LNRxBuffer[2]) {
      case 0x05:  // Write 4 bytes
        if ((LNRxBuffer[5] == EEPROM.read(EEPROM_ID2)) && (LNRxBuffer[6] == EEPROM.read(EEPROM_ID1)) && (LNRxBuffer[7] == 4) && (LNRxBuffer[8] == 0) ) { // write 4 bytes from address 4 
          MY_AL = LNRxBuffer[10] & 0x7F;
          EEPROM.write(EEPROM_ADR2,MY_AL);
          MY_AH = LNRxBuffer[11] & 0x7F;
          EEPROM.write(EEPROM_ADR1,MY_AH);
          MY_SS = LNRxBuffer[12] & 0x7F;
          EEPROM.write(EEPROM_SS,MY_SS);
          EEPROM.write(EEPROM_CFG,LNRxBuffer[13] & 0x7F);
          EEPROM.commit();
          LNSendRead4Response();
        }
        break;
      case 0x06:  // Read 4 bytes
        if ((LNRxBuffer[5] == EEPROM.read(EEPROM_ID2)) && (LNRxBuffer[6] == EEPROM.read(EEPROM_ID1)) && (LNRxBuffer[7] == 4) && (LNRxBuffer[8] == 0) ) { // read 4 bytes from address 4 
          LNSendRead4Response();
        }
        break;
      case 0x07:  // discover
        LNSendDiscoverResponse();
        break;
      case 0x08:  // identify
        if ((LNRxBuffer[5] == EEPROM.read(EEPROM_ID1)) && (LNRxBuffer[6] == EEPROM.read(EEPROM_ID2))) {LNSendDiscoverResponse();}
        break;
      case 0x09:  // change address
        if ((LNRxBuffer[7] == NMRA_ID) && (LNRxBuffer[8] == LN_DEVELOPER_ID) && (LNRxBuffer[10] == 0x20 ) && (LNRxBuffer[11] == 00) && (LNRxBuffer[12] == EEPROM.read(EEPROM_ID1)) && (LNRxBuffer[13] == EEPROM.read(EEPROM_ID2))) {
          MY_ID1=LNRxBuffer[5];
          MY_ID2=LNRxBuffer[6];
          EEPROM.write(EEPROM_ID1,LNRxBuffer[5]);
          EEPROM.write(EEPROM_ID2,LNRxBuffer[6]);
          EEPROM.commit();
          LNSendDiscoverResponse();
        }
        break;
    }
  }

}

void LNSendRead4Response() {
  while (LNTxBufferL) {}
  LNTxBuffer[0] = 0xE5;       // peer command
  LNTxBuffer[1] = 0x10;       // length
  LNTxBuffer[2] = 0x10;       // universal input device
  LNTxBuffer[3] = 0x40 | LNRxBuffer[2];       // Read/Write response
  LNTxBuffer[4] = 0x02;       // LNSV v 2
  LNTxBuffer[5] = 0x10;       // SVX1
  if (EEPROM.read(EEPROM_ID1) & 0x80) {LNTxBuffer[5] |= 0x08;}
  if (EEPROM.read(EEPROM_ID2) & 0x80) {LNTxBuffer[5] |= 0x04;}
  LNTxBuffer[6] = EEPROM.read(EEPROM_ID1) & 0x7F;       // DST_L
  LNTxBuffer[7] = EEPROM.read(EEPROM_ID2) & 0x7F;       // DST_H
  LNTxBuffer[8] = 4;                                    // SV_ADRL (only SV 4..7 are supported)
  LNTxBuffer[9] = 0;                                    // SV_ADRH
  LNTxBuffer[10] = 0x10;       // SVX2
  LNTxBuffer[11] = MY_AL;      // D1 = SV4 - Addres low
  LNTxBuffer[12] = MY_AH;      // D2 = SV5 - Address high
  LNTxBuffer[13] = MY_SS;      // D3 = SV6 - speed steps
  LNTxBuffer[14] = EEPROM.read(EEPROM_CFG) & 0x7F;       // D4 = SV7 - config
  LNTxBuffer[15] = 0xFF;   // XOR
  for (int i = 0; i<15; i++) {LNTxBuffer[15] ^= LNTxBuffer[i];}
  LNTxBufferL = 16;            // 16 bytes to send 
}

void LNSendDiscoverResponse() {
  while (LNTxBufferL) {}
  LNTxBuffer[0] = 0xE5;       // peer command
  LNTxBuffer[1] = 0x10;       // length
  LNTxBuffer[2] = 0x10;       // universal input device
  LNTxBuffer[3] = 0x40 | LNRxBuffer[2];       // Discover response
  LNTxBuffer[4] = 0x02;       // LNSV v 2
  LNTxBuffer[5] = 0x10;       // SVX1
  if (EEPROM.read(EEPROM_ID1) & 0x80) {LNTxBuffer[5] |= 0x08;}
  if (EEPROM.read(EEPROM_ID2) & 0x80) {LNTxBuffer[5] |= 0x04;}
  if (NMRA_ID & 0x80) {LNTxBuffer[5] |= 0x02;}
  if (LN_DEVELOPER_ID & 0x80) {LNTxBuffer[5] |= 0x01;}
  LNTxBuffer[6] = EEPROM.read(EEPROM_ID1) & 0x7F;       // DST_L
  LNTxBuffer[7] = EEPROM.read(EEPROM_ID2) & 0x7F;       // DST_H
  LNTxBuffer[8] = NMRA_ID & 0x7F;                       // SV_ADRL
  LNTxBuffer[9] = LN_DEVELOPER_ID & 0x7F;               // SV_ADRH
  LNTxBuffer[10] = 0x10;       // SVX2
  if (EEPROM.read(EEPROM_ID1) & 0x80) {LNTxBuffer[10] |= 0x02;}
  if (EEPROM.read(EEPROM_ID2) & 0x80) {LNTxBuffer[10] |= 0x01;}
  LNTxBuffer[11] = 0x20;     // D1 = product ID L
  LNTxBuffer[12] = 0;      // D2 = product ID H
  LNTxBuffer[13] = EEPROM.read(EEPROM_ID1) & 0x7F;       // D3 = serial number L
  LNTxBuffer[14] = EEPROM.read(EEPROM_ID2) & 0x7F;       // D4 = serial number H
  LNTxBuffer[15] = 0xFF;   // XOR
  for (int i = 0; i<15; i++) {LNTxBuffer[15] ^= LNTxBuffer[i];}
  LNTxBufferL = 16;            // 16 bytes to send
  
}


void SelfTestKey(uint8_t UsedKey) {   // some button is pressed, report it
  while (LNTxBufferL) {}
  LNTxBuffer[0] = 0xA8;       // 0xA8 -> code for keyboard
  LNTxBuffer[1] = 0x30;       // key press
  LNTxBuffer[2] = UsedKey; // key
  LNTxBuffer[3] = 0xA8 ^ 0x30 ^ 0xFF ^ LNTxBuffer[2];
  LNTxBufferL = 4;            // 4 bytes to send
  SelfTestMask |= 1 << (UsedKey - 1);
  WaitKeyRelease = UsedKey;
}

void SelfTestKeyDir(uint8_t UsedKey) {  // dir switch is activated (no wait for release)
  while (LNTxBufferL) {}
  LNTxBuffer[0] = 0xA8;       // 0xA8 -> code for keyboard
  LNTxBuffer[1] = 0x30;       // key press
  LNTxBuffer[2] = UsedKey; // key
  LNTxBuffer[3] = 0xA8 ^ 0x30 ^ 0xFF ^ LNTxBuffer[2];
  LNTxBufferL = 4;            // 4 bytes to send
  SelfTestMask |= 1 << (UsedKey - 1);
}

void SelfTestLoop() {
  ///////////// LED section //////////
  if (millis() - LastBlinkMillis >= BlinkSpeed) {
    LastBlinkMillis += BlinkSpeed;
    BlinkState++;
    if (BlinkState > 2) {BlinkState = 0;}
    switch (BlinkState) {
      case 0: 
        digitalWrite(LED_RED, LOW);
        digitalWrite(LED_GRE_L, HIGH);
        digitalWrite(LED_GRE_R, HIGH);
        break;
      case 1: 
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GRE_L, LOW);
        digitalWrite(LED_GRE_R, HIGH);
        break;
      case 2: 
        digitalWrite(LED_RED, HIGH);
        digitalWrite(LED_GRE_L, HIGH);
        digitalWrite(LED_GRE_R, LOW);
        break;
    }
  }
  if (millis() - LastKeyPotMillis >= 19) {
    LastKeyPotMillis += 19;
    ///////////// Potentiometer section //////////
    int32_t PotRead = analogRead(PA1);
    if (abs(PotRead - LastPot) > 12) {
      LongRefreshMillis = millis();
      LastPot = PotRead;
      if (PotRead < 10) {PotRead = 0;}   // remove small noise
      if (LNTxBufferL == 0) {
        while (LNTxBufferL) {}
        LNTxBuffer[0] = 0xAF;       // 0xAF -> code for potentiometer
        LNTxBuffer[1] = (PotRead >> 2) & 0x7F;  // LL
        LNTxBuffer[2] = (PotRead >> 9) & 0x7F; // HH
        LNTxBuffer[3] = 0xAF ^ 0xFF ^ LNTxBuffer[1] ^ LNTxBuffer[2];
        LNTxBufferL = 4;            // 4 bytes to send
      }
      if (PotRead == 0) {SelfTestMask |= 0x0400;}    // Pot lower limit exceeded
      if (PotRead > 4079) {SelfTestMask |= 0x0800;} // Pot upper limit exceeded
    }
    ///////////// Keyboard section //////////
    uint32_t ReadGPIO = ((GPIOB->INDR) << 16) | GPIOA->INDR;
    ReadGPIO = 0xFFFFFFFF ^ ReadGPIO;
    ReadGPIO = ReadGPIO & 0xC29EB4;         // mask for buttons and dir switch
    if ((WaitKeyRelease) && !(ReadGPIO & 0x829EB4)) {
      if (WaitKeyRelease<16) {
        while (LNTxBufferL) {}
        LNTxBuffer[0] = 0xA8;       // 0xA8 -> code for keyboard
        LNTxBuffer[1] = 0x40;       // key release
        LNTxBuffer[2] = WaitKeyRelease; // key
        LNTxBuffer[3] = 0xA8 ^ 0x40 ^ 0xFF ^ LNTxBuffer[2];
        LNTxBufferL = 4;            // 4 bytes to send
      }
      WaitKeyRelease=0;
    }
    if (LastGPIO ^ ReadGPIO) {              // some key pressed
      LastGPIO = ReadGPIO;
      // PA0 = power sense (wkup)
      // PA1 = Potentiometer (analog)
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000004)) {SelfTestKey(0x02);} // PA2 = F1/F9
      // PA3 = LN-Rx
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000010)) {SelfTestKey(0x01);} // PA4 = F0
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000020)) {SelfTestKey(0x03);} // PA5 = F2/F10
      // PA6 = LN-Vref
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000080)) {SelfTestKey(0x04);} // PA7 = F3/F11
      // PA8 = LN-Tx
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000200)) {SelfTestKey(0x05);} // PA9 = F4/F12
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000400)) {SelfTestKey(0x09);} // PA10 = F8/F16
      if (!(WaitKeyRelease) && (ReadGPIO & 0x000800)) {SelfTestKey(0x08);} // PA11 = F7/F15
      if (!(WaitKeyRelease) && (ReadGPIO & 0x001000)) {SelfTestKey(0x07);} // PA12 = F6/F14
      // PA13 = SWDIO
      // PA14 = SWDCLK
      if (!(WaitKeyRelease) && (ReadGPIO & 0x008000)) {SelfTestKey(0x06);} // PA15 = F5/F13
      //PB0 = LN-in
      if (!(WaitKeyRelease) && (ReadGPIO & 0x020000)) {SelfTestKey(0x0E);} // PB1 = Shift
      // PB2
      // PB3 = LED green
      // PB4 = LED green
      // PB5 = LED red
      if (!(WaitKeyRelease) && (ReadGPIO & 0x400000)) {SelfTestKeyDir(0x0D);} // PB6 = Dir sw
      if (!(WaitKeyRelease) && (ReadGPIO & 0x800000)) {SelfTestKey(0x0A);} // PB7 = ESTOP
    }
    if (digitalRead(PA0) == LOW) {MainState=3;}     // power down

  }
    ///////////// LocoNet section //////////
  if (NewPacket) {
    decodeSelfTest();
  }
    ///////////// Summary section //////////
  if (millis() - LongRefreshMillis >= 59900) {  // forced refresh every 59 sec
    LongRefreshMillis = millis();
    LastPot = 0xFFFF;           // unreal value to force refresh
  }
  if (SelfTestMask == 0x3FFF) {                  // all events appeared
    SelfTestMask |= 0x4000;
    BlinkSpeed = 500;
    while (LNTxBufferL) {}
    LNTxBuffer[0] = 0xA8;       // OPC_LOCO_ADR         0xBF    ;REQ  loco ADR
    LNTxBuffer[1] = 0x30;       // address 00 03
    LNTxBuffer[2] = 0x0B;       // address 00 03
    LNTxBuffer[3] = 0xA8 ^ 0x30 ^ 0x0B ^ 0xFF;       // Checksum = 0x43 = invert (0xBF xor 0x00 xor  0x03)
    LNTxBufferL = 4;            // 4 bytes to send
    EEPROM.write(EEPROM_CFG,0x50);    // write self test done to eeprom
    EEPROM.commit();
    while (LNTxBufferL) {}
    LNTxBuffer[0] = 0xA8;       // OPC_LOCO_ADR         0xBF    ;REQ  loco ADR
    LNTxBuffer[1] = 0x30;       // address 00 03
    LNTxBuffer[2] = 0x0C;       // address 00 03
    LNTxBuffer[3] = 0xA8 ^ 0x30 ^ 0x0C ^ 0xFF;       // Checksum = 0x43 = invert (0xBF xor 0x00 xor  0x03)
    LNTxBufferL = 4;            // 4 bytes to send
  }
}

void decodeWaitAddress() {
  NewPacket = 0;
  // E7 0E 01 30 03 00 00 07 00 00 00 00 00 23
  if ((RXOPCODE == 0xE7) && (LNRxBuffer[0] == 0x0E) && (LNRxBuffer[1] == 0)) {   // Slot #0 = accept command station type
    TRK = LNRxBuffer[6];              // command station status
    LNvU = ((LNRxBuffer[10] == 0x49) && (LNRxBuffer[11] == 0x42));       // Uhlenbrock type command stations identify themselves as "IB"
  }

  if (WaitSlot && (RXOPCODE == 0xB4) && (LNRxBuffer[0] == 0x3A)) {      // wait for dispatch, but received LACK
    WaitSlot = 0;
  }

  if (WaitSlot && (RXOPCODE == 0xE7) && (LNRxBuffer[0] == 0x0E) && ((LNRxBuffer[2] && 0x30) > 0x00)) {
    WaitSlot = 0;
    LOC_SLOT = LNRxBuffer[1];             // SLOT#
    MY_AL = LNRxBuffer[3];               // Address Low
    MY_AH = LNRxBuffer[8];               // Address High
    LOC_DIRF = LNRxBuffer[5];             // Dir & F0 to F4
    LOC_SND = LNRxBuffer[9];              // F5 to F8
    //F9F16;            // F9 to F16
    LOC_STAT = LNRxBuffer[2];             // slot status
    MY_SS = LOC_STAT;
    TRK = LNRxBuffer[6];              // command station status

    EEPROM.write(EEPROM_ADR1,MY_AH);    // loco address high byte. Address 00:00 mean no address, bit 7 mean no address
    EEPROM.write(EEPROM_ADR2,MY_AL);    // loco address low byte. Address 00:00 mean no address, bit 7 mean no address
    EEPROM.write(EEPROM_SS,MY_SS);    // Loco speed steps - low 3 bits from SLOT STATUS1; bit 7 mean no address
    EEPROM.commit();                 // update EEPROM;

    LastBlinkMillis = millis();      // refresh timers ...
    LongRefreshMillis = millis();
    if ((LNRxBuffer[10] == 0) && (LNRxBuffer[11] == 0)) {
      MainState = 2;        // ID 00:00 -> update my ID
      SubMenu = 2;
    } else if ((LNRxBuffer[10] == MY_ID1) && (LNRxBuffer[11] == MY_ID2)) {
      MainState = 2;        // ID = MY_ID -> check status
      SubMenu = 3;
    } else {
      MainState = 2;        // ID != MY_ID -> blink for update
      SubMenu = 0x0E;
    }

  }
  //if (WaitSlot && (RXOPCODE == 0xB4) && (LNRxBuffer[0] == 0x3A)) { WaitSlot = 0; }  // received LACK instead of slot => dispatch failed, ignore and do nothing
  WaitSlot = 0;   // not my slot -> some noise (including different communication)
}

void WaitAddressLoop() {
  if (millis() - LastKeyPotMillis >= 19) {
    LastKeyPotMillis += 19;
    ///////////// Keyboard section //////////
    uint32_t ReadGPIO = ((GPIOB->INDR) << 16) | GPIOA->INDR;
    ReadGPIO = 0xFFFFFFFF ^ ReadGPIO;
    ReadGPIO = ReadGPIO & 0xC29EB4;         // mask for buttons and dir switch
    if ((WaitKeyRelease) && !(ReadGPIO & 0x829EB4)) {
      WaitKeyRelease=0;
    }
    if (!(WaitKeyRelease) && (ReadGPIO == 0x820000)) {  // PB1 = Shift + PB7 = ESTOP
      WaitKeyRelease = 0xFF;    // release of double key
      while (LNTxBufferL) {}
      LNTxBuffer[0] = 0xBA;       // OPC_MOVE_SLOTS         0xBA    ;MOVE slot SRC to DEST
      LNTxBuffer[1] = 0x00;       // SRC = 0 -> DISPATCH GET
      LNTxBuffer[2] = 0x00;       // DEST = 0 -> do not cate
      LNTxBuffer[3] = 0xBA ^ 0x00 ^ 0x00 ^ 0xFF;       // Checksum
      LNTxBufferL = 4;            // 4 bytes to send
      WaitSlot = 1;               // mark I'm waiting for slot
    }
    if (digitalRead(PA0) == LOW) {MainState=3;}     // power down
  }
    ///////////// LocoNet section //////////
  if (NewPacket) {
    decodeWaitAddress();
  }

}