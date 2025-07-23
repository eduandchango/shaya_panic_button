#include <EEPROM.h>
#include <TinyLoRa.h>
#include <SPI.h>
#include <LowPower.h> // Manages sleep modes and peripheral power down
#include <Wire.h>
#include <DS3232RTC.h>

// Define DEBUG_MODE to enable Serial prints. Comment out for deployment.
//#define DEBUG_MODE

// --- Pin Definitions ---
// Confirmed for LoRa32U4II board from provided pinout:
// Arduino Pin 11 (PB7) -> PCINT7
// Arduino Pin 10 (PB6) -> PCINT6
// Arduino Pin 13 (PC7) -> Not a PCINT pin, but suitable for LED
// Arduino Pin A9 (PB5) -> PCINT5 (used as Analog Input)

#define BUTTON_PIN 11     // Connected to PB7, which is PCINT7 on ATmega32U4
#define RTC_INT_PIN 10    // Connected to PB6, which is PCINT6 on ATmega32U4
#define LED_PIN 13        // Onboard LED, confirmed as PC7 (distinct from BUTTON_PIN 11)
#define OPTO_PIN 12       // Digital output for external control (e.g., controlling a sensor power)
#define VBAT_PIN A9       // Analog input for battery voltage

// --- LoRa Session Keys (ABP Mode) ---
// These keys are specific to your LoRaWAN application and device.
// REPLACE WITH YOUR OWN KEYS FOR DEPLOYMENT!
uint8_t NwkSkey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t AppSkey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t DevAddr[4]  = {0x00, 0x00, 0x00, 0x00};

// TinyLoRa instance: (NSS_PIN, DIO0_PIN, RST_PIN)
// Verify these pins match your LoRa module's connection to the 32U4.
TinyLoRa lora = TinyLoRa(7, 8, 4);
DS3232RTC rtc;

// --- Volatile Flags for Interrupts ---#include <EEPROM.h>
#include <TinyLoRa.h>
#include <SPI.h>
#include <LowPower.h> // Manages sleep modes and peripheral power down
#include <Wire.h>
#include <DS3232RTC.h>

// Define DEBUG_MODE to enable Serial prints. Comment out for deployment.
//#define DEBUG_MODE

// --- Pin Definitions ---
// Confirmed for LoRa32U4II board from provided pinout:
// Arduino Pin 11 (PB7) -> PCINT7
// Arduino Pin 10 (PB6) -> PCINT6
// Arduino Pin 13 (PC7) -> Not a PCINT pin, but suitable for LED
// Arduino Pin A9 (PB5) -> PCINT5 (used as Analog Input)

#define BUTTON_PIN 11     // Connected to PB7, which is PCINT7 on ATmega32U4
#define RTC_INT_PIN 10    // Connected to PB6, which is PCINT6 on ATmega32U4
#define LED_PIN 13        // Onboard LED, confirmed as PC7 (distinct from BUTTON_PIN 11)
#define OPTO_PIN 12       // Digital output for external control (e.g., controlling a sensor power)
#define VBAT_PIN A9       // Analog input for battery voltage

// --- LoRaWAN Configuration Constants ---
// LoRaWAN ABP Mode Session Keys and Device Address.
// !!! IMPORTANT: REPLACE THESE WITH YOUR OWN KEYS FOR DEPLOYMENT !!!
// These are unique to your device and application on the LoRaWAN Network Server.
uint8_t NwkSkey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t AppSkey[16] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t DevAddr[4]  = {0x00, 0x00, 0x00, 0x00};

// TinyLoRa instance: (NSS_PIN, DIO0_PIN, RST_PIN)
// Verify these pins match your LoRa module's connection to the 32U4.
TinyLoRa lora = TinyLoRa(7, 8, 4);
DS3232RTC rtc;

// --- Volatile Flags for Interrupts ---
// These flags are set in the ISR and checked in the main loop after waking.
// 'volatile' keyword ensures the compiler doesn't optimize away reads/writes
// to these variables, as they can change unexpectedly by an ISR.
volatile bool buttonWakeFlag = false;
volatile bool rtcWakeFlag = false;

// --- Global Variables ---
uint16_t frameCounter = 0; // LoRaWAN uplink frame counter, stored in EEPROM
// Payload for LoRaWAN message: [Battery%, Label_byte1, Label_byte2, Label_byte3, Label_byte4, Label_byte5]
uint8_t loraData[6];

// --- Configuration Constants ---
const uint8_t RTC_ALARM_HOUR = 6; // RTC alarm set for 6 AM (06:00:00)
const int LORA_SEND_REPETITIONS = 3; // Number of times to send LoRa message per wake-up event
const unsigned long BUTTON_LED_BLINK_DURATION_MS = 120000UL; // 2 minutes (120,000 milliseconds)

// --- Interrupt Service Routine (ISR) ---
// PCINT0_vect handles all Pin Change Interrupts on Port B (PCINT0 to PCINT7) for ATmega32U4.
// This ISR will be triggered by a change on any pin enabled in PCMSK0.
ISR(PCINT0_vect) {
  // Briefly flash LED to confirm ISR entry. This is very short to minimize time in ISR.
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(100); // Minimal delay
  digitalWrite(LED_PIN, LOW);

  // Check BUTTON_PIN (11), which is PB7, corresponding to PCINT7.
  // Assuming button pulls pin LOW when pressed (with INPUT_PULLUP enabled).
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonWakeFlag = true;
    // Disable this specific PCINT bit to prevent re-triggering during handling
    // in the main loop. It will be re-enabled after processing.
    PCMSK0 &= ~(1 << PCINT7);
  }
  // Check RTC_INT_PIN (10), which is PB6, corresponding to PCINT6.
  // DS3232RTC INT/SQW pin is open-drain active-low, so it pulls LOW on alarm.
  if (digitalRead(RTC_INT_PIN) == LOW) {
    rtcWakeFlag = true;
    // Disable this specific PCINT bit to prevent re-triggering.
    PCMSK0 &= ~(1 << PCINT6);
  }
}

// --- Setup Function ---
void setup() {
#ifdef DEBUG_MODE
  Serial.begin(9600);
  delay(2000); // Give time for Serial monitor to connect, especially after USB re-enumeration on 32U4.
  Serial.println("\n--- Device Setup Starting ---");
#endif

  // Configure pin modes and initial states.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OPTO_PIN, OUTPUT); // Ensure OPTO_PIN is set as an OUTPUT
  digitalWrite(LED_PIN, LOW);
  digitalWrite(OPTO_PIN, LOW); // Ensure opto output is off initially

  // Load frame counter from EEPROM.
  EEPROM.get(0, frameCounter);
#ifdef DEBUG_MODE
  Serial.print("FrameCounter loaded from EEPROM: ");
  Serial.println(frameCounter);
#endif

  // Initialize LoRa module.
  lora.setChannel(MULTI); // Set to multi-channel mode
  lora.setDatarate(SF10BW125); // Set data rate (Spread Factor 10, Bandwidth 125kHz)
  if (!lora.begin()) {
#ifdef DEBUG_MODE
    Serial.println("LoRa init failed! Check wiring and module connections.");
#endif
    // Halt execution if LoRa fails to initialize, as it's a critical component.
    while (true) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  lora.setPower(17); // Set transmit power (e.g., 17 dBm)
#ifdef DEBUG_MODE
  Serial.println("LoRa initialized successfully.");
#endif

  // Initialize RTC module.
  rtc.begin();
  // Set Alarm 1 to trigger at RTC_ALARM_HOUR every day.
  // Format: (alarm_type, seconds, minutes, hours, day_of_week/day_of_month)
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, 0, 0, RTC_ALARM_HOUR, 0);
  rtc.alarmInterrupt(1, true); // Enable Alarm 1 interrupt output.
#ifdef DEBUG_MODE
  Serial.print("RTC Alarm 1 set for daily at ");
  Serial.print(RTC_ALARM_HOUR);
  Serial.println(":00:00.");
#endif

  // --- Configure Pin Change Interrupts for ATmega32U4 (Port B) ---
  // PCICR (Pin Change Interrupt Control Register):
  // PCIE0 (Pin Change Interrupt Enable 0) enables the PCINT0_vect ISR for PCINT0-PCINT7 (Port B pins).
  PCICR |= (1 << PCIE0);

  // PCMSK0 (Pin Change Mask Register 0):
  // Set the bits corresponding to the PCINT pins you want to enable.
  // PCINT7 for BUTTON_PIN (Arduino Pin 11 / ATmega32U4 PB7)
  // PCINT6 for RTC_INT_PIN (Arduino Pin 10 / ATmega32U4 PB6)
  PCMSK0 |= (1 << PCINT7) | (1 << PCINT6);

  sei(); // Enable global interrupts.

  // Clear any pending PCINT flags before entering the loop to prevent immediate false wake-ups.
  // Writing 1 to a PCIFR bit clears the corresponding flag.
  PCIFR |= (1 << PCIF0);
#ifdef DEBUG_MODE
  Serial.println("Pin Change Interrupts configured and enabled.");
  Serial.println("--- Setup Complete ---");
#endif
}

// --- Main Loop ---
void loop() {
#ifdef DEBUG_MODE
  Serial.println("Entering power-down sleep...");
#endif

  // Enter the deepest sleep mode (Power-down).
  // In this mode, most clocks are stopped, and the MCU consumes minimal power.
  // It will wake up only on an external interrupt (like PCINT).
  // ADC_OFF and BOD_OFF further reduce power consumption.
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  // --- MCU Woke Up! ---
  // Execution resumes here after an interrupt.

  // Disable interrupts temporarily to safely read and reset volatile flags.
  noInterrupts();
  bool btnWake = buttonWakeFlag;
  bool alarmWake = rtcWakeFlag;
  buttonWakeFlag = false; // Reset flag
  rtcWakeFlag = false;    // Reset flag
  interrupts();           // Re-enable interrupts

  // --- Handle Wake-up Events ---
  if (btnWake) {
    handleButtonWake();
  }
  if (alarmWake) {
    handleRTCWake();
  }
}

// --- Function to Handle Button Wake-up ---
void handleButtonWake() {
#ifdef DEBUG_MODE
  Serial.println("Woke up by Button! Processing event...");
#endif

  // ACTIVATE OPTO_PIN and LED for immediate feedback and external control.
  digitalWrite(OPTO_PIN, HIGH);
  digitalWrite(LED_PIN, HIGH);

  // Increment frame counter ONCE per wake-up event and save to EEPROM immediately.
  frameCounter++;
  EEPROM.put(0, frameCounter);
#ifdef DEBUG_MODE
  Serial.print("FrameCounter updated for this button event: ");
  Serial.println(frameCounter);
#endif

  // Send LoRaWAN message multiple times for reliability.
  for (int i = 0; i < LORA_SEND_REPETITIONS; i++) {
#ifdef DEBUG_MODE
    Serial.print("Sending LoRa message (Button) "); Serial.print(i + 1); Serial.print(" of "); Serial.println(LORA_SEND_REPETITIONS);
#endif
    // The sendLoRaMessage function uses the current global frameCounter
    sendLoRaMessage("00000"); // Send with a specific label for button wake-up
    delay(1000);             // Short delay between sends
  }
#ifdef DEBUG_MODE
  Serial.println("LoRa messages sent for button event.");
#endif

  // Longer LED Blink for visual indication that the button event was processed.
#ifdef DEBUG_MODE
  Serial.println("Starting 2-minute LED blink sequence.");
#endif
  unsigned long startBlink = millis();
  while (millis() - startBlink < BUTTON_LED_BLINK_DURATION_MS) {
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
#ifdef DEBUG_MODE
  Serial.println("LED blink sequence complete.");
#endif

  // DEACTIVATE OPTO_PIN and LED.
  digitalWrite(OPTO_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // Wait for the button to be released before re-enabling its interrupt.
  // This prevents immediate re-triggering if the button is held down.
#ifdef DEBUG_MODE
  Serial.println("Waiting for button release...");
#endif
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(10); // Small delay to prevent busy-waiting from consuming too much power
  }
  delay(50); // Small debounce delay after the button is released.
#ifdef DEBUG_MODE
  Serial.println("Button released. Re-enabling interrupt.");
#endif

  // Re-enable the button's Pin Change Interrupt (PCINT7 for pin 11).
  PCMSK0 |= (1 << PCINT7);
#ifdef DEBUG_MODE
  Serial.println("Button interrupt re-enabled. Returning to sleep.");
#endif
}

// --- Function to Handle RTC Alarm Wake-up ---
void handleRTCWake() {
#ifdef DEBUG_MODE
  Serial.println("Woke up by RTC Alarm! Processing event...");
#endif

  // Increment frame counter ONCE per wake-up event and save to EEPROM immediately.
  frameCounter++;
  EEPROM.put(0, frameCounter);
#ifdef DEBUG_MODE
  Serial.print("FrameCounter updated for this RTC event: ");
  Serial.println(frameCounter);
#endif

  // Send LoRaWAN message multiple times for reliability.
  for (int i = 0; i < LORA_SEND_REPETITIONS; i++) {
#ifdef DEBUG_MODE
    Serial.print("Sending LoRa message (RTC) "); Serial.print(i + 1); Serial.print(" of "); Serial.println(LORA_SEND_REPETITIONS);
#endif
    sendLoRaMessage("00000"); // Send with a specific label for RTC wake-up
    delay(1000);             // Short delay between sends
  }
#ifdef DEBUG_MODE
  Serial.println("LoRa messages sent for RTC event.");
#endif

  // Clear the current RTC alarm flag and re-set the alarm for the next occurrence.
  rtc.clearAlarm(1); // Clears the alarm flag in the DS3232RTC module.
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, 0, 0, RTC_ALARM_HOUR, 0); // Re-set for daily at RTC_ALARM_HOUR.
  rtc.alarmInterrupt(1, true); // Re-enable the RTC alarm interrupt output.
#ifdef DEBUG_MODE
  Serial.println("RTC Alarm cleared and re-set.");
#endif

  // Re-enable the RTC's Pin Change Interrupt (PCINT6 for pin 10).
  PCMSK0 |= (1 << PCINT6);
#ifdef DEBUG_MODE
  Serial.println("RTC interrupt re-enabled. Returning to sleep.");
#endif
}

// --- Function to Send LoRaWAN Message ---
void sendLoRaMessage(const char* label) {
#ifdef DEBUG_MODE
  Serial.println("Inside sendLoRaMessage()...");
#endif
  float batteryVoltage = readBattery();
  // Constrain battery voltage to a 0-100% range.
  // Adjust 3.3V (min) and 4.2V (max) based on your specific battery type (e.g., LiPo min/max).
  // This converts the raw voltage to a percentage.
  uint8_t batteryPercent = constrain(map(batteryVoltage * 100, 330, 420, 0, 100), 0, 100);

#ifdef DEBUG_MODE
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage, 2); // Print with 2 decimal places
  Serial.print(" V (");
  Serial.print(batteryPercent);
  Serial.println(" %)");
  Serial.print("LoRa Payload Label: ");
  Serial.println(label);
#endif

  loraData[0] = batteryPercent; // First byte of payload is battery percentage.
  // Copy the label (e.g., "00000", "00000") into the next 5 bytes of the payload.
  // Ensure the label is exactly 5 characters long or adjust memcpy size.
  memcpy(&loraData[1], label, 5);

#ifdef DEBUG_MODE
  Serial.println("Calling lora.sendData()...");
#endif
  lora.sendData(loraData, sizeof(loraData), frameCounter);
#ifdef DEBUG_MODE
  Serial.println("LoRa message sent.");
#endif
}

// --- Function to Read Battery Voltage ---
float readBattery() {
  // Read analog value from VBAT_PIN (A9).
  int raw = analogRead(VBAT_PIN);
  // Calculate actual voltage.
  // (raw_ADC_value * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION) * VOLTAGE_DIVIDER_CORRECTION_FACTOR
  // Assuming a 3.3V AREF for a 3.3V ATmega32U4 board (like LoRa32U4II).
  // The 1.3013 factor is your voltage divider ratio, calibrated with your measurements.
  // !!! IMPORTANT: VERIFY AND CALIBRATE THIS FACTOR (1.3013) !!!
  // Measure your battery voltage with a multimeter and compare it to the value
  // reported by this function. Adjust 1.3013 until they match.
  return (raw * 3.3 * 1.3013) / 1024.0;
}
