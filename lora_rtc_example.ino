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

// --- Volatile Flags for Interrupts ---
// These flags are set in the ISR and checked in the main loop after waking.
volatile bool buttonWakeFlag = false;
volatile bool rtcWakeFlag = false;

// --- Global Variables ---
uint16_t frameCounter = 0; // LoRaWAN uplink frame counter, stored in EEPROM
uint8_t loraData[6];       // Payload for LoRaWAN message: [Battery%, Label_byte1, Label_byte2, Label_byte3, Label_byte4, Label_byte5]

// --- Interrupt Service Routine (ISR) ---
// PCINT0_vect handles all Pin Change Interrupts on Port B (PCINT0 to PCINT7) for ATmega32U4.
// This ISR will be triggered by a change on any pin enabled in PCMSK0.
ISR(PCINT0_vect) {
  // Briefly flash LED to confirm ISR entry (very short to minimize power in ISR).
  digitalWrite(LED_PIN, HIGH);
  delayMicroseconds(100);
  digitalWrite(LED_PIN, LOW);

  // Check BUTTON_PIN (11), which is PB7, corresponding to PCINT7.
  // Assuming button pulls pin LOW when pressed (with INPUT_PULLUP enabled).
  if (digitalRead(BUTTON_PIN) == LOW) {
    buttonWakeFlag = true;
    // Disable this specific PCINT bit to prevent re-triggering during handling.
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
#endif

  // Configure pin modes and initial states.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OPTO_PIN, OUTPUT); // Ensure OPTO_PIN is set as an OUTPUT
  digitalWrite(LED_PIN, LOW);
  digitalWrite(OPTO_PIN, LOW);

  // Load frame counter from EEPROM.
  EEPROM.get(0, frameCounter);
#ifdef DEBUG_MODE
  Serial.print("FrameCounter loaded: ");
  Serial.println(frameCounter);
#endif

  // Initialize LoRa module.
  lora.setChannel(MULTI);
  lora.setDatarate(SF10BW125);
  if (!lora.begin()) {
#ifdef DEBUG_MODE
    Serial.println("LoRa init failed! Check wiring and module connections.");
#endif
    while (true); // Halt execution if LoRa fails to initialize.
  }
  lora.setPower(17);
#ifdef DEBUG_MODE
  Serial.println("LoRa initialized");
#endif

  // Initialize RTC module.
  rtc.begin();
  // Set Alarm 1 to trigger at 21:00:00 every day.
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, 0, 0, 21, 0);
  rtc.alarmInterrupt(1, true); // Enable Alarm 1 interrupt output.

  // --- Configure Pin Change Interrupts for ATmega32U4 (Port B) ---
  // PCICR (Pin Change Interrupt Control Register):
  // PCIE0 (Pin Change Interrupt Enable 0) enables the PCINT0_vect ISR for PCINT0-PCINT7 (Port B pins).
  PCICR |= (1 << PCIE0);

  // PCMSK0 (Pin Change Mask Register 0):
  // Set the bits corresponding to the PCINT pins you want to enable.
  // PCINT7 for BUTTON_PIN (11 / PB7)
  // PCINT6 for RTC_INT_PIN (10 / PB6)
  PCMSK0 |= (1 << PCINT7) | (1 << PCINT6);

  sei(); // Enable global interrupts.

  // Clear any pending PCINT flags before entering the loop to prevent immediate false wake-ups.
  PCIFR |= (1 << PCIF0); // Write 1 to clear the flag for PCINT0_vect
}

// --- Main Loop ---
void loop() {
#ifdef DEBUG_MODE
  // Debugging: Confirm PCINT registers before sleep
  Serial.print("DEBUG: PCICR before sleep: 0x"); Serial.println(PCICR, HEX);
  Serial.print("DEBUG: PCMSK0 before sleep: 0x"); Serial.println(PCMSK0, HEX);

  // Debugging: Quick LED blink to confirm loop is running before sleep
  digitalWrite(LED_PIN, HIGH);
  delay(50);
  digitalWrite(LED_PIN, LOW);
  delay(50);

  Serial.println("Entering power-down sleep...");
#endif

  // Removed lora.sleep() as it's not supported by this TinyLoRa library version.
  // The LoRa module will remain in its default state during MCU sleep.

  // Enter the deepest sleep mode (Power-down).
  // In this mode, most clocks are stopped, and the MCU consumes minimal power.
  // It will wake up only on an external interrupt (like PCINT).
  LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF);

  // --- MCU Woke Up! ---
  // Removed lora.wake() as it's not supported by this TinyLoRa library version.
  // The LoRa module will resume normal operation when the MCU wakes.

  // Disable interrupts temporarily to safely read and reset volatile flags.
  noInterrupts();
  bool btnWake = buttonWakeFlag;
  bool alarmWake = rtcWakeFlag;
  buttonWakeFlag = false;
  rtcWakeFlag = false;
  interrupts();

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
  Serial.println("Woke up by Button! Entering handleButtonWake()...");
#endif

  // ACTIVATE OPTO_PIN
  digitalWrite(OPTO_PIN, HIGH); // Activate opto output for the duration of the activity.
  digitalWrite(LED_PIN, HIGH);  // Turn on LED for immediate visual feedback.

  // --- Reliability: Increment frame counter ONCE per wake-up, then send multiple times ---
  frameCounter++; // Increment frame counter for this new event
  EEPROM.put(0, frameCounter); // Save updated frame counter to EEPROM
#ifdef DEBUG_MODE
  Serial.print("DEBUG: FrameCounter updated for this event: ");
  Serial.println(frameCounter);
#endif

  const int repetitions = 2; // Define how many times to send the message

  for (int i = 0; i < repetitions; i++) {
#ifdef DEBUG_MODE
    Serial.print("DEBUG: Sending LoRa message "); Serial.print(i + 1); Serial.print(" of "); Serial.println(repetitions);
#endif
    // The sendLoRaMessage function uses the current global frameCounter
    sendLoRaMessage("00000"); // Send with the fixed "00000" label
#ifdef DEBUG_MODE
    Serial.println("DEBUG: After sendLoRaMessage() call.");
#endif
    delay(1000);             // Short delay between sends
#ifdef DEBUG_MODE
    Serial.println("DEBUG: After delay(1000).");
#endif
  }
#ifdef DEBUG_MODE
  Serial.println("DEBUG: LoRa messages sent. Exiting loop.");
#endif

  // --- REINSTATED: Longer LED Blink for 2 minutes ---
#ifdef DEBUG_MODE
  Serial.println("DEBUG: Starting 2-minute LED blink sequence.");
#endif
  unsigned long startBlink = millis();
  while (millis() - startBlink < 120000UL) { // 120,000 milliseconds = 2 minutes
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
  }
#ifdef DEBUG_MODE
  Serial.println("DEBUG: LED blink sequence complete (2 minutes).");
#endif

  // DEACTIVATE OPTO_PIN
  digitalWrite(OPTO_PIN, LOW); // Deactivate opto output.

  // Wait for the button to be released before re-enabling its interrupt.
#ifdef DEBUG_MODE
  Serial.println("DEBUG: Waiting for button release...");
#endif
  while (digitalRead(BUTTON_PIN) == LOW) {
    delay(1); // Small delay to prevent busy-waiting from consuming too much power
  }
  delay(50); // Small debounce delay after the button is released.
#ifdef DEBUG_MODE
  Serial.println("DEBUG: Button released.");
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
  Serial.println("Woke up by RTC Alarm! Entering handleRTCWake()...");
#endif

  digitalWrite(LED_PIN, HIGH); // Briefly turn on LED for RTC alarm indication.

  // --- Reliability: Increment frame counter ONCE per wake-up, then send multiple times ---
  frameCounter++; // Increment frame counter for this new event
  EEPROM.put(0, frameCounter); // Save updated frame counter to EEPROM
#ifdef DEBUG_MODE
  Serial.print("DEBUG: FrameCounter updated for this event: ");
  Serial.println(frameCounter);
#endif

  const int repetitions = 2; // Define how many times to send the message

  for (int i = 0; i < repetitions; i++) {
#ifdef DEBUG_MODE
    Serial.print("DEBUG: Sending LoRa message "); Serial.print(i + 1); Serial.print(" of "); Serial.println(repetitions);
#endif
    sendLoRaMessage("00000"); // Send with the fixed "00000" label
#ifdef DEBUG_MODE
    Serial.println("DEBUG: After sendLoRaMessage() call.");
#endif
    delay(1000);             // Short delay between sends
#ifdef DEBUG_MODE
    Serial.println("DEBUG: After delay(1000).");
#endif
  }
#ifdef DEBUG_MODE
  Serial.println("DEBUG: LoRa messages sent. Exiting loop.");
#endif

  digitalWrite(LED_PIN, LOW);  // Turn off LED after sending messages.

  // Update and save the LoRaWAN frame counter.
  // This was moved to the beginning of the function, before the send loop.

  // Clear the current RTC alarm flag and re-set the alarm for the next occurrence.
  rtc.clearAlarm(1); // Clears the alarm flag in the DS3232RTC module.
  rtc.setAlarm(DS3232RTC::ALM1_MATCH_HOURS, 0, 0, 21, 0); // Re-set for 21:00:00 daily.
  rtc.alarmInterrupt(1, true); // Re-enable the RTC alarm interrupt output.

  // Re-enable the RTC's Pin Change Interrupt (PCINT6 for pin 10).
  PCMSK0 |= (1 << PCINT6);
#ifdef DEBUG_MODE
  Serial.println("RTC interrupt re-enabled. Returning to sleep.");
#endif
}

// --- Function to Send LoRaWAN Message ---
void sendLoRaMessage(const char* label) {
#ifdef DEBUG_MODE
  Serial.println("DEBUG: Inside sendLoRaMessage()...");
#endif
  float batteryVoltage = readBattery();
  // Constrain battery voltage (e.g., 3.2V to 4.2V) to a 0-100% range.
  // Adjust 3.2 and 4.2V based on your battery type (e.g., LiPo min/max).
  uint8_t batteryPercent = constrain((batteryVoltage - 3.2) / (4.2 - 3.2) * 100, 0, 100);

#ifdef DEBUG_MODE
  Serial.print("Battery Voltage: ");
  Serial.print(batteryPercent); // Changed to print percentage directly
  Serial.print(" %, Raw Voltage: ");
  Serial.print(batteryVoltage);
  Serial.println(" V");

  Serial.println("DEBUG: Preparing LoRa payload...");
#endif
  loraData[0] = batteryPercent; // First byte of payload is battery percentage.
  // Copy the label (e.g., "BTN01", "AWK01") into the next 5 bytes of the payload.
  memcpy(&loraData[1], label, 5); // Copy 5 characters for the label

#ifdef DEBUG_MODE
  Serial.println("DEBUG: Calling lora.sendData()...");
#endif
  lora.sendData(loraData, sizeof(loraData), frameCounter);
#ifdef DEBUG_MODE
  Serial.println("LoRa message sent.");
  Serial.println("DEBUG: Exiting sendLoRaMessage().");
#endif
}

// --- Function to Read Battery Voltage ---
float readBattery() {
  // Read analog value from VBAT_PIN (A9).
  int raw = analogRead(VBAT_PIN);
  // Calculate actual voltage.
  // (raw_ADC_value * ADC_REFERENCE_VOLTAGE / ADC_RESOLUTION) * VOLTAGE_DIVIDER_CORRECTION_FACTOR
  // Assuming a 3.3V AREF for a 3.3V ATmega32U4 board.
  // The 1.286 factor is your voltage divider ratio.
  // VERIFY THIS FACTOR based on your physical voltage resistors.
  return (raw * 3.3 * 1.286) / 1024.0;
}