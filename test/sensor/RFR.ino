// One single array/ pins for BOTH MUX modules (short circuited)

const int sharedSelectPins[4] = {14, 27, 26, 25};

// Individual signal lines
const int sensorSignalPin = 12;
const int ledSignalPin = 34;

// CALIBRATION
const int SENSOR_MIN = 200;  // Ambient/Dark threshold (LED off)
const int SENSOR_MAX = 4000; // Maximum reflection threshold (LED fully bright)

// Global array converted to int to store raw analog values (0 to 4095)
int sensorStates[16] = {0};

// FUNCTION DECLARATIONS
void initMux(const int selectPins[4], int signalPin, bool isOutput);
void setMuxAddress(const int selectPins[4], byte channel);
int readSensorChannel(const int selectPins[4], int signalPin, byte channel); // Changed to int for analog
void writeLedChannel(const int selectPins[4], int signalPin, byte channel, bool state);

void setup()
{
  Serial.begin(115200);

  // Initialize MUX hardware routing
  initMux(sharedSelectPins, sensorSignalPin, false);
  initMux(sharedSelectPins, ledSignalPin, true);

  Serial.println("Shared MUX Analog Debugger Initialized...");
}

void loop()
{
  for (byte channel = 0; channel < 16; channel++)
  {
    // Read the current IR sensor analog value (0 - 4095)
    int analogVal = readSensorChannel(sharedSelectPins, sensorSignalPin, channel);
    sensorStates[channel] = analogVal; // Save to global array

    // Map the analog raw value to a specific timing window in microseconds
    // More reflection = longer ON time = brighter looking LED
    int ledOnTime = map(analogVal, SENSOR_MIN, SENSOR_MAX, 0, 1000);
    ledOnTime = constrain(ledOnTime, 0, 1000); // Protect against boundary errors
    int ledOffTime = 1000 - ledOnTime;

    if (ledOnTime > 0)
    {
      writeLedChannel(sharedSelectPins, ledSignalPin, channel, HIGH);
      delayMicroseconds(ledOnTime);
    }

    if (ledOffTime > 0)
    {
      writeLedChannel(sharedSelectPins, ledSignalPin, channel, LOW);
      delayMicroseconds(ledOffTime);
    }
  }

  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500)
  {
    Serial.println("\n--- Analog Values [0-15] ---");
    for (int i = 0; i < 16; i++)
    {
      Serial.print(sensorStates[i]);
      Serial.print(" ");

      if ((i + 1) % 4 == 0)
      {
        Serial.println(); // Print format cleaner in 4x4 grid
      }
    }

    // Testing with only 1 IR + LED now (on CH: 06)

    lastPrint = millis();
  }
}

// FUNCTION DEFINITIONS

void initMux(const int selectPins[4], int signalPin, bool isOutput)
{
  for (int i = 0; i < 4; i++)
  {
    pinMode(selectPins[i], OUTPUT);
    digitalWrite(selectPins[i], LOW);
  }

  if (isOutput)
  {
    pinMode(signalPin, OUTPUT);
    digitalWrite(signalPin, LOW);
  }
  else
  {
    pinMode(signalPin, INPUT);
  }
}

void setMuxAddress(const int selectPins[4], byte channel)
{
  for (int bit = 0; bit < 4; bit++)
  {
    digitalWrite(selectPins[bit], bitRead(channel, bit));
  }
}

int readSensorChannel(const int selectPins[4], int signalPin, byte channel)
{
  setMuxAddress(selectPins, channel);
  delayMicroseconds(15);
  return analogRead(signalPin);
}

void writeLedChannel(const int selectPins[4], int signalPin, byte channel, bool state)
{
  setMuxAddress(selectPins, channel);
  digitalWrite(signalPin, state);
}