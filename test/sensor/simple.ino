byte channel = 0; //edit this shit


const int
    sharedSelectPins[4] = {5, 18, 19, 12},
    sensorSignalPin = 34,

    SensorMin = 200,
    SensorMax = 4000;

int sensorState = 0;



void initMux(const int selectPins[4], int signalPin)
{
    for (int i = 0; i < 4; i++)
    {
        pinMode(selectPins[i], OUTPUT);
        digitalWrite(selectPins[i], LOW);
    }

    pinMode(signalPin, INPUT);
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

void setup()
{
    Serial.begin(115200);

    // Initialize MUX hardware routing
    initMux(sharedSelectPins, sensorSignalPin);

    Serial.println("Shared MUX Analog Debugger Initialized...");
}

void loop()
{
    // Read the current IR sensor analog value (0 - 4095)
    int analogVal = readSensorChannel(sharedSelectPins, sensorSignalPin, channel);
    sensorState = analogVal; // Save to global array

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500)
    {
        Serial.print(sensorState);
        Serial.print(" ");

        // Testing with only 1 IR + LED now (on CH: 06)

        lastPrint = millis();
    }
}