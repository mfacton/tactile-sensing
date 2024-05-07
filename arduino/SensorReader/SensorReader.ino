#define NUM_ADC 9
#define BAUD 115200
#define DELAY_MS 5

const uint8_t pins[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8 };
uint16_t vals[NUM_ADC];
uint8_t buf[NUM_ADC*2];

unsigned long targetTime;

void setup() {
  for (int i = 0; i < NUM_ADC; i++) {
    pinMode(pins[i], INPUT);
  }

  Serial.begin(BAUD);
  while (!Serial);
  targetTime = millis();
}

void loop() {
  // Read pins
  for (int i = 0; i < NUM_ADC; i++) {
    vals[i] = analogRead(pins[i]);
  }

  //load data
  memcpy(buf, vals, NUM_ADC*2);

  // Wait until 10 ms has passed
  while (micros()/1000 < targetTime);
  targetTime += DELAY_MS;

  // Write data
  Serial.write(buf, NUM_ADC*2);
}
