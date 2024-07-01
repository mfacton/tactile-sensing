#define NUM_ADC 12
#define BAUD 115200
#define DELAY_MS 20

const uint8_t pins[] = { A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11 };
uint16_t vals[NUM_ADC];
uint8_t buf[NUM_ADC*2+2] = {'\r'};

unsigned long targetTime;

void setup() {
  for (int i = 0; i < NUM_ADC; i++) {
    pinMode(pins[i], INPUT);
  }

  buf[NUM_ADC*2] = '\n';

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
  while (millis() < targetTime);
  targetTime += DELAY_MS;

  // Write data
  Serial.write(buf, NUM_ADC*2+2);
}
