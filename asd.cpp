#include <Arduino.h>

const size_t num_adc_pins = 4;
const int8_t adc_pins[] = {A0, A1, A2, A3};

int val = 0;
unsigned long t_last_loop = 0;

void setup()
{
   for (int i = 0; i < num_adc_pins; i++)
   {
      pinMode(adc_pins[i], INPUT);
   }

   Serial.begin(115200);
   while (!Serial)
      ;
}

void loop()
{
   while (millis() - t_last_loop < 1000)
      ;
   for (int i = 0; i < num_adc_pins; i++)
   {
      val = analogRead(adc_pins[i]);
      Serial.print(val);
      Serial.print(",");
   }
   Serial.println();

   t_last_loop = millis();
}
