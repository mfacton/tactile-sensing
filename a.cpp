#include <Arduino.h>

const size_t num_adc_pins = 9;
const int8_t adc_pins[] = {A0, A1, A2, A3, A4, A5, 8, 6, 4};

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
   t = millis();
   while (t - t_last_loop < 10)
      ;
   for (int i = 0; i < num_adc_pins; i++)
   {
      val = analogRead(adc_pins[i]);
      Serial.print(val);
      if (i < num_adc_pins - 1)
      {
         Serial.print(",");
      }
   }
   Serial.println();

   t_last_loop = t;
}
