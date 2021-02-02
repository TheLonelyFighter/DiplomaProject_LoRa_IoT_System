// Read Atmega328P internal temperature sensor //
long read_temp()
{
  // Read temperature sensor against 1.1V reference
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX3);
  // Start AD conversion
  ADCSRA |= _BV(ADEN) | _BV(ADSC);
  // Detect end-of-conversion
  while (bit_is_set(ADCSRA,ADSC));
  // return raw data
  return ADCL | (ADCH << 8);
}
 
// Convert raw temperature data to °C
double conv_temp(long raw_temp)
{
  // f(x) = (raw - offset) / coeff
  return((raw_temp - 324.31) / 1.22);
}
 
void setup()
{
  Serial.begin(115200);
}
 
void loop()
{
  Serial.println(conv_temp(read_temp()), 1);
  delay(500);
}
