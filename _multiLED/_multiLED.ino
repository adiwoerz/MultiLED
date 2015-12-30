/*
LED VU meter for Arduino and Adafruit NeoPixel LEDs.
 
 Hardware requirements:
 - Most Arduino or Arduino-compatible boards (ATmega 328P or better).
 - Adafruit Electret Microphone Amplifier (ID: 1063)
 - Adafruit Flora RGB Smart Pixels (ID: 1260)
 OR
 - Adafruit NeoPixel Digital LED strip (ID: 1138)
 - Optional: battery for portable use (else power through USB or adapter)
 Software requirements:
 - Adafruit NeoPixel library
 
 Connections:
 - 3.3V to mic amp +
 - GND to mic amp -
 - Analog pin to microphone output (configurable below)
 - Digital pin to LED data input (configurable below)
 See notes in setup() regarding 5V vs. 3.3V boards - there may be an
 extra connection to make and one line of code to enable or disable.
 
 Written by Adafruit Industries.  Distributed under the BSD license.
 This paragraph must be included in any redistribution.
 
 fscale function:
 Floating Point Autoscale Function V0.1
 Written by Paul Badger 2007
 Modified from code by Greg Shakar

 Extend to drive multi LED strips
 
 */

#include <Adafruit_NeoPixel.h>
#include <math.h>

#define N_PIXELS1  17  // Number of pixels in strand
#define N_PIXELS2  29  // Number of pixels in strand
#define N_PIXELS3  45  // Number of pixels in strand
#define MIC_PIN   4  // Microphone is attached to this analog pin
#define LED_PIN1    7  // NeoPixel LED strand is connected to this pin
#define LED_PIN2    8  // NeoPixel LED strand is connected to this pin
#define LED_PIN3    9  // NeoPixel LED strand is connected to this pin
#define SAMPLE_WINDOW   2  // Sample window for average level
#define PEAK_HANG 2 //Time of pause before peak dot falls
#define PEAK_FALL 1 //Rate of falling peak dot
#define INPUT_FLOOR 10 //Lower range of analogRead input
#define INPUT_CEILING 200 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)



byte peak = 16;      // Peak level of column; used for falling dots
unsigned int sample;

byte dotCount = 0;  //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot

Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(N_PIXELS1, LED_PIN1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(N_PIXELS2, LED_PIN2, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(N_PIXELS3, LED_PIN3, NEO_GRB + NEO_KHZ800);

void setup() 
{
  // This is only needed on 5V Arduinos (Uno, Leonardo, etc.).
  // Connect 3.3V to mic AND TO AREF ON ARDUINO and enable this
  // line.  Audio samples are 'cleaner' at 3.3V.
  // COMMENT OUT THIS LINE FOR 3.3V ARDUINOS (FLORA, ETC.):
  //  analogReference(EXTERNAL);

  // Serial.begin(9600);
  strip1.begin();
  strip1.show(); // Initialize all pixels to 'off'
  
  strip2.begin();
  strip2.show(); // Initialize all pixels to 'off'

   strip3.begin();
   strip3.show(); // Initialize all pixels to 'off'


}

void loop() 
{
  unsigned long startMillis= millis();  // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level

  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y1, y2, y3;


  // collect data for length of sample window (in mS)
  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    sample = analogRead(MIC_PIN);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
 
  // Serial.println(peakToPeak);


  //Fill the strip with rainbow gradient
  
  for (int i=0;i<=strip1.numPixels()-1;i++)


  
  {
    strip1.setPixelColor(i,Wheel(map(i,0,strip1.numPixels()-1,30,255)));
      for (int i=0;i<=strip2.numPixels()-1;i++)
   //{
    //strip2.setPixelColor(i,Wheel(map(i,0,strip2.numPixels()-1,30,255)));

      //for (int i=0;i<=strip3.numPixels()-1;i++)
   // {
    strip3.setPixelColor(i,Wheel(map(i,0,strip3.numPixels()-1,30,255)));
  }


  //Scale the input logarithmically instead of linearly
  c = fscale(INPUT_FLOOR, INPUT_CEILING, strip1.numPixels(), 0, peakToPeak, 2);
  c = fscale(INPUT_FLOOR, INPUT_CEILING, strip2.numPixels(), 0, peakToPeak, 2);
  c = fscale(INPUT_FLOOR, INPUT_CEILING, strip3.numPixels(), 0, peakToPeak, 2);

  


  if(c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= strip1.numPixels()) { // Fill partial column with off pixels
    drawLine(strip1.numPixels(), strip1.numPixels()-c, strip1.Color(0, 0, 0));
     }
       
     if (c <= strip2.numPixels()) { // Fill partial column with off pixels
    drawLine(strip2.numPixels(), strip2.numPixels()-c, strip2.Color(0, 0, 0));
  }

  if (c <= strip3.numPixels()) { // Fill partial column with off pixels
    drawLine(strip3.numPixels(), strip2.numPixels()-c, strip3.Color(0, 0, 0));
  }



  // Set the peak dot to match the rainbow gradient
  y1 = strip1.numPixels() - peak;
  y2 = strip2.numPixels() - peak;
  y3 = strip3.numPixels() - peak;
  
  strip1.setPixelColor(y1-1,Wheel(map(y1,0,strip1.numPixels()-1,30,240)));
  strip2.setPixelColor(y2-1,Wheel(map(y2,0,strip2.numPixels()-1,30,240)));
  strip3.setPixelColor(y3-1,Wheel(map(y3,0,strip3.numPixels()-1,30,240)));
    
  strip1.show();
  strip2.show();
  strip3.show();

  // Frame based peak dot animation
  if(dotHangCount > PEAK_HANG) { //Peak pause length
    if(++dotCount >= PEAK_FALL) { //Fall rate 
      peak++;
      dotCount = 0;
    }
  } 
  else {
    dotHangCount++; 
  }
}

//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  for(int i=from; i<=to; i++){
    strip1.setPixelColor(i, c);
    strip2.setPixelColor(i, c);
    strip3.setPixelColor(i, c);
  }
}


float fscale( float originalMin, float originalMax, float newBegin, float
newEnd, float inputValue, float curve){

  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;


  // condition curve parameter
  // limit range

  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;

  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function

  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println(); 
   */

  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }

  // Zero Refference the values
  OriginalRange = originalMax - originalMin;

  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }

  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float

  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }

  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;

  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }

  return rangedValue;
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip1.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
     return strip2.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
      return strip3.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip1.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    return strip2.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    return strip3.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return strip1.Color(0, WheelPos * 3, 255 - WheelPos * 3);
      return strip2.Color(0, WheelPos * 3, 255 - WheelPos * 3);
        return strip3.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
