/*
TOF SENSOR HOOKUP GUIDE

GNF -> GND
3.3V -> 3.3V
SDA -> SDA
SCL -> SCL
INT -> 
SHUT ->

*/

#include <math.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include <VL53L1X.h>

#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1:
#define LED_PIN    6

// How many NeoPixels are attached to the Arduino?
#define LED_COUNT 45 //might change to 50??

#define MAX_PARTICLES 4
// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)

VL53L1X sensor;

// setup() function -- runs once at startup --------------------------------

bool reverse = false; // make particles go other way
bool showPlat = false; // show where hand/platform is detected
bool fading = true; // dots fade out after exceeding maxBounces

int alive[MAX_PARTICLES]; // 0 dead, 1 alive
int dying[MAX_PARTICLES]; // 0 alive/dead, 1 dying
uint32_t particleColor[MAX_PARTICLES];
int hue[MAX_PARTICLES]; // color of particle 
int val[MAX_PARTICLES]; // value/opacity(0 - off, 255 - full)
float yPos[MAX_PARTICLES];
float xPos[MAX_PARTICLES];
float ySpeed[MAX_PARTICLES];
int bounces[MAX_PARTICLES];

int spawnFrames = 4;
const float grav = .24; // 3
const float terminalVelocity = 7;
const float elasticity = .55; // bounciness
const float ground = 31.5;//40.0; // lowest point is ~40inches from top of strip
float plat = ground; // "platform" particles bounce off of
const float ledDist = ground/LED_COUNT; // physical space between leds
const float spawnChance = 0.00001; // chance of new particle every cycle
float high = 0; // value read from sensor when hand is at highest led
const int maxBounces = 2;
const int fadeSpeed = 3;
int particles = 0;

int frameCount = 0;

void setup() {
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  // END of Trinket-specific code.

  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
  Serial.begin(9600);

  Wire.begin();
  Wire.setClock(400000); // use 400 kHz I2C

  sensor.setTimeout(5);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
  }
  Serial.println("Lets goooo");
  //sensor.setDistanceMode(VL53L1X::Short);
  sensor.setDistanceMode(VL53L1X::Long);

  sensor.setROISize(4, 4);
  sensor.setROICenter(199);

  Serial.println(sensor.getROICenter());

  //sensor.setMeasurementTimingBudget(20000);
  sensor.startContinuous(15);
  Serial.println("new program");

  //Serial.println(alive[0]);
}

void loop() {
 frameCount += 1;
 //Serial.println(frameCount);

  while(high < 100){
      high = (float) sensor.read(); - 80;
      //Serial.println(high);
  }

  updatePlat();

  // calculate particles
  if (particles < MAX_PARTICLES && random(50.0) < (spawnChance)){
    //float col = random(45000, 60000);
    //col = random(0, 5000);
    float col = (44000) + (sin(frameCount * 0.0001) * 18000) + random(-5000,5000);
    //Serial.println(col);
    createParticle(0, random(-3,0), 0.2, col);
    // 40000 deep blues
    // 
  }
  move();

  // draw particles
  drawParticles();

  if (showPlat){
    drawPlat();
  }

  strip.show();

  //Serial.println(particles);
  }

// make new particles given position color and speed
void createParticle(float startX, float startY, float startSpeed, float hue){
  for (int i = 0; i < MAX_PARTICLES; i++){
    if (!alive[i]){
      spawnParticle(i, startX, startY, startSpeed, hue);
      return;
    }
  }
}

// instantiates particle with position, color, and speed
void spawnParticle(int i, float startX, float startY, float startSpeed, float h){
  alive[i] = 1;
  dying[i] = 0;
  bounces[i] = 0;
  xPos[i] = startX; // 0 through [number of strips]
  yPos[i] = startY;
  ySpeed[i] = startSpeed;
  hue[i] = h;
  val[i] = 255;
  //particleColor[i] = color;  
  particles += 1;
}

// makes particle inactive
void killParticle(int i){
  alive[i] = 0;
  dying[i] = 0;
  ySpeed[i] = 0;
  particles -= 1;
}

// begins fade of particle
void fadeParticle(int i){
  if (!fading){
    killParticle(i);
  } else {
    dying[i] = 1;
  }
}

// update position of all particles
void move(){
  for (int i = 0; i < MAX_PARTICLES; i++){
    if (alive[i]){
      ySpeed[i] += grav / 100.0;
      yPos[i] += ySpeed[i];
      if (yPos[i] >= ground){
        killParticle(i);
      }
      else if (ySpeed[i] > 0 && yPos[i] >= plat && (yPos[i] - plat)+2 > ySpeed[i]){
        bounce(i);
      }
    }

  }
}  

// weaken intensity of fading particle
void fade(int i){
  val[i] -= fadeSpeed;
  if (val[i] <= 10) {
    killParticle(i);    
  }
}

// make particle bounce off hand/platform
void bounce(int i){
  float diff = yPos[i] - plat;
  yPos[i] = plat - diff;
  ySpeed[i] *= -(elasticity);
  bounces[i] += 1;
  if (bounces[i] >= maxBounces) {
    fadeParticle(i);
  }
  if (abs(ySpeed[i]) < 0.1){
    fadeParticle(i);
  }
}

// show position of hand/platform
void drawPlat(){
  int p = worldToLed(plat);
  if (plat != 40){
    strip.setPixelColor(worldToLed(plat), strip.ColorHSV(4000,0,200));    
  }
}

// updates led strip to show new positions of particles
void drawParticles(){
  for (int i = 0; i < LED_COUNT; i++){
    strip.setPixelColor(i, 0);
  }
  for (int i = 0; i < MAX_PARTICLES; i++){
    if (alive[i]){

      if (dying[i]){
          fade(i);
      }

      strip.setPixelColor(getLED(i), strip.gamma32(strip.ColorHSV(hue[i], 255, val[i])));
      //strip.setPixelColor(getLED(i), (strip.ColorHSV(hue[i], 255, val[i])));

    }
  }
}

// gets the LED number of a particle
int getLED(int particle){
  int l = (int) round((yPos[particle]/ground) * LED_COUNT);
  if (reverse){
    return LED_COUNT - l;
  }
  return l;
}

// reads from sensor and update position of hand/platform
void updatePlat(){
  float reading = (float)sensor.read();
  

  if (reading != 0){
    Serial.println(reading);

    if (reading > 500 && reading < high-40){
      //reading -= 0.2*reading;
    }
    plat = (reading/high)*ground;
    
    if (plat > 30){
      plat = ground;
    }
    
    //Serial.println(rangingData.ambient_count_rate_MCPS);
    //Serial.println(plat);
  }
  //Serial.println(plat);
}

// converts LED coordinates to world coordinates
float ledToWorld(int led){
  return float(led)/ground;
}

// converts world coordinates to LED coordinates
int worldToLed(float pos){
  int l = (int) round((pos/ground) * LED_COUNT);
  //int l = map(pos/ground, 0, 1, 0, LED_COUNT);
  if (reverse){
    return LED_COUNT - l;
  }
  return l;
}
