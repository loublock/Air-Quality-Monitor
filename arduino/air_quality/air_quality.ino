// Created by Luis Garcia
// https://github.com/loublock 

/*
--------------- Quick Arduino Nano pinout ---------------
D7: button 1
D6: button 1 (or vise versa)
D2: common button GND (same interrupt)
D5: WS2812 LED stripe

A0: MQ135
A4: OLED + BME280 (I2C)
A4: OLED + BME280 (I2C)
*/

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_NeoPixel.h>
#include "MQUnifiedsensor.h"
#include <EEPROM.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)

#define LED_PIN 5 // LED stripe on D5
#define NUM_LEDS 15 // amount of WS2812 LEDs on the stripe

// useful LED defines
#define FAST 200
#define MEDIUM 500
#define SLOW 1000

#define PIN_BTN_0 7 // button 1 on D7
#define PIN_BTN_1 6 // button 2 on D6
#define COMMON_BTN_PIN 2 // buttons GND on D2 (interrupt)

Adafruit_BME280 bme;

// MQ135 defines as seen in the examples of Miguel A. Califa U.
#define SEALEVELPRESSURE_HPA (1013.25)
#define RatioMQ135CleanAir 3.6

#define TYPE "MQ-135"
#define BOARD "Arduino Nano"
#define MQ_PIN A0 // MQ135 on A0
#define ADC_BIT_RES 10
#define VOLTAGE_RES 5
// Resistor Load on ground pin of MQ135
#define RL 20

// heating time in ms
#define HEATING_TIME 300000 //~ 5 min

// temperature and rel humidity correction according to Georg Krocker
#define CORA 0.00035
#define CORB 0.02718
#define CORC 1.39538
#define CORD 0.0018

// approx. atmospheric CO2 in ppm measured at Mauna Loa Observatory, Hawaii (NOAA)
// oct 2020
#define ATMOCO2 410

// gas limits - edit if neccessary
// in theory triggering on one gas should be fine bc the curves are related
#define CO2_MAX 2000
#define CO2_LOW 1000
#define CO_MAX 70
#define CO_LOW 40

// decl & init of sensor values
float humidity = 0.0;
float temperature = 0.0;
float pressure = 0.0;
float altitude = 0.0;
float co2 = 0.0;
float co = 0.0;
float nh4 = 0.0;
// float toluol = 0.0;

// scrolling index
unsigned int train_idx = 0;
unsigned int rainbow_count = 0;
//r0 of mq135
int calcR0 = 0; 

// set this to false to calibrate the MQ 135
bool mq135_cal_done = true;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
MQUnifiedsensor MQ135(BOARD, VOLTAGE_RES, ADC_BIT_RES, MQ_PIN, TYPE);

// init button state
bool btn0_state = HIGH;
bool btn1_state = HIGH;

// connect interrupt pins with routines
unsigned long last_fire = 0;

// ----triggers to set in ISR----
// 0: hum/temp/press 1: CO2/CO/NH4
bool display_mode = 0;
// 0: gas conc. color 1: lamp (white 255)
bool led_mode = 0;

// mq135 pre heating variables
bool heating = true;
float timer_heat = 0;

// smoothing the reading values
unsigned int n_smooth = 0;
const int smooth_limit = 10;
float co2_buffer[smooth_limit];
float co_buffer[smooth_limit];
float nh4_buffer[smooth_limit];
// float toluol_buffer[smooth_limit];




void setup() 
{
  Serial.begin(9600);

  configure_common();
  attachInterrupt(0, btn_press_ISR, FALLING);

  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) 
  { 
    // Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  init_timer();
  init_mq135();

  // oled init
  display.setRotation(2);
  display.setTextSize(1);      
  display.setTextColor(WHITE); 
  display.setCursor(0, 0);     
  display.cp437(true);         // Use full 256 char 'Code Page 437' font

  pixels.begin();

  // start BME sensor
  if (!bme.begin(0x76)) 
  {
    // Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  // Clear the buffer
  display.clearDisplay();
  display.display();

  // setting up parameters
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  
  // Display static welcome text
  // \201 - ü \204 - ä \224 - ö \253 - ½
  display.println(F("Luft"));
  display.println(F("Qualit\204t"));
  display.display();

  delay(2000);

  display.clearDisplay();
  display.display();
}


void loop() 
{
  humidity = bme.readHumidity();
  temperature = bme.readTemperature(); 
  pressure = bme.readPressure() / 100.0F;
  altitude = bme.readAltitude(SEALEVELPRESSURE_HPA);

  // gets only executed when mq135_cal_done is initialized with false
  calibration_mq135();
  // get mq135 sensor values
  get_mq135_readings();

  // reset display or the display writes the new readings over the old ones
  display.display();
  display.clearDisplay();

  // wait for finishing heating
  if (timer_heat >= HEATING_TIME)
  {
    timer_heat = 0;
    heating = false;
//    Serial.println(F("Aufheizen abgeschlossen!"));
//    Serial.println(F("MQ135 betriebsbereit!"));
  }

  if (led_mode)
  {
    print_regular_readings();
    glow_LED('white', 255);
  }
  else if (!heating)
  {
    // co2: green: <=1000, orange: 2000>val>1500 red: >=2000
    // co: green: <=50, orange: 70>val>50 red: >=70
    if (co2 >= CO2_MAX || co >= CO_MAX)
    {
      // tehre's a bug: if you put the predefines color into the LED functions, the color is wrong.
      // Also if you put the dec/hex value as parameter. Only direct color set is working, so strings are given.
      blink_LED('red', FAST, 255);
      if (co2 >= CO2_MAX)
      {
        print_alarm_value_display("CO2: ", co2, "PPM", false, 0);
      }
      else
      {
        print_alarm_value_display("CO: ", co, "PPM", false, 0);
      }
    }
    else if (co2 <= CO2_LOW && co <= CO_LOW)
    {
      print_regular_readings();
      glow_LED('green', 15);
    }
    else
    {
      print_regular_readings();
      scroll_LED('orange', FAST, 5, 100);
    }
  }
  else
  {
    print_regular_readings();
    // rainbow_scroll_LED(); 
    scroll_LED('blue', MEDIUM, 5, 50);
  }
  

  // delay(10);
}


//----------------------OLED write routine---------------------------------------------------
void print_readings_display(float value, int line, String label, int offset, String unit, 
                            int textsize, bool is_spec_unit, int special_unit, bool blinking)
{
  display.setTextSize(textsize);
  display.setCursor(0, line);
  display.print(label);
  display.setCursor(offset, line);
  display.print(value);
  display.print(" ");

  if (is_spec_unit)
  {
    display.cp437(true);
    display.write(special_unit);
  }
 
  display.print(unit);
}


void print_alarm_value_display(String val_label, float alarm_value, String unit, 
                               bool is_spec_unit, int special_unit)
{
  // first print what value is alarming
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(val_label);

  // then the value in big
  display.setTextSize(2);
  display.setCursor(0, 10);
  display.print(alarm_value);
  display.print(" ");

    if (is_spec_unit)
  {
    display.cp437(true);
    display.write(special_unit);
  }
 
  display.print(unit);
}


void print_regular_readings()
{
  if (!display_mode)
  {
    print_readings_display(co2, 0, "CO2: ", 40, "PPM", 1, false, 0, false);
    print_readings_display(co, 10, "CO: ", 40, "PPM", 1, false, 0, false);
    print_readings_display(nh4, 20, "NH4: ", 40, "PPM", 1, false, 0, false);
    // print_readings_display(toluol, 23, "Toluol: ", 40, "PPM", 1, false, 0, false);
    // print_readings_display(calcR0, 20, "R0: ", 40, "", 1, false, 0, false);
  }
  else
  {
    print_readings_display(temperature, 0, "Temperatur: ", 75, "C", 1, true, 167, false);
    print_readings_display(humidity, 7, "Feuchte: ", 75, "%", 1, false, 0, false);
    print_readings_display(pressure, 15, "Druck: ", 60, "hPa", 1, false, 0, false);
    print_readings_display(altitude, 23, "H\224he: ", 60, "m", 1, false, 0, false);
  }
}


//--------------------------LED stripe routines---------------------------------------------
void blink_LED(int color, int speed, int brightness)
// lets the LEDs blink in a specific color, brightness and speed
{
  pixels.setBrightness(brightness);
  pixels.show();

  // if off turn on
  if (pixels.getPixelColor(0) == pixels.Color(0,0,0))
  {
    for (int n=0; n<NUM_LEDS; n++)
    {
      // turn on the train pixels
      if (color == 'red')
        pixels.setPixelColor(n, pixels.Color(255,0,0));
      else if (color == 'green')
        pixels.setPixelColor(n, pixels.Color(0,255,0));
      else if (color == 'blue')
        pixels.setPixelColor(n, pixels.Color(0,0,255));
      else if (color == 'orange')
        pixels.setPixelColor(n, pixels.Color(255,165,0));
      else
        pixels.setPixelColor(n, pixels.Color(255,255,255));
    }
    
    pixels.show();
    }
  // else turn off
  else
  {
    pixels.clear();
    pixels.show();
  }
  
  delay(speed);
}


void glow_LED(int color, int brightness)
// just turn on the LEDs at specific color and brightness
{
  pixels.setBrightness(brightness);
  pixels.show();

  for (int n=0; n<NUM_LEDS; n++)
  {
    // turn on the train pixels
    if (color == 'red')
      pixels.setPixelColor(n, pixels.Color(255,0,0));
    else if (color == 'green')
      pixels.setPixelColor(n, pixels.Color(0,255,0));
    else if (color == 'blue')
      pixels.setPixelColor(n, pixels.Color(0,0,255));
    else if (color == 'orange')
      pixels.setPixelColor(n, pixels.Color(255,165,0));
    else
      pixels.setPixelColor(n, pixels.Color(255,255,255));
  }

  pixels.show();
}


void scroll_LED(int color, int speed, int length, int brightness)
// this function lets a specific amount of LEDs travel around the stripe
// at a specific speed
{
  int diff = 0;
  pixels.setBrightness(brightness);
  pixels.show();

  // create a - let's call it a train, a train of LED light pixels
  // it has a length of pixels and a speed with which it moves on the LED stripe
  for (int t=train_idx; t<train_idx+length; t++)
  {
    // turn on the train pixels
    if (color == 'red')
      pixels.setPixelColor(t, pixels.Color(255,0,0));
    else if (color == 'green')
      pixels.setPixelColor(t, pixels.Color(0,255,0));
    else if (color == 'blue')
      pixels.setPixelColor(t, pixels.Color(0,0,255));
    else if (color == 'orange')
      pixels.setPixelColor(t, pixels.Color(255,165,0));
    else
      pixels.setPixelColor(t, pixels.Color(255,255,255));
    
  }
  // make them gloooow
  pixels.show();

  // blinking speed is too slow for here
  delay(speed/10);

  pixels.clear();
  // pixels.show();  

  train_idx++;
  if ((train_idx + length) > NUM_LEDS)
  {
    train_idx = 0;
  }
}


void rainbow_scroll_LED()
{
  if (rainbow_count >= 65536)
  {
    rainbow_count = 0;
  }
  else
  {
    for(int i=0; i<NUM_LEDS; i++) 
    {
      int pixel_hue = rainbow_count + (i * 65536L / NUM_LEDS);
      scroll_LED(pixels.gamma32(pixels.ColorHSV(pixel_hue)), MEDIUM, 5, 150);
    }
    rainbow_count++;
  }
  
}


// ----------------------button interrupt service routine-------------------------------------------------------
// you can use one ISR for more than one button, when you connect all buttons GND
// to one digital pin and set that to LOW --> COMMON_BTN_PIN
// when you connect the pin with the ISR you dont use the buttons digital pin, but your common pin
// finally you check in the ISR which button got pressed
void btn_press_ISR()
{
  // debounce the button, bc this is not Nicki Minaj
  if (millis() - last_fire < 200) 
  { 
    return;
  }
  last_fire = millis();

  configure_distinct();

  // get the button states - which one got pressed actully
  btn0_state = digitalRead(PIN_BTN_0);
  btn1_state = digitalRead(PIN_BTN_1);

  if (btn0_state == LOW)
  {
    display_mode = !display_mode;
  }
  else if (btn1_state == LOW)
  {
    led_mode = !led_mode;
  }

  configure_common();
}


// this magic is written by this guy 
// https://create.arduino.cc/projecthub/Svizel_pritula/10-buttons-using-1-interrupt-2bd1f8
void configure_common()
{
  pinMode(COMMON_BTN_PIN, INPUT_PULLUP);

  pinMode(PIN_BTN_0, OUTPUT);
  pinMode(PIN_BTN_1, OUTPUT);
  digitalWrite(PIN_BTN_0, LOW);
  digitalWrite(PIN_BTN_1, LOW);
}


void configure_distinct()
{
  pinMode(COMMON_BTN_PIN, OUTPUT);
  digitalWrite(COMMON_BTN_PIN, LOW);

  pinMode(PIN_BTN_0, INPUT_PULLUP);
  pinMode(PIN_BTN_1, INPUT_PULLUP);
}


// -----------------others------------------------------------
void init_mq135()
{
  //Set math model to calculate the PPM concentration and the value of constants
  MQ135.init();
  // set your RL resistor (china default: 1k, replace with one between 10k and 47k)
  // the greater the resistance on RL, the more sensitive the sensor becomes
  MQ135.setRL(RL); //20k
  MQ135.setRegressionMethod(1); //_PPM =  a*ratio^b

  calcR0 = EEPROM.read(0x00);
  // Serial.print("R0 aus EEPROM: ");
//  Serial.println(calcR0);
  MQ135.setR0(calcR0); 

  // Serial.println("3 Minuten heizen! Bitte warten...");
}


void calibration_mq135()
// calibration is done in pre heated sensor state! (replace 1k Ohm resistor on GND with a 20k)
{
  if (mq135_cal_done)
  {
    return;
  }

  if (!heating)
  {
    // Serial.println("Kalibrierung, bitte warten! ");
    int calcR0 = 0;

    for(int i=0; i<15; i++)
    {
      delay(2000);
      // get temp and hum for compensation
      humidity = bme.readHumidity();
      temperature = bme.readTemperature();
      // calculate correction factor
      float cf = CORA * temperature * temperature - CORB * temperature + CORC - (humidity - 33.) * CORD;

      MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin
      //Serial.println(MQ135.calibrate(RatioMQ135CleanAir));
      calcR0 += MQ135.calibrate(RatioMQ135CleanAir, cf);
    }

    // Serial.println("Berechne Widerstandswert:");
    calcR0 = calcR0/15;
    // Serial.println(calcR0);
    MQ135.setR0(calcR0);
    //Kalibrierwert in Festspeicher schreiben!
    EEPROM.write(0, calcR0);
    // Serial.println("Kalibrierung beendet!");
    mq135_cal_done = true;
  }
}


void get_mq135_readings()
{
  // https://github.com/miguel5612/MQSensorsLib

  /*
    Exponential regression:
  GAS      | a      | b
  CO       | 605.18 | -3.937  
  Alcohol  | 77.255 | -3.18 
  CO2      | 110.47 | -2.862
  Tolueno  | 44.947 | -3.445
  NH4      | 102.2  | -2.473
  Acetona  | 34.668 | -3.369
  */
  // smooth the readings over n values
  if (n_smooth >= smooth_limit)
  {
    co = 0.0;
    co2 = 0.0;
    nh4 = 0.0;
    
    for (int n=0; n<smooth_limit; n++)
    {
      co += co_buffer[n];
      co2 += co2_buffer[n];
      nh4 += nh4_buffer[n];
      // toluol += toluol_buffer[n];
    }
    co /= smooth_limit;
    co2 /= smooth_limit;
    nh4 /= smooth_limit;
    // toluol /= smooth_limit;

    n_smooth = 0;
  } 

  // correction factor dependant of temperature and humidity 
  // (credits to George K. --> https://github.com/GeorgK/MQ135)
  float cf = CORA * temperature * temperature - CORB * temperature + CORC - (humidity - 33.) * CORD;

  MQ135.update(); // Update data, the arduino will be read the voltage on the analog pin

  MQ135.setA(605.18); MQ135.setB(-3.937); // Configurate the ecuation values to get CO concentration
  co_buffer[n_smooth] = MQ135.readSensor(cf); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(110.47); MQ135.setB(-2.862); // Configurate the ecuation values to get CO2 concentration
  co2_buffer[n_smooth] = MQ135.readSensor(cf) * 100 + ATMOCO2; // Sensor will read PPM concentration using the model and a and b values setted before or in the setup
  // when the library is calibrated it assumes the current state of the
  // air as 0 PPM, and it is considered today that the CO2 present in the atmosphere is around 400 PPM.

  // MQ135.setA(44.947); MQ135.setB(-3.445); // Configurate the ecuation values to get Tolueno concentration
  // toluol_buffer[n_smooth] = MQ135.readSensor(cf); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  MQ135.setA(102.2); MQ135.setB(-2.473); // Configurate the ecuation values to get NH4 concentration
  nh4_buffer[n_smooth] = MQ135.readSensor(cf); // Sensor will read PPM concentration using the model and a and b values setted before or in the setup

  n_smooth++;
}


void init_timer()
{
  //use timer register 1, because regist 0 interfers with our button interrupts
  //timer for heating MQ135 time: 1kHz 
  TCCR2A=(1<<WGM21);    //Set CTC mode   
  OCR2A=249; //Value for ORC0A for 1ms
  
  TIMSK2|=(1<<OCIE2A);   //Set the interrupt request
  
  TCCR2B|=(1<<CS22);    //Set prescaler to 64

  sei(); //Enable interrupt
}


//interrupt request t1
ISR(TIMER2_COMPA_vect)
{   
  if (heating)
  {
    timer_heat++;
  }
}
