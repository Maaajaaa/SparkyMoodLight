#define HUD
#define ENABLE_BOBLIGHT

#ifdef HUD
  #include "HID-Project.h"

#endif
#include <IRLremote.h>
#include <Adafruit_PWMServoDriver.h>
#include "pca9685_rgb_led.h"

/*
  Updated Fade RGB LED Smoothly through 7 colours
  Fades an RGB LED using PWM smoothly through 7 different colours pausing for 1 seconds on each colour.
  Re-writted code to non blocking program using timers.
  
  Connect an common Cathode RGB LED with appropriate resistors on each anode to your Arduino Uno; 
  Red to pin 6, Green to pin 5, Blue to pin 3, Cathode to GND.
  
  Developed for Arduino Uno by Joshua David - TechHelpBlog.com
   
  Please Feel Free to adapt and use this code in your projects. 
  Contact me at techhelpblog.com and let me know how you've used it!  
*/

/*---------------------PWM-STUFF-------------------*/
//set the PCA9685 up
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x60);
//define the RBG-Channels
PCA9685_RGB_LED rgb0 = PCA9685_RGB_LED(pwm, 2, 0, 1, false);
PCA9685_RGB_LED rgb1 = PCA9685_RGB_LED(pwm, 5, 3, 4, false);
PCA9685_RGB_LED rgb2 = PCA9685_RGB_LED(pwm, 8, 10, 9, false);
PCA9685_RGB_LED rgb3 = PCA9685_RGB_LED(pwm, 13, 11, 12, false);

//mode state
bool fading = false;


/*--------------------IR-STUFF----------------------------*/
// do you want to block until the last data is received
// or do you always want to update the data to the newest input
#define IRL_BLOCKING true

// IR input pin (must be a interrupt capable pin)
#define pinIR 7

// variables to temporary save latest IR input
uint8_t IRProtocol = 0;
uint16_t IRAddress = 0;
uint32_t IRCommand = 0;
uint16_t lastIRAddress = 0;
uint32_t lastIRCommand = 0;

/*-------------------FADING-STUFF--------------------------*/
byte RED, GREEN, BLUE; 
byte RED_A = 0;
byte GREEN_A = 0; 
byte BLUE_A = 0;
int led_delay = 0;
byte colour_count = 1;                //Count the colours out
#define colour_count_max 7              //Set this to the max number of colours defined
#define colour_delay 4000             //Define the delay between changing colours in ms
#define time_at_colour 750           //Time to stay on a colour in ms

//Some Time values
unsigned long TIME_LED = 0;
unsigned long TIME_COLOUR = 0;

//Factor for Boblight
#define boblightFactor 1

//Colours
//Blue
#define C1_R 0
#define C1_G 0
#define C1_B 4095
//Red
#define C2_R 4095
#define C2_G 0
#define C2_B 0
//White
#define C3_R 4095
#define C3_G 4095
#define C3_B 4095
//Orange
#define C4_R 4095
#define C4_G 2976
#define C4_B 0
//Light Blue
#define C5_R 0
#define C5_G 2688
#define C5_B 4095
//Purple
#define C6_R 4095
#define C6_G 0
#define C6_B 4095
//Yellow
#define C7_R 4095
#define C7_G 2688
#define C7_B 0

/*--------------------BOBLIGHT-STUFF----------------*/
#define FIRST_PREFIX 0x55
#define SECOND_PREFIX 0xAA
#define NROUTPUTS 12

uint16_t values[12];
uint8_t first = 0;

boolean colorNull = true;

/*-----------------STATUS-LEDS-------------------*/
#define BLUE_LED 5
#define RED_LED 6

/*------------------------STATUS-VARIABLES-----------*/
bool device_on = false;
bool screen_on = true;

/*---------------SETUP--------------------*/
void setup()
{
  // start serial debug output
  //while(!Serial);
  Serial.begin(115200);
  Serial.println("Startup");

  //status leds
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(BLUE_LED, LOW);

  //pwm
  pwm.begin();
  pwm.setPWMFreq(1600);
  
  setAll(4095, 0, 0);

  //IR
  attachInterrupt(digitalPinToInterrupt(pinIR), IRLinterrupt<IR_USER>, CHANGE);

  //RAINBOW-LIGHT
  //Assign initial values
  RED = C1_R;
  GREEN = C1_G;
  BLUE = C1_B;
  //Get the led_delay speed
  led_delay = (colour_delay - time_at_colour) / 255; 

  //blink thrue all colors
  setAll(4095,0,0);
  delay(200);
  setAll(0,4095,0);
  delay(200);
  setAll(0,0,4095);
  delay(200);
  setAll(0,0,0);

  //Mediacontroller
  #ifdef HUD Consumer.begin(); 
#endif
}

/*-----------------LOOP---------------*/
void loop()
{
    
  // temporary disable interrupts and print newest input
  uint8_t oldSREG = SREG;
  cli();
  
  if (IRProtocol) 
  {
    SREG = oldSREG;    
    //Serial.println();
    //Serial.print("Protocol:");
    //Serial.println(IRProtocol);

    //was here before
    switch(IRAddress)
    {
      case 0xFF00:  //Thomson
        switch(IRCommand)
        {
          case 0xF708: //POWER
            /*if(device_on)
            {//turn off            
              Serial.println("RGB off");
              digitalWrite(RED_LED, HIGH);
              digitalWrite(BLUE_LED, LOW);
              device_on = false;
              setAll(0,0,0);
              fading=false;
            }
            else
            {//turn on
              Serial.println("RGB on");
              digitalWrite(RED_LED, LOW);
              digitalWrite(BLUE_LED, HIGH);
              device_on = true;
            }    */
            #ifdef HUD Consumer.write(CONSUMER_SLEEP); 
            #endif
          break;
          
          case 0xFF00: //S-Key          
            if(fading){
              Serial.println("FADE OFF");
              fading = false;
            }
            else {
              Serial.println("FADE ON");
              fading = true;              
            }
          break;

          case 0xFC03: //p-key            
            /*Serial.println("BLUE");
            setAll(0,0,4095);*/
            Serial.println("Thomeson p-key");
            if(screen_on)
            {
              #ifdef HUD 
                Consumer.write(CONSUMER_PROGRAMMABLE_BUTTON_CONFIGURATION);
              #endif
              screen_on = false;          
            }
            else
            {
              #ifdef HUD 
                Consumer.write(CONSUMER_CONTROL_CONFIGURATION);  
              #endif
              screen_on = true;              
            }
            
          break;

          //MEDIA-keys                
          case 0xFB04:  //Vol+
            Serial.println("Thomson Vol+");
            #ifdef HUD 
              Consumer.write(MEDIA_VOLUME_UP);  
            #endif
          break;

          case 0xFE01:  //Vol-
            Serial.println("Thomson Vol-");
            #ifdef HUD 
              Consumer.write(MEDIA_VOLUME_DOWN); 
            #endif
          break;
          
          case 0xF609:  //Play/Pause
            Serial.println("Thomson Play/Pause");            
            #ifdef HUD 
              Consumer.write(MEDIA_PLAY_PAUSE); 
            #endif
          break;

          case 0xFA05:  //Stop
            Serial.println("Thomson Stop");
            #ifdef HUD 
              Consumer.write(MEDIA_STOP);
            #endif
          break;

          case 0xF807:  //Next Song
            Serial.println("Thomson next Song");
            #ifdef HUD
              Consumer.write(MEDIA_NEXT);  
            #endif
          break;

          case 0xF40B:  //Previous Song
            Serial.println("Thomson previous Song");
            #ifdef HUD
            Consumer.write(MEDIA_PREV);  
            #endif
          break;

          case 0xF50A:  //Loop
            Serial.println("Thomson loop-key");            
            #ifdef HUD 
              Consumer.write(CONSUMER_CALCULATOR);  
            #endif            
          break;

          case 0xF906:  //Fast Forward
            Serial.println("Thomson Fast Forward");
            #ifdef HUD
              Consumer.write(MEDIA_FAST_FORWARD);  
            #endif
          break;

          case 0xFD02:  //Rewind
            Serial.println("Thomson Rewind");
            #ifdef HUD 
              Consumer.write(MEDIA_REWIND);  
            #endif
          break;
          
          default:
            Serial.print("Thomson unnown command:");
            Serial.println(IRCommand, HEX);
          break;
        }
      break;
      
      case 0xEF00:  //RBG
        switch(IRCommand)
        {
          case 0xFC03:  //ON
            Serial.println("RGB on");
            digitalWrite(RED_LED, LOW);
            digitalWrite(BLUE_LED, HIGH);
            device_on = true;
          break;
          
          case 0xFD02:  //OFF
            Serial.println("RGB off");
            digitalWrite(RED_LED, HIGH);
            digitalWrite(BLUE_LED, LOW);
            device_on = false;
            setAll(0,0,0);
            fading=false;
          break;
          
          case 0xFB04:  //RED
            Serial.println("RED");
            setAll(4095,0,0);
          break;
          
          case 0xFA05:  //GREEN
            Serial.println("GREEN");
            setAll(0,4095,0);
          break;

          case 0xF906:  //BLUE
            Serial.println("BLUE");
            setAll(0,0,4095);
          break;

          case 0xF807:  //WHITE
            Serial.println("WHITE");
            setAll(4095,4095,4095);
          break;

          case 0xF40B:  //FLASH
            Serial.println("FLASH");
            
          break;

          case 0xF00F:  //STROBE
            Serial.println("STROBE");
          break;

          case 0xEC13:  //FADE
            if(fading){
              Serial.println("FADE OFF");
              fading = false;
            }
            else {
              Serial.println("FADE ON");
              fading = true;              
            }
          break;

          case 0xE817:  //SMOOTH
            Serial.println("SMOOTH");
          break;

          case 0xF708:  //RED ORANGE
            Serial.println("RED ORANGE");
            setAll(4095,928,0);
          break;

          case 0xF30C:  //ORANGE
            Serial.println("ORANGE");            
            setAll(4095,218,0);
          break;

          case 0xEF10:  //YELLOW ORANGE
            Serial.println("YELLOW ORANGE");
            setAll(4095,2850,0);
          break;

          case 0xEB14:  //YELLOW
            Serial.println("YELLOW");
            setAll(4095,1600,0);
          break;

          case 0xF609:  //LIGHT GREEN
            Serial.println("LIGHT GREEN");
            setAll(1456,4095,0);
          break;

          case 0xF20D:  //CYAN
            Serial.println("CYAN");
            setAll(4095,4095,0);
          break;

          case 0xEE11:  //BLUE GREEN
            Serial.println("BLUE GREEN");
            setAll(816,1792,4095);
          break;

          case 0xEA15:  //DARK BLUE GREEN
            Serial.println("DARK BLUE GREEN");
            setAll(0,1024,4095);
          break;

          case 0xF50A:  //BLUE VIOLET
            Serial.println("BLUE VIOLET");
            setAll(1920,0,255);
          break;

          case 0xF10E:  //VIOLET
            Serial.println("VIOLET");
            setAll(4095,0,4095);
          break;


          case 0xED16:  //PURPLE
            Serial.println("PURPLE");
            setAll(4095,0,2688);
          break;

          case 0xE916:  //PINK
            Serial.println("PINK");
            setAll(4095,0,1280);
          break;
          
          default:
            Serial.print("RGB unnown command:");
            Serial.println(IRCommand, HEX);
          break;
        }
      break;

      case 0: //universal (repeat)
      {
        if(IRCommand == 0xFFFF) //repeat
          if(lastIRAddress == 0xFF00 && lastIRCommand == 0xF906)
          {
            Serial.println("Repeat FFDW");            
            #ifdef HUD 
              Consumer.write(MEDIA_FAST_FORWARD);  
            #endif
          }
          else
          {
            Serial.println("RPT");
          }
      }
      break;
      
      default:  //dump
        Serial.print("Address:");
        Serial.println(IRAddress, HEX);
        Serial.print("Command:");
        Serial.println(IRCommand, HEX);
      break;
    }

    // reset variable to not read the same value twice
    IRProtocol = 0;
  }
  SREG = oldSREG;
  

//Rest of your program - Avoid using delay(); function!
  if(fading)
  {
    if(millis() - TIME_LED >= led_delay){
      TIME_LED = millis();
  
      //Run the LED Function to check and adjust the values
      LED();
    }
  
    if(millis() - TIME_COLOUR >= colour_delay){
      TIME_COLOUR = millis();
  
      //Run the Colour Change function
      COLOUR();
    }
  }
  #ifdef ENABLE_BOBLIGHT
  if(Serial.available())
    serialEvent();
  #endif
}

void setAll(int R, int G, int B)
{
  rgb0.set(R, G, B);
  rgb1.set(R, G, B);
  rgb2.set(R, G, B);
  rgb3.set(R, G, B);
  Serial.print("R:");
  Serial.print(R);
  Serial.print(" G:");
  Serial.print(G);
  Serial.print(" B:");
  Serial.println(B);
}


void IREvent(uint8_t protocol, uint16_t address, uint32_t command) 
{
  //startTime = micros(); //StopTime
  
  // called when directly received a valid IR signal.
  // do not use Serial inside, it can crash your program!

  // dont update value if blocking is enabled
  if (IRL_BLOCKING && !IRProtocol) {
    //save old (last) address and command if theye arent repeat
    if(IRAddress != 0 && IRCommand != 0xFFFF)
    { 
      lastIRAddress = IRAddress;
      lastIRCommand = IRCommand;
    }
    // update the values to the newest valid input
    IRProtocol = protocol;
    IRAddress = address;
    IRCommand = command;
  }
  
  /*elapsedTime = micros() - startTime; //DisplayTime
  Serial.print("InterruptTime: ");  Serial.println(elapsedTime);*/
}

void decodeIR(const uint16_t duration) {
  // called when directly received an interrupt CHANGE.
  // do not use Serial inside, it can crash your program!

  // add the protocols you want to use here to reduce flash/ram/performance.
  // you can use this to decode e.g. two remotes at the same time
  // but dont need to check all of the other protocols.
  // calling this function takes some small overhead though.
  // you can also change the priority here
  decodeNec<IR_EXTRA_ACCURACY>(duration);
  //decodePanasonic<IR_EXTRA_ACCURACY>(duration);
}

void LED()
{

  //Check Values and adjust "Active" Value
  if(RED != RED_A){
    if(RED_A > RED) RED_A--;
    if(RED_A < RED) RED_A++;
  }
  if(GREEN != GREEN_A){
    if(GREEN_A > GREEN) GREEN_A--;
    if(GREEN_A < GREEN) GREEN_A++;
  }
  if(BLUE != BLUE_A){
    if(BLUE_A > BLUE) BLUE_A--;
    if(BLUE_A < BLUE) BLUE_A++;
  }

  //Assign modified values to the pwm outputs for each colour led
  setAll(RED_A*16, GREEN_A*16, BLUE_A*16);

}

void COLOUR()
{

  //Increment the colour by one or go back to 1 if maxed out
  if(colour_count < colour_count_max) colour_count++;
  else colour_count = 1;

  if(colour_count == 1){
    RED = C1_R;
    GREEN = C1_G;
    BLUE = C1_B;
  } else if(colour_count == 2){
    RED = C2_R;
    GREEN = C2_G;
    BLUE = C2_B;
  } else if(colour_count == 3){
    RED = C3_R;
    GREEN = C3_G;
    BLUE = C3_B;
  } else if(colour_count == 4){
    RED = C4_R;
    GREEN = C4_G;
    BLUE = C4_B;
  } else if(colour_count == 5){
    RED = C5_R;
    GREEN = C5_G;
    BLUE = C5_B;
  } else if(colour_count == 6){
    RED = C6_R;
    GREEN = C6_G;
    BLUE = C6_B;
  } else if(colour_count == 7){
    RED = C7_R;
    GREEN = C7_G;
    BLUE = C7_B;
  }
}
#ifdef ENABLE_BOBLIGHT

//boblightd needs to send 0x55 0xAA before sending the channel bytes
void WaitForPrefix()
{
  uint8_t first = 0, second = 0;
  while (second != FIRST_PREFIX || first != SECOND_PREFIX)
  {
    while (!Serial.available());
    second = first;
    first = Serial.read();
  }
}

void serialEvent()
{
  colorNull = false;
  WaitForPrefix();
  for (uint8_t i = 0; i < NROUTPUTS*2; i++)
  {
    while(!Serial.available());
    values[i] = Serial.read();
  }
  for(uint8_t i = 0; i < NROUTPUTS; i++)
  {
    values[i] = values[2*i]+values[2*i+1];
  }
  rgb0.set(values[0]*boblightFactor, values[1]*boblightFactor, values[2]*boblightFactor);
  rgb1.set(values[3]*boblightFactor, values[4]*boblightFactor, values[5]*boblightFactor);
  rgb2.set(values[6]*boblightFactor, values[7]*boblightFactor, values[8]*boblightFactor);
  rgb3.set(values[9]*boblightFactor, values[10]*boblightFactor, values[11]*boblightFactor);
  
  //for (uint8_t i = 0; i < NROUTPUTS; i++)
    //analogWrite(outputs[i], values[i]);
}

#endif
