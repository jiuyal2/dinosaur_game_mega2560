#include <arduinoFFT.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>

#define PENDING 0
#define DONE 1

#define TRIGPIN  9
#define ECHOPIN  10
#define SWPIN    5

#define OP_DECODEMODE  8
#define OP_SCANLIMIT   10
#define OP_SHUTDOWN    11
#define OP_DISPLAYTEST 14
#define OP_INTENSITY   10
#define dinosaur_x     1

int DIN = 48;
int CS =  49;
int CLK = 51;

////////////////////////////////////////////////
// APPROVED FOR ECE 474   Spring 2021
//
//  NOTE: modify analogRead() on line 113 according
//   to your setup.
////////////////////////////////////////////////


byte spidata[2]; //spi shift register uses 16 bits, 8 for ctrl and 8 for data

/*
 * Pin 22~29: 7-segment display
 * Pin 37~34: 7-segment display digit selection
 */

uint8_t seven_seg_digits[20] = {0b0111111,  // = 0
                                0b0000110,  // = 1
                                0b1011011,  // = 2
                                0b1001111,  // = 3
                                0b1100110,  // = 4
                                0b1101101,  // = 5
                                0b1111101,  // = 6
                                0b0000111,  // = 7
                                0b1111111,  // = 8
                                0b1101111,  // = 9
                                0b0000000,  // = off
                                0b0000001,  // = countdown 1
                                0b0000011,  // = countdown 3
                                0b0000111,  // = countdown 3
                                0b0001111,  // = countdown 4
                                0b0011111,  // = countdown 5
                                0b0111111,  // = countdown 6 
                                0b1100100,  // = left
                                0b0001001,  // = middle
                                0b1010010   // = right
                               };

int jump_dia[5] = {2, 3, 4, 3, 2};

uint8_t digits[4] = {10, 10, 10, 10};
int distance = 0;
bool game_running = false;


// define two tasks for Blink & AnalogRead
void RT1( void *pvParameters );
void Seg7(void *pvParameters);
void Game_ctrl(void *pvParameters);
void clear_7seg();
void game(void *pvParameters);
void spiTransfer(volatile byte row, volatile byte data);


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(19200);

  pinMode(TRIGPIN, OUTPUT); // Sets the trigPin as an Output
  pinMode(ECHOPIN, INPUT); // Sets the echoPin as an Input
  pinMode(SWPIN, INPUT);
  digitalWrite(SWPIN, HIGH);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  } 

  // Now set up two tasks to run independently.
  xTaskCreate(
    RT1
    ,  "Task1"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    Seg7
    ,  "seven_seg"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
 
  xTaskCreate(
    Game_ctrl
    ,  "Game_control"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    Ultrasonic
    ,  "Ultrasonic"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  // Set up 7-segment display
  DDRA = 0xFF;
  DDRC = 0xF;
  PORTC = 0xF;

  pinMode(DIN, OUTPUT);
  pinMode(CS, OUTPUT);
  pinMode(CLK, OUTPUT);
  digitalWrite(CS, HIGH);
  spiTransfer(OP_DISPLAYTEST,0);
  spiTransfer(OP_SCANLIMIT,7);
  spiTransfer(OP_DECODEMODE,0);
  spiTransfer(OP_SHUTDOWN,1);

  clear_spi();
  vTaskStartScheduler();
}

void loop()
{
  // Empty. Things are done in Tasks.
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void RT1(void *pvParameters) {

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the UNO, LEONARDO, MEGA, and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care 
  of use the correct LED pin whatever is the board used.
  
  The MICRO does not have a LED_BUILTIN available. For the MICRO board please substitute
  the LED_BUILTIN definition with either LED_BUILTIN_RX or LED_BUILTIN_TX.
  e.g. pinMode(LED_BUILTIN_RX, OUTPUT); etc.
  
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products
  
  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
  
  modified 2 Sep 2016
  by Arturo Guadalupi
*/

  // initialize digital LED_BUILTIN on pin 47 as an output.
  pinMode(47, OUTPUT);

  for (;;) { // A Task shall never return or exit.
    digitalWrite(47, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 100 / portTICK_PERIOD_MS ); // wait for 100 milisecond
    digitalWrite(47, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 200 / portTICK_PERIOD_MS ); // wait for 200 milisecond
  }
}

void Seg7(void *pvParameters) {
  int i = 0;
  for(;;) {
    if (i == 4)
      i = 0; // reset i = 0;

    if (i == 0) {
      PORTC = 0xE;
    } else if (i == 1) {
      PORTC = 0xD;
    } else if (i == 2) {
      PORTC = 0xB;
    } else if (i == 3) {
      PORTC = 0x7;
    }
    PORTA = seven_seg_digits[digits[i]];
    vTaskDelay( 1 / portTICK_PERIOD_MS );
    PORTA = 0;
    i++;
  }
}

void Game_ctrl(void *pvParameters) {
  int hardness = 0;
  long start = xTaskGetTickCount();
  TaskHandle_t xHandle = NULL; // handle for game.
  for(;;) {
    if (!game_running) {
      while (true) {
          int selected_hard = (distance / 10)%10;
          if (selected_hard < 3) {
            if (selected_hard != hardness) {
              start = xTaskGetTickCount();
              digits[hardness] = 10; // clear previous digit
              hardness = selected_hard;
            }
            digits[hardness] = hardness;
            long time_last = (xTaskGetTickCount() - start) * portTICK_PERIOD_MS;

            if(time_last < 400) {
              digits[3] = 11;
            } else if(time_last < 800) {
              digits[3] = 12;
            } else if(time_last < 1200) {
              digits[3] = 13;
            } else if(time_last < 1600) {
              digits[3] = 14;
            } else if(time_last < 2000) {
              digits[3] = 15;
            } else if(time_last < 2400) {
              digits[3] = 16;
            } else {
              if (digitalRead(SWPIN) == 0)
                hardness = 4;
              break;
            }

          } else {
            start = xTaskGetTickCount();
            clear_7seg();
          }
          vTaskDelay( 100 / portTICK_PERIOD_MS );
      }

      clear_7seg();
      vTaskDelay( 100 / portTICK_PERIOD_MS );
      if (hardness == 4) {
        digits[0] = 17;
        digits[1] = 18;
        digits[2] = 18;
        digits[3] = 19;
        vTaskDelay( 1000 / portTICK_PERIOD_MS );

        clear_7seg();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );

        digits[0] = 17;
        digits[1] = 18;
        digits[2] = 18;
        digits[3] = 19;
        vTaskDelay( 1000 / portTICK_PERIOD_MS );

        clear_7seg();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );

        digits[0] = 17;
        digits[1] = 18;
        digits[2] = 18;
        digits[3] = 19;
        vTaskDelay( 1000 / portTICK_PERIOD_MS );

        clear_7seg();
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
      }

      //for each row, clear the LEDs
      for (int i = 0; i < 8; i++){
        spiTransfer(i, 0b00000001);
      }

      digits[0] = 3;
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digits[0] = 2;
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digits[0] = 1;
      vTaskDelay( 500 / portTICK_PERIOD_MS );
      digits[0] = 10;

      xTaskCreate(
        game
        ,  "game"   // A name just for humans
        ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
        ,  &hardness // pass hardness to game
        ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,  &xHandle );
        game_running = true;
    } else {
      // wait
      if ((distance / 10)%10 < 3) { // logic for pause
        vTaskSuspend(xHandle);
        clear_7seg();
        digits[0] = hardness;
      } else { 
        vTaskResume(xHandle);
      }
      vTaskDelay( 50 / portTICK_PERIOD_MS );
    }
  }
}

void game(void *pvParameters) {
  int hardness = * (int*)pvParameters;
  int dinosaur_y = 1;
  int score = 0;
  bool flip = false; // flag for flipping the color
  int move_rate = 2 * (hardness + 1);
  int count = 0;
  int jump_index = -1;

  int frame[8] = {1, 1, 1, 1, 1, 1, 1, 1};
  

  while(true) {
    if (count % 20 == 0) {
      move_rate += hardness;
    } 
    
    if (jump_index > 4) { // if jump complete, reset y and index
      dinosaur_y = 1;
      jump_index = -1;
    } else if (jump_index >= 0) { // jump in progress, update y and index
      dinosaur_y = jump_dia[jump_index];
      jump_index++;
    }

    if (digitalRead(SWPIN) == 0 && dinosaur_y == 1) {  // SW is pressed and dinosaur is not in the air, initate jump
      jump_index = 0;
    }

    for (int i = 0; i < 7; i++) { // shift the frame
          frame[i] = frame[i+1];
    }

    int num = random(0, 20); // logic to generate obstacles(bird and rock)
    if (num == 8) {
      frame[7] = 0b00000011;
    } else if (num == 9) {
      frame[7] = 0b00110001;
    } else {
      frame[7] = 0b00000001;
    }


    for (int i = 0; i < 8; i++){ // render the background
      if (i == dinosaur_x)
        spiTransfer(i, frame[i] | 1 << dinosaur_y);
      else
        spiTransfer(i, frame[i]);
    }

    if ((frame[dinosaur_x] & (1 << dinosaur_y)) != 0) { // collide detection
      break;
    }

    count++;
    digits[0] = count/1000 % 10;
    digits[1] = count/100  % 10;
    digits[2] = count/10   % 10;
    digits[3] = count%10;

    vTaskDelay( (500.0/move_rate) / portTICK_PERIOD_MS );
  }

  clear_7seg();

  vTaskDelay( 1000 / portTICK_PERIOD_MS );

  digits[0] = count/1000 % 10;
  digits[1] = count/100  % 10;
  digits[2] = count/10   % 10;
  digits[3] = count%10;
  
  vTaskDelay( 1000 / portTICK_PERIOD_MS );
  
  clear_7seg();

  vTaskDelay( 1000 / portTICK_PERIOD_MS );

  digits[0] = count/1000 % 10;
  digits[1] = count/100  % 10;
  digits[2] = count/10   % 10;
  digits[3] = count%10;

  vTaskDelay( 1000 / portTICK_PERIOD_MS );

  clear_7seg();

  vTaskDelay( 1000 / portTICK_PERIOD_MS );

  digits[0] = count/1000 % 10;
  digits[1] = count/100  % 10;
  digits[2] = count/10   % 10;
  digits[3] = count%10;
  
  vTaskDelay( 1000 / portTICK_PERIOD_MS );
  
  clear_7seg();

  game_running = false;
  vTaskDelete(NULL);
}


void Ultrasonic(void *pvParameters) {
  for(;;) {
    digitalWrite(TRIGPIN, LOW);

    vTaskDelay( 1 / portTICK_PERIOD_MS );

    digitalWrite(TRIGPIN, HIGH);

    vTaskDelay( 1 / portTICK_PERIOD_MS );

    digitalWrite(TRIGPIN, LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    // Calculating the distance
    distance = pulseIn(ECHOPIN, HIGH) * 0.01715;
    }
}



void clear_7seg() {
  for (int i = 0; i < 4; i++)
    digits[i] = 10;
}

void clear_spi() {
  for (int i = 0; i < 8; i++){
    spiTransfer(i, 0b00000000);
  }
}

void spiTransfer(volatile byte opcode, volatile byte data){
  int offset = 0; //only 1 device
  int maxbytes = 2; //16 bits per SPI command

  for(int i = 0; i < maxbytes; i++) { //zero out spi data
    spidata[i] = (byte)0;
  }
  //load in spi data
  spidata[offset+1] = opcode+1;
  spidata[offset] = data;
  digitalWrite(CS, LOW); //
  for(int i=maxbytes;i>0;i--)
    shiftOut(DIN,CLK,MSBFIRST,spidata[i-1]); //shift out 1 byte of data starting with leftmost bit
  digitalWrite(CS,HIGH);
}

