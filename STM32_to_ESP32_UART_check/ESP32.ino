//data receive from stm32
uint8_t led_number;
//declare led gpio pin number
uint8_t led_1=25, led_2=32, led_3=33;

void setup() {
  //declare LEDs pin as output
  pinMode(25, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  //set pins 25, 32, 33 as HIGH to turn off LEDs when startup
  digitalWrite(25, HIGH);
  digitalWrite(32, HIGH);
  digitalWrite(33, HIGH);
  //Software serial begin                                          // !!!! Hardware serial UART 17-Tx, 16-Rx
  Serial1.begin(9600, SERIAL_8N1, 17, 16);  // 17 -Rx, 16-Tx


  while(1)
  { 
    //Get data from STM32 UART 7
    led_number=Serial1.read();
    if(led_number=='1')
      { 
        //set pin 25 as LOW to turn on LED
        digitalWrite(25, LOW);
      }
    else if(led_number=='2')
      { 
        //set pin 32 as LOW to turn on LED
        digitalWrite(32, LOW);
      }
    else if(led_number=='3')
      { 
        //set pin 33 as LOW to turn on LED
        digitalWrite(33, LOW);
      }
    else if(led_number=='0')
     {
      //set 25, 32, 33 as HIGH to turn off all the LEDs.
      digitalWrite(25, HIGH);
      digitalWrite(32, HIGH);
      digitalWrite(33, HIGH);
     }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
}
