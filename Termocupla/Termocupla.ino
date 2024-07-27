/*    Max6675 Module  ==>   Arduino
 *    CS              ==>     D10
 *    SO              ==>     D12
 *    SCK             ==>     D13
 *    Vcc             ==>     Vcc (5v)
 *    Gnd             ==>     Gnd      */

#include <SPI.h>
#include <PID_v1.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

#define MAX6675_CS   10
#define MAX6675_SO   12
#define MAX6675_SCK  13

#define SETPOINT_INIT 150;
#define SLEEP 20


// Entradas del encoder rotatorio 
#define CLK 2
#define DT 3
#define BOTONCENTRO 7
#define BOTONBAJAR 4

volatile int counter = 0;
int currentStateCLK;
int lastStateCLK;
String currentDir ="";

bool enableShow = false;

LiquidCrystal_I2C lcd(0x27,16,2);
//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
float temperature_read;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(115200);
  pinMode(A2, OUTPUT);
  pinMode(BOTONCENTRO, INPUT_PULLUP);
  pinMode(BOTONBAJAR, INPUT_PULLUP);
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  lastStateCLK = digitalRead(CLK);
  
  // LLamar a updateEncoder() cuando un high o un low haya cambiado 
  // sobre la interrupcion 0 (pin 2), o interrupcion 1 (pin 3)
  attachInterrupt(0, updateEncoder, CHANGE);
  attachInterrupt(1, updateEncoder, CHANGE);
  Input = 20;
  Setpoint = SETPOINT_INIT;
  counter = Setpoint;
  myPID.SetMode(AUTOMATIC);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("Solder Station");
  lcd.setCursor(0,1);
  lcd.print("candelo@gmail.com");
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Set:");
  lcd.setCursor(0,1);
  lcd.print("Real:");
  Pitodoble();
}


unsigned long previousMillis = 0;    
unsigned long previousMillis2 = 0;   
unsigned long previousMillis3 = 0;
const long interval = 1000; 
const long interval2 = 300; 
const long interval3 = 150; 
unsigned int seconds = 0;
unsigned int minutes = 0;

bool isReady = false;

void loop() {

  
    unsigned long currentMillis = millis();
    if ((currentMillis - previousMillis >= interval) || enableShow) {
      enableShow = false;
      previousMillis = currentMillis;
      ShowTemp();
      ShowTime();
    }
    if (currentMillis - previousMillis2 >= interval2) {
      previousMillis2 = currentMillis;
      CautinProcess();
    }
    if (currentMillis - previousMillis3 >= interval3) {
      previousMillis3 = currentMillis;
      ButtonProcess();
    }

    if(isReady){
      int a = Setpoint;
      int b = Input;
      Serial.println(a);
      Serial.println(b);
      if(a == b){
        isReady = false;
        Pitodoble();
      }
    }
    
}

void ShowTime(){
    
    if(seconds<59){
      seconds++;  
    }else{
      seconds = 0;
      minutes++;
    } 

    if(minutes >= SLEEP){
      Setpoint = SETPOINT_INIT;
      lcd.setCursor(10,1);  
      lcd.print("Sleep");
    }else{
      lcd.setCursor(10,1);  
      lcd.print("Ready");
    }

    lcd.setCursor(11,0);
    if(minutes < 10){
        lcd.print("0");
    }
    lcd.print(minutes);
    lcd.print(":");
    if(seconds < 10){
        lcd.print("0");
    }
    lcd.print(seconds);   
}

void ButtonProcess(){
      if(digitalRead(BOTONCENTRO) == 0){
        delay(80);
        if(Setpoint < 600.0 && digitalRead(BOTONCENTRO) == 0){
          //Setpoint +=10;
          Setpoint = 300;
          ShowTemp();
          isReady = true;
          seconds = 0;
          minutes = 0;
          //Pito();
        }
      }else if(digitalRead(BOTONBAJAR) == 0){
        delay(80);
        if(Setpoint > 150.0 && digitalRead(BOTONBAJAR) == 0){
          Setpoint -=10;
          ShowTemp();
          isReady = true;
          seconds = 0;
          minutes = 0;          
          //Pito();
        }
      }
}

void Pitodoble(){
  digitalWrite(A2,HIGH);
  delay(100);
  digitalWrite(A2,LOW);  
  delay(100);
  digitalWrite(A2,HIGH);
  delay(100);
  digitalWrite(A2,LOW);  
}


void Pito(){
  digitalWrite(A2,HIGH);
  delay(100);
  digitalWrite(A2,LOW);   
}

void CautinProcess(){
  temperature_read = readThermocouple();
  temperature_read/=1.5;
  Input = temperature_read;
  myPID.Compute();
  analogWrite(6, Output);     
}


void ShowTemp(){
  
  Serial.print("TEMP: ");
  Serial.print(temperature_read);    
  Serial.print(" - ");    
  Serial.println(Output); 
  
  lcd.setCursor(0,0);
  lcd.print("Set: ");
  lcd.print(Setpoint,0);
  lcd.print("C");
  lcd.setCursor(0,1);
  lcd.print("Real:");  
  lcd.print(Input,0);
  lcd.print("C");
}


double readThermocouple() {

  uint16_t v;
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);
  pinMode(MAX6675_SCK, OUTPUT);
  
  digitalWrite(MAX6675_CS, LOW);
  delay(1);

  // Read in 16 bits,
  //  15    = 0 always
  //  14..2 = 0.25 degree counts MSB First
  //  2     = 1 if thermocouple is open circuit  
  //  1..0  = uninteresting status
  
  v = shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  v <<= 8;
  v |= shiftIn(MAX6675_SO, MAX6675_SCK, MSBFIRST);
  
  digitalWrite(MAX6675_CS, HIGH);
  if (v & 0x4) 
  {    
    // Bit 2 indicates if the thermocouple is disconnected
    return NAN;     
  }

  // The lower three bits (0,1,2) are discarded status bits
  v >>= 3;

  // The remaining bits are the number of 0.25 degree (C) counts
  return v*0.25;
}

void updateEncoder(){
  // leer el estado actual del CLK
  currentStateCLK = digitalRead(CLK);

// Si el ultimo estado actual del CLK es distinto, entonces ocurrió un pulso 
// reacciona solo a un cambio de estado para evitar un conteo doble
  if (currentStateCLK != lastStateCLK && currentStateCLK == 1){

// si el estado del DT es diferente que el estado del CLK 
// entonces el encoder está rotando en sentido antihorario CCW asi que //decrementa   
if (digitalRead(DT) != currentStateCLK) {
    if(counter <= 590)
        counter += 10;
     currentDir ="CCW";
    }else {
// Encoder está rotando en sentido horario CW así que incrementa
     if(counter > 150)
        counter -=10;
     currentDir ="CW";
    }
    Setpoint = counter;
    //ShowTemp();
    isReady = true;
    seconds = 0;
    minutes = 0;
    enableShow = true;
    Serial.print("Direction: ");
    Serial.print(currentDir);
    Serial.print(" | Counter: ");
    Serial.println(counter);
  }

  // guarda el ultimo estado del CLK 
  lastStateCLK = currentStateCLK;
}
