#define INPUT_SIZE             8
#define OUTPUT_SIZE            50

// PIN DEFINITIONS
#define Tsensor                A5
#define Input0                 A0
#define Input1                 A1
#define Input2                 A2
#define Input3                 A3
#define Input4                 A4
#define FanPWM                 10                                          // Pin PWM
#define LED                    13 

// COMMUNICATION COMMANDS
#define D1ST                  "D1ST"                                      // D1 Start Transmission
#define D2ST                  "D2ST"                                      // D2 Start Transmission
#define D1SP                  "D1SP"                                      // D1 Start Pairing
#define D2SP                  "D2SP"                                      // D2 Start Pairing
#define D1DC                  "D1DC"                                      // D1 Disconnect
#define D2DC                  "D2DC"                                      // D2 Disconnect
#define D1CP                  "D1CP"                                      // D1 Confirm Pairing
#define D1RT                  "D1RT"                                      // D1 Received Transmission
#define RESET                 "_RST"                                      // RESET
#define FAN_SPEED             "FANS"                                      //
#define FAN_AIRING_OFF        "AIR0"                                      //
#define FAN_AIRING_ON         "AIR1"                                      //
#define DATA                  "DATA"                                      // DATA tag
#define READY                 "RDY_"                                      // DATA tag

// FAN DEFINITIONS
#define FAN_PWM_SPEED_DEFAULT 72                                          // initial fan speed
#define FAN_PWM_START_SPEED   225

int FAN_PWM_current_speed = FAN_PWM_START_SPEED;

unsigned long curr_time;                                                  //
unsigned long time_start;                                                 //
unsigned long sensor_data[5];                                             // Aarray with sensor data
unsigned long Temp;                                                       // temperature
unsigned int N_oversampling = 4^1;                                        // oversampling
char buffer[OUTPUT_SIZE];                                                 // output buffer
bool in_test_state = false;                                               // state var
char str_input[INPUT_SIZE + 1];                                           // input buffer

void(*resetFunc) (void) = 0;                                              // Reset Function

void setup() {
  Serial.setTimeout(5U);
  analogReference(DEFAULT);                                               // set reference voltage for ADC
  Serial.begin(115200);                                                   // open serial port
  pinMode(FanPWM, OUTPUT);                                                // fan control signal pin
  analogWrite(FanPWM, 0);                                                 
  pinMode(LED, OUTPUT); 
  delay(500);
  FAN_PWM_current_speed = FAN_PWM_SPEED_DEFAULT;
  writeFan();                                                             // set initial fan speed
}

void loop() {
  digitalWrite(LED, LOW);
  while (!pairing()) {};                                                // pair devices
  digitalWrite(LED, HIGH);
  while(1)
  { 
    readDataFromSensors();
    readLine();
    if (!(memcmp(str_input, DATA, 4))) { 
      writeBufferToSerial();       
      sendDataToPC();
      readLine();
    } 
    else if (!(memcmp(str_input, RESET, 4))) resetFunc();
    else if (!(memcmp(str_input, FAN_SPEED, 4)))  getFanSpeed();
    else if (!(memcmp(str_input, D1DC, 4)))  break;
  }
}              


inline void readLine()
{
  Serial.readBytes(str_input, INPUT_SIZE);
  str_input[INPUT_SIZE] = 0;
}

inline void readDataFromSensors() {
  Temp = analogRead_(Tsensor, N_oversampling);                           // read temperature
  sensor_data[0] = analogRead_(Input0, N_oversampling);                  // read sensors
  sensor_data[1] = analogRead_(Input1, N_oversampling);
  sensor_data[2] = analogRead_(Input2, N_oversampling);
  sensor_data[3] = analogRead_(Input3, N_oversampling);
  sensor_data[4] = analogRead_(Input4, N_oversampling);
}

inline void sendDataToPC()
{
  clearBuffer();
  sprintf(buffer, "%s%4lu;%4lu;%4lu;%4lu;%4lu;%4lu;%3d\n",DATA,sensor_data[0],sensor_data[1],sensor_data[2],sensor_data[3],sensor_data[4],Temp,FAN_PWM_current_speed); // write to buffer
  writeBufferToSerial();
}

bool pairing() {
  while (true) { 
    if (!(Serial.available() < INPUT_SIZE)) readLine();
    if (!(memcmp(str_input, D1SP, 4)))
    {
      clearBuffer();
      sprintf(buffer, "%s\n", D2SP); 
      writeBufferToSerial();
    }
    else if (!(memcmp(str_input, D1CP, 4))) break;
    else
    {
      clearBuffer();
      sprintf(buffer, "%s\n", D2DC); 
      writeBufferToSerial();
    }
  }
  return true;
}

inline unsigned long analogRead_(int pin, unsigned int samples) {
  double sum = 0.0;
  analogRead(pin); // dummy
  for(int i=0; i < samples; i++) {
    sum += (double) analogRead(pin); // reading from ADC
  }
  sum = round((sum / (double) samples) * 10.0);
  return  sum;
}

inline void writeFan() {
  analogWrite(FanPWM, FAN_PWM_current_speed);
}

inline void clearBuffer(){
  memset(buffer, 32, OUTPUT_SIZE);
}

inline void writeBufferToSerial(){
  Serial.write(buffer);
  Serial.flush();
}

void serialInputFlush() {
 while (true)
   {
    delay(1);
    if (Serial.available ())
    {
      while (Serial.available ()) Serial.read ();
      continue;
    }
    else break;
   }
}

void getFanSpeed() {
  char sfanspeed[4];
  memcpy(sfanspeed, str_input + 4, 4);
  FAN_PWM_current_speed = (int) atoi(sfanspeed);
  clearBuffer();
  sprintf(buffer, "%4s%3d\n", FAN_SPEED, FAN_PWM_current_speed); // Write to buffer
  writeBufferToSerial();
}
