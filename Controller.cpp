#include<Wire.h>
#include <RTClib.h>
#include <MPU6050_light.h>
MPU6050 mpu(Wire);
RTC_DS3231 rtc;
DateTime now; // for acessing the time

#define cs1 A0
#define cs2 A1
#define cs3 A2
#define cs4 A3
#define yaw_clock 3
#define yaw_anti_clock 5
#define pitch 6
#define gripper 9
void yaw_clockwise_pwm(unsigned char );
void yaw_anticlockwise_pwm(unsigned char );
void pitch_pwm(unsigned char);
void gripper_pwm(unsigned char);
void get_currents(void);
void mpu_angles(void);
void get_time(void);
 // for storing the timer value
struct timer
{
    unsigned char hour;
    unsigned char min;
    unsigned char sec;

}current_time;


double current_sensor[4]={0}; // for storing current sensor values
double x,y,z; // for storing orientation angles in x,y,z direction
void setup(){
     Wire.begin();
    Serial.begin(9600);
    pinMode(cs1,INPUT);
    pinMode(cs2,INPUT);
    pinMode(cs3,INPUT);
    pinMode(cs4,INPUT);
    pinMode(yaw_clock,OUTPUT);
    pinMode(yaw_anti_clock,OUTPUT);
    pinMode(pitch,OUTPUT);
    pinMode(gripper,OUTPUT);

}
// Main program starts
void loop(){





}
// Main program ends
void yaw_clockwise_pwm(unsigned char pwm){

    Serial.print("Rotating clockwise with pwm: ");
    Serial.println(pwm);
    analogWrite(yaw_clock,pwm);
    
}
void yaw_anticlockwise_pwm(unsigned char pwm){

    Serial.print("Rotating anti clockwise with pwm: ");
    Serial.println(pwm);
    analogWrite(yaw_anti_clock,pwm);

}
void pitch_pwm(unsigned char pwm){
    Serial.print("Pitch motion with pwm: ");
    Serial.println(pwm);
    analogWrite(pitch,pwm);
}
void gripper_pwm(unsigned char pwm){
    Serial.print("Gripper is actuating with pwm: ");
    Serial.println(pwm);
    analogWrite(gripper,pwm);
}
void get_currents(void){

    double cur[4] = {0.0}, samples[4] = {0.0},voltage[4] = {0};
  for (unsigned char x = 0; x < 10; x++)          //Get 10 samples for better measurment
  {
    cur[0] = analogRead(cs1);    //Read current sensor values 
    cur[1] = analogRead(cs2); 
    cur[2] = analogRead(cs3); 
    cur[3] = analogRead(cs4);            
    samples[0] += cur[0];        //Add samples together
    samples[1] += cur[1];  
    samples[2] += cur[2];  
    samples[3] += cur[3];  
    delay (3);                           // let ADC settle before next sample 3ms
  }
  cur[0]=samples[0]/10.0;                   //Taking Average of Samples
  cur[1]=samples[1]/10.0;  
  cur[2]=samples[2]/10.0;  
  cur[3]=samples[3]/10.0; 

  voltage[0]=cur[0]*(5.0 / 1024.0);         //((AvgAcs * (5.0 / 1024.0)) is converitng the read voltage in 0-5 volts
  voltage[1]=cur[1]*(5.0 / 1024.0); 
  voltage[2]=cur[2]*(5.0 / 1024.0); 
  voltage[3]=cur[3]*(5.0 / 1024.0); 
  
  current_sensor[0] = (voltage[0]-2.5)/0.185; //2.5 is offset,,,   0.185v is rise in output voltage when 1A current flows at input
  current_sensor[1] = (voltage[1]-2.5)/0.185;
  current_sensor[2] = (voltage[2]-2.5)/0.185;
  current_sensor[3] = (voltage[3]-2.5)/0.185;
  Serial.println("Current reading is done, stored in current_sensor array in Amperes :");
 
}
void mpu_angles(void){

    Wire.beginTransmission(0x68); // enable the communication
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ 
        Serial.println("MPU6050 is not Found Stucked in while loop");
        while(1);

   } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
  mpu.update();

	x = mpu.getAngleX();
	
	y = mpu.getAngleY();
	
	z = mpu.getAngleZ();
	
    Serial.print("Orientation calculated: ");
    Serial.print("X: ");
    Serial.print(x);
    Serial.print(" | ");
    Serial.print("Y: ");
    Serial.print(y);
    Serial.print(" | ");
    Serial.print("Z: ");
    Serial.println(z);
    Wire.endTransmission(true);  // Disable the communication with stop bit
}

void get_time(void){

    Wire.beginTransmission(0x68); // enable the communication

    if (! rtc.begin()) 
    {
            Serial.println("DS1307 RTC Module not Found Stucked in while loop");
             while (1);
    }

  if (rtc.lostPower()) 
  {

       Serial.println("RTC power failure, resetting the time!");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  }
    
    now = rtc.now();
    current_time.hour = now.hour();
    current_time.min = now.minute();
    current_time.sec = now.second();
     // now.day(), now.year(), now.month() for monitoring the day year month
    Serial.print("System time calculated: ");
    Serial.print(current_time.hour);
    Serial.print(" : ");
    Serial.print(current_time.min);
    Serial.print(" : ");
    Serial.println(current_time.sec);

     Wire.endTransmission(true);  // Disable the communication with stop bit

}
