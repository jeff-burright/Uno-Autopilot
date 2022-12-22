// Jeff's Arduino Autopilot Version 6. Stable on Arduino UNO. Uses IBT-2 motor controller, IR remote, red button, 9DOF compass, and LCD. No rudder feedback yet.

 #define Arduino 0
 #define Teensy 1
 #define Board Arduino//  0 = Arduino or  1 = Teensy

  #include <Keypad.h>
  #include <LiquidCrystal.h>
  #include <LiquidCrystal_I2C.h>
  #include <Wire.h>
#include <Bounce2.h>
  #include <L3G.h>
#include <BTS7960.h> 
#include <IRremote.h>
#include <IRremote.hpp>
 #include <LSM303.h>

/******       USER INPUT       ********/

#define Compass 0 //  0 for Pololu, 1 for BNO055
#define IMU 2 // Used to select Pololu IMUs Versions and calibration set. 
    //Allowed values: 2 IMU9 V2; 93 (jacksIMU9V3); 103 (jacks IMU10V3; 51 (Jacks IMU9V5 #1); 52 (Jacks IMU9 V5 #2)
    //determines which set of calibration data is used these extra versions were added to code 7/11/17 J14.4
    // see code lines this tab about 390 to 438 to enter your calibration data
#define GPS_Used 0 // 1 to include GPS code, 0 to exclude it
#define Motor_Controller 3  // 1 for Pololu Qik dual controller, 2 for Pololu Trex dual controller, 3 for Pololu Simple Controller
#define Clutch_Solenoid 0 // 1 a clutch solenoid is used, 0 is not used, currently clutch solenoid does not work with single Simple Pololou Controller
#define RUDDER_MODE 1 // 0 uses rudder position, 1 does not
#define RF24_Attached 0 // 0 if RF 24 radio modules are not attached, 1 if they are used
#define Wind_Input 0 // 1 to use NMEA wind data. 0 to not use wind data
#define RUDDER_OFFSET 0 // 1 uses rudder offset, 0 does not
#define BEARINGRATE_OFFSET 1 // 1 to use 0 to not use
 // float PID_Ks[4] = {.75,.4,.01,0}; // use with rudder_command = rudder_command  + PID_output;
 float PID_Ks[4] = {2, .4, 2, .0005};  // [ K(overall), K(heading error), K(differential error), K(integral error)] // use with rudder_command = PID_output
 // when tested summer 2013 used 2, .4, 4, 0.  Used {2, .4, 1, .000} in 2015. Used {1, .4, 2, .000} operationg without Rudder indicator
 #define PID_MODE 0 // See description PID tab.
 boolean Accept_Terms = 0; //  1 user must accept terms to run.  0 user accepts terms by setting this value to 0
 boolean just_started = 0; // to do a second setup so get a better gyro startup
 //float Kxte[3] = {0, 0, 0}; // used for XTE PID, use this to zero out XTE tracking
//float Kxte[3] = {.2, 0, 0}; // {.2, 4, .0004} baseline; {.05, .5, .0005}last used;  0 is proportional, 1 is differential, 2 is integral error, see GPS_Steer() in PID
                        // .36 will give 45 deg correction at 120 ft XTE to emulate my Garmin GPSMAP 740s see PID tab, voidActual_Gps_Steering()
 float K_course_avg = .999; //used to smooth gps course in PID/ void Actual_GPS_Steering().999 will smooth over 1000 points
 float Maximum_Rudder = 42; // rudder in degrees
 // User set motorspeedMIN around lines 359, 360, 371 for your controller type and rudder steering motor Use crtl-F to find motorspeedMIN 
 float Tack_Angle = 100;  // angle throug which boat will tack when tack L or R pressed (keys 4 and 6 in TACK mode(3)
 int Tack_rudder_MAX = 32;// limits rudder so it doesn't slow boat too much,  need to tune
 float Tack_rudder_speed = .5; // rudder speed during tack , value * full speed, will use min of tack speed and regular speed, user adjust
 float Rudder_Offset = 0; // see notes 10.21.16
 float bearingrate_Offset = 0;
 float MagVar_default = 18.4;// 18.4 Seattle   User should keep up to date for loaction.  Pgm will use GPS value if available + east, - west
 boolean Serial_Remote_Is_Connected = 0  ; // 1 remote connected, 0 remote not connected Pgm hangs up if remote data is sent and remote is not connected
 boolean GPS_Auto_Steer = 0; // 0 will cause GPS steering to go to compass mode and maintain heading if waypoint reached. 
                             //  1 will cause GPS to automatically steer to next waypoint.  NOTE THIS CAN BE DANGEROUS
 boolean Use_CTS = 0  ;// 0 no, 1 yes;  if the GPS generates a GPS course to steer and you want to use it instead of calculating course to steer from BOD and XTE                            
 //float bearingrate_correction = 0.0; //use to get avg stationary bearing rate to read 0 on screen 2


 #define SW2_Used 0 // 0 if SW not used. 1 if SW2 used, must be 1 to use SW2 on wired remote
 #define UseBarometer 0 // 1 to use, 0 to not use, this works with Pololu Alt IMU-10 v3 and v5 Prints barometric pressure and temperature on screen 4, measured at the compass 
 #define Wind_Steer_Direct 0 /* 0 should be used for default which uses wind-error or GPS_error to compute heading_error but then all steering is based on compass steering.
                              If set to 1 the PID will use the Wind error or the GPS course error directly without using the compass (compass still needed for bearing rate)
                             see lines around PID 45 for implementation.  This change implemented 8/18/17 version J14.4 barometer 
                             */
 #define GPS_Steer_Direct 0 // same as above
 #define TFT_Used 0 //  1 to use tft
 int print_level_max = 1;  //  0 to 4, used to set how much serial monitor detail to print
 // 0 = none, 1=PID Output, 2 Adds parsed GPS data, 3 adds raw GPS input, 4 adds checksum results
 int print_time =5;  // print interval in sec
 boolean Print_ETdata = 0; //prints GPS incoming data turn this off to see actual loop time
 boolean Print_ETtime = 0;  // prints Easy Transfer loop time 1 on 0 off
 boolean Print_heading  = 0 ; // diagnostic serial print heading, interval set in A_P_loop
 boolean Print_LCD_IMU9 = 0;  //prints Head, Pitch, Roll, Yaw on LCD conflicts with other LCD Prints
 boolean Print_LCD_AP = 1; // prints main A/P LCD output data and Menus, only do one of these at a time
 boolean Print_Gyro = 0; //  Prints LCD and Serial scaled X(roll), Y(pitch), Z(Yaw) deg/sec
 boolean Print_PID = 0;
 boolean Print_UTC = 0;
 boolean print_Nav_Data = 0; // Print_1 Tab
 boolean Print_Motor_Commands = 0;  // prints rudder commands in PID tab
 boolean Print_Anticpate_Turn = 0;  // prints data from void Actual_GPS_Steering to evaluate Anticipate turn function
 int print_level=print_level_max;
//  print modes for MinIMU9
/*For debugging purposes*/
//OUTPUTMODE=1 will print the corrected data, 
//OUTPUTMODE=0 will print uncorrected data of the gyros (with drift)
#define OUTPUTMODE 1
#define PRINT_DATA 0 // 1 to print serial data or run python display
#define PRINT_EULER 0  //Will print the Euler angles Roll, Pitch and Yaw, needed for python display
//#define PRINT_DCM 0     //Will print the whole direction cosine matrix
#define PRINT_ANALOGS 0 //Will print the analog raw data

 
 //*************  GPS USERS INPUT  *******************

/*
// GPS parameters needed elsewhere (wind and LCD)
int PT_old = 0; // for pritnt timer
String GPS_status= "NO GPS";
float Avg_course; 
float CTS_GPS2;
float GPS_course_to_steer;
float course_error;
float AVG_tracking_error;
float XTE_integral_error;
float XTE_course_correction;
unsigned long UTC_timer;
unsigned long UTC_timer_old;
boolean GPS_Available = 0;
boolean GPS_Was_Available = 0;
int MSG = 0; // message to send number for message display on serial remote
 int j_MAX = 0;
double float1;
double float2;
double float3;
char *brkb, *pEnd;
String string1;
String string2;
unsigned  long long1;
char char_buf[16];
float course; 
long checksum_received = 0; // or static byte
int checksum = 0;
boolean checksum_status = false;
String data_IN[17];
String Active_waypoint;
float XTE;
//float XTE_differential_error;
boolean GPRMC_fix = false;
double Bearing_to_destination = 0;
float course_to_steer;
double Bearing_origin_to_destination = 0; 
unsigned long UTC; 
float NEXT_TURN;
float Next_Turn[20];
double Range_Destination;
double Bearing_to_destination_by_LatLon = 0;
int WPT_index = 0;
double Waypoint_Bearing_From[20]; // beraing from previous waypoint to this waypoint 

*/
float MagVar; //Magnetic Variation E is plus, W is minus 

  
//******  COMPASS  ***************

 #include <L3G.h>  // get library from Pololu http://www.pololu.com/product/1268
 #include <LSM303.h>  // get library from Pololu, use version that corresponds to your version of the IMU




// LCD library code:

// initialize the LCD with the numbers of the interface pins FOR LCD pins (RS,E,D4,D5,D6,D7)
//LiquidCrystal lcd(41,43,45,47,49,39); // matches Jack's wiring
LiquidCrystal_I2C lcd(0x27,16,2); //Addr: 0x3F, 20 chars & 4 lines, for serial LCD

//LiquidCrystal lcd(39,41,43,45,47,49); matches Fritz JNE Autopilot V6D
long lcdtimer=0;   //LCD Print timer
 

// Red button settings
boolean toggle = false;
#define BUTTON_PIN 4 
Bounce debouncer = Bounce(); // Instantiate a Bounce object



// IR Remote settings
int IR_RECEIVE_PIN = 13;
  



//const int motorspeed = 32; // 0 t0 127  // use with pololu motor controller

 //Servo myservo;  // used to operate a servo to simulate the rudder
 float Magnetic_Variation; 
 float heading_error =0;
 float differential_error = 0;
 float integral_error = 0; 
 float deadband;  // steering deadband
 float rudder_position = 0;
 float rudder_command = 0;
 float rudder_error;
 int motorspeed;  // sets rudder motor speed in Pololu controller can be constant or variable
 int rudder_MAX; //allows rudder to be changed for various maneuvers like TACK
 boolean TACK_ON = false;  // goes on for a tack and off when within 10 deg final course
// float rudder_total_time;
 float bearingrate=0;
 float bearingrate_smoothed;
 float bearingrate2;
 unsigned long bearingrate2_time;
 float heading_old;
 float delta_heading;
 long delta_compass_time; // computed in compass tab, used in PID for integral error
 float PID_output = 0; 
// float GPS_PID = 0;
 boolean GPS_Steering = false;
 int Steering_Mode = 0;
 String Mode = "OFF";
 String Previous_Mode;
 boolean Steering = false;
 boolean sw1_turned_on = false;
 boolean sw1 = false;
 boolean sw2 = false;
 boolean sw1Status = 1;
 boolean DODGE_MODE = false;
 int Screen=0;
 boolean rudder_on;
 boolean rudder_was_off;
 unsigned long rudder_time_old;
  

  #if Motor_Controller == 3 // Pololu Simple controller
 // Jeff addition for IBT-2 motor controller

const uint8_t EN = 8;
const uint8_t L_PWM = 5;
const uint8_t R_PWM = 6;
BTS7960 motorController(EN, L_PWM, R_PWM);


//L298N motor(11,12); Jeff addition commented out for IBT-2 controller

   int motorspeedMIN = 30; // was 555. this value is the minimum speed sent to controller if left or right rudder is commanded
                           //  rudder stop will still send 0. Use to overcome starting torque. Set so if rudder error greater than the deadband the rudder
                           //  moves at a noticable but slow speed.  Higher values will be more responsive. 
   int motorspeedMAX = 255; // was 3200. jeff changed.
 #endif


 // wind variables defined outside #if Wind_Input to compile without complexity in LCD tab

 
/***********  COMPASS  SET UP  **************************************/
float heading;
float heading_to_steer=0; //  see PID
float MAG_Heading_Degrees; //LCD_compass tab

//float system_cal; float gyro_cal;  float accel_cal; float mag_cal;
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
long timer=0;   //general purpuse timer
long timer_old;
long timer24=0; //Second timer used to print values 
unsigned int counter=0;
unsigned int counter2=0;
unsigned long counter3=0; // used for 10 minute print interval in A_P_Loop
float roll;
float pitch;
float yaw;

#if Compass == 0
// setup data for MinIMU9 from Tab minIMU9AHRS of Pololu software
int SENSOR_SIGN[9] = {1,-1,-1, -1,1,1, 1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168
// LSM303 accelerometer: 8 g sensitivity
// 3.8 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 
// L3G4200D gyro: 2000 dps full scale  see tab I2C line 60 to set Full scale value
// 70 mdps/digit; 1 dps = 0.07 use .07 for FS 2000dps, use .00875 for FS 245 dps. see I2C about line 64 to set full scale
#define Gyro_Gain_X .00875 //X axis Gyro gain .07 for FS 2000 DPS, .00875 for full scale of 245 degrees per second
#define Gyro_Gain_Y .00875 //Y axis Gyro gain
#define Gyro_Gain_Z .00875 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board

// this is data for Jacks IMU9V2,
#if IMU == 2 // Jack's version 2 IMU calibration 
  #define M_X_MIN -663   
  #define M_Y_MIN -683
  #define M_Z_MIN -611   
  #define M_X_MAX 453
  #define M_Y_MAX 427
  #define M_Z_MAX 460 
#endif

#if IMU == 93 // This is the data for Jacks IMU9V3  new data 7/11/17 taken on the boat
  #define M_X_MIN -3525 // -3350   
  #define M_Y_MIN -3473 // -3276
  #define M_Z_MIN -3398 //-3284   
  #define M_X_MAX 3600 //3500
  #define M_Y_MAX 3226 //3150
  #define M_Z_MAX 2802 //2670 
#endif

#if IMU == 103 // IMU-10 V3
  // cal data for jack's IMU10 V3 calibrated on the boat 7/11/17
  #define M_X_MIN -3290  
  #define M_Y_MIN -3288
  #define M_Z_MIN -2840  
  #define M_X_MAX 3611
  #define M_Y_MAX 3553
  #define M_Z_MAX 4127 
#endif

#if IMU == 51 // this is for IMU V5 #1, put your cal data here
#define IMU_V5 //  This is an added line that came with Pololu IMU AHRS code for using Pololu IMU V5
   // necessary because library function calls are different for V2 - V3 and the calls for V5
// comment out one of these two calibration data sets
#define M_X_MIN -1955 // values for Jack's IMU V5 #1 cal 7/11/17 on the boat
#define M_Y_MIN -4857
#define M_Z_MIN 728
#define M_X_MAX 5459
#define M_Y_MAX 5518
#define M_Z_MAX 7758
#endif

#if IMU == 52 // this is for IMU V5 #2, Can be used for a second IMU 9 or 10 second calibration set for testing
#define IMU_V5 //  This is an added line that came with Pololu IMU AHRS code for using Pololu IMU V5
#define M_X_MIN -2582 // values for Jack's IMU V5 #2 cal 2/2/17
#define M_Y_MIN -6418
#define M_Z_MIN 194
#define M_X_MAX 4837
#define M_Y_MAX 1381
#define M_Z_MAX 7236
#endif

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible
int AN[6]; //array that stores the gyro and accelerometer data
int AN_OFFSET[6]={0,0,0,0,0,0}; //Array that stores the Offset of the sensors
int gyro_x;
int gyro_y;
int gyro_z;
int accel_x;
int accel_y;
int accel_z;
int magnetom_x;
int magnetom_y;
int magnetom_z;
float c_magnetom_x;
float c_magnetom_y;
float c_magnetom_z; 
float MAG_Heading;

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Gyro_Vector[3]= {0,0,0};//Store the gyros turn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

// Euler angles

float errorRollPitch[3]= {0,0,0}; 
float errorYaw[3]= {0,0,0};

byte gyro_sat=0;

float DCM_Matrix[3][3]= {
  {
    1,0,0  }
  ,{
    0,1,0  }
  ,{
    0,0,1  }
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}}; //Gyros here
float Temporary_Matrix[3][3]={
  {
    0,0,0  }
  ,{
    0,0,0  }
  ,{
    0,0,0  }
};
// end IMU-9 data
#endif // end if compass == 0
/************************************/

/*
#if Board == Arduino
#if GPS_Used == 1
// Set up for Easy Transfer to receive GPS data 
 //create object
  EasyTransfer ET, ET2; 
 
  struct RECEIVE_DATA_STRUCTURE{
    //put your variable definitions here for the data you want to receive
    //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float SD_course;
  float SD_course_to_steer;
 // char SD_CTS_MorT[2];
  float SD_SOG;
  float SD_MagVar;
  float SD_XTE;
  float SD_XTE_differential_error;
//  char SD_XTE_LR[2];
 // char SD_XTE_unit[2];
  char SD_Waypoint_next[11]; 
//  boolean SD_NEMA_sentence;
  boolean SD_GPRMC_fix;
  boolean SD_GPAPB_fix;
  float SD_Bearing_origin_to_destination; // Same as course to steer
  char SD_BOD_MorT[2];
 // char SD_Origin_Waypoint[11];
  float SD_Bearing_to_destination;
 // char SD_BTD_MorT[2];
  float SD_Range_Destination;
 // float SD_Velocity_towards_destination; 
  long SD_UTC;
  //long SD_Date;
 // float SD_Lat_current;
//  float SD_Lon_current;
  float SD_NEXT_TURN;
  // int SD_Wind;
 // int SD_Speed;  // wind speed
 // float SD_Time_decimal;
  };
  
  //give a name to the group of data
  RECEIVE_DATA_STRUCTURE ETdata;
  #endif //
  #endif
/*********************************************************************************/  
  // Second ETData structure to send data to to remote controller
/*    
  struct SEND_DATA_STRUCTURE{  //Arduino to Arduino communication Eazy Transfer by Bill Porter
    //put your variable definitions here for the data you want to send
    //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  float course;
  float SOG;
  float MagVar;
  float XTE;
  char Waypoint_next[11]; 
  float Bearing_origin_to_destination;
  float Bearing_to_destination;
  float Range_Destination;
  //long UTC;
  float Next_Turn;
  float heading;
  float heading_to_steer;
  float course_to_steer;
  float bearingrate;
  float rudder_position;
  char Mode[5];
  int MSG;

  
  }; // End Data Sttructure  
  
  SEND_DATA_STRUCTURE ET2data;
*/  





 //*  SETUP    SETUP   SETUP   ***** 

 void setup() {

     Serial.begin(57600); // Serial conection to Serial Monitor   ---- Jeff changed from 57600


     // Pins D5 and D6 - 4 kHz to fix motor noise hopefully?
TCCR0B = 0b00000010; // x8
TCCR0A = 0b00000001; // phase correct


debouncer.attach(BUTTON_PIN,INPUT_PULLUP); // Attach the debouncer to a pin with INPUT_PULLUP mode
debouncer.interval(25); // Use a debounce interval of 25 milliseconds

IrReceiver.begin(IR_RECEIVE_PIN, DISABLE_LED_FEEDBACK);


  // pinMode(RPWM, OUTPUT); //IBT-2
  // pinMode(LPWM, OUTPUT); //
  //#define Serial_MotorControl Serial2

delay(1000); // give chip some warmup on powering up
 
   //Serial1.begin(57600); //Communication to Serial Remote


  //pinMode (LCD_Contrast_Pin, OUTPUT); //LCD Contrast Pin //  conflict with teensy Serial3 could be reassigned
  //analogWrite(LCD_Contrast_Pin, LCD_Contrast); // pwm control of LCD contrast  Replaces 10K pot for contrast control 
 lcd.init(); 
 lcd.setCursor(0,0);

   
 #if Compass == 0
 //SETUP FOR MinIMU9 
 lcd.print("Starting Compass");
     I2C_Init();
   //Serial.println("Pololu MinIMU-9 + Arduino AHRS");
   // digitalWrite(STATUS_LED,LOW);
  delay(1500);
  Accel_Init();
  Compass_Init();
  Gyro_Init();  
  delay(20);  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro();
    Read_Accel();
    for(int y=0; y<6; y++)   // Cumulate values
      AN_OFFSET[y] += AN[y];
    delay(20);
    }    
  for(int y=0; y<6; y++)
    AN_OFFSET[y] = AN_OFFSET[y]/32;   
    AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];  
  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
   // Serial.println(AN_OFFSET[y]);
 
  delay(2000);
 // digitalWrite(STATUS_LED,HIGH);
    
  timer=millis();
  delay(20);
  counter=0;
  // End setup data for MinIMU9
 #endif
 
  //if(Motor_Controller == 1) Serial_MotorControl.write(170); // sends packet that is detected for auto baud rate detection and starts normal operation not need for Trex
 // if(Motor_Controller == 3)  // ditto for Pololu Simple controller
  {
  //Serial_MotorControl.write(170);
  //Serial_MotorControl.write(131);
  }

 Key0_Pressed();

 }  // end setup
  
/*********************************************/
 //motorController.Enable();



void loop()
{

 #if Compass == 0
  if(just_started)
  {  setup();  //this runs setup a second time because I (JE) find gyros zero state does not initialize when powered up, runs once
     just_started = 0;
  }
 #endif 
 

//Rudder_Stop(); // Jeff added this in to begin in motor off position


redbutton();
IRREMOTE();

A_P_Loop(); // Autopilot Loop


 }    

