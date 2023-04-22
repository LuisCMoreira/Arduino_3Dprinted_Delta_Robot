#include <Servo.h>

int x_axis=A5;
int y_axis=A3;

int grip_out=A1;

int inp_joy=A0;

int z_up_inp=5;
int z_dw_inp=6;

int func_1=7;

float p0 = 109.26, p1 = 57.5, p2 = 255, p3 = 2.5;

//Testes

 // robot geometry
 // (look at pics above for explanation)
 const float e = 6*(p3)/sqrt(3.0);     // end effector
 const float f = 6*(p0/2)/sqrt(3.0);     // base
 const float re = p2;
 const float rf = p1;
 
 // trigonometric constants
 const float sqrt3 = sqrt(3.0);
 const float pi = 3.141592653;    // PI
 const float sin120 = sqrt3/2.0;   
 const float cos120 = -0.5;        
 const float tan60 = sqrt3;
 const float sin30 = 0.5;
 const float tan30 = 1/sqrt3;

unsigned long tempo_dirkin=0;

unsigned long tempo_invkin=0;

float last_spk_x=0;
float last_spk_y=0;
float last_spk_z=0;

float theta1_last,theta2_last,theta3_last;

/////////////////////////////////////////////////////////////////////////////////////////////////////////7

int speedp=100;

int accustep=5;


int fun_1_loop_aux=LOW;


int servo1_dw_lim=-50;
int servo1_up_lim=90;
int servo2_dw_lim=-50;
int servo2_up_lim=90;
int servo3_dw_lim=-50;
int servo3_up_lim=90;



int z_dw_lim=-170;

//0's do robot; 
int downJ1=160;
int downJ2=145;
int downJ3=160;

//90's do robot
int midJ1=65;
int midJ2=60;
int midJ3=75;

float trans_lin_x=0;
float trans_lin_y=0;
float trans_lin_z=-220;

float Tjoy_xm=trans_lin_x;
float Tjoy_ym=trans_lin_y;
float Tjoy_zm=trans_lin_z;

float z0_last=trans_lin_x;
float y0_last=trans_lin_y;
float x0_last=trans_lin_z;

float Tjoy_x_last=0,Tjoy_y_last=0,Tjoy_z_last=0;

//float  Tjoy_z, Tjoy_y, Tjoy_x;


unsigned long timer_zup;
unsigned long timer_zdw;

unsigned long timer_ijoy;

unsigned long timer_serial;

bool MANUAL=LOW, SERIALCOM=LOW, CYCLE=HIGH;
String Mode="Cyc";
String ModeString=":"+Mode;

  bool cycle_on=LOW;

    int step_var=0;



String T_State="O";
  
bool h_start=HIGH;
bool busy_move=LOW;
bool grip=LOW;
bool joy_out=LOW;

int prog_1_x[100];
int prog_1_y[100];
int prog_1_z[100];


  int mode_var=0;
  bool mode_change=LOW;


  bool no_lmove=true;
 int cyc_count=0;
 bool run_cycl=false;

Servo myservo1;  // create servo object to control a servo
Servo myservo2;  // create servo object to control a servo
Servo myservo3;  // create servo object to control a servo

int pos = 0;    // variable to store the servo position



// include the library code:
#include <LiquidCrystal.h>

// Creates an LCD object. Parameters: (rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(8, 9, 10, 11, 12, 13);



String readString, data;
float X, Y, Z, CL, CHK; 







///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

byte servoMin=-50;

byte servoPosX = 0;
byte newServoPosX = servoMin;
byte servoPosY = 0;
byte newServoPosY = servoMin;
byte servoPosZ = 0;
byte newServoPosZ = servoMin;

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};
int newFlashInterval = 0;
float servoFractionX = 0.0; // fraction of servo range to move
float servoFractionY = 0.0;
float servoFractionZ = -280;

unsigned long curMillis;

unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 1000;

float last_sFX=0;
float last_sFY=0;
float last_sFZ=0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {
  
  myservo1.attach(2);  // pin frontmotor - y axiz pos
  myservo2.attach(3);  // pin leftmotor
  myservo3.attach(4);  // pin rightmotor

   pinMode(z_up_inp, INPUT);
   pinMode(z_dw_inp, INPUT);
   
   pinMode(inp_joy, INPUT);   
   pinMode(func_1, INPUT);
   
   pinMode(grip_out, OUTPUT);


  

Serial.begin(115200);





// include the library code:

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  // Clears the LCD screen
  lcd.clear();



  
  //moveServo();
  jmove_trans(0,0,-280);
  delay(500);
  
    // tell the PC we are ready
  Serial.println("<Arduino is ready>");
}

void loop() {


if(run_cycl==true)
{
goto pone;
  }

  
go_home();

state_control();


pone:
 run_cycle(); 



       
}



void getDataFromPC() {

    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

//=============
 
void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
  
  //strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  //newFlashInterval = atoi(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  servoFractionX = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ","); 
  servoFractionY = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ","); 
  servoFractionZ = atof(strtokIndx);     // convert this part to a float

}

//=============

void replyToPC() {

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<Msg ");
    //Serial.print(messageFromPC);
    //Serial.print(" X ");
    //Serial.print(servoFractionX);
    //Serial.print(" Y ");
    //Serial.print(servoFractionY);
    //Serial.print(" Z ");
    //Serial.print(servoFractionZ);
    //Serial.print(" Time ");
    //Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");
  }
}

/*
void updateServoPos() {

  byte servoRange = servoMax - servoMin;
  if (servoFractionX >= 0 && servoFractionX <= 1) {
    newServoPosX = servoMin + ((float) servoRange * servoFractionX);
  }
    if (servoFractionY >= 0 && servoFractionY <= 1) {
    newServoPosY = servoMin + ((float) servoRange * servoFractionY);
  }
  
    if (servoFractionZ >= 0 && servoFractionZ <= 1) {
    newServoPosZ = servoMin + ((float) servoRange * servoFractionZ);
  }
}

*/





//*************************************************************************************************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//COMUNICAÇÃO


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************************************************************************************************************************************************************************


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PUBLICADOR

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void serial_talker()
{
 int fun_1_loop=digitalRead(func_1);
 

 if (fun_1_loop==HIGH)
 {fun_1_loop_aux=HIGH;}


if ((abs(z0_last-last_spk_z)>5)||(abs(y0_last-last_spk_y)>5)||(abs(x0_last-last_spk_x)>5)||(fun_1_loop_aux==HIGH))
{


 

String string_lin_x ="X"+String(x0_last,0); 
  
String string_lin_y = "Y"+String(y0_last,0); 

String string_lin_z = "Z"+String(z0_last,0);



if (grip==HIGH)
{T_State="X";}
else
{T_State="O";}

String string_T_State = "CL "+T_State;


if (fun_1_loop==HIGH)
  {ModeString=">"+Mode;}
else
  {ModeString=":"+Mode;}


String full_data="  "+string_lin_x+"  "+string_lin_y+"  "+string_lin_z+" "+string_T_State +" "+ModeString+" ";







lcd_com(String(x0_last,0),String(y0_last,0),String(z0_last,0),T_State,ModeString);


last_spk_z=z0_last;
last_spk_y=y0_last;
last_spk_x=x0_last;


 if (fun_1_loop==LOW)
 {fun_1_loop_aux=LOW;}


}
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CONTROLO DE LCD 16X2

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void lcd_com(String xdata,String ydata,String zdata, String T_State, String ModeString)

  

{



  lcd.clear();

  
  //print X
  lcd.setCursor(0, 0);
  lcd.print("X");
  
  lcd.setCursor((1+4-xdata.length() ), 0);
  lcd.print(xdata);

    lcd.setCursor(6, 0);
  lcd.print("Y");
  
  lcd.setCursor((7+4-ydata.length() ), 0);
  lcd.print(ydata);


  lcd.setCursor(12, 0);
  lcd.print("CL");
  
  lcd.setCursor(15, 0);
  lcd.print(T_State);


  
  lcd.setCursor(0, 1);
  lcd.print("Z");
  
  lcd.setCursor((1+4-zdata.length() ), 1);
  lcd.print(zdata);

    lcd.setCursor(6, 1);
  lcd.print("M");
  
  lcd.setCursor((6+4-zdata.length() ), 1);
  lcd.print(ModeString);

    lcd.setCursor(12, 1);
  lcd.print("S");

   lcd.setCursor((13+3-String(step_var).length() ), 1);
  lcd.print(String(step_var));
 
  
  
  }


void SpC(){

if (SERIALCOM==HIGH)
{
 
  if (Serial.available())  {
    char c = Serial.read();  //gets one byte from serial buffer
   
    if (c == ',') {
      
     
      if(readString.indexOf("X") >=0) {
        readString=readString.substring(2);
        
        X = readString.toInt();
       


      }
      if(readString.indexOf("Y") >=0) {
        readString=readString.substring(2);
  
        Y = readString.toInt();
 
      }
      if(readString.indexOf("Z") >=0) {
        readString=readString.substring(2);
 
        Z = readString.toInt();

      }
       if(readString.indexOf("CL") >=0) {
        readString=readString.substring(2);

        CL = readString.toInt();

      }

      
      if(readString.indexOf("CHK") >=0) {
        readString=readString.substring(2);

        CHK = readString.toInt();


   
      }

 
                          
   float CHKsum=X+Y+Z+CL;
   
                      if (/*CHK==(CHKsum) &&*/ Z<-180)
                        {

                          Serial.println("OK");
                        
                           //deve ler - ,X 0,Y 20,Z -260,CL 0,CHK -240;
                          jmove_trans(X,Y,Z);
                          Tjoy_xm=X;
                          Tjoy_ym=Y;
                          Tjoy_zm=Z;
                          X=0;
                          Y=0;
                          Z=0;
                          CL=0;
                          CHK=0;
                          

                               readString=""; //clears variable for new input
      data="";

                         
                            
                         }
                        
                       else
                        {
                          Serial.println("NOK");
                        }
                      



  


     
      //do some stuff


      readString=""; //clears variable for new input
      data="";

    } 
    else {     
      readString += c; //makes the string readString
    }
  }
}
//Serial.flush();
}


//*************************************************************************************************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//CONTROLO DE ESTADO


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************************************************************************************************************************************************************************

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//SELEÇÃO DE ESTADOS

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void state_control()
{


  int fun_1_loop=digitalRead(func_1);

  if (fun_1_loop==HIGH)
  {
      
      if (analogRead(x_axis)>824 && mode_change==LOW) 
        {
          mode_var ++;
          if (mode_var>2)
            {mode_var=0;}
          mode_change=HIGH;   
        }

      if (analogRead(x_axis)<200 && mode_change==LOW) 
        {
          mode_var --;
          if (mode_var<0)
            {mode_var=2;}

          mode_change=HIGH;   
        }

    switch (mode_var) {
      case 1: 
           MANUAL=HIGH, SERIALCOM=LOW, CYCLE=LOW;
           Mode="Man";

        break;
      case 2: 
           MANUAL=LOW, SERIALCOM=HIGH, CYCLE=LOW;
           Mode="SpC";
           
        break;
      case 0: 
           MANUAL=LOW, SERIALCOM=LOW, CYCLE=HIGH;
           Mode="Cyc";
           
        break;  
      }


      if (analogRead(x_axis)>400 && analogRead(x_axis)<624) 
        {
          mode_change=LOW;   
        }

        serial_talker();
        
        fun_1_loop=digitalRead(func_1);

        
        
  }

     serial_talker();
  
if (MANUAL==HIGH)
 {joy_move(); 

  }

if (SERIALCOM==HIGH)
 {
  
 SpC(); 
  }

if (CYCLE==HIGH)
 {
  cyc_count=0;
  run_cycl=true; 
  delay(1);
  
  }


        digitalWrite(grip_out, grip);

}



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//CORRER CYCLO GRAVADO

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  void run_cycle()
    {
  

  if ((abs(servoFractionX-last_sFX)>5 || (abs(servoFractionY-last_sFY)>5) || abs(servoFractionZ-last_sFZ)>5)) {
    
      lmove_trans(servoFractionX,servoFractionY,servoFractionZ);
      last_sFX=servoFractionX;
      last_sFY=servoFractionY;
      last_sFZ=servoFractionZ;
      delay(250);
      
    }
   else
   {
      cyc_count=cyc_count+1;  
      
        
     }


if(cyc_count>10)
{
  run_cycl=false;
  }

if(no_lmove==false)
{
  getDataFromPC();
  curMillis = millis();
  replyToPC();
}

     
    }
    
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//COMANDO MANUAL

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void joy_move(){

  if (digitalRead(func_1)==LOW)
{
int joy_x=analogRead(x_axis);
int joy_y=analogRead(y_axis);
int zup=digitalRead(z_up_inp);
int zdw=digitalRead(z_dw_inp);
bool bool_joy=digitalRead(inp_joy);



float Tjoy_x=x0_last;
float Tjoy_y=y0_last;
float Tjoy_z=z0_last;




  if (joy_x>824) 
    {
 bool valid_move_xup=check_move(x0_last+5, y0_last, z0_last);
      if (valid_move_xup==HIGH){
        Tjoy_x=Tjoy_x+5;
        Tjoy_xm=Tjoy_x;
        }
        
    }

   if (joy_x<200) 
    {

 bool valid_move_xdw=check_move(x0_last-5, y0_last, z0_last);
 if (valid_move_xdw==HIGH){
      Tjoy_x=Tjoy_x-5;
      Tjoy_xm=Tjoy_x;
 }
     
    }

  if (joy_y>824) 
    {
      bool valid_move_yup=check_move(x0_last, y0_last+5, z0_last);
   if (valid_move_yup==HIGH){
      Tjoy_y=Tjoy_y+5;
      Tjoy_ym=Tjoy_y;
   }
    }

   if (joy_y<200)
    {
      bool valid_move_ydw=check_move(x0_last, y0_last-5, z0_last);
    if (valid_move_ydw==HIGH){
      Tjoy_y=Tjoy_y-5;
      Tjoy_ym=Tjoy_y;
    }
    }

      if (zup==1 && zdw==0) 
    {

     
     
      bool valid_move_zup=check_move(x0_last, y0_last, z0_last+5);

      
    if (valid_move_zup==HIGH){
      


    Tjoy_z=Tjoy_z+5;
    Tjoy_zm=Tjoy_z;

    }
    }

  if (zdw==1 && zup==0)
    {
      bool valid_move_zdw=check_move(x0_last, y0_last, z0_last-5);
    if (valid_move_zdw==HIGH){
      Tjoy_z=Tjoy_z-5; 
        Tjoy_zm=Tjoy_z; 

           
    }
    }

    


 

    if (bool_joy==HIGH)
    {joy_out=LOW;
    
     }

  if (bool_joy==LOW && joy_out==LOW)
    {
    if (grip==HIGH){
     grip=LOW;
     
    }
    else
    
    {
      grip=HIGH;
     
      }
    joy_out=HIGH; 
    }

 
jmove_trans( Tjoy_xm,  Tjoy_ym,  Tjoy_zm);


  }
}

//*************************************************************************************************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//CONTROLO DE MOVIMENTOS


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************************************************************************************************************************************************************************

//*************************************************************************************************************************************************************************************

//CINEMATICA INVERSA

//*************************************************************************************************************************************************************************************

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ROTAÇÃO DA JUNTA DO MOTOR 1 (ALINHADO COM O EIXO Y)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float front_val(float x, float y, float z){
  
 float  mp2, h, theta; // refer to picture < picture of linkages > for explanations of names. all units in mm
 
 mp2 = sqrt( pow(p2,2) - pow(x,2) ); // adjusts the linkage length to compensate for the tilt of the linkages
 y = y + p3 - p0; // p0 is the distance from the base of the lever arm to the center of the lever arms
 
 h = sqrt( pow(y,2) + pow(z,2) ); // calculates h. Refer to < picture of linkages >
 theta = atan( y / z ) + acos( ( pow(h,2) + pow(p1,2) - pow(mp2,2) ) / (2*h*p1) );
 theta = theta / PI * 180; // adjust theta to match setLever() parameters
 
 return theta;
 }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ROTAÇÃO DA JUNTA DO MOTOR 2

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float right_val(float x, float y, float z){
  
 float right, p2 = 315; // these are the variables for the lever angles

 float ry = -y*sin(PI/6) + x*cos(PI/6); // adjust x, y, and z values for the left lever
 float rx = -y*cos(PI/6) - x*sin(PI/6);
 right = front_val(rx, ry, z); //calculates lever angle for the left lever

 return right;
 }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//ROTAÇÃO DA JUNTA DO MOTOR 3

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float left_val(float x, float y, float z){
  
 float front, right, left, p2 = 315; // these are the variables for the lever angles

 float ly = -y*sin(PI/6) - x*cos(PI/6); // adjust x, y, and z values for the right lever
 float lx = y*cos(PI/6) - x*sin(PI/6);
 
 left = front_val(lx, ly, z); //calculates lever angle for the right lever
  
 return left;
 }

//*************************************************************************************************************************************************************************************

//FUNÇÕES DE MOVIMENTOS

//*************************************************************************************************************************************************************************************

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//MOVIMENTOS DE JUNTA

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void jmove(int pos1in1, int pos1in2, int pos1in3)
{

int var = 0,var1 = 0,var2 = 0,var3 = 0;

busy_move=HIGH;

while (var < 3) {

  int pos1 = myservo1.read() ;
  int pos2 = myservo2.read() ;
  int pos3 = myservo3.read() ;

  int pos1f=map(pos1in1,90,0,downJ1,midJ1);
  int pos2f=map(pos1in2,90,0,downJ2,midJ2);
  int pos3f=map(pos1in3,90,0,downJ3,midJ3);

  if (pos1 != pos1f)
    {
      if (pos1<pos1f)
        {pos1 ++;}
      else
        {pos1 --;}
      }
    else
    {var1 = 1;}

  if (pos2 != pos2f)
       if (pos2<pos2f)
        {pos2 ++;}
      else
        {pos2 --;}
    else
    {var2 = 1;}

  if (pos3  !=  pos3f)
      if (pos3<pos3f)
        {pos3 ++;}
      else
        {pos3 --;}
    else
    {var3 = 1;}

    var = var1+var2+var3;

    myservo1.write(pos1);

    myservo2.write(pos2);

    myservo3.write(pos3);
     
    delay(map(speedp,1,100,10,1));

    delta_calcForward();
    serial_talker();
    
}
 busy_move=LOW;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//MOVIMENTOS DE JUNTA COM COORDENADAS TRANSFORMADAS

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


 float jmove_trans(float x, float y, float z){
  
 float front, right, left;
 
 front = delta_calcInverse_1(x,y,z);
 right = delta_calcInverse_2(x,y,z);
 left = delta_calcInverse_3(x,y,z);

  jmove(front,right,left);

 return ;
 }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//MOVIMENTOS LINEARES COM COORDENADAS TRANSFORMADAS

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void lmove_trans(float x, float y, float z){

  int case_xyz=0,fin=0;
  float front_last=0,right_last=0,left_last=0;

  float trans_lin_x=x0_last;
  float trans_lin_y=y0_last;
  float trans_lin_z=z0_last;


  float last_tlx=0;
  float last_tly=0;
  float last_tlz=0;
    
  float linmax=max (max (abs(x-x0_last), (abs(y-y0_last))), (abs(z-z0_last)));

  float trans_lin_xb=x0_last;
  float trans_lin_yb=y0_last;
  float trans_lin_zb=z0_last;

  no_lmove=true;

  if (linmax<1){
    case_xyz=0;
 
  }
    
  if (linmax==abs(x-x0_last)){
    case_xyz=1;
  }

  if (linmax==abs(y-y0_last)){
    case_xyz=2;
  }

  if (linmax==abs(z-z0_last)){
    case_xyz=3;
  }

  while (fin==0){

    switch (case_xyz) {
      case 0: 
           fin=1;

           
        break;
      case 1: 


              if ((x-x0_last)>accustep)
                {
                  trans_lin_x=trans_lin_x+accustep;
                }
              else
                {
                  if ((x-x0_last)<accustep){
                  trans_lin_x=trans_lin_x-accustep;
                  }
                }

                if (abs(y-y0_last)>accustep){
              trans_lin_y= map(trans_lin_x, trans_lin_xb, x, trans_lin_yb, y);
                }

              if (abs(z-trans_lin_z)>accustep){
              trans_lin_z= map(trans_lin_x, trans_lin_xb, x, trans_lin_zb, z);
              }

              if (abs(x0_last-x)<=accustep){

                fin=1;
              }
              
        break;
      case 2:   

              if ((y-y0_last)>accustep)
                {
                  trans_lin_y=trans_lin_y+accustep;
                }
              else
                {
                  if ((y-y0_last)<accustep){
                  trans_lin_y=trans_lin_y-accustep;
                  }
                }

                if (abs(x-x0_last)>accustep){
              trans_lin_x= map(trans_lin_y, trans_lin_yb, y, trans_lin_xb, x);
                }

              if (abs(z-trans_lin_z)>accustep){
              trans_lin_z= map(trans_lin_y, trans_lin_yb, x, trans_lin_zb, z);
              }

              if (abs(y0_last-y)<=accustep){

                fin=1;
              }       

       break;
      case 3:    

              if ((z-z0_last)>accustep)
                {
                  trans_lin_z=trans_lin_z+accustep;
                }
              else
                {
                  if ((z-z0_last)<accustep){
                  trans_lin_z=trans_lin_z-accustep;
                  }
                }

                if (abs(y-y0_last)>accustep){
              trans_lin_y= map(trans_lin_z, trans_lin_zb, z, trans_lin_yb, y);
                }

              if (abs(x-trans_lin_x)>accustep){
              trans_lin_x= map(trans_lin_z, trans_lin_zb, z, trans_lin_xb, x);
              }

              if (abs(z0_last-z)<=accustep){

                fin=1;
              }
        break;
      }

if ((abs(trans_lin_x-last_tlx)>accustep)||(abs(trans_lin_y-last_tly)>accustep)||(abs(trans_lin_z-last_tlz)>accustep))
{

  jmove(delta_calcInverse_1(trans_lin_x, trans_lin_y, trans_lin_z), delta_calcInverse_2(trans_lin_x, trans_lin_y, trans_lin_z), delta_calcInverse_3(trans_lin_x, trans_lin_y, trans_lin_z));   
        
last_tlx=trans_lin_x;
last_tly=trans_lin_y;
last_tlz=trans_lin_z;
}        
      

  }
  no_lmove=false;
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//FUNÇÃO DE VALIDAÇÃO DE POSIÇÃO

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool check_move(float x, float y, float z){

bool check_valid;

float  front, right,left;

 front = delta_calcInverse_1(x,y,z);

 right = delta_calcInverse_2(x,y,z);

 left = delta_calcInverse_3(x,y,z);
 

 
if ((front>servo1_dw_lim && front<servo1_up_lim) && (right>servo2_dw_lim && right<servo2_up_lim) && (left>servo3_dw_lim && left<servo3_up_lim) )
  {check_valid=HIGH;
  
  
  }


  else
    {check_valid=LOW;

    }

  //if (z<z_dw_lim)
 // {check_valid=LOW;}



  



return check_valid;
}

//*************************************************************************************************************************************************************************************
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//PROGRAMAS DE MOVIMENTOS


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//*************************************************************************************************************************************************************************************

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PROGRAMA DE TRABALHO

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void prog_1(int p_step)
  {

float  prog_1_x[100];
float  prog_1_y[100];
float  prog_1_z[100];
  //float test_prog_T[100];







int Psteps=14;


if (CYCLE==HIGH && step_var>0)
 {
  lmove_trans(prog_1_x[p_step], prog_1_y[p_step], prog_1_z[p_step]);

  if (p_step>=Psteps)
  {  cycle_on=LOW;
  step_var=0;
  
  }


 }

 
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PROGRAM DE TESTE

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void test_prog(){



float  test_prog_x[]={   0,    0,   80,  -80,    0,    0,    0,    0,    0,   90,  -90,  -90,  90,   0,   0,  60,  -60,  -60,  60,   0,   0};
float  test_prog_y[]={   0,    0,    0,    0,    0,   80,  -80,    0,    0,   10,   10,  -10, -10,   0,   0,  60,   60,  -60, -60,   0,   0};
float  test_prog_z[]={-250, -200, -200, -200, -200, -200, -200, -200, -220, -220, -220, -220,-220,-250,-200,-260, -260, -260,-260,-280,-200};
int    test_prog_T[]={   0,    0,    1,    1,    0,    0,    0,    0,    0,    1,    1,    1,   1,   0,   0,    1,    0,   1,   0,   1,   0};


int Psteps=21;


if (CYCLE==HIGH)
 {
  lmove_trans(test_prog_x[step_var], test_prog_y[step_var], test_prog_z[step_var]);
  step_var=step_var+1;
  if (test_prog_T[step_var]==1)
  { grip=HIGH;}
 else
 { grip=LOW;}
  
  if (step_var>(Psteps-1))
  {  cycle_on=LOW;
  step_var=0;
  
  }
 }
 }
 
 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PROGRAM DE MOVIMENTO PARA POSIÇÃO HOME

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 void go_home(){



  
  if (h_start==HIGH){



  jmove_trans(Tjoy_xm,Tjoy_ym,Tjoy_zm);
  
  h_start=LOW;

  }
 }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//PROGRAM DE MOVIMENTO DE TESTE DOS MOTORES

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float reset_motors(){

myservo1.write(servo1_up_lim);
myservo2.write(servo2_up_lim);

myservo3.write(servo3_up_lim);

delay(1000);

myservo1.write(servo1_dw_lim);
myservo2.write(servo2_dw_lim);

myservo3.write(servo3_dw_lim);

delay(1000);
  
}



























 // forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
 // returned status: 0=OK, -1=non-existing position
 //int delta_calcForward(float theta1, float theta2, float theta3, float &x0, float &y0, float &z0) {
     
void delta_calcForward()

{
     


  float theta1=map(myservo1.read(),downJ1,midJ1,90,0);
  float theta2=map(myservo2.read(),downJ2,midJ2,90,0);
 float theta3=map(myservo3.read(),downJ3,midJ3,90,0);

/*
   float theta1=map(myservo1.read(),downJ1,midJ1,0,pi/2);
  float theta3=map(myservo2.read(),downJ2,midJ2,0,pi/2);
 float theta2=map(myservo3.read(),downJ3,midJ3,0,pi/2);
 */         
     
     float x0, y0, z0; 
     
     
     
     
     float t = (f-e)*tan30/2;
     float dtr = pi/(float)180.0;
 
     theta1 *= dtr;
     theta2 *= dtr;
     theta3 *= dtr;
 
     float y1 = -(t + rf*cos(theta1));
     float z1 = -rf*sin(theta1);
 
     float y2 = (t + rf*cos(theta2))*sin30;
     float x2 = y2*tan60;
     float z2 = -rf*sin(theta2);
 
     float y3 = (t + rf*cos(theta3))*sin30;
     float x3 = -y3*tan60;
     float z3 = -rf*sin(theta3);
 
     float dnm = (y2-y1)*x3-(y3-y1)*x2;
 
     float w1 = y1*y1 + z1*z1;
     float w2 = x2*x2 + y2*y2 + z2*z2;
     float w3 = x3*x3 + y3*y3 + z3*z3;
     
     // x = (a1*z + b1)/dnm
     float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
     float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;
 
     // y = (a2*z + b2)/dnm;
     float a2 = -(z2-z1)*x3+(z3-z1)*x2;
     float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;
 
     // a*z^2 + b*z + c = 0
     float a = a1*a1 + a2*a2 + dnm*dnm;
     float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
     float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);
  
     // discriminant
     float d = b*b - (float)4.0*a*c;
    // if (d < 0) return -1; // non-existing point
 
     z0 = -(float)0.5*(b+sqrt(d))/a;
     x0 = (a1*z0 + b1)/dnm;
     y0 = (a2*z0 + b2)/dnm;

 

     z0_last=z0;
     y0_last=y0;
     x0_last=x0;


     
     //return 0;
 }




  
  // inverse kinematics
 // helper functions, calculates angle theta1 (for YZ-pane)
float delta_calcInverse_1(float x0, float y0, float z0)
{
  float theta;
  
     float y1 = -0.5 * 0.57735 * f; // f/2 * tg 30
     y0 -= 0.5 * 0.57735    * e;    // shift center to edge
     // z = a + b*y
     float a = (x0*x0 + y0*y0 + z0*z0 +rf*rf - re*re - y1*y1)/(2*z0);
     float b = (y1-y0)/z0;
     // discriminant
     float d = -(a+b*y1)*(a+b*y1)+rf*(b*b*rf+rf); 

     float yj = (y1 - a*b - sqrt(d))/(b*b + 1); // choosing outer point
     float zj = a + b*yj;
     theta = 180.0*atan(-zj/(y1 - yj))/pi + ((yj>y1)?180.0:0.0);

      //Serial.println(theta);
   return theta;
 }
 
 // inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
 // returned status: 0=OK, -1=non-existing position
 float delta_calcInverse_2(float x0, float y0, float z0) {
     float theta1,theta2,theta3;
     
     theta2 = 0;
     

     theta2 = delta_calcInverse_1(x0*cos120 + y0*sin120, y0*cos120-x0*sin120, z0);  // rotate coords to +120 deg

      //Serial.println(theta2);
     return theta2;
 }

  float delta_calcInverse_3(float x0, float y0, float z0) {
     float theta1,theta2,theta3;
     
     theta3 = 0;
     

     theta3 = delta_calcInverse_1(x0*cos120 - y0*sin120, y0*cos120+x0*sin120, z0);  // rotate coords to -120 deg

     //Serial.println(theta3);
     return theta3;
 }
 
 
