#include <ESP8266WiFi.h>

String readString;

String last_incoming=" ";

String   incoming_req,incoming,incomingX,incomingY,incomingZ,incomingCL,incomingSP;

String   incoming_read;

int incomingCHK;

int serialcount=0;

bool run_ok;

const char* ssid = "some_ssid";//type your ssid
const char* password = "password";//type your password

//const char* ssid = "ADSLPT-AB23655";//type your ssid
//const char* password = "fp20ene1";//type your password

WiFiServer ESPserver(80);//Service Port

void setup() 
{
Serial.begin(115200);


Serial.println();
Serial.println();
Serial.print("Connecting to: ");
Serial.println(ssid);

WiFi.begin(ssid, password);
delay(5000);

/*
 The following four line of the 
 code will assign a Static IP Address to 
 the ESP Module. If you do not want this, 
 comment out the following four lines.  
*/

IPAddress ip(192,168,43,38);   
IPAddress gateway(192,168,43,1);


//IPAddress ip(192,168,1,38);   
//IPAddress gateway(192,168,1,1);
  


IPAddress subnet(255,255,255,0);   
WiFi.config(ip, gateway, subnet); 
delay(5000);

while (WiFi.status() != WL_CONNECTED) 
{
delay(100);
Serial.print("*");
}
Serial.println("");
Serial.println("WiFi connected");

// Start the server
ESPserver.begin();
Serial.println("Server started");

// Print the IP address
Serial.print("The URL to control ESP8266: ");
Serial.print("http://");
Serial.print(WiFi.localIP());
}

void loop() 
{
// Check if a client has connected
WiFiClient client = ESPserver.available();
if (!client) 
{
return;
}

// Wait until the client sends some data
//Serial.println("New Client");
while(!client.available())
{
delay(1);
}

// Read the first line of the request
String request = client.readStringUntil('\r');
Serial.println(request);
client.flush();

// Match the request

if (request.indexOf("/POSCOM") != -1)
{



  
incoming_req = request.substring((request.indexOf(",X")), (request.indexOf(":&")));


incomingX = incoming_req.substring((2+incoming_req.indexOf(",X")), (incoming_req.indexOf(",Y")));
incomingY = incoming_req.substring((2+incoming_req.indexOf(",Y")), (incoming_req.indexOf(",Z")));
incomingZ = incoming_req.substring((2+incoming_req.indexOf(",Z")), (incoming_req.indexOf(",CL")));
incomingCL = incoming_req.substring((3+incoming_req.indexOf(",CL")), (incoming_req.indexOf(",SP")));
incomingSP = incoming_req.substring((3+incoming_req.indexOf(",SP")), (incoming_req.indexOf(":&")));


incomingCHK=(incomingX.toInt()+incomingY.toInt()+incomingZ.toInt()+incomingCL.toInt())/(incomingSP.toInt()*0.01);

Serial.println(incomingCHK);

incoming =",X "+incomingX+",Y "+incomingY+",Z "+incomingZ+",CL "+incomingCL+",SP "+incomingSP+",CHK "+incomingCHK+";";

if (incoming!=last_incoming)
{
run_ok = true;
last_incoming=" ";
}

}




while (run_ok==true)
{

Serial.println(incoming);

serialcount=serialcount+1;


if (Serial.available() > 0) {
    
 incoming_read = Serial.readString();
}

 if ((incoming_read=="OK")||(serialcount>=10))
  {
    run_ok = false;
    incoming_read="ir";
    incoming=" ";
    serialcount=0;
  }


}


delay(10);
client.stop();
//Serial.println("Client disconnected");
Serial.println("");
}
