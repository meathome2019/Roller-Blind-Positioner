
#include <Arduino.h>
#define _TASK_MICRO_RES  //Force task scheduler to run in microsecond mode
#include <TaskScheduler.h>
#include <esp_task_wdt.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ThingSpeak.h>
#include <ESP32Ping.h>
#include "OTA.h"

const char *SSID = "";
const char *PWD = "";

const char* PARAM_INPUT_1 = "output";
const char* PARAM_INPUT_2 = "state";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
Scheduler ts;
WiFiClient client;


#define Pin1 19
#define Pin2 18
#define Pin3 5
#define Pin4 17
#define Pin5 4
#define WDT_TIMEOUT 8

int Hall_Count=0; 
int steps=0;  //The number of steps stepper motor has taken
int Position=0; //The number of steps for stepper motor (target)
int Step_Top=0;
int hall=16;  //Set input pin for hall magnetic sensor
int Last_Pos=0;
int Position_t;
unsigned int inputM1=0;
unsigned int inputM2=0;
const unsigned long UpdateInterval1 = 30 * 1000;
//unsigned long myChannelNumber1 =  ;
//const char *myWriteAPIKey1 = "";
String inputMessage1="";
String inputMessage2="";

IPAddress local_IP(192, 168, 1, 118);
IPAddress gateway(192, 168, 1, 254);    // Set your network Gateway usually your Router base address
IPAddress subnet(255, 255, 255, 0); 
IPAddress dns(192,168,1,254);           // Set your network DNS usually your Router base address
IPAddress dns2(8,8,8,8);

//########################################  VOID  ###################################################
String outputState(int output);
void Flashon();
void Flashoff();
void setupOTA();
void runner1();
void watchdog();
void step4();
void step3();
void step2();
void step1_c();
void callback1();
void step1();
void step1ac();
void step2ac();
void step3ac();
void step4ac();
void callback2();
void callback3();
void disablet1();
void disablet2();
void enablet1();
void enablet2();
void Step_Hall_enable();
void Rssi_Fail();
void WiFi_Check();
void clear_pois();
void clear_pois1();
void clear_pois2();
void rssi();
void step_off();
void step1_ac();
void Top_on();
void Top_off();
void OneQuarter_on();
void OneQuarter_off();
void Middle_on();
void Middle_off();
void ThreeQuarters_on();
void ThreeQuarters_off();
void Bottom_on();
void Bottom_off();
void Toggled();

//###################################################################################################

Task t1(1200  , TASK_FOREVER, &callback1 ,&ts); //1400   Step motor delay for Down loop
Task t2(1299  , TASK_FOREVER, &callback2 ,&ts); //1500   Step motor delay for UP loop

//Task OTA1(3040* TASK_MILLISECOND, TASK_FOREVER,&OTA,&ts,true);
Task watchdog1(3060* TASK_MILLISECOND, TASK_FOREVER, &watchdog,&ts,true);
Task Step_Hall_enable1(300 * TASK_MILLISECOND ,TASK_FOREVER , &callback3 ,&ts);

Task WiFi_Check1(4*TASK_MINUTE, TASK_FOREVER,&WiFi_Check,&ts,true);
//Task Thing_send1(10*TASK_MINUTE, TASK_FOREVER,&rssi,&ts,true);


//####################################################
void callback1(){}
//####################################################
void callback2(){}
//#####################################################
void callback3(){}
//###################################################################################################
void IRAM_ATTR Stepper_stop(){   //If magnetic sensor tripped
Hall_Count=100;
  step_off();
	//Serial.print(".");
}
//###################################################################################################
void Rssi_Fail(){
//Thing_send1.setCallback(&rssi);
//Thing_send1.delay(30*TASK_SECOND);
}
//##########################################  WEB PAGE  #############################################
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>ESP Web Server</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 3.0rem;}
		h3 {font-size: 2.0rem;}
    p {font-size: 3.0rem;}
		body { background-color: lightblue; }
    body {max-width: 600px; margin:0px auto; padding-bottom: 25px;}
    .switch {position: relative; display: inline-block; width: 120px; height: 68px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #66ff66; border-radius: 6px}
    .slider:before {position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 3px}
    input:checked+.slider {background-color: #b30000}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}
  </style>
</head>
<body>
  <h2>Roller Blind Positioner</h2>
	<h3>Press Bution To Goto Position</h3>
  %BUTTONPLACEHOLDER%
<script>function toggleCheckbox(element) {
  var xhr = new XMLHttpRequest();
  if(element.checked){ xhr.open("GET", "/update?output="+element.id+"&state=1", true); }
  else { xhr.open("GET", "/update?output="+element.id+"&state=0", true); }
  xhr.send();
}
</script>
</body>
</html>
)rawliteral";
//###################################################################################################
// Replaces placeholder with button section in your web page
String processor(const String& var){
  //Serial.println(var);
  if(var == "BUTTONPLACEHOLDER"){
    String buttons = "";
    buttons += "<h4> Top </h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"19\" " + outputState(Pin1) + "><span class=\"slider\"></span></label>";

    buttons += "<h4>OneQuarter</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"18\" " + outputState(Pin2) + "><span class=\"slider\"></span></label>";

    buttons += "<h4>Middle</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"5\" " + outputState(Pin3) + "><span class=\"slider\"></span></label>";

    buttons += "<h4>ThreeQuarters</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"4\" " + outputState(Pin4) + "><span class=\"slider\"></span></label>";

		buttons += "<h4>Bottom</h4><label class=\"switch\"><input type=\"checkbox\" onchange=\"toggleCheckbox(this)\" id=\"17\" " + outputState(Pin5) + "><span class=\"slider\"></span></label>";
    return buttons;
  }
  return String();
}
//##################################################################################################
String outputState(int output){
  if(digitalRead(output)){
    return "checked";
  }
  else {
    return "";
  }
}
//###################################################################################################
void setup(){
  // Serial port for debugging purposes
  Serial.begin(115200);
	
  pinMode(Pin1, OUTPUT);
  digitalWrite(Pin1, LOW);
  pinMode(Pin2, OUTPUT);
  digitalWrite(Pin2, LOW);
  pinMode(Pin3, OUTPUT);
  digitalWrite(Pin3, LOW);
	pinMode(Pin4, OUTPUT);
	pinMode(Pin5, OUTPUT);
  digitalWrite(Pin5, LOW);

  digitalWrite(Pin4, LOW);
	pinMode(hall,INPUT_PULLDOWN);
  
	//step_off();

  //########################    Connect to Wi-Fi   ##################################################
	WiFi.disconnect();
  WiFi.mode(WIFI_STA );       // switch off AP
  WiFi.setAutoConnect(true);
  WiFi.setAutoReconnect(true);
  WiFi.config(local_IP, gateway, subnet, dns, dns2);
  WiFi.begin(SSID, PWD);
	int i=0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    if(i>=10){
      Serial.println("Connection Failed! Rebooting...");
      delay(2000);
      ESP.restart();
    }
		i++;
  }
	//setupOTA();
	//OTA1.enableDelayed();
  watchdog1.enableDelayed();
  //ThingSpeak.begin(client);
  //Thing_send1.enableDelayed();
	WiFi_Check1.enableDelayed();
	Serial.println("Configuring WDT...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL); //add current thread to WDT watch



  // Print ESP Local IP Address
  Serial.println("\nConnected to " + WiFi.SSID() + " Use IP address: " + WiFi.localIP().toString()); 
  Serial.println("WiFi ON");

  // Route for root / web page
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html, processor);
  });

  // Send a GET request to <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
  server.on("/update", HTTP_GET, [] (AsyncWebServerRequest *request) {
    inputMessage1="";
    inputMessage2="";
    // GET input1 value on <ESP_IP>/update?output=<inputMessage1>&state=<inputMessage2>
    if (request->hasParam(PARAM_INPUT_1) && request->hasParam(PARAM_INPUT_2)) {
      inputMessage1 = request->getParam(PARAM_INPUT_1)->value();
      inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
      digitalWrite(inputMessage1.toInt(), inputMessage2.toInt());
			inputM1=inputMessage1.toInt();
			inputM2=inputMessage2.toInt();
			Serial.println(inputM1 );
			Serial.println(inputM2 );
    }
    else {
      inputMessage1 = "No message sent";
      inputMessage2 = "No message sent";
    }
    Serial.print("GPIO: ");
    Serial.print(inputMessage1);
    Serial.print(" - Set to: ");
    Serial.println(inputMessage2);
    request->send(200, "text/plain", "OK");
		Toggled();
	});
  // Start server

  server.begin();
}
	//#############################  If webpage button toggled  ######################################
void Toggled(){
	//Serial.println(" " );

	if (inputM1 == 19 && inputM2 == 1){
		Top_on();
		Serial.print("U0");
		Serial.println(inputM1 );
	}
	if (inputM1 == 19 && inputM2 == 0){
		Top_off();
		Serial.println("U0 off");
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (inputM1 == 18 && inputM2 == 1){
		OneQuarter_on ();
		Serial.print("U2");
		Serial.println(inputM1 );
	}
	if (inputM1 == 18 && inputM2 == 0){
		 OneQuarter_off();
		 Serial.println("U2 off");
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (inputM1 == 5 && inputM2 == 1){
		Middle_on ();
		Serial.print("U3");
		Serial.println(inputM1 );
	}
	if (inputM1 == 5 && inputM2 == 0){
		 Middle_off ();
		 Serial.println("U3 off");
	}

	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (inputM1 == 4 && inputM2 == 1){
		ThreeQuarters_on ();
		Serial.print("U3");
		Serial.println(inputM1 );
	}
	if (inputM1 == 4 && inputM2 == 0){
		 ThreeQuarters_off ();
		 Serial.println("U3 off");
	}
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	if (inputM1 == 17 && inputM2 == 1){
		Bottom_on ();
		Serial.print("U4");
		Serial.println(inputM1 );
	}
	if (inputM1 == 17 && inputM2 == 0){
		Bottom_off ();
		Serial.println("U4 off");
	}
}

//###################################################################################################
void loop() {
	ts.execute();
}
//###################################################################################################
void step1_c(){  
  WiFi_Check1.disable();
  //Thing_send1.disable();
  if (steps<=Position ){  //Compare steps to position
    steps++;
    step1();
		Serial.print(".");
  }
  else{
    if(steps>=Position){
      t1.disable();
      t2.enable();
      step1_ac();
    }
  } 
  if(steps==Position){ //If stepper motor reached position disable timers one and two and zero steps
    t1.disable();
    t2.disable();
    step_off();  //Turn stepper motor drivers off
  }
}
//################################################################################################## 
void step1(){
      digitalWrite(Pin1,HIGH);
      digitalWrite(Pin2,LOW);
      digitalWrite(Pin3,LOW);
      digitalWrite(Pin4,HIGH);
      t1.setCallback(&step2);
  }
void step2(){
      digitalWrite(Pin1,LOW);
      digitalWrite(Pin2,LOW);
      digitalWrite(Pin3,HIGH);
      digitalWrite(Pin4,HIGH);
      t1.setCallback(&step3);
}
void step3(){
      digitalWrite(Pin1,LOW);
      digitalWrite(Pin2,HIGH);
      digitalWrite(Pin3,HIGH);
      digitalWrite(Pin4,LOW);
      t1.setCallback(&step4);
}
void step4(){
      digitalWrite(Pin1,HIGH);
      digitalWrite(Pin2,HIGH);
      digitalWrite(Pin3,LOW);
      digitalWrite(Pin4,LOW);
      t1.setCallback(&step1_c);
}
  
  //#################################################################################################
void step1_ac(){
  if (steps>=Position){
    steps--;
    step1ac();
  }
  if(steps==Position){
    t1.disable();
    t2.disable();
    step_off();
  }
}    
void step1ac(){
      digitalWrite(Pin1,HIGH);
      digitalWrite(Pin2,HIGH);
      digitalWrite(Pin3,LOW);
      digitalWrite(Pin4,LOW);
      t2.setCallback(&step2ac);
}  
void step2ac(){
       digitalWrite(Pin1,LOW);
      digitalWrite(Pin2,HIGH);
      digitalWrite(Pin3,HIGH);
      digitalWrite(Pin4,LOW);
      t2.setCallback(&step3ac);
}
void step3ac(){  
      digitalWrite(Pin1,LOW);
      digitalWrite(Pin2,LOW);
      digitalWrite(Pin3,HIGH);
      digitalWrite(Pin4,HIGH);
      t2.setCallback(&step4ac);
}
void step4ac(){   
      digitalWrite(Pin1,HIGH);
      digitalWrite(Pin2,LOW);
      digitalWrite(Pin3,LOW);
      digitalWrite(Pin4,HIGH);
      t2.setCallback(&step1_ac);
}
//###################################################################################################
void step_off(){  //Turn stepper motor drivers off
  detachInterrupt(hall);
  t1.disable();
  t2.disable();
  Step_Hall_enable1.disable();
  digitalWrite(Pin1,LOW);
  digitalWrite(Pin2,LOW);
  digitalWrite(Pin3,LOW);
  digitalWrite(Pin4,LOW);
	digitalWrite(Pin5,LOW);
	
  if(Step_Top==1){
    Step_Top=0;
    steps=0;
    Position=0;
  }
  WiFi_Check1.enableDelayed();
  //Thing_send1.enableDelayed(); 
	Serial.println("Step Off");       
}
//###################################################################################################
void enablet1(){   //Enabled timer at start of job
  t1.enable();
  //Step_Hall_enable1.enableDelayed();
	Step_Hall_enable1.enableDelayed ();
  Step_Hall_enable1.setCallback(&Step_Hall_enable);
  //attachInterrupt(hall, Stepper_stop, RISING);
	Serial.println("Enable1");
}
//###################################################################################################
void enablet2(){
  t2.enable();
  //Step_Hall_enable1.enableDelayed();
  Step_Hall_enable1.enableDelayed ();
	Step_Hall_enable1.setCallback(&Step_Hall_enable);
	Serial.println("Enable2");
}
//###################################################################################################
void Step_Hall_enable(){
  attachInterrupt(hall, Stepper_stop, RISING);
  Hall_Count=0;
}
//###################################################################################################
void disablet1(){  //Disabled timer at the end of job
  t1.disable();
}
//###################################################################################################
void disablet2(){
  t2.disable();
}
//////////////////////////////////  LED ON OR OFF SELECT  ///////////////////////////////////////////
void Top_on(){
  enablet1();
  disablet2();
  //if(steps>=10 && Hall_Count==0){
  if(Hall_Count==0){
    Position=-40000;
    Step_Top=1;
  }
  step1_c();
  //LED_ONE_STATE = "on";
  Last_Pos=0;
	Serial.println("Top_On");
}
void Top_off(){
  disablet1();
  disablet2();
  //LED_ONE_STATE="off";
  step_off();
	Serial.println("Top_Off");
}
//###################################################################################################
void OneQuarter_on(){
  enablet1();
  disablet2();
  Position=967;
  step1_c();
  //LED_TWO_STATE = "on";
  Last_Pos=1;
	Serial.println("OneQuart_On");
}
void OneQuarter_off(){
  disablet2();
  disablet1();
  //LED_TWO_STATE ="off";
  step_off();
	Serial.println("OneQuart_Off");
  }
//###################################################################################################
void Middle_on(){
  enablet1();
  disablet2();
  Position=1935;
  step1_c();
  //LED_THREE_STATE = "on";
  Last_Pos=2;
	Serial.println("Mid_On");
}
void Middle_off(){
  disablet1();
  disablet2();
  step_off();
  //LED_THREE_STATE ="off";
	Serial.println("Mid_Off");
  }
//###################################################################################################
void ThreeQuarters_on(){
  enablet1();
  disablet2();
  Position=2800;
  step1_c();
  //LED_FOUR_STATE = "on";
  Last_Pos=3;
	Serial.println("3Qrt_On");
}
void ThreeQuarters_off(){
  disablet1();
  disablet2();
  step_off();
  //LED_FOUR_STATE ="off";
	Serial.println("3Qrt_Off");
  }
//###################################################################################################
void Bottom_on(){
  enablet1();
  disablet2();
  Position=3670;
  step1_c();
  //LED_Flashg_STATE="on";
  Last_Pos=4;
	Serial.println("Bottom_On");
}
void Bottom_off(){
  disablet1();
  disablet2();
  step_off();
  //LED_Flashg_STATE="off";
	Serial.println("Bottom_Off");
} 
//###################################################################################################
void WiFi_Check(){  //Ping Router to establish connection if no connection sleep for five               minutes and try again.
  Serial.println();
  Serial.print(F("WiFi connected with ip "));  
  Serial.println(WiFi.localIP());
  watchdog();
  Serial.print F("Pinging ip ");
  Serial.println(gateway);
	watchdog();
  if(Ping.ping(gateway)) {
    Serial.println(F("Success!!"));
  } else {
    Serial.println(F("Error :("));
    
    esp_sleep_enable_timer_wakeup(UpdateInterval1);
    Serial.println(F("Starting deep-sleep period..."));
    esp_deep_sleep_start(); // Sleep for e.g. 30 minutes
  }
}

//##################################  WATCHDOG  ####################################################
void watchdog(){     //Watchdog reset point
  esp_task_wdt_reset();
}
//###################################################################################################
void rssi(){
  long Rssi = WiFi.RSSI();
  Serial.print("RSSI:");
  Serial.println(Rssi);
/*
  int x = ThingSpeak.writeField(myChannelNumber1, 1, Rssi, myWriteAPIKey1);
  if(x == 200){
    Serial.println("Channel update successful.");
  }
  else{
    Serial.println("Problem updating channel. HTTP error code " + String(x));
    Rssi_Fail();
  }
	*/
}
//###################################################################################################
