/*
 * Rui Santos 
 * Complete Project Details https://randomnerdtutorials.com
*/

//=================================== Working Successfully ==========================================
//===================================================================================================
//===================================================================================================
//===================================================================================================
//===================================================================================================

#define BLYNK_AUTH_TOKEN "RZzoUf85eYxevDkOpnsFzQtl-5WrCYJ0"

#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include<stdio.h>
#include<stdlib.h>
///////
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ESP32_MailClient.h"
#include<HTTPClient.h>
#include <BlynkSimpleEsp32.h>
#define BLYNK_TEMPLATE_ID "TMPLgY6mg8Y0"
#define BLYNK_DEVICE_NAME "DHT"
#define BLYNK_AUTH_TOKEN "RZzoUf85eYxevDkOpnsFzQtl-5WrCYJ0"

char auth[] = BLYNK_AUTH_TOKEN;
Adafruit_BMP085 bmp;
// char ssidd[] = "realme 6";  // type your wifi name
// char pass[] = "123456789";  // type your wifi password
BlynkTimer timer;

#define DHTPIN 32      // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11  // DHT 11
const char* ssid = "realme 6";
const char* password = "123456789";
// To send Emails using Gmail on port 465 (SSL)
#define emailSenderAccount "mohamedali404m@gmail.com"
#define emailSenderPassword "qyotcaterzbzwkky"
#define smtpServer "smtp.gmail.com"
#define smtpServerPort 465
#define emailSubject "[ALERT] ESP32 Temperature"





DHT dht(DHTPIN, DHTTYPE);
// Default Recipient Email Address
String input_email = "your_email_recipient@gmail.com";
String enableEmailChecked = "checked";
String inputMessage2 = "true";
// Default Threshold Temperature Value
String temp_threshold = "1000000.0";
String humidity_threshold = "1000000.0";
String pressure_threshold = "1000000.0";
String altitude_threshold = "1000000.0";
String sealevelpressure_threshold = "1000000.0";
String realaltitude_threshold = "1000000.0";
String lastTemperature, lastHumedity, lastPressure, lastAltitude, lastSealevelPressure, lastreal_Altitude;





const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Weather Station</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  </head>
    <body style="background-color: darksalmon; margin-left: 30px;margin-top: 20px;">
     <h2 style = "margin-left: 60px;color: brown; " >Weather Station</h2>
  <span style= " font-size: 20px; color:blueviolet"> Temperature</span> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; :
  <span style = "font-size: 15px;" >%TEMPERATURE% &deg;C</span>
  <br>
  <br>
   <span style= " font-size: 20px;color:blueviolet ">Humidity</span> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  : 
   <span style = "font-size: 15px;" >%Humidity% &#8453;</span>
   <br>
   <br>
   <span style= " font-size: 20px;color:blueviolet"> Pressure</span> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  : 
    <span style = "font-size: 15px;" >%Pressure% Pa</span>
    <br>
    <br>
    <span style= " font-size: 20px;color:blueviolet"> Altitude</span>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;  : 
    <span style = "font-size: 15px;" >%Altitude% meters</span>
   <br>
   <br>
   <span style= " font-size: 20px;color:blueviolet"> SealevelPressure</span>  : 
    <span style = "font-size: 15px;" >%SealevelPressure% Pa</span>
   <br>
   <br>
   <span style= " font-size: 20px;color:blueviolet"> real_Altitude</span> &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; :  
    <span style = "font-size: 15px;" >%readAltitude% meters</span>
   <br>
   <br>
  <h2 style= "color: brown;">ESP Email Notification</h2>


  <form action="/get" style = "color: blueviolet;">
    
    Email Address 	&nbsp;	&nbsp;	&nbsp;	&nbsp;	&nbsp;	&nbsp;	&nbsp;	&nbsp;	&nbsp;	&nbsp;	&nbsp; <input type="email" name="email_input" value="%EMAIL_INPUT%" required><br>
    <br>
    Enable Email Notification 	&nbsp;	&nbsp;<input type="checkbox" name="enable_email_input" value="true" %ENABLE_EMAIL%><br>
    <br>
    Temperature Threshold 	&nbsp;	&nbsp;	&nbsp;	&nbsp;<input type="number" step="0.1" name="threshold_input" value="%TEMPERATURE_THRESHOLD%" required>
    <input style="border-radius: 10px; color: red; background-color: aqua; border-color: azure;" type="submit" value="Submit">
    <br>
    <br>
    Humidity Threshold 	&nbsp; 		&nbsp;	&nbsp;	&nbsp;	&nbsp;&nbsp;	&nbsp;  <input type="number" step="0.1" name="Humidity_input" value="%Humidity_THRESHOLD%" required>
    <input  style="border-radius: 10px; color: red; background-color: aqua; border-color: azure;" type="submit" value="Submit">
    <br>
    <br>
    Pressure Threshold 		&nbsp;	&nbsp;	&nbsp;	&nbsp;&nbsp;	&nbsp;	&nbsp;	&nbsp;<input type="number" step="0.1" name="Pressure_input" value="%Pressure_THRESHOLD%" required>
    <input style="border-radius: 10px; color: red; background-color: aqua; border-color: azure;" type="submit" value="Submit">
    <br>
    <br>
    Altitude Threshold 		&nbsp; 	&nbsp;	&nbsp;	&nbsp;&nbsp;	&nbsp;	&nbsp;	&nbsp; <input type="number" step="0.1" name="Altitude_input" value="%Altitude_THRESHOLD%" required>
    <input style="border-radius: 10px; color: red; background-color: aqua; border-color: azure;"  type="submit" value="Submit">
    <br>
    <br>
    Sealevel Pressure Threshold 	 <input type="number" step="0.1" name="SealevelPressure_input" value="%SealevelPressure_THRESHOLD%" required>
    <input style="border-radius: 10px; color: red; background-color: aqua; border-color: azure;" type="submit" value="Submit">
    <br>
    <br>
    readAltitude Threshold 	&nbsp;	&nbsp;	&nbsp;	&nbsp;  <input type="number" step="0.1" name="readAltitude_input" value="%realAltitude_THRESHOLD%" required>
    <input style="border-radius: 10px; color: red; background-color: aqua; border-color: azure;" type="submit" value="Submit">
    <br>
    <br>
  </form>
</body>
<script>


</script>
</html>)rawliteral";
void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}
AsyncWebServer server(80);

String read_DHT_BMP_Data(int flag) {
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();
  float H = dht.readHumidity();
  float P = bmp.readPressure();
  float A = bmp.readAltitude();
  double SLP = bmp.readSealevelPressure();
  double real_Altitude = bmp.readAltitude(102000);



  // Read temperature as Fahrenheit (isFahrenheit = true)
  //float t = dht.readTemperature(true);
  // Check if any reads failed and exit early (to try again).
  if (flag == 0) {
    if (isnan(t)) {
      Serial.println("Failed to read from DHT sensor!");
      return "--";
    } else {
      Serial.print(t);
      // Blynk.virtualWrite(V0, t);
      Serial.print(",");
      return String(t);
    }
  } else if (flag == 1) {
    if (isnan(H)) {
      Serial.println("Failed to read from DHT sensor!");
      return "--";
    } else {
      Serial.print(H);
      // Blynk.virtualWrite(V1, H);      
      Serial.print(",");
      return String(H);
    }
  } else if (flag == 2) {
    if (isnan(P)) {
      Serial.println("Failed to read from DHT sensor!");
      return "--";
    } else {
      Serial.print(P);
      // Blynk.virtualWrite(V2, P);      
      Serial.print(",");
      return String(P);
    }
  } else if (flag == 3) {
    if (isnan(A)) {
      Serial.println("Failed to read from DHT sensor!");
      return "--";
    } else {
      Serial.print(A);
      // Blynk.virtualWrite(V3, A);
      Serial.print(",");
      return String(A);
    }
  } else if (flag == 4) {
    if (isnan(SLP)) {
      Serial.println("Failed to read from DHT sensor!");
      return "--";
    } else {
      Serial.print(SLP);
      // Blynk.virtualWrite(V4, SLP);
      Serial.print(",");
      return String(SLP);
    }
  } else if (flag == 5) {
    if (isnan(real_Altitude)) {
      Serial.println("Failed to read from DHT sensor!");
      return "--";
    } else {
      Serial.print(real_Altitude);
      Serial.print("\n");
    //  Blynk.virtualWrite(V5, real_Altitude);

      return String(real_Altitude);
    }
  }
  else
     return "--";  
}
void sendSensor()
{

  float t = dht.readTemperature();
  float H = dht.readHumidity();
  float P = bmp.readPressure();
  float A = bmp.readAltitude();
  double SLP = bmp.readSealevelPressure();
  double real_Altitude = bmp.readAltitude(102000);

   
  
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
    Blynk.virtualWrite(V0, t);
    Blynk.virtualWrite(V1, H);
    Blynk.virtualWrite(V2, P);
    Blynk.virtualWrite(V3, A);
    Blynk.virtualWrite(V4, SLP);
    Blynk.virtualWrite(V5, real_Altitude);
    
}
// Replaces placeholder with DS18B20 values
String processor(const String& var) {
  //Serial.println(var);
   if (var == "EMAIL_INPUT") {
    return input_email;
  } 
  else if (var == "ENABLE_EMAIL") {
    return enableEmailChecked;
  }
  if (var == "TEMPERATURE") {
    return lastTemperature;
  }
  else if (var == "Humidity") {
    return lastHumedity;
  }
  else if (var == "Pressure") {
    return lastPressure;
  }
  else if (var == "Altitude") {
    return lastAltitude;
  }
  else if (var == "SealevelPressure") {
    return lastSealevelPressure;
  }
  else if (var == "readAltitude") {
    return lastreal_Altitude;
  }
  else if (var == "TEMPERATURE_THRESHOLD") {
    return temp_threshold;
  }
  else if (var == "Humidity_THRESHOLD") {
    return humidity_threshold;
  }
  else if (var == "Pressure_THRESHOLD") {
    return pressure_threshold;
  }
  else if (var == "Altitude_THRESHOLD") {
    return altitude_threshold;
  }
  else if (var == "SealevelPressure_THRESHOLD") {
    return sealevelpressure_threshold;
  }
  else if (var == "realAltitude_THRESHOLD") {
    return realaltitude_threshold;
  }
  
  
  return String();
}
// Flag variable to keep track if email notification was sent or not
bool emailSent = false;

const char* PARAM_INPUT_1 = "email_input";
const char* PARAM_INPUT_2 = "enable_email_input";
const char* PARAM_INPUT_3 = "threshold_input";
const char* PARAM_INPUT_4 = "Humidity_input";
const char* PARAM_INPUT_5 = "Pressure_input";
const char* PARAM_INPUT_6 = "Altitude_input";
const char* PARAM_INPUT_7 = "SealevelPressure_input";
const char* PARAM_INPUT_8 = "readAltitude_input";

// Interval between sensor readings. Learn more about timers: https://RandomNerdTutorials.com/esp32-pir-motion-sensor-interrupts-timers/
unsigned long previousMillis = 0;
const long interval = 5000;

// The Email Sending data object contains config and data to send
SMTPData smtpData;


void setup() {
  Serial.begin(9600);
  Blynk.begin(auth, ssid, password, "blynk.cloud", 80);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("ESP IP Address: http://");
  Serial.println(WiFi.localIP());

  // Start the DHT11 sensor
  dht.begin();
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/BMP180 sensor, check wiring!");
    while (1) {}
  }
  // Send web page to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send_P(200, "text/html", index_html, processor);
  });

  // Receive an HTTP GET request at <ESP_IP>/get?email_input=<inputMessage>&enable_email_input=<inputMessage2>&threshold_input=<inputMessage3>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
    // GET email_input value on <ESP_IP>/get?email_input=<inputMessage>
    if (request->hasParam(PARAM_INPUT_1)) {
      input_email = request->getParam(PARAM_INPUT_1)->value();
      // GET enable_email_input value on <ESP_IP>/get?enable_email_input=<inputMessage2>
      if (request->hasParam(PARAM_INPUT_2)) {
        inputMessage2 = request->getParam(PARAM_INPUT_2)->value();
        enableEmailChecked = "checked";
      } else {
        inputMessage2 = "false";
        enableEmailChecked = "";
      }
      // GET threshold_input value on <ESP_IP>/get?threshold_input=<inputMessage3>
      if (request->hasParam(PARAM_INPUT_3)) {
        temp_threshold = request->getParam(PARAM_INPUT_3)->value();
      }
      if (request->hasParam(PARAM_INPUT_4)) {
        humidity_threshold = request->getParam(PARAM_INPUT_4)->value();
      }
      if (request->hasParam(PARAM_INPUT_5)) {
        pressure_threshold = request->getParam(PARAM_INPUT_5)->value();
      }
      if (request->hasParam(PARAM_INPUT_6)) {
        altitude_threshold = request->getParam(PARAM_INPUT_6)->value();
      }
      if (request->hasParam(PARAM_INPUT_7)) {
        sealevelpressure_threshold = request->getParam(PARAM_INPUT_7)->value();
      }
      if (request->hasParam(PARAM_INPUT_8)) {
        realaltitude_threshold = request->getParam(PARAM_INPUT_8)->value();
      }
    }
     else {
      input_email = "No message sent";
    }
    // Serial.println(input_email);
    // Serial.println(inputMessage2);
    // Serial.println(temp_threshold);
    // Serial.println(humidity_threshold);
    // Serial.println(pressure_threshold);
    // Serial.println(altitude_threshold);
    // Serial.println(sealevelpressure_threshold);
    // Serial.println(realaltitude_threshold);
    
   
    request->send(200, "text/html", "HTTP GET request sent to your ESP.<br><a href=\"/\">Return to Home Page</a>");
  });
  server.onNotFound(notFound);
  server.begin();
  timer.setInterval(100L,sendSensor );
}



void loop() {
  // Serial.print("Temperature = ");
  // Serial.print(dht.readTemperature());
  // Serial.println(" *C");
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;


    lastTemperature = read_DHT_BMP_Data(0);
    float temperature = lastTemperature.toFloat();

    lastHumedity = read_DHT_BMP_Data(1);
    float Humedity = lastHumedity.toFloat();

    lastPressure = read_DHT_BMP_Data(2);
    float Pressure = lastPressure.toFloat();

    lastAltitude = read_DHT_BMP_Data(3);
    float Altitude = lastAltitude.toFloat();

    lastSealevelPressure = read_DHT_BMP_Data(4);
    float SealevelPressure = lastSealevelPressure.toFloat();

    lastreal_Altitude = read_DHT_BMP_Data(5);
    float real_Altitude = lastreal_Altitude.toFloat();

    
    // Check if temperature is above threshold and if it needs to send the Email alert
    if (temperature > temp_threshold.toFloat() && inputMessage2 == "true" && !emailSent) {
      String emailMessage = String("Temperature above threshold. Current temperature: ") + String(temperature) + String("C");
      if (sendEmailNotification(emailMessage)) {
        // Serial.println(emailMessage);
       //emailSent = true;
      } else {
        Serial.println("Email failed to send");
      }
    }
    // Check if temperature is below threshold and if it needs to send the Email alert
     if ((Humedity > humidity_threshold.toFloat()) && inputMessage2 == "true" && !emailSent) {
      String emailMessage = String("Humedity above threshold. Current Humedity: ") + String(Humedity) + String(" C");
      if (sendEmailNotification(emailMessage)) {
        // Serial.println(emailMessage);
        //emailSent = true;
      } else {
        Serial.println("Email failed to send");
      }
    }

    if (Pressure > pressure_threshold.toFloat() && inputMessage2 == "true" && !emailSent) {
      String emailMessage = String("Pressure above threshold. Current Pressure: ") + String(Pressure) + String("C");
      if (sendEmailNotification(emailMessage)) {
        // Serial.println(emailMessage);
        //emailSent = true;
      } else {
        Serial.println("Email failed to send");
      }
    }
    // Check if temperature is below threshold and if it needs to send the Email alert
    if ((Altitude > altitude_threshold.toFloat()) && inputMessage2 == "true" && !emailSent) {
      String emailMessage = String("Altitude above threshold. Current Altitude: ") + String(Altitude) + String(" C");
      if (sendEmailNotification(emailMessage)) {
        // Serial.println(emailMessage);
        //emailSent = true;
      } else {
        Serial.println("Email failed to send");
      }
    }

    if (SealevelPressure > sealevelpressure_threshold.toFloat() && inputMessage2 == "true" && !emailSent) {
      String emailMessage = String("SealevelPressure above threshold. Current SealevelPressure: ") + String(SealevelPressure) + String("C");
      if (sendEmailNotification(emailMessage)) {
        // Serial.println(emailMessage);
        //emailSent = true;
      } else {
        Serial.println("Email failed to send");
      }
    }
    // Check if temperature is below threshold and if it needs to send the Email alert
    if ((real_Altitude > realaltitude_threshold.toFloat()) && inputMessage2 == "true" && !emailSent) {
      String emailMessage = String("real_Altitude above threshold. Current real_Altitude: ") + String(real_Altitude) + String(" C");
      if (sendEmailNotification(emailMessage)) {
        // Serial.println(emailMessage);
        //emailSent = true;
      } else {
        Serial.println("Email failed to send");
      }
    }
   
      // String stemperature(temperature);
      // String sHumedity(Humedity);
      // String sPressure(Pressure);
      // String sAltitude(Altitude);
      // String sSealevelPressure(SealevelPressure);
      // String sreal_Altitude(real_Altitude);
      // sendData("Temperature="+stemperature+"&Humidity="+sHumedity+"&Pressure="+sPressure+"&Altitude="+sAltitude+"&SealevelPressure="+sSealevelPressure+"&RealAltitude="+sreal_Altitude);
        
  }
  
  
}


bool sendEmailNotification(String emailMessage) {
  // Set the SMTP Server Email host, port, account and password
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);

  // For library version 1.2.0 and later which STARTTLS protocol was supported,the STARTTLS will be
  // enabled automatically when port 587 was used, or enable it manually using setSTARTTLS function.
  //smtpData.setSTARTTLS(true);

  // Set the sender name and Email
  smtpData.setSender("ESP32", emailSenderAccount);

  // Set Email priority or importance High, Normal, Low or 1 to 5 (1 is highest)
  smtpData.setPriority("High");

  // Set the subject
  smtpData.setSubject(emailSubject);

  // Set the message with HTML format
  smtpData.setMessage(emailMessage, true);

  // Add recipients
  smtpData.addRecipient(input_email);

  smtpData.setSendCallback(sendCallback);

  // Start sending Email, can be set callback function to track the status
  if (!MailClient.sendMail(smtpData)) {
    Serial.println("Error sending Email, " + MailClient.smtpErrorReason());
    return false;
  }
  // Clear all data from Email object to free memory
  smtpData.empty();
  return true;
}

// Callback function to get the Email sending status
void sendCallback(SendStatus msg) {
  // Print the current status
  Serial.println(msg.info());

  // Do something when complete
  if (msg.success()) {
    Serial.println("----------------");
  }
  ////////////////

  
  Serial.println();
  Blynk.run();
  timer.run();
  delay(500);
}