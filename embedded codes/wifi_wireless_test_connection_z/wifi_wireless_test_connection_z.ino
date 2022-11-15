#define ESP8266 Serial3
String SSID = "alper";
String PASSWORD = "12345678";

int LED = 13;



boolean FAIL_8266 = false;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);
  delay(500);
  digitalWrite(LED, HIGH);
  delay(500);
  digitalWrite(LED, LOW);


  do{
    Serial.begin(115200);
    ESP8266.begin(115200);
  
    //Wait Serial Monitor to start
    while(!Serial);
    Serial.println("--- Start ---");

    ESP8266.println("AT+RST");
    delay(1000);
    if(ESP8266.find("OK"))
    {
      Serial.println("Module is ready");
      ESP8266.println("AT+CWMODE=1");
      delay(2000);
      
      //Quit existing AP, for demo
      Serial.println("Quit AP");
      ESP8266.println("AT+CWQAP");
      delay(1000);
      
      clearESP8266SerialBuffer();
      if(cwJoinAP())
      {
        Serial.println("CWJAP Success");
        FAIL_8266 = false;
        
        delay(3000);
        cwGetIP();
      }else{
        Serial.println("CWJAP Fail");
        delay(500);
        FAIL_8266 = true;
      }
    }else{
      Serial.println("Module have no response.");
      delay(500);
      FAIL_8266 = true;
    }
  }while(FAIL_8266);
  
  digitalWrite(LED, HIGH);
}

void loop() {

}

boolean waitOKfromESP8266(int timeout)
{
  do{
    Serial.println("wait OK...");
    delay(1000);
    if(ESP8266.find("OK"))
    {
      return true;
    }

  }while((timeout--)>0);
  return false;
}

boolean cwJoinAP()
{
  String cmd="AT+CWJAP=\"" + SSID + "\",\"" + PASSWORD + "\"";
  ESP8266.println(cmd);
  return waitOKfromESP8266(10);
}

//show IP address on Serial Monitor
void cwGetIP()
{
  clearESP8266SerialBuffer();
  ESP8266.println("AT+CIFSR");
  delay(1000);
  
  while (ESP8266.available() > 0) {
    char a = ESP8266.read();
    Serial.write(a);
  }
}

void clearESP8266SerialBuffer()
{
  Serial.println("= clearESP8266SerialBuffer() =");
  while (ESP8266.available() > 0) {
    char a = ESP8266.read();
    Serial.write(a);
  }
  Serial.println("==============================");
}
