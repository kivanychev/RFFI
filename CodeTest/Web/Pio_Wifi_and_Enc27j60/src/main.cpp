// Load Wi-Fi library
#include <WiFi.h>

//------------------------------------------------------------------
//            Enc28j60 library configuration
//------------------------------------------------------------------


#include <SPI.h>

#include <Ethernet.h>


// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {
  0x94, 0x3C, 0xC6, 0x8C, 0x77, 0xA4
};

IPAddress eth_ip(192, 168, 1, 177);

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer eth_server(80);

//------------------------------------------------------------------
//             WiFi library configuration
//------------------------------------------------------------------

// Replace with your network credentials
const char* ssid     = "ESP32-Access-Point";
const char* password = "123456789";

// Set web server port number to 80
WiFiServer server(80);

// Variable to store the HTTP request
String header;

// Auxiliar variables to store the current output state
String output26State = "off";
String output27State = "off";

// Assign output variables to GPIO pins
const int output26 = 13;
const int output27 = 27;

void setup() {

  Serial.begin(115200);
  // Initialize the output variables as outputs
  pinMode(output26, OUTPUT);
  pinMode(output27, OUTPUT);

  // Set outputs to LOW
  digitalWrite(output26, LOW);
  digitalWrite(output27, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open

  WiFi.softAP(ssid, password);

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  server.begin();

  // -------------------------------------------------------------
  // Ethernet server configuration
  // -------------------------------------------------------------

  // Setting CS pin connected to GPIO
  Ethernet.init(CONFIG_EXAMPLE_ENC28J60_CS_GPIO);

  Serial.println("\r\nEthernet WebServer:");

  // start the Ethernet connection and the server:
  Serial.println(WiFi.macAddress());
  Ethernet.begin(mac, eth_ip);

  // Check for Ethernet hardware present

  Serial.print("Ethernet hardware status: ");
  Serial.println(Ethernet.hardwareStatus());

  if (Ethernet.hardwareStatus() == EthernetNoHardware) 
  {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true)
    {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }

  Serial.print("Ethernet connecting ");

  while (Ethernet.linkStatus() == LinkOFF) 
  {
    Serial.print(".");    
    delay(500);
  }

  // start the Ethernet server
  eth_server.begin();
  Serial.print("Ethernet server started at ");
  Serial.println(Ethernet.localIP());

}


// ---------------------------------------------------------------------
//                               MAIN LOOP
// ---------------------------------------------------------------------


void loop()
{
  // -------------------------------------------------------------------
  //              WiFi client handling
  // -------------------------------------------------------------------

  WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /26/on") >= 0) {
              Serial.println("GPIO 26 on");
              output26State = "on";
              digitalWrite(output26, HIGH);
            } else if (header.indexOf("GET /26/off") >= 0) {
              Serial.println("GPIO 26 off");
              output26State = "off";
              digitalWrite(output26, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP32 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 26  
            client.println("<p>GPIO 26 - State " + output26State + "</p>");
            // If the output26State is off, it displays the ON button       
            if (output26State=="off") {
              client.println("<p><a href=\"/26/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/26/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

  // -------------------------------------------------------------------
  //              Ethernet client handling
  // -------------------------------------------------------------------

  // listen for incoming clients

  EthernetClient eth_client = eth_server.available(); 
  if (eth_client)
  {
    Serial.println("new client");
    // an http request ends with a blank line
    boolean currentLineIsBlank = true;

    while (eth_client.connected()) 
    {
      if (eth_client.available()) 
      {
        char c = eth_client.read();
        Serial.write(c);
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the http request has ended,
        // so you can send a reply
        if (c == '\n' && currentLineIsBlank) 
        {
          // send a standard http response header
          eth_client.println("HTTP/1.1 200 OK");
          eth_client.println("Content-Type: text/html");
          eth_client.println("Connection: close");  // the connection will be closed after completion of the response
          eth_client.println("Refresh: 2");  // refresh the page automatically every 2 sec
          eth_client.println();
          eth_client.println("<!DOCTYPE HTML>");
          eth_client.println("<html>");

          // output the value of each analog input pin
          for (int analogChannel = 0; analogChannel < 6; analogChannel++) 
          {
            int sensorReading = analogRead(analogChannel);
            eth_client.print("analog input ");
            eth_client.print(analogChannel);
            eth_client.print(" is ");
            eth_client.print(sensorReading);
            eth_client.println("<br />");
          }
          eth_client.println("</html>");
          break;
        }

        if (c == '\n') 
        {
          // you're starting a new line
          currentLineIsBlank = true;
        } 
        else if (c != '\r') 
        {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }

      }
    }
    // give the web browser time to receive the data
    delay(1);

    // close the connection:
    eth_client.stop();
    Serial.println("eth_client disconnected");
  }
  else {
      Serial.println("No Ethernet client");
      delay(500);

  }



}