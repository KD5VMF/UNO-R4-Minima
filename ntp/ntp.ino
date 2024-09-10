#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <Adafruit_GPS.h>
#include <SD.h>
#include <TimeLib.h>
#include <SoftwareSerial.h>  // Include SoftwareSerial for GPS communication

// GPS Configuration
#define GPS_BAUD 9600
SoftwareSerial mySerial(8, 7);  // RX, TX (pins 8 and 7)
Adafruit_GPS GPS(&mySerial);  // Using SoftwareSerial for GPS communication

// SD Card Settings
const int chipSelect = 4;  // Pin for SD card on the Adafruit GPS shield (use pin 4)
const char* settingsFile = "settings.txt";

// Ethernet Configuration
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 160);  // Default static IP
bool dhcpEnabled = true;  // DHCP enabled by default

// UDP for NTP
EthernetUDP Udp;
unsigned int ntpPort = 123;
const unsigned long seventyYears = 2208988800UL;
byte packetBuffer[48];  // Buffer to hold NTP request

// Flags
bool gpsDebug = false;  // To toggle NMEA data display
bool gpsStatusEnabled = false;  // Flag to track GPS status display
bool gpsFixReported = false;  // Track if GPS fix has been reported

// CLI Variables
String commandBuffer = "";

// Setup function
void setup() {
  Serial.begin(115200);
  clearScreen();
  Serial.println("Initializing GPS NTP Server...");

  // Initialize SD card
  if (!SD.begin(chipSelect)) {
    Serial.println("SD card initialization failed! Check wiring and SD card format.");
  } else {
    loadSettingsFromSD();  // Load settings from SD card at boot
  }

  // Initialize Ethernet with DHCP or Static IP
  initializeEthernet();

  // Initialize UDP
  Udp.begin(ntpPort);

  // Initialize GPS
  mySerial.begin(GPS_BAUD);  // Start the software serial for GPS
  GPS.begin(GPS_BAUD);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);     // 1 Hz update rate

  Serial.println("Waiting for GPS fix...");
  showMenu();  // Show CLI menu
}

// Reinitialize Ethernet based on DHCP setting
void initializeEthernet() {
  clearScreen();
  if (dhcpEnabled) {
    if (Ethernet.begin(mac) == 0) {
      Serial.println("Failed to configure Ethernet using DHCP, falling back to static IP.");
      Ethernet.begin(mac, ip);
    }
  } else {
    Ethernet.begin(mac, ip);
  }

  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
}

void loop() {
  processCLI();  // Handle user input for CLI

  // GPS data handling
  if (GPS.available()) {
    char c = GPS.read();
    if (gpsDebug) {
      Serial.write(c);  // Output NMEA data when debug is enabled
    }
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) {
        Serial.println("Failed to parse GPS data.");
      } else {
        if (GPS.fix) {
          if (!gpsFixReported) {  // Only report once when the GPS fix is obtained
            Serial.println("GPS Fix obtained!");
            gpsFixReported = true;
          }
          if (gpsStatusEnabled) {
            Serial.print("Satellites: ");
            Serial.println(GPS.satellites);
          }
        } else {
          if (gpsFixReported) {  // Report when the GPS fix is lost
            Serial.println("No valid GPS fix.");
            gpsFixReported = false;
          }
        }
      }
    }
  }

  // Handle NTP requests
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    processNTP();
  }
}

// Process incoming CLI commands
void processCLI() {
  while (Serial.available()) {
    char c = Serial.read();
    Serial.print(c);  // Echo character

    if (c == '\n' || c == '\r') {
      commandBuffer.trim();
      handleCommand(commandBuffer);  // Process the command
      commandBuffer = "";  // Clear buffer for next input
      showMenu();  // Show menu after each command
    } else {
      commandBuffer += c;  // Append char to command buffer
    }
  }
}

// Handle individual commands (toggle states)
void handleCommand(String command) {
  clearScreen();  // Clear the screen for a clean output
  if (command == "1") {
    toggleGPSDebug();
  } else if (command == "2") {
    toggleGPSStatus();
  } else if (command == "3") {
    toggleDHCP();
  } else if (command == "4") {
    resetSettings();
  } else if (command == "5") {
    forceSDLoad();
  } else if (command == "6") {
    forceSDSave();
  } else if (command == "7") {
    programReset();
  } else if (command == "8") {
    showCurrentStatus();
  } else {
    Serial.println("Invalid command.");
  }
}

// Show menu options
void showMenu() {
  Serial.println("\nAvailable commands:");
  Serial.println("1. Toggle GPS Debug (currently " + String(gpsDebug ? "ON" : "OFF") + ")");
  Serial.println("2. Toggle GPS Status Display (currently " + String(gpsStatusEnabled ? "ON" : "OFF") + ")");
  Serial.println("3. Toggle DHCP (currently " + String(dhcpEnabled ? "ON" : "OFF") + ")");
  Serial.println("4. Reset settings to default");
  Serial.println("5. Force SD Load");
  Serial.println("6. Force SD Save");
  Serial.println("7. Program Reset");
  Serial.println("8. Show Current System Status");
  Serial.print("> ");
}

// Toggle GPS debug (enable/disable)
void toggleGPSDebug() {
  gpsDebug = !gpsDebug;
  saveSettingsToSD();  // Save the change immediately to SD card
  Serial.println(gpsDebug ? "GPS debug enabled." : "GPS debug disabled.");
}

// Toggle GPS Status Display (enable/disable)
void toggleGPSStatus() {
  gpsStatusEnabled = !gpsStatusEnabled;
  saveSettingsToSD();  // Save the change immediately to SD card
  Serial.println(gpsStatusEnabled ? "GPS status display enabled." : "GPS status display disabled.");
}

// Toggle DHCP on/off
void toggleDHCP() {
  dhcpEnabled = !dhcpEnabled;
  saveSettingsToSD();  // Save the change immediately to SD card
  Serial.println(dhcpEnabled ? "DHCP enabled." : "DHCP disabled.");
  initializeEthernet();  // Reinitialize Ethernet after changing DHCP
}

// Reset settings to default
void resetSettings() {
  gpsDebug = false;
  gpsStatusEnabled = false;
  dhcpEnabled = true;
  saveSettingsToSD();  // Save the reset settings to SD card
  Serial.println("Settings reset to default.");
  initializeEthernet();
}

// Force SD card load
void forceSDLoad() {
  loadSettingsFromSD();
  Serial.println("Settings loaded from SD card.");
}

// Force SD card save
void forceSDSave() {
  saveSettingsToSD();
  Serial.println("Settings saved to SD card.");
}

// Program reset (reinitialize everything)
void programReset() {
  Serial.println("Resetting program...");
  setup();  // Re-run the setup function to reset everything
}

// Show current system status
void showCurrentStatus() {
  Serial.println("Current System Status:");
  Serial.print("IP Address: ");
  Serial.println(Ethernet.localIP());
  Serial.print("Subnet Mask: ");
  Serial.println(Ethernet.subnetMask());
  Serial.print("Gateway: ");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("DHCP Enabled: ");
  Serial.println(dhcpEnabled ? "Yes" : "No");
  
  // GPS Status
  if (GPS.fix) {
    Serial.println("GPS Status: FIXED");
    Serial.print("Satellites: "); Serial.println(GPS.satellites);
    Serial.print("Latitude: "); Serial.println(GPS.latitude, 6);
    Serial.print("Longitude: "); Serial.println(GPS.longitude, 6);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
  } else {
    Serial.println("GPS Status: No Fix");
  }
}

// Process NTP request
void processNTP() {
  Udp.read(packetBuffer, 48);

  if (GPS.fix) {
    unsigned long currentEpoch = gpsToUnixTime();
    unsigned long ntpTime = currentEpoch + seventyYears;

    // Prepare NTP response
    packetBuffer[40] = (ntpTime >> 24) & 0xFF;
    packetBuffer[41] = (ntpTime >> 16) & 0xFF;
    packetBuffer[42] = (ntpTime >> 8) & 0xFF;
    packetBuffer[43] = ntpTime & 0xFF;

    Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
    Udp.write(packetBuffer, 48);
    Udp.endPacket();
  } else {
    Serial.println("No valid GPS fix, skipping NTP response.");
  }
}

// Convert GPS time to Unix time using TimeLib
unsigned long gpsToUnixTime() {
  int year = GPS.year + 2000;
  int month = GPS.month;
  int day = GPS.day;
  int hour = GPS.hour;
  int minute = GPS.minute;
  int second = GPS.seconds;

  // Using TimeLib's tmElements_t to store date/time info
  tmElements_t tm;
  tm.Year = CalendarYrToTm(year);  // Converts year to correct format
  tm.Month = month;
  tm.Day = day;
  tm.Hour = hour;
  tm.Minute = minute;
  tm.Second = second;

  return makeTime(tm);  // Converts to Unix timestamp
}

// Save settings to SD card
void saveSettingsToSD() {
  File file = SD.open(settingsFile, FILE_WRITE);
  if (file) {
    file.print("gpsDebug=");
    file.println(gpsDebug ? "true" : "false");
    file.print("gpsStatusEnabled=");
    file.println(gpsStatusEnabled ? "true" : "false");
    file.print("dhcpEnabled=");
    file.println(dhcpEnabled ? "true" : "false");
    file.close();
    Serial.println("Settings saved to SD card.");
  } else {
    Serial.println("Error saving settings to SD card.");
  }
}

// Load settings from SD card
void loadSettingsFromSD() {
  File file = SD.open(settingsFile, FILE_READ);
  if (file) {
    while (file.available()) {
      String line = file.readStringUntil('\n');
      line.trim();  // Ensure the line is trimmed before comparison

      if (line.startsWith("gpsDebug=")) {
        gpsDebug = (line.substring(9) == "true");
      }
      if (line.startsWith("gpsStatusEnabled=")) {
        gpsStatusEnabled = (line.substring(17) == "true");
      }
      if (line.startsWith("dhcpEnabled=")) {
        dhcpEnabled = (line.substring(12) == "true");
      }
    }
    file.close();
    Serial.println("Settings loaded from SD card.");
  } else {
    Serial.println("No settings found on SD card, using defaults.");
  }
}


// Clear screen (for CLI to look professional and clean)
void clearScreen() {
  Serial.write(27);  // ESC character
  Serial.print("[2J");  // Clear screen command
  Serial.write(27);
  Serial.print("[H");  // Cursor to home position
}
