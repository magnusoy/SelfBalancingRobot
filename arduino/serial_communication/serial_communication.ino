// Defining global variables for recieving data
boolean newData = false;
const byte numChars = 64;
char receivedChars[numChars]; // An array to store the received data

// Variables holding buttonvalues
const byte numButtons = 5;
int buttonStates[numButtons] = {0, 0, 0, 0, 0};

void setup() {
  Serial.begin(115200);
  pinMode(13, OUTPUT);
}

void loop() {
  readStringFromSerial();
  updateButtonValues();
  if(buttonStates[0] == '1') {
    digitalWrite(13, HIGH);
  } else {
    digitalWrite(13, LOW);
  }
}

/**
  Reads a string from Serial Monitor.
*/
void readStringFromSerial() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;

  while ((Serial.available() > 0) && (!newData)) {
    rc = Serial.read();

    if (rc != endMarker) {
      receivedChars[ndx] = rc;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0'; // Terminate the string
      ndx = 0;
      newData = true;
    }
  }
}

/**
  Fetches the value from a substring,
  wich is seperated with a given symbol.
  @param data your String to be seperated
  @param seperator your symbol to seperate by
  @param index where your value is located
  @return substring before seperator
*/
String getValueFromSerial(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  strIndex[0] = strIndex[1] + 1;
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/**
  Reads from the recived char array and
  fills an array with the stored values.
*/
void updateButtonValues() {
  if (newData) {
    buttonStates[0] = receivedChars[0];
    buttonStates[1] = receivedChars[2];
    buttonStates[2] = receivedChars[4];
    buttonStates[3] = receivedChars[6];
    buttonStates[4] = receivedChars[8];
  }
  newData = false;
}
