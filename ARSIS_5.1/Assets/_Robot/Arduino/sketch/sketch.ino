#include <WiFiNINA.h>
#include <FlashAsEEPROM.h>
#include <FlashStorage.h>

// Motor Pins
#define MOTOR_1_PIN 2
#define MOTOR_2_PIN 3

#define TIME_FOR_SERIAL 1 // seconds to wait
int timeWaitedForSerial;

// The wifi credentials
char wifiName[32];
char wifiPasskey[32];

// The ponter to response from the server
char *response = "";

// The IP address of the server
uint8_t first, second, third, fourth;
IPAddress target(first, second, third, fourth);
WiFiClient connection;
int wifiStatus = WL_IDLE_STATUS;

// the motor speed values
byte motor1Speed = 0;
byte motor2Speed = 0;

// The server connection status
int resultStatus;

// The variable to store the loop activation status
bool initLoop = false;

void setup()
{
    timeWaitedForSerial = 0;
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(MOTOR_1_PIN, OUTPUT);
    pinMode(MOTOR_2_PIN, OUTPUT);
    Serial.setTimeout(100);

    // null terminate all strings
    wifiName[31] = '\0';
    wifiPasskey[31] = '\0';
}

void WriteConfigs()
{
    // first 32 bytes are network name
    for (int i = 0; i < 32; i++)
    {
        EEPROM.write(i, wifiName[i]);
    }
    // next 32 bytes are network pass
    for (int i = 0; i < 32; i++)
    {
        EEPROM.write(i + 32, wifiPasskey[i]);
    }
    // write the IP addrress parts
    // 64 is empty
    EEPROM.write(65, first);
    EEPROM.write(66, second);
    EEPROM.write(67, third);
    EEPROM.write(68, fourth);
    // bytes after that are not allocated
    //  save the data
    EEPROM.commit();
}

void LoadConfigs()
{
    // read the first 32 bytes into the SSID
    for (int i = 0; i < 32; i++)
    {
        wifiName[i] = EEPROM.read(i);
    }
    // read next 32 bytes are network pass
    for (int i = 0; i < 32; i++)
    {
        wifiPasskey[i] = EEPROM.read(32 + i);
    }
    // load IP address
    first = EEPROM.read(65);
    second = EEPROM.read(66);
    third = EEPROM.read(67);
    fourth = EEPROM.read(68);
}

// This handles network input
void HandleUserInput(const char *input)
{
    // we only care about the first 2 bytes
    digitalWrite(LED_BUILTIN, HIGH);
    motor1Speed = input[0];
    motor2Speed = input[1];
    digitalWrite(LED_BUILTIN, LOW);
    // This can segfault if the input is not large enough.
    // That is why the led is on and off
}

// This handles the user serial input
void ShowMenu()
{
    Serial.print("Configure Device:\nw - Change wifi info\np - Print Network Host IP Address\nt - Change IP target\nn - Print network name\nc - Connect to network\ns - Save configuration\nr - Read configs\nq - Exit Setup\n\n>\n");
    while (Serial.available() == 0)
    {
    }
    char result;
    Serial.readBytes(&result, 1);
    switch (result)
    {
    case 'p':
        wifiStatus = WiFi.begin(wifiName, wifiPasskey);
        if (wifiStatus != WL_CONNECTED)
        {
            Serial.print("Couldn't connect. Status code: ");
            Serial.println(wifiStatus);
        }
        else
        {
            Serial.println("The gatetway ip is:");
            Serial.println(WiFi.gatewayIP());
        }
        break;
    case 'w':
        while (Serial.available() != 0)
        {
            Serial.readBytes(&result, 1);
        }
        Serial.print("Enter wifi name:\n");
        while (Serial.available() == 0)
        {
        }
        Serial.readBytes(wifiName, 31);
        Serial.print("Enter wifi password:\n");
        while (Serial.available() == 0)
        {
        }
        Serial.readBytes(wifiPasskey, 31);
        break;
    case 't':
        char tmpchar[3];
        while (Serial.available() != 0)
        {
            Serial.readBytes(&result, 1);
        }
        Serial.print("Enter address:\n");
        while (Serial.available() == 0)
        {
        }
        Serial.readBytes(tmpchar, 3);
        first = atoi(tmpchar);
        Serial.readBytes(tmpchar, 1);
        Serial.readBytes(tmpchar, 3);
        second = atoi(tmpchar);
        Serial.readBytes(tmpchar, 1);
        Serial.readBytes(tmpchar, 3);
        third = atoi(tmpchar);
        Serial.readBytes(tmpchar, 1);
        Serial.readBytes(tmpchar, 3);
        fourth = atoi(tmpchar);

        Serial.print("IP address is: ");
        Serial.print(first);
        Serial.print(".");
        Serial.print(second);
        Serial.print(".");
        Serial.print(third);
        Serial.print(".");
        Serial.println(fourth);
        break;
    case 'n':
        Serial.println("Network name: ");
        Serial.println(wifiName);
        break;
    case 'c':
        Serial.print("Connecting to ");
        target = {first, second, third, fourth};
        Serial.println(wifiName);
        wifiStatus = WiFi.begin(wifiName, wifiPasskey);
        if (wifiStatus != WL_CONNECTED)
        {
            Serial.print("Couldn't connect. Status code: ");
            Serial.println(wifiStatus);
        }
        else
        {
            Serial.println("Connection was succussful.");
            WiFiClient connection;
            int resultStatus;
            if ((resultStatus = connection.connect(target, 28512)))
            {
                Serial.println("posted");
                // Make a HTTP request:
                connection.println("CONNNECTION START");
                unsigned char result[33];
                result[0] = '\0';
                while (result[0] != 'q')
                {
                    connection.println("STATUS: WAITING");
                    while (connection.available() == 0)
                    {
                    }
                    int size = connection.read(result, 32);
                    result[size] = '\0';
                    Serial.print("recieved data: ");
                    Serial.println((char *)result);
                    HandleUserInput((char *)result);
                }
                connection.stop();
            }
            else
            {
                Serial.println("could not transmit data");
                Serial.print("the ip is: ");
                Serial.println(target);
                Serial.println("the status is");
                Serial.println(resultStatus);
                Serial.println("Attempting to ping ip");
                int pingResult = WiFi.ping(target);
                Serial.println(pingResult);
            }
            wifiStatus = WiFi.disconnect();
        }
        break;
    case 'r':
        LoadConfigs();
        Serial.print("Loaded Configuration... ");
        Serial.print("Network name: ");
        Serial.println(wifiName);
        Serial.print("target IP address is: ");
        Serial.print(first);
        Serial.print(".");
        Serial.print(second);
        Serial.print(".");
        Serial.print(third);
        Serial.print(".");
        Serial.println(fourth);
        break;
    case 's':
        Serial.print("Saving Configuration:\n");
        WriteConfigs();
        break;
    case 'q':
        if (EEPROM.isValid())
        {
            Serial.print("Exiting setup...\n");
            timeWaitedForSerial = TIME_FOR_SERIAL * 10 + 1;
        }
        else
        {
            Serial.print("Invalid configs. Try saving or entering network info.\n");
        }
        break;
    default:
        break;
    }
}

void loop()
{
    // handle the config
    while (!Serial && timeWaitedForSerial < TIME_FOR_SERIAL * 10)
    {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(100);
        timeWaitedForSerial++;
    }
    if ((Serial && timeWaitedForSerial < TIME_FOR_SERIAL * 10) || !EEPROM.isValid())
    {
        digitalWrite(LED_BUILTIN, HIGH);
        ShowMenu();
    }
    // handle the wifi stuff
    else
    {
        if (!initLoop)
        {
            initLoop = true;

            // This is nessasary to actually load the info from the eeprom
            LoadConfigs();
            LoadConfigs();
            delay(500);
            LoadConfigs();
            // idk why this is nessesisary

            target = {first, second, third, fourth};
            wifiStatus = WiFi.begin(wifiName, wifiPasskey);
            if (wifiStatus != WL_CONNECTED)
            {
                // Conection failed.
                digitalWrite(LED_BUILTIN, LOW);
                while (true)
                {
                }
            }
            else
            {
                // Connection was successful.
                resultStatus = connection.connect(target, 28512);
            }
        }
        if ((resultStatus))
        {
            digitalWrite(LED_BUILTIN, HIGH);
            unsigned char result[33];
            result[0] = '\0';
            connection.println(response);
            while (result[0] != 'q')
            {
                if (connection.available() != 0)
                {
                    int size = connection.read(result, 32);
                    result[size] = '\0';
                    Serial.print("recieved data: ");
                    Serial.println((char *)result);
                    HandleUserInput((char *)result);
                    connection.println("STATUS: WAITING");
                }
                // if (connection.status() != WL_CONNECTED)
                // {
                //     analogWrite(MOTOR_1_PIN, 0);
                //     analogWrite(MOTOR_2_PIN, 0);
                //     while (true)
                //     {
                //         //do nothing yet: disconnected
                //     }
                // }

                // Motor PWM
                analogWrite(MOTOR_1_PIN, motor1Speed * 2);
                analogWrite(MOTOR_2_PIN, motor2Speed * 2);

                // This is higher-frequency, but probably not nessasary
                /* digitalWrite(MOTOR_1_PIN, HIGH);
                digitalWrite(MOTOR_2_PIN, HIGH);
                delayMicroseconds(min(motor2Speed, motor1Speed));
                //Time to stop the slower motor
                if (motor1Speed < motor2Speed)
                {
                    digitalWrite(MOTOR_1_PIN, LOW);
                }
                else
                {
                    digitalWrite(MOTOR_2_PIN, LOW);
                }
                delayMicroseconds(abs(motor2Speed - motor1Speed));
                //Time to stop the faster motor
                if (motor1Speed < motor2Speed)
                {
                    digitalWrite(MOTOR_2_PIN, LOW);
                }
                else
                {
                    digitalWrite(MOTOR_1_PIN, LOW);
                }
                delayMicroseconds(128 - max(motor2Speed, motor1Speed)); */
            }
        }
        else
        {
            digitalWrite(LED_BUILTIN, LOW);
            while (true)
            {
            }
        }
    }
}
