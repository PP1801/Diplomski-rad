/*  amt203_nano.ino
 *  23.06.2023.   Putanec
 *
 *  RADI !!!
 * 
 *  u kombinaciji sa ( uno_i2c_enc_of.ino )
 * 
 *  Arduino NANO     ->  COM 10
 *  Arduino Pro Mini ->  COM 7      (USBasp programmer)
 * 
 */

#include <SPI.h>
#include <Wire.h>
//#include <tca9548a.h>  // mux

#define baudRate 115200     //this is the serial baud rate for talking to the Arduino
#define timoutLimit 100     //this will be our SPI timout limit

String sketch_name = "amt203_nano_of.ino";

// SPI commands used by the AMT20
#define nop             0x00 //no operation
#define rd_pos          0x10 //read position
#define set_zero_point  0x70 //set zero point
#define set_zero_ack    0x80 //set zero acknowledge

//set the chip select pin for the AMT20
#define AMT203_CS_PIN 10 // 4
#define AMT203_READ_FREQ 50 // in Hz
uint16_t currentPosition;   //this 16 bit variable will hold our 12-bit position
uint16_t oldPosition = 0;

static uint32_t amt203_scan_time;
uint8_t retryLimit = 100; //number of retries while waiting for the sensor's response
volatile bool flag_enc_set_zero = 0;


byte high;
byte low;

// SPI initialization for PAA5100JE optical flow sensor
#include "PAA5100JE.h"
#define OF_CS_PIN     9     // define SPI chip select pin
#define OF_READ_FREQ 100    //100 Define reading frequency in Hz

PAA5100JE_OF of_sensor(OF_CS_PIN);

static uint32_t opti_scan_time;
double of_working_height = 18.0; // in mm
double of_delta_xy_mm[2] = {0.0, 0.0};
double of_xy_mm[2] = {0.0, 0.0};

bool opti_initialized = 0;
volatile bool flag_of_set_zero = 0;

float of_x_float = 0.00F;
float of_y_float = 0.00F;
char  of_x_buffer[7];
char  of_y_buffer[7];


void setup()
{
    
    Serial.begin(baudRate);  //Initialize the UART serial connection

    //Set I/O mode of all SPI pins.
    pinMode(SCK, OUTPUT);
    pinMode(MOSI, OUTPUT);
    pinMode(MISO, INPUT);
    pinMode(AMT203_CS_PIN, OUTPUT);

    Wire.begin(40);               // join i2c bus with address #40
    Wire.onRequest(requestEvent); // register event
    Wire.onReceive(receiveEvent); // register event

    if(OPTI_ON()){
        opti_initialized = 1;
        Serial.println(F("Opti OK!"));}
    else{
        Serial.println(F("Opti NOT OK!"));}

    delay(1000);
    Serial.println(sketch_name);
}


//after the setup() method this loop gets entered and is the main() function for our program
void loop()
{
    uint32_t amt203_time = millis();
    if ((amt203_time - amt203_scan_time) >= (1000 / AMT203_READ_FREQ))
    {
        AMT_getPos();
        if(currentPosition != oldPosition)
        {
            oldPosition = currentPosition;
            Serial.print("P: ");
            Serial.println(currentPosition, DEC);
        }
        amt203_scan_time = amt203_time;
    }


    if(opti_initialized)
    {
        uint32_t opti_time = millis();
        if ((opti_time - opti_scan_time) >= (1000 / OF_READ_FREQ))
        {
            if (of_sensor.getDistance(of_delta_xy_mm) == false)
                Serial.println("Error reading sensor values");
            else
            {
                of_xy_mm[0] += of_delta_xy_mm[0];
                of_xy_mm[1] += of_delta_xy_mm[1];
                Serial.print("X:\t");
                Serial.print(of_xy_mm[0]);
                Serial.print(" mm \t Y:\t");
                Serial.print(of_xy_mm[1]);
                Serial.print(" mm\n");
            }
            opti_scan_time = opti_time;
        }   
    }

    if (flag_enc_set_zero && AMT_setZero())
    {
        Serial.println(F("done!"));
        flag_enc_set_zero = 0;
    }
    if (flag_of_set_zero)
    {
        of_xy_mm[0] = 0.00;
        of_xy_mm[1] = 0.00;
        flag_of_set_zero = 0;
        // seems to only work once, perhaps the flag refuses to be changed back to 1?
    }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void requestEvent(void)
{

    Wire.write(high);
    Wire.write(low);
    //Wire.endTransmission();
    //Serial.print("S: ");
    //Serial.println(currentPosition, DEC);

    dtostrf(of_xy_mm[0],7,2,of_x_buffer);
    Wire.write(of_x_buffer);

    dtostrf(of_xy_mm[1],7,2,of_y_buffer);
    Wire.write(of_y_buffer);
}

void receiveEvent(int HowMany)
{
    if(HowMany == 1)        // set encoder zero
    {
        flag_enc_set_zero = 1;
    }
    else if (HowMany == 2)  // reset of readings
    {
        flag_of_set_zero = 1;
    }
}



bool OPTI_ON(void)
{
    if (!of_sensor.init()) return 0;
    else
    {
        of_sensor.setWorkingHeight(of_working_height);
        of_sensor.setOrientation(true, true, false);
        return 1;
    }
}

//We will use this function to handle transmitting SPI commands in order to keep our code clear and concise.
//It will return the byte received from SPI.transfer()
uint8_t SPIWrite(uint8_t sendByte)
{
    //holder for the received over SPI
    uint8_t data;
    //SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
    //the AMT20 requires the release of the AMT203_CS_PIN line after each byte
    digitalWrite(AMT203_CS_PIN, LOW);
    delayMicroseconds(1);

    data = SPI.transfer(sendByte);
    delayMicroseconds(5);

    digitalWrite(AMT203_CS_PIN, HIGH);
    SPI.endTransaction();
    delayMicroseconds(5);
    
    return data;
}

void AMT_getPos(void)
{
    uint8_t data;               //this will hold our returned data from the AMT20
    uint8_t timeoutCounter;     //our timeout incrementer
    uint16_t currentPos;   //this 16 bit variable will hold our 12-bit position    
    //reset the timoutCounter;
    timeoutCounter = 0;

    //send the rd_pos command to have the AMT20 begin obtaining the current position
    data = SPIWrite(rd_pos);
    
    //we need to send nop commands while the encoder processes the current position. We
    //will keep sending them until the AMT20 echos the rd_pos command, or our timeout is reached.
    while (data != rd_pos && timeoutCounter++ < timoutLimit)
    {
        data = SPIWrite(nop);
    }
    delayMicroseconds(10);
    if (timeoutCounter < timoutLimit)
    {  //rd_pos echo received
        //We received the rd_pos echo which means the next two bytes are the current encoder position.
        //Since the AMT20 is a 12 bit encoder we will throw away the upper 4 bits by masking.
        //Obtain the upper position byte. Mask it since we only need it's lower 4 bits, and then
        //shift it left 8 bits to make room for the lower byte.

        high = (SPIWrite(nop)& 0x0F);
        currentPos = high << 8;  // < --
        delayMicroseconds(1);
        //OR the next byte with the current position
        low = SPIWrite(nop);
        currentPos |= low;
        delayMicroseconds(1);
        currentPosition = currentPos;  // <--
    }
    else
    {   //timeout reached
        //This means we had a problem with the encoder
        Serial.print(F("Error obtaining position.\n"));
        delay(1);
        //timeoutCounter = 0;
        //while(true);

    }
}

bool AMT_setZero(void)
{
    uint8_t data;               //this will hold our returned data from the AMT20
    uint8_t retryCounter = 0;   //reset the retryCounter;

    //send the set_zero_point command
    data = SPIWrite(set_zero_point);
    //we need to send nop commands while the encoder sets the current position as zero.
    while (data != set_zero_ack && retryCounter++ < retryLimit)
    {
        data = SPIWrite(nop);
    }
    delayMicroseconds(10);
    if (retryCounter < retryLimit)
    {  //set_zero_ack received
        //Set zero was successful and the new position offset is stored in EEPROM.
        //Now the encoder must be power cycled, otherwise the position values will not
        //be calculated off the latest zero position.
        return 1;
    }
    else
    {
        digitalWrite(AMT203_CS_PIN, HIGH); // <-- novo
        SPI.endTransaction();      // <--
        return 0;
    }  
}