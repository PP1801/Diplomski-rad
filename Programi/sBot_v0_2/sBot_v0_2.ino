/*
    sBot_testing           07.07.2023.
    OpenCR              Putanec

    za dobivanje parametara modela
    > nema natezanja sajle
    > ima 1 tof (LR_F)
*/

/*
 *   _________________________________
 *  | Spojeno | Radi  | !Radi | Note  |
 *  |2x Tof_SR|       |       |       |
 *  |2x Tof_LR|       |       |       |
 *  | i2c mux |       |       |       |
 *  | PAA OF  |       |       |       |
 *  | 9-dof   |       |       |       |
 *  | AMT203  |       |       |       |
 *  | AMT102  |       |       |       |
 *  | NRF24L01|       |       |       |
 *  | INA260  |       |       |       |
 *  |1x  force|       |       |       |
 *  |2x  force|       |       |       |
 *  |RoboClaw |       |       |       |
 *  ˙"""""""""""""""""""""""""""""""""˙
 */

#include <SPI.h>

#include <Wire.h>
#include <tca9548a.h>
TCA9548A multiplexer;

#define THIS_BOT_NUMBER     2   // 1/2 <---

#define sketch_name "sBot_v0_2.ino"
#define serial_baud_rate    115200 

#define BDPIN_LED_USER_1    22
#define BDPIN_LED_USER_2    23
#define BDPIN_LED_USER_3    24

#define PRINT_FREQ 25
static uint32_t print_time = 0;

// _____________________________________  AMT203 (apsolutni)
uint16_t enc_abs_position = 0;   //16 bit variable for 12-bit position
uint16_t old_abs_position = 0;

// _____________________________________  AMT102 (inkrementalni)
const byte enc_pin_A = 3;
const byte enc_pin_B = 2;
int32_t enc_rel_position = 0;
int32_t old_rel_position = -1;
byte rel_state;     // stanje na kanalima pinova A i B

//// _____________________________________  Force sensor
//#include "HX711.h"
//#define force_scaling_factor 60.7f 
//#define FORCE_READ_FREQ      10   // [Hz]
//
//HX711 force_sensor;
//
//const byte force_DOUT_pin = 53;
//const byte force_SCK_pin = 52;
//float force = 0.0;
//static uint32_t force_scan_time = 0;

// _____________________________________  PI regulator brzine
#include "PID_v1.h"

double PI_w_in, PI_w_out, PI_w_ref;  // ulaz, izlaz i referenca za kutnu brzinu 
float Kcw = 0.1783;      // pojacanje PI regulatora brzine
float Tcw = 0.0314;     // integracijska konstanta PI-reg brzine
float Kp_w = Kcw;       // P pojacanje regulatora brzine
//float Kp_w = 2.0*Kcw;
float Ki_w = Kcw/Tcw*7.0; // I pojacanje regulatora brzine
//float Ki_w = ;
float Kd_w = 0.0;       // D pojacanje regulatora brzine
PID PI_w(&PI_w_in, &PI_w_out, &PI_w_ref, Kp_w, Ki_w, Kd_w, P_ON_M, DIRECT);

int min_voltage_speed = 22; // mrtva zona motora [V] ->  22 ~= 2 V

#define w_filter_freq 20    // frekvencija filtriranja za izracun
float w_k = 0.0;            // kutna brzina kotaca
float w_k_f = 0.0;          // kutna brzina kotaca -> filtrirano
float w_n = 0.0;            // kutna brzina, memorija w(n)
float w_n_p1 = 0.0;         // kutna brzina, memorija w(n+1)

float T_s_w = 20.0/1000.0;  // vrijeme uzorkovanja enkodera [ms/1000 -> s]
float T_f_w = 50.0/1000.0;  // vremenska konstanta filtera brzine [s]
float K_f_w = 1.0;          // pojacanje filtera brzine [/]
static uint32_t w_filt_time = 0;

float w_konst = 50.0/8192.0;

static uint32_t drive_timer = 0;
int drive_time = 0;
int drive_voltage = 0;
bool manual_drive = 0;

static uint32_t begin_time = 0;

bool show_printfs = 0;

float k_ramp = 0.0;             // nagib zeljenog rasta napona
float rise_time = 2000.0;          // zeljeno vrijeme rasta napona do postavljene vrijednosti [ms]
static uint32_t tick = 0;
static uint32_t rise_timer = 0;
int temp_wheel_voltage = 0;

// _____________________________________  P regulator sile sajle
bool regulate_force = 0;        // za upaliti/ugasiti reg. petlju sile
float ref_force = 0.0;          // memorija za referencu sile 
float force_tolerance = 100.0;  // dopustena tolerancija oko reference sile
float K_f = 0.1;                // pojacanje P-regulatora sile
int min_voltage_force = 22;     // mrtva zona motora [V] ->  22 ~= 2 V 
int temp_wire_voltage = 0;      // napon natezanja (samo za manual natezanje)

// _____________________________________  P regulator pozicije robota
bool regulate_position = 0;     // za upaliti/ugasiti reg. petlju pozicije  
float ref_position = 0.0;       // memorija za referencu pozicije
float Kcz = 60.4;               // pojacanje P-regulatora polozaja
float Rsr = 34.4; //(40 srednji)// kontakni polumjer kotaca [mm] -> eksperimentalno
// Rsr (bot 1) = 36.273*1.5   = 54.4                    
// Rsr (bot 2) = 36.273*0.912 = 33.08  mozda je bio 40  -v
// Rsr (bot 2) = 40.0  *0.912 = 36.48  <- onda probaj ovo

// Rsr (bot 1) = 54.4 * 0.877 = 47.7
// Rsr (bot 2) = 36.48*0.943 = 34.4



float f_pos = PI*Rsr/4096.0;    // faktor za dobivanje pozicije iz broja inpulsa
float enc_calc_position = 0.0;  // izracunati prevaljeni put po enkoderu
float wr_max = 1.2; //(2.81 ?) // maksimalna kutna brzina kotaca

// wr_max (bot1) = 1.0   <-  sporiji da bi ga (bot2) pricekao
// wr_max (bot2) = 1.2

int pid_compute_times = 0;

float position_diff = 0.0;


bool bot_sync = 0;              // za upaliti/ugasiti sinkronizaciju preko nagiba sajle
float K_sink = 0.05;            // pojacanje korekcije brzine za sinkronizaciju
uint16_t enc_abs_reference = 0; // referenca koja predstavlja nulti nagib sajle
int16_t nagib = 0;             // nagib sajle (razlika mjerene pozicije i referentne)

#include "RoboClaw.h"
//SoftwareSerial serial(10,11);
RoboClaw roboclaw(&Serial1, 10000);
#define rc_addr 0x80

//// _____________________________________  INA260 Current, Voltage and Power meter
//#include "Adafruit_INA260.h"
//#define INA_READ_FREQ   25      // [Hz]
//
//Adafruit_INA260 ina260 = Adafruit_INA260();
//static uint32_t ina_scan_time = 0;
//
//bool ina_initialized = 0;

// _____________________________________  ToF senzori
#include <VL6180X.h>   // short range tof sensor
#include <VL53L1X.h>   // long range tof sensor

VL6180X tof_SR_F;  // SR_F    multiplexer channels:
VL6180X tof_SR_B;  // SR_B    | SR_F |1    | LR_F |3
VL53L1X tof_LR_F;  // LR_F     ______       ______
VL53L1X tof_LR_B;  // LR_B    | SR_B |2    | LR_B |4


// _____________________________________  Optical flow
#define NANO_READ_FREQ 50
static uint32_t nano_read_time = 0;
double of_xy_mm[2] = {0.0, 0.0};

// _____________________________________  nrf24l01
#include "RF24.h"
#define NRF_CE_PIN  7
#define NRF_CNS_PIN 8
RF24 radio(NRF_CE_PIN, NRF_CNS_PIN);  // Let these addresses be used for the pair

const byte bot_reading_address[5] = {'1','s','b','o','t'};
const byte bot_master_address[5] = {'t','e','n','s','y'};
bool radio_initialized = 0;

typedef struct command_struct   // tip strukture koji sadrzi zadane/zeljene vrijednosti
{
    byte    bot_number;     // adresa za raspoznavanje kome se master obraca  1 byt
    byte    in_command;     // dolazeca naredba od mastera                    1 byt
    int16_t wheel_voltage;  // direktni napon motora kotaca                   2 byta
    float   ref_wire_force; // referentna sila sajle                          2 byta
    float   ref_position;   // referentna pozicija/visina robota              4 byta
    float   ref_speed;      // referentna brzina gibanja robota               4 byta
};                          //                                        suma = 14 byta

typedef struct sensor_struct_1  // tip strukture koji formatira ocitanja sa senzora
{
    int16_t tof3;          // Long_R_UP     2 byta
    uint16_t position_abs;  // wire encoder  2 byta
    float   position_rel;  // wheel encoder 4 byta
    float    wire_force;    // wire force    2 byta
    float  of_distance_y;   // OF distance x 2 byta
    float  of_distance_x;   // OF distance y 2 byta
};                          //       suma = 14 byta

  //double   of_dy;         // OF distance y 4 byta


typedef struct sensor_struct_2  // nastavak podataka/ocitanja sa senzora
{
    uint16_t tof1;          // Short_R_UP    2 byta
    uint16_t tof2;          // Short_R_DOWN  2 byta
    uint16_t tof4;          // Long_R_DOWN   2 byta
    float    ina_c;         // ina current   4 byta
    float    ina_v;         // ina voltage   4 byta
};                          //       suma = 14 byta

command_struct command_data = {0, 0, 0, 0.0, 0.0, 0.0};
sensor_struct_1 sensor_data1 = {0, 0, 0.0, 0.0, 0.0, 0.0};
sensor_struct_2 sensor_data2 = {0, 0, 0, 0.0, 0.0};

void PID_INIT(void)
{
    PI_w.SetSampleTime(20);
    PI_w.SetOutputLimits(-126,126);
    PI_w_ref = 0.0;
}

// > > > > > > > > > > > > > > > > > > >  nRF24l01 inicijalizacija
bool RADIO_ON(void)
{
    if (!radio.begin()) 
        return 0;
    else
    {
        radio.setPALevel(RF24_PA_LOW);

        //radio.enableDynamicPayloads();

        radio.openWritingPipe(bot_master_address);      // svi roboti odasilju na istu adresu mastera
        radio.openReadingPipe(1, bot_reading_address);  //

        radio.setPayloadSize(sizeof(command_data));

        radio.setRetries(3,5);      // delay, count
        radio.startListening();
        return 1;
    }
}

void NANO_GET(void)
{
    multiplexer.openOneChannel(7);

    uint32_t nano_time = millis();
    if ((nano_time - nano_read_time) >= (1000 / NANO_READ_FREQ))
    {
        Wire.requestFrom(40, 16);   // zatrazi 16 byta od perifernog nano/micro pod adresom 40
                                    // 2 byta za 16-bitnu poziciju enkodera
                                    // 7 byta za float (0.00F) -> x pomak of senzora
                                    // 7 byta za float (0.00F) -> y pomak of senzora
        nano_read_time = nano_time;
    }
    
    if (15 <= Wire.available())
    {
        byte high = Wire.read();    // recieve high byte
        byte low = Wire.read();     // recieve low byte

        enc_abs_position = (high << 8) + low; // merge into final reading

        String dataString = "";
        while(Wire.available() >= 8){
            char in_char = Wire.read();
            dataString = dataString + in_char;}

        of_xy_mm[0] = dataString.toFloat();

        dataString = "";
        while(Wire.available()){
            char in_char = Wire.read();
            dataString = dataString + in_char;}

        of_xy_mm[1] = dataString.toFloat();
    }
    multiplexer.closeOneChannel(7);
}

//bool INA_ON(void)
//{
//    multiplexer.openOneChannel(6);
//    if(!ina260.begin()) return 0;
//    else
//    {
//        ina260.setMode(INA260_MODE_CONTINUOUS);
//        ina260.setCurrentConversionTime(INA260_TIME_140_us);
//        ina260.setVoltageConversionTime(INA260_TIME_140_us);
//        return 1;
//    }
//    multiplexer.closeOneChannel(6);
//}

bool TOF_ON(void)
{
    static bool LR_F_initialized = 0;
    //static bool LR_B_initialized = 0;

    //multiplexer.openOneChannel(1);
    //tof_SR_F.init();        tof_SR_F.configureDefault();
    //tof_SR_F.setTimeout(0); tof_SR_F.startRangeContinuous(20);
    //multiplexer.closeOneChannel(1);
    //delay_ms(1);

    //multiplexer.openOneChannel(2);
    //tof_SR_B.init();        tof_SR_B.configureDefault();
    //tof_SR_B.setTimeout(0); tof_SR_B.startRangeContinuous(20);
    //multiplexer.closeOneChannel(2);
    //delay_ms(1);

    multiplexer.openOneChannel(3);
    tof_LR_F.setTimeout(0);
    if(!tof_LR_F.init()) LR_F_initialized = 0;
    else{
        tof_LR_F.startContinuous(20);
        LR_F_initialized = 1;}
    multiplexer.closeOneChannel(3);
    delay_ms(1);

    //multiplexer.openOneChannel(4);
    //tof_LR_B.setTimeout(0);
    //if(!tof_LR_B.init()) LR_B_initialized = 0;
    //else{
    //    tof_LR_B.startContinuous(20);
    //    LR_B_initialized = 1;}
    //multiplexer.closeOneChannel(4);
    //delay_ms(1);

    return (LR_F_initialized); //&& LR_B_initialized);
}

//void FORCE_ON(void)
//{
//    force_sensor.begin(force_DOUT_pin, force_SCK_pin);  // data, clock, gain f
//    force_sensor.set_scale(60.7f);
//    force_sensor.tare();
//    delay_ms(100);
//}

void AMT102_ON(void)
{
    pinMode(enc_pin_A, INPUT_PULLUP);
    pinMode(enc_pin_B, INPUT_PULLUP);
    delay_ms(2);

    AMT102_reset_position();

    attachInterrupt(0, pin_A_isr, CHANGE);
    attachInterrupt(1, pin_B_isr, CHANGE);
    interrupts();
}
// AMT102 functions
void AMT102_reset_position(void)
{
    enc_rel_position = 0;
    byte state = 0;
    if(digitalRead(enc_pin_A)) state |= 1;
    if(digitalRead(enc_pin_B)) state |= 2;
    rel_state = state;
}

void pin_A_isr(void) { update_state();}

void pin_B_isr(void) { update_state();}

void update_state(void)
{
    byte state = rel_state & 3;
    if(digitalRead(enc_pin_A)) state |= 4;
    if(digitalRead(enc_pin_B)) state |= 8;
    switch(state)
    {
        case 0: case 5: case 10: case 15:
            break;
        case 1: case 7: case 8: case 14:
            enc_rel_position++; break;
        case 2: case 4: case 11: case 13:
            enc_rel_position--; break;
        case 3: case 12:
            enc_rel_position += 2; break;
        default:
            enc_rel_position -= 2; break;
    }
    rel_state = (state >> 2);
}

void setup()
{
    Serial.begin(serial_baud_rate);
    delay(200);
    inif(3);
    Serial.println(sketch_name);

    roboclaw.begin(38400);

    PID_INIT();
    
    Wire.begin();
    multiplexer.begin();

    if(RADIO_ON()){
        radio_initialized = 1;
        Serial.println(F("Radio OK!"));}
    else{Serial.println(F("Radio FAILED!"));}

    //if(INA_ON()){
    //    ina_initialized = 1;
    //    Serial.println(F("INA260 OK!"));}
    //else{Serial.println(F("INA260 FAILED!"));}
    
    //FORCE_ON();

    TOF_ON();

    AMT102_ON();

}

void loop()
{
    if (radio_initialized)
    {
        uint8_t recieve_pipe;
        if (radio.available(&recieve_pipe))     // PRISTIGLA PORUKA
        {
            uint8_t recieve_bytes = radio.getPayloadSize(); // treba biti kao i command_struct (16 byta)
            if (recieve_bytes > 1)
            {
                radio.read(&command_data, recieve_bytes);

                if (command_data.in_command != 2)
                {
                    Serial.print("\n_____master:_____");
                    Serial.print("Recieved "); Serial.print(recieve_bytes);
                    Serial.print(" on pipe "); Serial.print(recieve_pipe); Serial.println(":");
                    PRINT_COMMAND();
                }

            }
        }

        if (command_data.in_command == 1)   // ***** EMERGENCY STOP COMMAND
        {
            roboclaw.ForwardM1(rc_addr, 0); // odmah smanji napon motora
            //roboclaw.ForwardM2(rc_addr, 0); //      -||-
            //regulate_force = 0;             // iskljuci reg. petlju sile
            PI_w.SetMode(MANUAL);           // iskljuci reg. petlju brzine
            PI_w_out = 0;
            regulate_position = 0;          // iskljuci reg. petlju pozicije

            bot_sync = 0;
            manual_drive = 0;

            command_data.in_command = 0;    
            Serial.println("STOPPED!");
        }

        else if (command_data.in_command == 2    // ***** SEND SENSOR READINGS 
            && command_data.bot_number == THIS_BOT_NUMBER)
        {
            if(manual_drive || bot_sync)
            {
                sensor_data1.position_abs = nagib;       // AMT203_nagib
                sensor_data1.wire_force = w_k_f;         //  w_wheel
                sensor_data1.tof3 = temp_wheel_voltage;  //  u_motor
                SEND_SENSORS();
            }
            else
                SEND_SENSORS();
                                                                                                                                                                                            
            command_data.in_command = 0;    // reset command -> executed
            command_data.bot_number = 0;    // reset bot number
        }

        //else if (command_data.in_command == 3)      // ***** START FORCE REG
        //{
        //    ref_force = command_data.ref_wire_force;
        //    regulate_force = 1;
        //}

        else if (command_data.in_command == 4)      // ***** START SPEED REG
        {
            PI_w_ref = command_data.ref_speed;
            PI_w.SetMode(AUTOMATIC);
        }

        //else if (command_data.in_command == 5)      // ***** START POSITION REG
        //{
        //    //float ref_position = command_data.ref_position;   // OF -> robot 4
        //    ref_position = command_data.ref_position;       // enkoder -> robot 3
        //    regulate_position = 1; 
        //    PI_w.SetMode(AUTOMATIC);
        //}

        else if (command_data.in_command == 10)     // ***** START POSITION W SYNC
        {
            ref_position = command_data.ref_position;
            regulate_position = 1;
            bot_sync = 1;
            PI_w.SetMode(AUTOMATIC);
            command_data.in_command = 0;

            drive_timer = millis();

            enc_abs_reference = enc_abs_position; // <-- i ovdje
        }

        //else if (command_data.in_command == 6       // ***** TOGGLE SINGLE POSITION REG
        //    && command_data.bot_number == THIS_BOT_NUMBER)
        //{
        //    ref_position = command_data.ref_position;
        //    regulate_position = !regulate_position;
        //    if (regulate_position)
        //    {
        //        PI_w.SetMode(AUTOMATIC);
        //    }
        //    else
        //    {
        //        PI_w.SetMode(MANUAL);
        //        roboclaw.BackwardM1(rc_addr, 0);
        //    }
        //    command_data.in_command = 0;
        //}

        else if (command_data.in_command == 9)      // ***** SET ENCODER REFERENCE
        {
            enc_abs_reference = enc_abs_position;
            command_data.in_command = 0;
        }

        else if (command_data.in_command == 11      // ***** SET NEW PARAMETERS (1)
                && (command_data.bot_number == 0    //              Kcw, Tcw, Kcz
                    || command_data.bot_number == THIS_BOT_NUMBER))
        {
            Kcw = command_data.ref_wire_force;
            Tcw = command_data.ref_position;
            Kcz = command_data.ref_speed;
            
            // recalculate Kcw, Tcw and Kcz dependencies:
            Kp_w = Kcw;
            Ki_w = Kcw/Tcw;
            PI_w.SetTunings(Kp_w, Ki_w*7.0, Kd_w);

            Serial.println(F("Novi parametri (1):"));
            Serial.print(F("Kcw = ")); Serial.println(Kcw, 4);
            Serial.print(F("Tcw = ")); Serial.println(Tcw, 4);
            Serial.print(F("Kcz = ")); Serial.println(Kcz, 4);

            command_data.in_command = 0;
        }

        else if (command_data.in_command == 12      // ***** SET NEW PARAMETERS (2)
                && (command_data.bot_number == 0    //       K_sink, Rsr, wr_max
                    || command_data.bot_number == THIS_BOT_NUMBER))
        {
            K_sink = command_data.ref_wire_force;
            Rsr = command_data.ref_position;
            wr_max = command_data.ref_speed;

            // recalculate K_sink, Rsr and wr_max dependencies
            f_pos = PI*Rsr/4096.0;

            Serial.println(F("Novi parametri (2):"));
            Serial.print(F("K_sink = ")); Serial.println(K_sink, 4);
            Serial.print(F("Rsr    = ")); Serial.println(Rsr, 4);
            Serial.print(F("wr_max = ")); Serial.println(wr_max, 4);

            command_data.in_command = 0;
        }

        else if (command_data.in_command == 15)     // ***** Voznja rampa-konst-Napon 
        {                                           //          t-sekundi
            drive_time = command_data.bot_number;
            drive_voltage = command_data.wheel_voltage;
            manual_drive = 1;
            command_data.in_command = 0;

            k_ramp = (float)drive_voltage/rise_time*1000.0;

            int voltage = round((float)(drive_voltage/12.0)*126.0);
            Serial.print("Drive time: "); Serial.print(drive_time); Serial.println(" s");
            Serial.print("Rise time: "); Serial.print(rise_time); Serial.println(" ms");
            Serial.print("Drive voltage: "); Serial.print(drive_voltage);
            Serial.print(" = "); Serial.println(voltage);
            Serial.print("k_ramp: "); Serial.println(k_ramp);

            k_ramp = k_ramp*0.0105;

            drive_timer = millis();
            Serial.print("start millis: "); Serial.println(drive_timer);

            roboclaw.BackwardM1(rc_addr, voltage);
        }

        else if (command_data.in_command == 13      // ***** HIDE/SHOW Serial.printf()s
                && (command_data.bot_number == 0    //      
                    || command_data.bot_number == THIS_BOT_NUMBER))
        {
            show_printfs = !show_printfs;
            if (show_printfs) Serial.println(F("Show ON"));
            else Serial.println(F("Show OFF"));
            command_data.in_command = 0;
        }



    }   // radio_initialized

    GET_SENSORS();      // citanje stanja senzora

    FILTER_SPEED();

    //REG_WIRE_FORCE();

    REG_WHEEL_SPEED();

    MANUAL_DRIVE();

}

void FILTER_SPEED(void)
{
    uint32_t start_timer = millis();
    if ((start_timer - w_filt_time) >= 1000/w_filter_freq)  // filtriranje brzine
    {
        w_k = (float)(enc_rel_position - old_rel_position)*w_konst;
        old_rel_position = enc_rel_position;            // odredjivanje brzine diferencijom

        w_n = w_n_p1;                                   // novi korak (n+1)
        w_n_p1 = (1.0 - T_s_w/T_f_w)*w_n + (T_s_w/T_f_w)*w_k;
        w_k_f = w_n_p1;                                 // filtrirana brzina

        //PI_w_in = w_k_f;                                // ulaz u regulator
        w_filt_time = start_timer;

        // REG_POSITION();
    }
}

void MANUAL_DRIVE(void)
{
    if (manual_drive)
    {
        uint32_t start_timer = millis();
        //Serial.print("millis: "); Serial.println(start_timer);
        uint16_t elapsed_time = start_timer - drive_timer;

        if (elapsed_time <= (uint16_t)rise_time)
        {
            if ((start_timer - rise_timer) >= 20)  // filtriranje brzine
            {
                tick = tick + 1;
                temp_wheel_voltage = round((float)elapsed_time*k_ramp);

                if (show_printfs)
                {
                    Serial.print(tick); Serial.print("\tt: "); Serial.print(elapsed_time);
                    Serial.print("\tvoltage: "); Serial.println(temp_wheel_voltage);
                }
                
                roboclaw.BackwardM1(rc_addr, temp_wheel_voltage);
                rise_timer = start_timer;

                // REG_POSITION();
            }
        }

        if (start_timer >= (drive_timer + (uint32_t)(drive_time*1000)))
        {
            roboclaw.ForwardM1(rc_addr, 0);
            manual_drive = 0;

            if (show_printfs)
            {
                Serial.println("_________________");
                Serial.print(drive_time); Serial.println(" s elapsed.");

                //Serial.print(F("encS :\t")); Serial.print(enc_rel_position*f_pos, 2);
                Serial.print(F("encS :\t")); Serial.print(sensor_data1.position_rel, 2);
                Serial.print(F("of_s :\t")); Serial.println(sensor_data1.of_distance_y);
            }
        }
    }
}

void GET_SENSORS(void)
{
    //uint32_t start_timer = millis();

    NANO_GET();                         // PAA & AMT203

    //sensor_data1.position_rel = enc_rel_position;
    //sensor_data1.of_distance_x = of_xy_mm[0];
    sensor_data1.of_distance_y = of_xy_mm[1]*1.1722;    
    // (bot 1) correction = 1.0
    // (bot 2) correction = 1.23        

    // (bot 1) correction = 1.0 * 0.96 = 0.96       
    // (bot 2) correction = 1.23*1.072 = 1.31856                         

    //multiplexer.openOneChannel(1);      // TOF#1 SR_F
    //sensor_data2.tof1 = tof_SR_F.readRangeContinuousMillimeters();
    //multiplexer.closeOneChannel(1);

    //multiplexer.openOneChannel(0x02);   // TOF#2 SR_B
    //sensor_data2.tof2 = tof_SR_B.readRangeContinuousMillimeters();
    //multiplexer.closeOneChannel(0x02);

    multiplexer.openOneChannel(0x03);   // TOF#3 LR_F
    if (tof_LR_F.dataReady()) sensor_data1.tof3 = tof_LR_F.read(false);
    multiplexer.closeOneChannel(0x03);

    //multiplexer.openOneChannel(0x04);   // TOF#4 LR_B
    //if (tof_LR_B.dataReady()) sensor_data2.tof4 = tof_LR_B.read(false);
    //multiplexer.closeOneChannel(0x04);

    //if (ina_initialized)                // INA260
    //{
    //    if ((start_timer/1000 - ina_scan_time) >= 1000/INA_READ_FREQ)
    //    {
    //        multiplexer.openOneChannel(0x06);
    //        sensor_data2.ina_c = ina260.readCurrent();
    //        sensor_data2.ina_v = ina260.readBusVoltage();
    //        ina_scan_time = start_timer;
    //        multiplexer.closeOneChannel(0x06);
    //    }
    //}

    //                                    // FORCE
    //if ((start_timer - force_scan_time) >= 1000/FORCE_READ_FREQ)
    //{
    //    sensor_data1.wire_force = force_sensor.get_units();
    //    force_scan_time = start_timer;
    //}

                                        // AMT102
    if (enc_abs_position != old_abs_position)
    {
        old_abs_position = enc_abs_position;
        sensor_data1.position_abs = old_abs_position;
        //Serial.print("abs:\t");
        //Serial.println(old_abs_position);
    }

    sensor_data1.position_rel = enc_rel_position*f_pos;

    //if ((start_timer - print_time) >= 1000/PRINT_FREQ) 
    //{
    //    PRINT_SENSOR();
    //    print_time = start_timer;
    //}

}

void SEND_SENSORS(void)
{
    radio.stopListening();
    //radio.setPayloadSize(sizeof(in_data));
    radio.openWritingPipe(bot_master_address);
    unsigned long send_timer = micros();
    //bool re_report = radio.write(&command_data, sizeof(command_data));
    bool re_report = radio.write(&sensor_data1, sizeof(sensor_data1));
    unsigned long end_send_timer = micros();
    if(!manual_drive)
    {
        if (re_report)
        {
            if (!show_printfs)
            {
                Serial.println("Sent ok! :)"); Serial.print("Took me: ");
                Serial.print(end_send_timer - send_timer); Serial.println(" us.");
            }
        }
        else
        {
            Serial.println(".failed to send :c");
        }
    }
    //flag_send_data = 0;
    radio.openReadingPipe(1, bot_reading_address);
    radio.startListening();
}

void REG_POSITION(void)
{
    if(regulate_position)
    {
        float velocity;
      //float position_diff;
        //enc_calc_position = enc_rel_position*f_pos; // dosad prijedjeni put po enkoderu

        //float robot_position = sensor_data1.of_distance_y;  //OF robot 4
        //float robot_position = enc_calc_position;       //  enkoder robot 3
        //position_diff = (ref_position - robot_position);
      //position_diff = (ref_position - sensor_data1.of_distance_y);

        nagib = enc_abs_position - enc_abs_reference;

        velocity = position_diff*Kcz;

        if (position_diff > 120.0)       // ** jos nismo tamo
        {
            //velocity = min(position_diff, 0.001) * Kcz;
            //velocity = min(position_diff*Kcz, wr_max);
            velocity = min(velocity, wr_max);

            //velocity = velocity - (float)nagib*K_sink;

            velocity = max(velocity, 0.0);
        }
        else if (position_diff < -120.0)  // ** prebacili smo poziciju      2(260)
        {
            //velocity = max(position_diff, -0.001) * Kcz;
            //velocity = min(position_diff*Kcz, -0.01);
            velocity = min(velocity, 0.0);

            //velocity = velocity - (float)nagib*K_sink;

            velocity = max(velocity, -wr_max);
        }
        else 
        {
            velocity = position_diff*0.0002;  //0.0002
            //velocity = 0.0;
            //PI_w_out = 0;
            //PI_w_in = 0.0;
            //PI_w_ref = 0.0;
            //roboclaw.BackwardM1(rc_addr, 0);
            //PI_w.SetMode(MANUAL);
        }
        
        PI_w_ref = (double)(velocity - (float)nagib*K_sink);

        //if (show_printfs)
        //{
        //    Serial.print(millis()-drive_timer); Serial.print(F(" <-"));
        //    Serial.println(F("\n_______________"));
        //    Serial.print(F("refS :\t")); Serial.print(ref_position, 2);
        //    Serial.print(F("\tof_s :\t"));  //Serial.println(robot_position);
        //                                    Serial.print(sensor_data1.of_distance_y);
        //    Serial.print(F("\tencS :\t"));//Serial.print(enc_calc_position, 2);
        //                                    Serial.println(sensor_data1.position_rel, 2);
        //    Serial.print(F("diff :\t")); Serial.print(position_diff, 2);
        //    Serial.print(F("\tnagib:\t")); Serial.println(nagib);
        //}
    }
}

void REG_WHEEL_SPEED(void)
{
    //if ((position_diff < 50.0) && (position_diff > -50.0))  // 260    // 2(50)
    //{
    //    PI_w_in = w_k_f*2.0;
    //}
    //else 
    PI_w_in = w_k_f;

    position_diff = (ref_position - sensor_data1.of_distance_y);


    REG_POSITION();                                 // korekcija reference brzine
    
    if (PI_w.Compute())
    {

        bool close = 0;
        if ((position_diff < 8.0) && (position_diff > -8.0))  // (260)
        {
            close = 1;
        }
        //else
            temp_wheel_voltage = PI_w_out;                     // rezultat PI regulatora


        if((temp_wheel_voltage > 10) && !close)
        {
            roboclaw.BackwardM1(rc_addr, temp_wheel_voltage);
        }
        else if ((temp_wheel_voltage < -10) && !close)
        {
            roboclaw.ForwardM1(rc_addr, -1*temp_wheel_voltage);
        }
        else
        {
            roboclaw.BackwardM1(rc_addr, 0);
            //velocity = 0.0;
            PI_w_out = 0;
            PI_w_in = 0.0;
            PI_w_ref = 0.0;
            temp_wheel_voltage = 0;

        }

        pid_compute_times += 1;

        if (show_printfs && (pid_compute_times >= 5))
        {
            pid_compute_times = 0;
            Serial.print(millis()-drive_timer); Serial.print(F(" <-"));
            Serial.println(F("\n_______________"));
            Serial.print(F("refS :\t")); Serial.print(ref_position, 2);
            Serial.print(F("\tof_s :\t"));  //Serial.println(robot_position);
                                            Serial.print(sensor_data1.of_distance_y);
            Serial.print(F("\tencS :\t"));//Serial.print(enc_calc_position, 2);
                                            Serial.println(sensor_data1.position_rel, 2);
            Serial.print(F("diff :\t")); Serial.print((ref_position - sensor_data1.of_distance_y), 2);
            Serial.print(F("\tnagib:\t")); Serial.println(nagib);
        
            Serial.print(F("w_ref:\t")); Serial.print(PI_w_ref);
            Serial.print(F("\tw_k  :\t")); Serial.print(w_k);
            Serial.print(F("\tw_k_f:\t")); Serial.println(w_k_f);
            Serial.print(F("PI   :\t")); Serial.print(PI_w_out);
            Serial.print(F("\tvolts:\t")); Serial.println(temp_wheel_voltage);
            Serial.println("\n");

            //enc_calc_position = enc_rel_position*PI*Rsr/4096.0;
            //Serial.print(F("encS :\t")); Serial.println(enc_calc_position, 2);
        }
    }

}

void PRINT_SENSOR(void)
{
    Serial.println(F("\n_______________"));
    //Serial.print(F("tof#1:\t")); Serial.print(sensor_data2.tof1);
    //Serial.print(F("\tAbs:\t")); Serial.println(sensor_data1.position_abs);
    //Serial.print(F("tof#2:\t")); Serial.print(sensor_data2.tof2);
    //Serial.print(F("\tRel:\t")); Serial.println(sensor_data1.position_rel);
    //Serial.print(F("tof#3:\t")); Serial.print(sensor_data1.tof3);
    //Serial.print(F("\tof_dx:\t")); Serial.println(sensor_data1.of_distance_x);
    //Serial.print(F("tof#4:\t")); Serial.print(sensor_data2.tof4);
    //Serial.print(F("\tof_dy:\t")); Serial.println(sensor_data1.of_distance_y);
    //Serial.print(F("force:\t")); Serial.println(sensor_data1.wire_force);
    //Serial.print(F("\tina_v:\t")); Serial.print(sensor_data2.ina_v);
    //Serial.print(F("\tina_c:\t")); Serial.println(sensor_data2.ina_c);

    Serial.print(F("\t\tAbs: ")); Serial.println(sensor_data1.position_abs);
    Serial.print(F("\t\tRel: ")); Serial.println(sensor_data1.position_rel);
    Serial.print(F("tof#3: \t")); Serial.print(sensor_data1.tof3);
    //Serial.print(F("\tof_dx:\t")); Serial.println(sensor_data1.of_distance_x);
    Serial.print(F("\t\tof_dy:\t")); Serial.println(sensor_data1.of_distance_y);
}

void PRINT_COMMAND(void)
{
    //Serial.println(in_data);
    //Serial.print("sbot_slave :\t"); Serial.println(command_data.sbot_slave);
    //Serial.print("in_command :\t"); Serial.println(command_data.in_command);
    //radio.flush_rx();
    Serial.print("Sbot: "); Serial.print(command_data.bot_number);
    Serial.print("  Command: "); Serial.println(command_data.in_command);
    Serial.print("Wheel: "); Serial.print(command_data.wheel_voltage);
    Serial.print("  Wire: "); Serial.println(command_data.ref_wire_force);
    Serial.print("R_pos: "); Serial.print(command_data.ref_position);
    Serial.print("  R_speed: "); Serial.println(command_data.ref_speed);
    Serial.println("_________________");
}


// > > > > > > > > > > > > > > > > > > > Initialization flash
void inif (uint8_t nTimes){
    int led_pin_user[3] = { BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3};
    pinMode(led_pin_user[0], OUTPUT);
    pinMode(led_pin_user[1], OUTPUT);
    pinMode(led_pin_user[2], OUTPUT);
    for (u_int8_t j = 0; j < nTimes; j++){
        for ( u_int8_t i = 0; i < 3; i++ ){
            digitalWrite(led_pin_user[i], LOW);
            delay(100);
        }
        for ( u_int8_t i = 0; i < 3; i++ ){
            digitalWrite(led_pin_user[i], HIGH);
            delay(100);
        }
    }
}