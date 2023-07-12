/*
    node_cmnd_test      26.06.2023.
    OpenCR              Putanec

    > kod za jedan SafetyBod master mikroupravljac (Teensy 4.0)

    > treba imati strukturu iz koje moze procitati naredbe i reagirat na njih
    > treba imati strukturu koja salje mjerenja sa senzora
    > mora cijelo vrijeme slusati, osim kada master trazi podatke

    > prvo treba napraviti osnovnu komunikaciju (slanje i primanje)

*/

#include <SPI.h>

#define sketch_name "node_teensy_test.ino"
#define serial_baud_rate    115200 

#define UPDATE_FREQ 10

// > > > > > > > > > > > > > > > > > > >  nrf24l01
#include "RF24.h"
#include <nRF24L01.h>
#define NRF_CE_PIN  7
#define NRF_CNS_PIN 8
RF24 radio(NRF_CE_PIN, NRF_CNS_PIN);  // Let these addresses be used for the pair

const byte bot_reading_address[5] = {'1','s','b','o','t'};
const byte bot_master_address [5] = {'t','e','n','s','y'};
bool radio_initialized = 0;

typedef struct command_struct
{
    byte    bot_number;     // adresa za raspoznavanje kome se master obraca  1 byt
    byte    in_command;     // dolazeca naredba od mastera                    1 byt
    int16_t wheel_voltage;  // direktni napon motora kotaca                   2 byta
    float   ref_wire_force; // referentna sila sajle                          2 byta
    float   ref_position;   // referentna pozicija/visina robota              4 byta
    float   ref_speed;      // referentna brzina gibanja robota               4 byta
};  // int32_t              //                                        suma = 14 byta


typedef struct sensor_struct_1  // tip strukture koji formatira ocitanja sa senzora
{
    int16_t tof3;          // Long_R_UP     2 byta
    uint16_t position_abs;  // wire encoder  2 byta
    float    position_rel;  // wheel encoder 4 byta
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

command_struct command_data = {0, 0, 0, 0.0, 0.0, 0.05};
sensor_struct_1 sensor_data1 = {0, 0, 0.0, 0.0, 0.0, 0.0};
sensor_struct_2 sensor_data2 = {0, 0, 0, 0.0, 0.0};

uint8_t command_handler = 0;    // pomocna varijabla za slaganje i raspored slozenih funkcija

bool flag_send_data = 0;
int flag_get_data = 0;

bool bot_get_data = 0;
static uint32_t bot_get_time = 0;
static uint32_t reading_timer = 0;

int fail_counter = 0;
int bot_number = 1;
bool new_data = 0;
bool read_now = 0;

int help_index = 0;

long int n_measurement = 0;

uint16_t AMT203_nagibi[4] = {0, 0, 0, 0};           // = sensor_data1.position_abs;
float AMT102_pozicije[4] = {0.0, 0.0, 0.0, 0.0};    // = sensor_data1.position_rel;
float OF_dy_pozicije[4] = {0.0, 0.0, 0.0, 0.0};     // = sensor_data1.of_distance_y;
float w_wheel[4] = {0.0, 0.0, 0.0, 0.0};            // = sensor_data1.wire_force;
int u_motor[4] = {0, 0, 0, 0};                      // = sensor_data1.tof3;

// > > > > > > > > > > > > > > > > > > >  nRF24l01 inicijalizacija
bool RADIO_ON(void)
{
    if (!radio.begin()) return 0;
    else
    {
        radio.setPALevel(RF24_PA_LOW);
        //radio.enableDynamicPayloads();
        radio.openWritingPipe(bot_reading_address);    // svi roboti odasilju na istu adresu mastera
        radio.openReadingPipe(1, bot_master_address);  //
        //radio.setPayloadSize(sizeof(payload));
        radio.setPayloadSize(sizeof(command_data));

        radio.setRetries(3,5);      // delay, count
        radio.startListening();
        return 1;
    }
}

void setup()
{
    Serial.begin(serial_baud_rate);
    Serial.println(sketch_name);

    if(RADIO_ON())
    {
        radio_initialized = 1;
        Serial.println(F("Radio OK!"));
    }
    else 
        Serial.println(F("Radio NOT OK!"));
}

void loop()
{
    if (Serial.available() > 0)
    {
        String command;
        while (Serial.available() > 0)  // Dohvati naredbu sa serial monitora
        {
            char buffer_char = Serial.read();
            if (buffer_char == '\n') break;
            else command += (char)buffer_char;
        }
                                        // Dekodiraj i izvrsi...
        if      (command_handler == 1)  // >koja naredba?
        {
            byte byte_buff = command.toInt();
            Serial.println(byte_buff);
            command_data.in_command = byte_buff;
            command_handler = 0;
        }        
        
        else if (command_handler == 2)  // >koji Sbot?
        {
            byte byte_buff = command.toInt();
            Serial.println(byte_buff);
            command_data.bot_number = byte_buff;
            if (help_index == 0)
            {
                if(byte_buff == 0) Serial.println("\t(svi)");
                command_handler = 0;
            }
            else 
            {
                Serial.print(F("Voltage: "));
                command_handler = 6;
            }
        }
        else if (command_handler == 3)  // >kolika sila?
        {
            float ref_force = (float)command.toInt()*1.0;
            Serial.println(ref_force);
            command_data.ref_wire_force = ref_force;
            command_handler = 0;
        }
        else if (command_handler == 4)  // >kolika brzina?
        {
            float ref_speed = (float)command.toFloat();
            Serial.println(ref_speed);
            command_data.ref_speed = ref_speed;
            command_handler = 7;
            help_index = 3;
            Serial.print("Press s for start ");
        }
        else if (command_handler == 5)  // >koja pozicija?
        {
            float ref_position = (float)command.toInt()*1.0;
            Serial.println(ref_position);
            command_data.ref_position = ref_position;
            if (help_index == 0) command_handler = 0;
            else
            {
                command_handler = 7;
                Serial.print("Press s for start ");
            }
        }
        else if (command_handler == 6)  // >koliki napon?
        {
            byte byte_buff = command.toInt();
            Serial.println(byte_buff);
            command_data.wheel_voltage = byte_buff;
            if (help_index == 0) command_handler = 0;
            else 
            {
                command_handler = 7;
                Serial.print("Press s for start ");
            }
        }
        else if (command_handler == 7)  // >pokreni regulaciju/testiranje
        {
            char char_buff = command[0];
            if (char_buff == 's')
            {
                Serial.println("proceed");
                if (help_index == 1)        // pokreni napon + vrijeme    !!
                {
                    command_data.in_command = 15;
                }
                else if (help_index == 2)   // pokreni regulaciju pozicije!!
                {
                    command_data.in_command = 10;
                }
                else if (help_index == 3)   // pokreni regulaciju brzine  !!
                {
                    command_data.in_command = 4;
                }
                flag_send_data = 1;
                bot_number = 0;
            }
            else
            {
                Serial.println("aborted");
            }
            command_handler = 0;
            help_index = 0;
        }

        else if (command_handler == 8)  // > Kcw, Tcw, Kcz
        {
            float new_value = (float)command.toFloat();
            Serial.println(new_value, 4);
            if (help_index == 0)        // read Kcw
            {
                command_data.ref_wire_force = new_value; // ref_wire_force = Kcw
                Serial.print(F("Tcw (0.0314) -> "));
                command_handler = 8;
                help_index++;           // next Tcw
            }
            else if (help_index == 1)   // read Tcw
            {
                command_data.ref_position = new_value;  // ref_position = Tcw
                Serial.print(F("Kcz (0.0100) -> "));
                command_handler = 8;
                help_index++;           // next Kcz
            }
            else if (help_index == 2)   // read Kcz
            {
                command_data.ref_speed = new_value;     // ref_speed = Kcz
                Serial.println(F("ok_________"));
                command_handler = 0;
                help_index = 0;          // all set

                command_data.in_command = 11;  // cmd da sBot ucita nove parametre (1)
                flag_send_data = 1;
            }
        }
        else if (command_handler == 9)  // > K_sink, Rsr, wr_max
        {
            float new_value = (float)command.toFloat();
            Serial.println(new_value, 4);
            if (help_index == 0)        // read K_sink
            {
                command_data.ref_wire_force = new_value; // ref_wire_force = K_sink
                Serial.print(F("Rsr (40.0)   -> "));
                command_handler = 9;
                help_index++;           // next Rsr
            }
            else if (help_index == 1)   // read Rsr
            {
                command_data.ref_position = new_value;  // ref_position = Rsr
                Serial.print(F("wr_max (2.82)-> "));
                command_handler = 9;
                help_index++;           // next wr_max
            }
            else if (help_index == 2)   // read wr_max
            {
                command_data.ref_speed = new_value;     // ref_speed = wr_max
                Serial.println(F("ok_________"));
                command_handler = 0;
                help_index = 0;          // all set

                command_data.in_command = 12;  // cmd da sBot ucita nove parametre (2)
                flag_send_data = 1;
            }           
        }

        else if (command == "" || command == " ")    // STOP !!!
        {
            command_data.in_command = 1;
            flag_send_data = 1;
            Serial.println(F("Stopped!"));
            command_handler = 0;
            bot_get_data = 0;
            help_index = 0;
        }
        else if (command == "cmd")
        {
            Serial.print(F("Command = "));
            command_handler = 1;
        }
        else if (command == "bot")
        {
            Serial.print(F("For Sbot #"));
            command_handler = 2;
        }
        else if (command == "force")
        {
            Serial.print(F("Force = "));
            command_handler = 3;
        }
        else if (command == "speed")
        {
            Serial.print(F("Speed = "));
            command_handler = 4;
        }
        else if (command == "position")     // priredi testiranje sa regulacijom 
        {
            Serial.print(F("Position = "));
            command_handler = 5;
            help_index = 2;
        }
        else if (command == "pos")
        {
            Serial.print(F("Position = "));
            command_handler = 5;
        }
        else if (command == "voltage")
        {
            Serial.print(F("Voltage = "));
            command_handler = 6;
        }

        else if (command == "timer")        // priredi testiranje sa naponom
        {
            help_index = 1;
            Serial.println(F("Drive setup______"));
            Serial.print(F("Time:\t "));
            command_handler = 2;
        }
        else if (command == "par1")     // > Kcw, Tcw, Kcz
        {
            Serial.println(F("Parametri regulatora (1)"));
            Serial.print(F("Kcw (0.1783) -> "));
            command_handler = 8;
        }
        else if (command == "par2")     // > K_sink, Rsr, wr_max
        {
            Serial.println(F("Parametri regulatora (2)"));
            Serial.print(F("K_sink (0.0500) -> "));
            command_handler = 9;
        }
        else if (command == "send")             // POSALJI PORUKU
        {
            flag_send_data = 1;
            SEND_PRINT();
        }
        else if (command == "get")
        {
            bot_get_data = !bot_get_data;
            if (bot_get_data) Serial.println(F("Show ON"));
            else Serial.println(F("Show OFF"));
            new_data = 0;
        }
        else if (command == "try")
        {
            command_data.in_command = 2;
            flag_send_data = 1;
        }
        else if (command == "rst")
        {
            n_measurement = 0;
        }
        else if (command == "info") Serial.println(sketch_name);
        else if (command == "+")
        {
            Serial.println(F("Wire +"));
            command_data.in_command = 7;
            flag_send_data = 1;
        }
        else if (command == "-")
        {
            Serial.println(F("Wire -"));
            command_data.in_command = 8;
            flag_send_data = 1;
        }
        else if (command == "encset")
        {
            Serial.println(F("Setting encoders"));
            command_data.in_command = 9;
            flag_send_data = 1;
        }
        else if (command == "show")
        {
            Serial.print(F("Toggle bot ")); Serial.print(command_data.bot_number);
            Serial.println(F(" Serial.prints"));
            command_data.in_command = 13;
            flag_send_data = 1;
        }
    }

    if (flag_send_data)         // send command_data
    {
        radio.stopListening();
        radio.openWritingPipe(bot_reading_address);
        unsigned long send_timer = micros();
        //bool re_report = radio.write(&command_data, sizeof(command_data));
        //bool re_report = radio.write(&payload, sizeof(payload));
        bool re_report = radio.write(&command_data, sizeof(command_data));
        unsigned long end_send_timer = micros();
        //Serial.print(".");
        if (!bot_get_data)
        {
            if (re_report)
            {

                Serial.println("Sent "); Serial.print(sizeof(sensor_data1));
                Serial.println(" bytes"); Serial.print("Took me: ");
                Serial.print(end_send_timer - send_timer); Serial.println(" us.");
                if (command_data.in_command == 15 || command_data.in_command == 10)
                {
                    bot_get_data = 1;
                    reading_timer = millis();
                }
            }
            else
            {
                Serial.println(F("failed to send :c  "));
            }
        }

        flag_send_data = 0;
        radio.openReadingPipe(1, bot_master_address);
        radio.startListening();
    }

    uint8_t recieve_pipe;
    if(radio.available(&recieve_pipe))  // recieve sensor_data1
    {
        uint8_t recieve_bytes = radio.getPayloadSize(); // treba biti kao i comman_struct (2 byta)
        //radio.read(&command_data, recieve_bytes);
        radio.read(&sensor_data1, recieve_bytes);
        
        if (!bot_get_data)
        {
            Serial.print("Recieved "); Serial.print(recieve_bytes); Serial.print(" bytes");
            Serial.print(" on pipe "); Serial.print(recieve_pipe); Serial.println(": ");
            //Serial.println(payload);
            Serial.print(F("\t\tAbs: ")); Serial.println(sensor_data1.position_abs);
            Serial.print(F("\t\tenc_s: ")); Serial.println(sensor_data1.position_rel);
            Serial.print(F("tof#3: \t")); Serial.print(sensor_data1.tof3);
            //Serial.print(F("\tof_dx:\t")); Serial.println(sensor_data1.of_distance_x);
            Serial.print(F("\t\tof_dy:\t")); Serial.println(sensor_data1.of_distance_y);
            Serial.print(F("force:\t")); Serial.println(sensor_data1.wire_force);

            //Serial.print("sbot_slave :\t"); Serial.println(command_data.sbot_slave);
            //Serial.print("in_command :\t"); Serial.println(command_data.in_command);
        }
    
        new_data = 1;
    }

    GET_READINGSv2();

}

void GET_READINGS(void)
{
    int n_bots = 2;

    uint32_t get_time = millis();
    if (bot_get_data)
    {
        bool complete_data = 0;

        if ((get_time - bot_get_time) >= 1000/UPDATE_FREQ)
        {
            read_now = 1;            // start a new reading cycle
            bot_get_time = get_time; // log time of the last call
        }

        if (read_now)
        {
            if (new_data) // new_data = 1
            {
                AMT102_pozicije[bot_number - 1] = sensor_data1.position_rel;
                OF_dy_pozicije[bot_number -1] = sensor_data1.of_distance_y;
                AMT203_nagibi[bot_number - 1] = sensor_data1.position_abs;
                w_wheel[bot_number - 1] = sensor_data1.wire_force;
                u_motor[bot_number - 1] = sensor_data1.tof3;
                new_data = 0;

                bot_number = bot_number + 1;

                if(bot_number > n_bots)  // all 4 bots
                {
                    bot_number = 1;
                    read_now = 0;   // completed read clyce
                    complete_data = 1;
                }
                else
                {
                    flag_send_data = 1; // continue to next bot
                    command_data.bot_number = bot_number;
                }
                fail_counter = 0;
            }
            else // no new_data
            {
                command_data.bot_number = bot_number;
                command_data.in_command = 2;

                fail_counter = fail_counter + 1;    //
                if (fail_counter%5000 == 0)      // if multiple of 2000 passes and still no recieve
                {
                    flag_send_data = 1;         // try to send again
                }
                if (fail_counter > 10000)     // failed geting data from this bot
                {                           // move on to next bot
                    new_data = 1;       // proceed to next bot    
                }
                //new_data = 0;
            }

            if (new_data)   // artificial new data
            {
                AMT102_pozicije[bot_number - 1] = -0.1;
                OF_dy_pozicije[bot_number -1] = -0.1;
                AMT203_nagibi[bot_number - 1] = 0;
                w_wheel[bot_number - 1] = 0.0;
                u_motor[bot_number - 1] = -1;
                new_data = 0;

                bot_number = bot_number + 1;

                if(bot_number > n_bots)  // all 4 bots
                {
                    bot_number = 1;
                    read_now = 0;   // completed read clyce
                    complete_data = 1;
                }
                else
                {
                    flag_send_data = 1; // continue to next bot
                    command_data.bot_number = bot_number;
                }
                fail_counter = 0;                
            }
        }

        if (complete_data)
        {
            Serial.print(millis() - reading_timer); Serial.print("\t");
            n_measurement = n_measurement + 1;
            Serial.print(n_measurement); Serial.print("\t");
            for (int i=0; i<n_bots; i++)
            {
                Serial.print(i+1);
                Serial.print("\t"); Serial.print(AMT102_pozicije[i]);
                Serial.print("\t"); Serial.print(OF_dy_pozicije[i]);
                Serial.print("\t"); Serial.print(AMT203_nagibi[i]);
                Serial.print("\t"); Serial.print(w_wheel[i]);
                Serial.print("\t"); Serial.print(u_motor[i]);
                Serial.print("\t");
            }
            Serial.print("\n");
            complete_data = 0;
        }

    }
}

void GET_READINGSv2(void)
{
    int n_bots = 2;

    uint32_t get_time = millis();
    if (bot_get_data)
    {
        bool complete_data = 0;

        if ((get_time - bot_get_time) >= 500/UPDATE_FREQ)       // time to get readings
        {
            if (read_now == 1)      // was stil anticipating anwser
            {
                AMT102_pozicije[bot_number -1] = 0.0;   // place dummy readings
                OF_dy_pozicije[bot_number -1] = 0.0;
                AMT203_nagibi[bot_number - 1] = 0;
                w_wheel[bot_number - 1] = 0.0;
                u_motor[bot_number - 1] = 0;
            }

            read_now = 1;            // start a new reading cycle
            bot_get_time = get_time; // log time of the last call
            
            bot_number = bot_number + 1;    // new bot

            if (bot_number > n_bots)  // all 4 bots
            {
                bot_number = 1;
                complete_data = 1;
            }
            //Serial.println(bot_number);

            command_data.bot_number = bot_number;   // prepare interregation
            command_data.in_command = 2;
            flag_send_data = 1;                     // send request
            fail_counter = 0;                       // reset counter
        }

        if (read_now)
        {
            if (new_data)           // new_data = 1, from radio recieve
            {
                //Serial.println("read data");
                AMT102_pozicije[bot_number-1] = sensor_data1.position_rel;    //
                OF_dy_pozicije[bot_number-1] = sensor_data1.of_distance_y;
                AMT203_nagibi[bot_number-1] = sensor_data1.position_abs;
                w_wheel[bot_number-1] = sensor_data1.wire_force;
                u_motor[bot_number-1] = sensor_data1.tof3;
                new_data = 0;       // data has been read
                read_now = 0;       // stop anticipating
            }

            //else // no new_data
            //{
            //    //Serial.println("no new data");
            //    if (((get_time - bot_get_time) >= 100/UPDATE_FREQ)      // (120) (200)
            //        && ((get_time - bot_get_time) <= 120/UPDATE_FREQ)) // if waited 1/2 of cycle
            //    //if ((get_time - bot_get_time) <= 200/UPDATE_FREQ)
            //    {
            //        //Serial.print("trying again");
            //        flag_send_data = 1;          // try to send again
            //        command_data.bot_number = bot_number;
            //        command_data.in_command = 2;
            //    }
            //}

            //else // now new_data
            //{
            //    fail_counter = fail_counter + 1;    //
            //    //if (fail_counter%5000 == 0)      // if multiple of 2000 passes and still no recieve
            //    if (fail_counter%5000 == 0
            //        && fail_counter <= 10000)
            //    {
            //        flag_send_data = 1;         // try to send again
            //    }
            //    //if (fail_counter > 10000)     // failed geting data from this bot
            //    //{                           // move on to next bot
            //    //    new_data = 1;       // proceed to next bot    
            //    //}
            //}
        }

        if (complete_data)
        {
            //Serial.println("complete");
            Serial.print(millis() - reading_timer); Serial.print("\t");
            n_measurement = n_measurement + 1;
            Serial.print(n_measurement); Serial.print("\t");
            for (int i=0; i<n_bots; i++)
            {
                Serial.print(i+1);
                Serial.print("\t"); Serial.print(AMT102_pozicije[i]);
                Serial.print("\t"); Serial.print(OF_dy_pozicije[i]);
                Serial.print("\t"); Serial.print(AMT203_nagibi[i]);
                Serial.print("\t"); Serial.print(w_wheel[i]);
                Serial.print("\t"); Serial.print(u_motor[i]);
                Serial.print("\t");
            }
            Serial.print("\n");
            complete_data = 0;
        }

    }
}

void SEND_PRINT(void)
{
    Serial.println(F("Sending: "));
    Serial.print(F("Sbot: "));        Serial.print(command_data.bot_number);
    Serial.print(F("  Command: ")); Serial.println(command_data.in_command);
    Serial.print(F("Wheel: "));    Serial.print(command_data.wheel_voltage);
    Serial.print(F("  Wire: ")); Serial.println(command_data.ref_wire_force);
    Serial.print(F("R_pos: "));    Serial.print(command_data.ref_position);
    Serial.print(F("  R_speed: ")); Serial.println(command_data.ref_speed);
}