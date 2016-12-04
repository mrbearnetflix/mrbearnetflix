#include "IRTransmitter/IRTransmitter.h"

// Vizio TV pwr on + netflix
static unsigned int vizio_data[] =      {9000, 4450, 600, 550, 600, 550, 550, 1700, 550, 550, 
                                        600, 550, 550, 550, 600, 550, 550, 550, 550, 1700, 
                                        550, 1700, 550, 550, 600, 1650, 600, 1650, 600, 1650, 
                                        600, 1650, 600, 1650, 600, 1650, 600, 1650, 600 ,550, 
                                        550, 1700, 550, 550, 600, 1650, 550, 1650, 600, 1650, 
                                        600, 550, 600, 550, 600, 1650, 550, 550, 600, 1650, 
                                        600, 550, 600, 500, 600, 500, 650};  // NEC 20DFD728
    
#define SQUEEZE_THRESHOLD 100
#define BOOST_EN    D3
#define HEART_PIN   D2
#define IR_PIN2     D1
#define IR_PIN1     D0

#define SENSOR2     A0
#define SENSOR1     A1

#define COUNTER_PERIOD 20
#define LED_LUT_MAX 180
uint8_t led_lut_idx = 0;
static const uint8_t LED_LUT[] = {15,17,20,22,25,28,30,33,36,38,41,43,46,48,51,54,56,59,61,64,66,69,71,73,76,78,81,83,85,88,90,92,94,97,99,101,103,105,107,109,111,113,115,117,119,121,123,125,127,128,130,132,133,135,136,138,139,141,142,144,145,146,148,149,150,151,152,153,154,155,156,157,158,159,159,160,161,161,162,162,163,163,164,164,164,165,165,165,165,165,165,165,165,165,165,165,164,164,164,163,163,162,162,161,161,160,159,159,158,157,156,155,154,153,152,151,150,149,148,146,145,144,142,141,139,138,136,135,133,132,130,128,127,125,123,121,119,117,115,113,111,109,107,105,103,101,99,97,94,92,90,88,85,83,81,78,76,73,71,69,66,64,61,59,56,54,51,48,46,43,41,38,36,33,30,28,25,22,20,17,15};
#define LED_FADE_LUT_MAX 90
uint8_t led_fade_lut_idx = 0;
static const uint8_t LED_FADE_LUT[] = {255,254,254,254,254,254,253,253,252,251,251,250,249,248,247,246,245,243,242,241,239,238,236,234,232,231,229,227,225,223,220,218,216,213,211,208,206,203,200,198,195,192,189,186,183,180,177,173,170,167,163,160,156,153,149,146,142,138,135,131,127,123,119,115,111,107,103,99,95,91,87,83,78,74,70,65,61,57,53,48,44,39,35,31,26,22,17,13,8,4,0};
uint16_t timeout_counter=0;

#define COUNTS_PER_SECOND 1000/COUNTER_PERIOD
#define N_BLINKS 3
#define N_DELAY 3
uint8_t blink_counter=0;
uint16_t delay_counter=0;

#define WAITING 0
#define REMOTE_HUG 1
#define LOCAL_HUG 2
#define CONNECTED 3
uint8_t bear_state = WAITING;

Timer LEDTimer(COUNTER_PERIOD, ctrlLED);

uint8_t published = false;
uint16_t readings[8] = {0};
int8_t head_pos = 0;
uint16_t avg = 0;
int32_t sum = 0;
uint8_t i;

FuelGauge battery;
float battery_level;

IRTransmitter transmitter_front(IR_PIN1, D7);
IRTransmitter transmitter_rear(IR_PIN2, D7);
    
void setup() 
{
    
    Particle.function("initiatehug", RemoteInitiateHugHandler);
    Particle.function("completehug", RemoteCompleteHugHandler);
    Particle.variable("BattStat", battery_level);
    
    Serial.begin(9600);
    delay(1000);

    //Setup the GPIO for the LEDs
    digitalWrite(D7,LOW);
    digitalWrite(BOOST_EN,LOW);
    digitalWrite(HEART_PIN,LOW);
    digitalWrite(IR_PIN1,LOW);
    digitalWrite(IR_PIN2,LOW);
    
    pinMode(D7, OUTPUT);
    pinMode(BOOST_EN, OUTPUT);
    pinMode(HEART_PIN, OUTPUT);
    pinMode(IR_PIN1, OUTPUT);
    pinMode(IR_PIN2, OUTPUT);    

    //Setup the GPIO for the hug sensor
    pinMode(SENSOR1, INPUT);
    
    //Accumulate a buffer of readings
    for(i=0; i<8; i++)
    {
        readings[i] = analogRead(SENSOR1);
        sum += readings[i];
        delay(100);
    }
    avg = sum >> 3;
    head_pos = 0;
    
    LEDTimer.start();
}

void loop() 
{
    //Read the battery percentage
    battery_level = battery.getSoC();
    
    int8_t tail_pos;
    uint16_t val;
    
    //Read the input pin
    val = analogRead(SENSOR1);
    
    //Remove the value at the tail of the buffer (about to be the head of the buffer)
    sum -= readings[head_pos];
    
    //Store the most recent reading at the head
    readings[head_pos] = val;
    
    //Update the average
    sum += readings[head_pos];
    avg = sum >> 3;
    
    //Update the head position
    head_pos++;
    head_pos = head_pos & 0b0111;
    
    if(val > avg)
    {
        uint16_t diff = val - avg;
        
        if(diff > SQUEEZE_THRESHOLD)
        {
            if(published == false)
            {
                 switch(bear_state)
                {
                    case(WAITING):
                        //We're just waiting so try to initiate a new sequence
                        Particle.publish("initiatehug","0",60,PRIVATE);
                        published = true;
                        Serial.print("Squeezed!");
                        LocalHugHandler();
                        transmitter_front.Transmit(vizio_data, sizeof(vizio_data) / sizeof(vizio_data[0]));
                        transmitter_rear.Transmit(vizio_data, sizeof(vizio_data) / sizeof(vizio_data[0]));
                    break;
                    case(REMOTE_HUG):
                        //If we've already detected a remote hug send a hug to complete the sequence
                        Particle.publish("completehug","0",60,PRIVATE);
                        published = true;
                        Serial.print("Squeezed!");
                        LocalHugHandler();
                    break;
                default:
                    //Do Nothing
                break;
                }
            }
        }
    }
    else
    {
        published = false;
    }
    
    //TODO - Go to sleep here instead of using a delay
    delay(500);
}


int RemoteInitiateHugHandler(String command)
{
    switch(bear_state)
    {
        case(WAITING):
            bear_state = REMOTE_HUG;
            Serial.print("REMOTE HUG!");
        break;
        case(LOCAL_HUG):
            //Bears are out of sync immediately respond with a complete hug
            Particle.publish("completehug","0",60,PRIVATE);
            published = true;
            bear_state = CONNECTED;
            Serial.print("CONNECTED!");
        break;
        case(REMOTE_HUG):
            //Do Nothing
        break;
        case(CONNECTED):
            //Do Nothing
        break;
        default:
            //Do Nothing
        break;
    }
    return(0);
}



int RemoteCompleteHugHandler(String command)
{
    switch(bear_state)
    {
        case(WAITING):
            //Do Nothing
        break;
        case(LOCAL_HUG):
            bear_state = CONNECTED;
            //Turn on the TV
            transmitter_front.Transmit(vizio_data, sizeof(vizio_data) / sizeof(vizio_data[0]));
            transmitter_rear.Transmit(vizio_data, sizeof(vizio_data) / sizeof(vizio_data[0]));
            Serial.print("CONNECTED!");
        break;
        case(REMOTE_HUG):
            //Do Nothing
        break;
        case(CONNECTED):
            //Do Nothing
        break;
        default:
            //Do Nothing
        break;
    }
    return(0);
}

void LocalHugHandler(void)
{
    switch(bear_state)
    {
        case(WAITING):
            bear_state = LOCAL_HUG;
            Serial.print("LOCAL HUG!");
        break;
        case(LOCAL_HUG):
            //Do Nothing
        break;
        case(REMOTE_HUG):
            bear_state = CONNECTED;
            //Turn on the TV
            transmitter_front.Transmit(vizio_data, sizeof(vizio_data) / sizeof(vizio_data[0]));
            transmitter_rear.Transmit(vizio_data, sizeof(vizio_data) / sizeof(vizio_data[0]));
            Serial.print("CONNECTED!");
        break;
        case(CONNECTED):
            //Do Nothing
        break;
        default:
            //Do Nothing
        break;
    }
}

void ctrlLED(void)
{
    switch(bear_state)
    {
        case(WAITING):
            digitalWrite(HEART_PIN, LOW);
        break;
        case(LOCAL_HUG):
        case(REMOTE_HUG):
            if(led_lut_idx < LED_LUT_MAX)
            {
                //Blink the LED
                analogWrite(HEART_PIN, LED_LUT[led_lut_idx]);
                led_lut_idx++;
            }
            else
            {
                led_lut_idx = 0;
            }
            
            //Only stay in the hugged state for 60 seconds
            if(timeout_counter++ > COUNTS_PER_SECOND*60)
            {
                bear_state = WAITING;
                timeout_counter = 0;
                led_lut_idx = 0;
            }
        break;
        case(CONNECTED):
            timeout_counter = 0;
            //On for N seconds
            if(blink_counter == 0)
            {
                digitalWrite(HEART_PIN, HIGH);
                //The first blink is N seconds long
                if(delay_counter < COUNTS_PER_SECOND*N_DELAY)
                {
                    delay_counter++;
                }
                else
                {
                    digitalWrite(HEART_PIN, LOW);
                    delay_counter = 0;
                    blink_counter++;
                }
            }
            //Else blink N Times
            else if(blink_counter < (N_BLINKS*2))
            {
                if(delay_counter < (COUNTS_PER_SECOND/2))
                {
                    delay_counter++;
                }
                else
                {
                    //Toggle LED on
                    digitalWrite(HEART_PIN, !digitalRead(HEART_PIN));
                    delay_counter = 0;
                    blink_counter++;
                }
            }
            //Fade out and switch back to a Waiting state
            else
            {
                if(led_fade_lut_idx < LED_FADE_LUT_MAX)
                {
                    //Blink the LED
                    analogWrite(HEART_PIN, LED_FADE_LUT[led_fade_lut_idx]);
                    led_fade_lut_idx++;
                }
                else
                {
                    //We're finished
                    led_fade_lut_idx = 0;
                    blink_counter = 0;
                    delay_counter = 0;
                    bear_state = WAITING;
                }
            }
        break;
        default:
            //Do Nothing
        break;
    }
}

