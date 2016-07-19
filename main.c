// Power management system code
// Author: Andy Li (liy92@mcmaster.ca)
// Copyright 2016, McMaster Solar Car Project
// Controls the motor and MPPT relays, precharge, and the horn, reads the
// voltage of the aux pack and the temperature of the DC/DC converter and sends
// it over CAN bus

// Includes
#include "main.h"
#include "can_telem.h"
#include "can18F4580_mscp.c"

// Timing periods
#define SENDING_PERIOD_MS     1000 // Telemetry data is sent over CAN bus at this period
#define PRECHARGE_DURATION_MS 2000 // CANNOT PRECHARGE FOR MORE THAN 7 SECONDS
#define HORN_DURATION_MS       500 // Duration of the horn honk
#define DEBOUNCE_PERIOD_MS      10 // Hardware switch debounce period

// BMS temperature limits
#define BPS_TEMP_WARNING       60 // 60°C charge limit
#define BPS_TEMP_CRITICAL      70 // 70°C discharge limit

// Miscellaneous defines
#define N_AUX_CELLS 4

// CAN bus defines
#define TX_PRI 3
#define TX_EXT 0
#define TX_RTR 0

#define ARRAY_ON                \
    gb_array_connected = true;  \
    output_high(MPPT_PIN);

#define ARRAY_OFF               \
    gb_array_connected = false; \
    output_low(MPPT_PIN);

#define MOTOR_ON                \
    gb_motor_connected = true;  \
    output_high(MOTOR_PIN);

#define MOTOR_OFF               \
    gb_motor_connected = false; \
    output_low(MOTOR_PIN);

// Debounces a hardware pin
#define DEBOUNCE                               \
    int16 i;                                   \
    for (i = 0 ; i < DEBOUNCE_PERIOD_MS ; i++) \
    {                                          \
        delay_ms(1);                           \
    }

static int1        gb_send;
static int32       g_can0_id;
static int8        g_can0_data[8];
static int8        g_can0_len;
static int1        gb_can0_hit = false;
static int32       g_can1_id;
static int8        g_can1_data[8];
static int8        g_can1_len;
static int1        gb_can1_hit = false;
static int32       g_rx_id;
static int8        g_rx_len;
static int8        g_rx_data[8];
static pms_state_t g_state;
static int1        gb_motor_connected;
static int1        gb_array_connected;
static int1        gb_brake_pressed;
static int1        gb_battery_temperature_safe;
static int8        g_aux_pack_voltage[N_AUX_CELLS];
static int8        g_pms_data_page[CAN_PMS_DATA_LEN];

void pms_init(void)
{
    gb_motor_connected          = false;
    gb_array_connected          = false;
    gb_brake_pressed            = false;
    gb_battery_temperature_safe = true;
    
    // Set up the ADC channels
    setup_adc(ADC_CLOCK_INTERNAL);
    setup_adc_ports(AUX1_ANALOG_PIN | AUX2_ANALOG_PIN | AUX3_ANALOG_PIN | AUX4_ANALOG_PIN |
                    DCDC_TEMP_ANALOG_PIN);
}

// Accepts a packet of BPS temperature data and the length of the packet
// Checks if each point of data is below the temperature warning threshold
int1 check_bps_temperature(int * data, int length)
{
    int i;
    
    for (i = 0 ; i < length ; i++)
    {
        if (data[i] >= BPS_TEMP_WARNING)
        {
            // Temperature is above the warning threshold, return false
            return 0;
        }
    }
    
    // The temperatures are below the warning threshold, return true
    return 1;
}

void read_aux_voltages(void)
{
    // Connect the aux pack cell terminals to the ADCs
    output_high(AUX_READ_PIN);
    delay_us(10);
    
    // Cell 1
    set_adc_channel(AUX1_ADC_CHANNEL);
    g_aux_pack_voltage[0] = read_adc();
    delay_us(10);
    
    // Cell 2
    set_adc_channel(AUX2_ADC_CHANNEL);
    g_aux_pack_voltage[1] = read_adc();
    delay_us(10);
    
    // Cell 3
    set_adc_channel(AUX3_ADC_CHANNEL);
    g_aux_pack_voltage[2] = read_adc();
    delay_us(10);
    
    // Cell 4
    set_adc_channel(AUX4_ADC_CHANNEL);
    g_aux_pack_voltage[3] = read_adc();
    delay_us(10);
    
    // Disconnect the aux pack to avoid draining current
    output_low(AUX_READ_PIN);
}

int8 read_dcdc_temp(void)
{
    int8 temp;
    set_adc_channel(DCDC_TEMP_ADC_CHANNEL);
    temp = read_adc();
    delay_us(10);
    return temp;
}

void update_pms_data(void)
{
    // PMS CAN bus heartbeat signal
    static int1 b_can_heartbeat = 0;
    
    read_aux_voltages(); // Read the aux voltages
    g_pms_data_page[0] = g_aux_pack_voltage[0]; // Aux cell 1 voltage
    g_pms_data_page[1] = g_aux_pack_voltage[1]; // Aux cell 2 voltage
    g_pms_data_page[2] = g_aux_pack_voltage[2]; // Aux cell 3 voltage
    g_pms_data_page[3] = g_aux_pack_voltage[3]; // Aux cell 4 voltage
    g_pms_data_page[4] = read_dcdc_temp();      // DC/DC converter temperature
    g_pms_data_page[5] = gb_array_connected;    // Motor state
    g_pms_data_page[6] = gb_motor_connected;    // Array state
    g_pms_data_page[7] = b_can_heartbeat;       // CAN bus heartbeat
    
    b_can_heartbeat = !b_can_heartbeat;
}

// Honks the horn for a predefined duration
void honk(void)
{
    int16 i;
    output_high(HORN_PIN);
    for (i = 0 ; i < HORN_DURATION_MS ; i++)
    {
        delay_ms(1);
    }
    output_low(HORN_PIN);
}

// INT_TIMER2 programmed to trigger every 1ms with a 20MHz clock
// This interrupt will send out telemetry data for the aux pack and the dcdc converter
// This interrupt will also toggle the status LED
#int_timer2
void isr_timer2(void)
{
    static int16 ms = 0;
    if (ms >= SENDING_PERIOD_MS)
    {
        ms = 0;                    // Reset timer
        output_toggle(STATUS_LED); // Toggle the status LED
        gb_send = true;
    }
    else
    {
        ms++;
    }
}

// CAN receive buffer 0 interrupt
#int_canrx0
void isr_canrx0()
{
    struct rx_stat rxstat;
    if (can_getd(g_can0_id, g_can0_data, g_can0_len, rxstat))
    {
        gb_can0_hit = true;
    }
    else
    {
        gb_can0_hit = false;
    }
}

// CAN receive buffer 1 interrupt
#int_canrx1
void isr_canrx1()
{
    struct rx_stat rxstat;
    if (can_getd(g_can1_id, g_can1_data, g_can1_len, rxstat))
    {
        gb_can1_hit = true;
    }
    else
    {
        gb_can1_hit = false;
    }
}

void idle_state(void)
{
    if (gb_can0_hit == true)
    {
        // Data received in buffer 0, transfer contents
        g_rx_id = g_can0_id;
        g_rx_len = g_can0_len;
        memcpy(g_rx_data,g_can0_data,8);
        gb_can0_hit = false;
        g_state = DATA_RECEIVED;
    }
    else if (gb_can1_hit == true)
    {
        // Data received in buffer 1, transfer contents
        g_rx_id = g_can1_id;
        g_rx_len = g_can1_len;
        memcpy(g_rx_data,g_can1_data,8);
        gb_can1_hit = false;
        g_state = DATA_RECEIVED;
    }
    else if (can_tbe() && (gb_send == true))
    {
        // Ready to send data
        g_state = DATA_SENDING;
    }
    else
    {
        // Nothing, proceed to check switches
        g_state = CHECK_SWITCHES;
    }
}

void check_switches_state(void)
{
    // Check the array switch
    if ((input_state(MPPT_SWITCH) == 1) && (gb_array_connected == false) && (gb_battery_temperature_safe == true))
    {
        DEBOUNCE;
        if (input_state(MPPT_SWITCH) == 1)
        {
            // If the switch was turned on and the battery temperature is safe, turn on the array
            ARRAY_ON;
        }
    }
    else if ((input_state(MPPT_SWITCH) == 0) && (gb_array_connected == true))
    {
        DEBOUNCE;
        if (input_state(MPPT_SWITCH) == 0)
        {
            // If the switch was turned off, turn off the array
            ARRAY_OFF;
        }
    }
    
    // Check the motor switch
    if ((input_state(MOTOR_SWITCH) == 1) && (gb_motor_connected == false))
    {
        DEBOUNCE;
        if (input_state(MOTOR_SWITCH) == 1)
        {
            // If the switch was turned on, precharge the motor and turn it on
            output_high(PRECHARGE_PIN);
            for (i = 0 ; i < PRECHARGE_DURATION_MS ; i++)
            {
                delay_ms(1);
            }
            MOTOR_ON;
            delay_ms(10);
            output_low(PRECHARGE_PIN);
        }
    }
    else if ((input_state(MOTOR_SWITCH) == 0) && (gb_motor_connected == true))
    {
        DEBOUNCE;
        if (input_state(MOTOR_SWITCH) == 0)
        {
            // If the switch was turned off, turn off the motor
            MOTOR_OFF;
        }
    }
    
    // Check the brake lights
    if ((input_state(BRAKE_SWITCH) == 1) && (gb_brake_pressed == false))
    {
        DEBOUNCE;
        if (input_state(BRAKE_SWITCH) == 1)
        {
            // If the brake was pressed, signal the blinker to turn on the brake lights
            can_putd(COMMAND_PMS_BRAKE_LIGHT_ID,0,0,TX_PRI,TX_EXT,TX_RTR);
            gb_brake_pressed = true;
        }
    }
    else if ((input_state(BRAKE_SWITCH) == 0) && (gb_brake_pressed == true))
    {
        DEBOUNCE;
        if (input_state(BRAKE_SWITCH) == 0)
        {
            // If the brake was released, signal the blinker to turn off the brake lights
            can_putd(COMMAND_PMS_BRAKE_LIGHT_ID,0,0,TX_PRI,TX_EXT,TX_RTR);
            gb_brake_pressed = false;
        }
    }
    
    // Return to idle state
    g_state = IDLE;
}

void data_received_state(void)
{
    switch(g_rx_id)
    {
        case COMMAND_PMS_DISCONNECT_ARRAY_ID:
            // Received a command to disconnect the array
            // Turn off the array and send a response
            ARRAY_OFF;
            can_putd(RESPONSE_PMS_DISCONNECT_ARRAY_ID,0,0,TX_PRI,TX_EXT,TX_RTR);
            g_state = BPS_TRIP;
            return; // Break out of this state early, fall into the bps trip state
            break;
        case COMMAND_PMS_ENABLE_HORN_ID:
            // Received a command to honk the horn
            honk();
            break;
        case CAN_BPS_TEMPERATURE1_ID:
        case CAN_BPS_TEMPERATURE2_ID:
        case CAN_BPS_TEMPERATURE3_ID:
            if (check_bps_temperature(g_rx_data,g_rx_len) == 0)
            {
                // One of the battery temperatures is above the warning threshold
                // Turn off the array
                gb_battery_temperature_safe = false;
                ARRAY_OFF;
            }
            else if ((gb_array_connected == false) && (input_state(MPPT_SWITCH) == 1))
            {
                // The battery temperatures are all below the warning threshold
                // Turn on the array if it is off and the switch is pressed
                gb_battery_temperature_safe = true;
                ARRAY_ON;
            }
            break;
        default:
            break;
    }
    
    // Return to idle state
    g_state = IDLE;
}

void data_sending_state(void)
{
    // Sends a packet of telemetry data
    update_pms_data();
    can_putd(CAN_PMS_DATA_ID,g_pms_data_page,CAN_PMS_DATA_LEN,TX_PRI,TX_EXT,TX_RTR);
    gb_send = false; // Reset sending flag
    
    // Return to idle state
    g_state = IDLE;
}

void bps_trip_state(void)
{
    // The PMS will assume a bps trip when it receives a CAN command to disconnect the array
    // The state machine will never exit this state if it falls in, the PMS will need to be reset
    g_state = BPS_TRIP;
}

// Main
void main()
{
    // Enable CAN receive interrupts
    clear_interrupt(INT_CANRX0);
    enable_interrupts(INT_CANRX0);
    clear_interrupt(INT_CANRX1);
    enable_interrupts(INT_CANRX1);
    
    // Setup timer interrupts
    setup_timer_2(T2_DIV_BY_4,79,16); // Timer 2 set up to interrupt every 1ms with a 20MHz clock
    enable_interrupts(INT_TIMER2);
    enable_interrupts(GLOBAL);
    
    pms_init();
    can_init();
    
    // Start in idle state
    g_state = IDLE;
    
    while(true)
    {
        switch(g_state)
        {
            case IDLE:
                idle_state();
                break;
            case CHECK_SWITCHES:
                check_switches_state();
                break;
            case DATA_RECEIVED:
                data_received_state();
                break;
            case DATA_SENDING:
                data_sending_state();
                break;
            case BPS_TRIP:
                bps_trip_state();
                break;
            default:
                break;
        }
    }
}
