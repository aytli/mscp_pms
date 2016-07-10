#include "main.h"
#include "can_telem.h"
#include "can18F4580_mscp.c"

#define SENDING_PERIOD_MS      200
#define PRECHARGE_DURATION_MS 2000 // CANNOT PRECHARGE FOR MORE THAN 7 SECONDS
#define HORN_DURATION_MS       500
#define DEBOUNCE_PERIOD_MS     100

// BMS temperature limits
#define BPS_TEMP_WARNING       60 // 60°C charge limit
#define BPS_TEMP_CRITICAL      70 // 70°C discharge limit

// CAN bus defines
#define TX_PRI 3
#define TX_EXT 0
#define TX_RTR 0

// Only turn on the array if the bps did not trip
#define ARRAY_ON                   \
    if (gb_bps_trip == false)      \
    {                              \
        gb_array_connected = true; \
        output_high(MPPT_PIN);     \
    }

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
static int1        gb_bps_trip;
static int1        gb_battery_temperature_safe;
static int8        g_pms_data_page[CAN_PMS_DATA_LEN];

void pms_init(void)
{
    gb_motor_connected          = false;
    gb_array_connected          = false;
    gb_bps_trip                 = false;
    gb_battery_temperature_safe = true;
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
            return 0;
        }
    }
    return 1;
}

void update_pms_data(void)
{
    g_pms_data_page[0] = 1; // Aux cell 1 voltage
    g_pms_data_page[1] = 2; // Aux cell 2 voltage
    g_pms_data_page[2] = 3; // Aux cell 3 voltage
    g_pms_data_page[3] = 4; // Aux cell 4 voltage
    g_pms_data_page[4] = 5; // DC/Dc converter temperature
    g_pms_data_page[5] = gb_array_connected; // Motor state
    g_pms_data_page[6] = gb_motor_connected; // Array state
    g_pms_data_page[7] = 8; // Unused?
}

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
    static int8 ms = 0;
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
    int16 i;
    
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
            can_putd(COMMAND_PMS_DISCONNECT_ARRAY_ID,0,0,TX_PRI,TX_EXT,TX_RTR);
            gb_bps_trip = true; // The array disconnect command is interpreted as a BPS trip
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
    gb_send = false; // Reset sending flag
    can_putd(CAN_PMS_DATA_ID,g_pms_data_page,CAN_PMS_DATA_LEN,TX_PRI,TX_EXT,TX_RTR);
    
    // Return to idle state
    g_state = IDLE;
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
            default:
                break;
        }
    }
}
