#include "main.h"
#include "can_telem.h"
#include "can18F4580_mscp.c"

#define HEARTBEAT_PERIOD_MS 200

#define BPS_TEMP_WARNING  60 // 60°C charge limit
#define BPS_TEMP_CRITICAL 70 // 70°C discharge limit

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

static int1        gb_array_connected;

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

// INT_TIMER2 programmed to trigger every 1ms with a 20MHz clock
// This interrupt will toggle the heartbeat LED
#int_timer2
void isr_timer2(void)
{
    static int8 ms = 0;
    if (ms >= HEARTBEAT_PERIOD_MS)
    {
        ms = 0;         // Reset timer
        output_toggle(STATUS_LED);
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
    else if (gb_send == true)
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
            break;
        case CAN_BPS_TEMPERATURE1_ID:
        case CAN_BPS_TEMPERATURE2_ID:
        case CAN_BPS_TEMPERATURE3_ID:
            if (check_bps_temperature(g_rx_data,g_rx_len) == 0)
            {
                // One of the battery temperatures is above the warning threshold
                // Turn off the array
                ARRAY_OFF;
            }
            else if (gb_array_connected == false)
            {
                // The battery temperatures are all below the warning threshold
                // Turn on the array if it is off
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
}

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
    
    can_init();
    
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
