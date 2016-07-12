#include <18F26K80.h>
#device adc=8

#FUSES NOWDT                    //No Watch Dog Timer
#FUSES SOSC_DIG                 //Digital mode, I/O port functionality of RC0 and RC1
#FUSES NOXINST                  //Extended set extension and Indexed Addressing mode disabled (Legacy mode)
#FUSES HSH                      //High speed Osc, high power 16MHz-25MHz
#FUSES NOPLLEN                  //4X HW PLL disabled, 4X PLL enabled in software
#FUSES BROWNOUT
#FUSES PUT
#FUSES NOIESO
#FUSES NOFCMEN
#FUSES NOPROTECT
#FUSES CANC                     //Enable to move CAN pins to C6(TX) and C7(RX)

#use delay(clock = 20000000)

// SWITCHES
#define MPPT_SWITCH   PIN_B4
#define MOTOR_SWITCH  PIN_B5

// OUTPUTS
#define STATUS_LED    PIN_A5
#define HORN_PIN      PIN_B3
#define AUX_READ_PIN  PIN_C0
#define PRECHARGE_PIN PIN_C1
#define MOTOR_PIN     PIN_C2
#define MPPT_PIN      PIN_C3

// ANALOG PINS
#define AUX1_PIN      PIN_A0
#define AUX2_PIN      PIN_A1
#define AUX3_PIN      PIN_A2
#define AUX4_PIN      PIN_A3
#define DCDC_TEMP_PIN PIN_B0

// Aux pack ADC channels and pins
#define AUX1_ADC_CHANNEL          0
#define AUX2_ADC_CHANNEL          1
#define AUX3_ADC_CHANNEL          2
#define AUX4_ADC_CHANNEL          3
#define AUX1_ANALOG_PIN        sAN0
#define AUX2_ANALOG_PIN        sAN1
#define AUX3_ANALOG_PIN        sAN2
#define AUX4_ANALOG_PIN        sAN3

// DC/DC temperature ADC channel and pin
#define DCDC_TEMP_ADC_CHANNEL    10
#define DCDC_TEMP_ANALOG_PIN  sAN10

// State machine states
typedef enum
{
    IDLE,
    CHECK_SWITCHES,
    DATA_RECEIVED,
    DATA_SENDING,
    BPS_TRIP,
    N_STATES
} pms_state_t;
