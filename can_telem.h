#ifndef CAN_TELEM_H
#define CAN_TELEM_H

// NOTE: The following tables are x-macros
// X macro tutorial: http://www.embedded.com/design/programming-languages-and-tools/4403953/C-language-coding-errors-with-X-macros-Part-1

//////////////////////////
// CAN BUS DEFINES ///////
//////////////////////////

#define EXPAND_AS_CAN_ID_ENUM(a,b,c)  a##_ID  = b,
#define EXPAND_AS_CAN_LEN_ENUM(a,b,c) a##_LEN = c,
#define EXPAND_AS_CAN_ID_ARRAY(a,b,c)           b,

// X macro table of CANbus packets
//        Packet name            ,    ID, Length
#define CAN_ID_TABLE(ENTRY)                   \
    ENTRY(CAN_BPS_TEMPERATURE1   , 0x608,  8) \
    ENTRY(CAN_BPS_TEMPERATURE2   , 0x609,  8) \
    ENTRY(CAN_BPS_TEMPERATURE3   , 0x60A,  8) \
    ENTRY(CAN_PMS_DATA           , 0x60E,  8)
#define N_CAN_ID 3

enum {CAN_ID_TABLE(EXPAND_AS_CAN_ID_ENUM)};
enum {CAN_ID_TABLE(EXPAND_AS_CAN_LEN_ENUM)};


//////////////////////////////
// CAN COMMAND DEFINES ///////
//////////////////////////////

#define EXPAND_AS_MISC_ID_ENUM(a,b)  a##_ID  = b,

// X macro table of miscellaneous CANbus packets
//        Packet name                  ,    ID
#define CAN_MISC_TABLE(ENTRY)                   \
    ENTRY(COMMAND_PMS_DISCONNECT_ARRAY , 0x777) \
    ENTRY(COMMAND_PMS_ENABLE_HORN      , 0x780)
#define N_CAN_COMMAND 2

enum {CAN_MISC_TABLE(EXPAND_AS_MISC_ID_ENUM)};


#endif
