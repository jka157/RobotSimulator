#ifndef _STATE_ID_HEADER_
#define _STATE_ID_HEADER_

#define STATEID_MOD_ERROR           0x00000001L
#define STATEID_MOD_HOME            0x00000002L
#define STATEID_MOD_HALT            0x00000004L
#define STATEID_MOD_POWERFAULT      0x00000008L
#define STATEID_MOD_TOW_ERROR       0x00000010L
#define STATEID_MOD_COMM_ERROR      0x00000020L
#define STATEID_MOD_SWR             0x00000040L
#define STATEID_MOD_SW1             0x00000080L
#define STATEID_MOD_SW2             0x00000100L
#define STATEID_MOD_BRAKEACTIVE     0x00000200L
#define STATEID_MOD_CURLIMIT        0x00000400L
#define STATEID_MOD_MOTION          0x00000800L
#define STATEID_MOD_RAMP_ACC        0x00001000L
#define STATEID_MOD_RAMP_STEADY     0x00002000L
#define STATEID_MOD_RAMP_DEC        0x00004000L
#define STATEID_MOD_RAMP_END        0x00008000L
#define STATEID_MOD_INPROGRESS      0x00010000L
#define STATEID_MOD_FULLBUFFER      0x00020000L
#define STATEID_MOD_POW_VOLT_ERR    0x00040000L
#define STATEID_MOD_POW_FET_TEMP    0x00080000L
#define STATEID_MOD_POW_WDG_TEMP    0x00100000L
#define STATEID_MOD_POW_SHORTCUR    0x00200000L
#define STATEID_MOD_POW_HALLERR     0x00400000L
#define STATEID_MOD_POW_INTEGRALERR 0x00800000L
#define STATEID_MOD_CPU_OVERLOAD    0x01000000L
#define STATEID_MOD_BEYOND_HARD     0x02000000L
#define STATEID_MOD_BEYOND_SOFT     0x04000000L

/*
        ID                       Description
-----------------------------------------------------------------------------------------------------------
STATEID_MOD_ERROR             An error occured in the module (Generic Error flag).
STATEID_MOD_HOME              The module was homed (Info). 
STATEID_MOD_HALT              The module is in HALT state. Motion commands are not processed (Info).
STATEID_MOD_POWERFAULT        An error occured in the power bridge (Error).
STATEID_MOD_TOW_ERROR         The drive was unable to follow the interpolated position. (Error).
STATEID_MOD_COMM_ERROR        Problems in communication occured (Error).
STATEID_MOD_SWR               The Reference switch is active (Info).
STATEID_MOD_SW1               Limit switch 1 is active (Info).
STATEID_MOD_SW2               Limit switch 2 is active (Info).
STATEID_MOD_BRAKEACTIVE       The brake is active (Info).
STATEID_MOD_CURLIMIT          The PID loop output has reached the limit (Info).
STATEID_MOD_MOTION            The module is in motion (Info).
STATEID_MOD_RAMP_ACC          The module is in a phase of acceleration (Info). Only valid for ramp motion commands.
STATEID_MOD_RAMP_STEADY       The module is in a phase of steady speed (Info). Only valid for ramp motion commands.
STATEID_MOD_RAMP_DEC          The module is in a phase of deceleration (Info). Only valid for ramp motion commands.
STATEID_MOD_RAMP_END          The module has reached the target position (Info). Only valid for ramp motion commands.
STATEID_MOD_INPROGRESS        The module processes an actual Step motion command. (Info).
STATEID_MOD_FULLBUFFER        A Step motion command is waiting in the message buffer (Info).
STATEID_MOD_POW_VOLT_ERR      A voltage drop occured (Error).
STATEID_MOD_POW_FET_TEMP      A temperature problem occured in the power bridge (Error).
STATEID_MOD_POW_WDG_TEMP      A temperature problem occured in the motor (Error).
STATEID_MOD_POW_SHORTCUR      A short current occured (Error).
STATEID_MOD_POW_HALLERR       A communication error occured (Error).
STATEID_MOD_POW_INTEGRALERR   An error according to the integral criteria occured (current too high over time ¨C Error).
STATEID_MOD_CPU_OVERLOAD      The CPU is too busy (Error).
STATEID_MOD_BEYOND_HARD       The module has reached the hard limit switch (Error).
STATEID_MOD_BEYOND_SOFT       The module has reached the soft limit (Error).
*/

#endif
