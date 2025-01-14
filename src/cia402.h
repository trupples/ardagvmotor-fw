#ifndef CIA402_H_
#define CIA402_H_

enum cia402_state {
    CIA402_NOT_READY_TO_SWITCH_ON, // Doing initialization
    CIA402_SWITCH_ON_DISABLED, // Motor not electrically connected + control inactive
    CIA402_READY_TO_SWITCH_ON, 
    CIA402_SWITCHED_ON, // Motor electrically connected + control INACTIVE
    CIA402_OPERATION_ENABLED, // Motor control ACTIVE
    CIA402_QUICK_STOP_ACTIVE, // Quick stop in progress
    CIA402_FAULT_REACTION_ACTIVE, // ???
    CIA402_FAULT, // ???
};

#define CIA402_BIT_READYTOSWITCHON (1 << 0)
#define CIA402_BIT_SWITCHON (1 << 1)
#define CIA402_BIT_OPERATIONEN (1 << 2)
#define CIA402_BIT_FAULT (1 << 3)
#define CIA402_BIT_VOLTAGEEN (1 << 4)
#define CIA402_BIT_QUICKSTOP (1 << 5)
#define CIA402_BIT_SWITCHONDISABLED (1 << 6)
#define CIA402_BIT_WARNING (1 << 7)
#define CIA402_BIT_REMOTE (1 << 9)
#define CIA402_BIT_TARGET (1 << 10)
#define CIA402_BIT_LIMIT (1 << 11)

#define CIA402_BIT_PV_SPEED (1 << 11)
#define CIA402_BIT_PV_MAXSLIP (1 << 11)

struct cia402 {
    struct canopen *co;
    enum cia402_state state;
};

#endif
