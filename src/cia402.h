#ifndef CIA402_H_
#define CIA402_H_

#include <zephyr/smf.h>

enum cia402_state {
    CIA402_NOT_READY_TO_SWITCH_ON, // Low power, disabled drive, brake enabled if exists. Run self-test + init
    CIA402_SWITCH_ON_DISABLED,     // Idle powered-down state
    CIA402_READY_TO_SWITCH_ON,     // Intermediary state during motor power-up. Maybe high power (depends on application), drive disabled
    CIA402_SWITCHED_ON,            // Intermediary state during motor power-up. High power, drive disabled
    CIA402_OPERATION_ENABLED,      // Motor powered on, drive enabled
    CIA402_QUICK_STOP_ACTIVE,      // Quick stop active
    CIA402_FAULT_REACTION_ACTIVE,  // Stopping because of a fault condition
    CIA402_FAULT                   // Final fault state. Needs explicit action to return to the rest of the state machine
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
    struct smf_ctx smf_ctx;
    struct canopen *co;

    /* Application-specific callbacks 
     *
     * FIXME: Is this the sanest way around this? I would like cia402 to have
     *        the state machine logic, but as few side effects (communication
     *        with the motor driver) as possible.
     */

    // Callback for initializing motor drive and performing self-test.
    void (*cb_initialize)(struct cia402 *);

    // Callback for initiating motor enable
    void (*cb_start)(struct cia402 *);

    // Callback for checking if the motor has started and can be controlled. Only called after a previous cb_start call.
    bool (*cb_is_started)(struct cia402 *);

    // Callback for powering off the motor (ordinary stop, quick stop, fault reaction). This may initiate a slow down sequence, but should finally result in a "limp" drive.
    void (*cb_stop)(struct cia402 *);

    // Callback for checking if motor has been stopped. Only called after a previous cb_stop call.
    bool (*cb_is_stopped)(struct cia402 *);

    // Callback for cleanup in case of unrecoverable errors. May call k_oops() at the end.
    void (*cb_oops)(struct cia402 *);

    // CANopenNode requires extensions for event-based TPDO requests
    OD_extension_t statusword_extension;
    OD_extension_t mod_extension;
    OD_extension_t vel_extension;
    OD_extension_t pos_extension;

    bool _unrecoverable_error_flag;
};

// Initialize cia402 struct, OD extensions, state machine
void cia402_init(struct cia402* cia402);

// Enter fault and trigger application specific motor stop logic
void cia402_enter_fault(struct cia402 *cia402);

// Get current cia402 state machine state
enum cia402_state cia402_current_state(struct cia402 *cia402);

void cia402_thread_run(struct cia402 *cia402, void *unused2, void *unused3);

#endif /* CIA402_H_ */
