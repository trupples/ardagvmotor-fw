#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>
#include <canopennode.h>

#include "cia402.h"
#include "OD.h"

LOG_MODULE_REGISTER(cia402, LOG_LEVEL_INF);

static const struct smf_state cia402_states[];

void cia402_enter_fault(struct cia402 *cia402) {
    smf_set_state(&cia402->smf_ctx, &cia402_states[CIA402_FAULT_REACTION_ACTIVE]);
}

enum cia402_state cia402_current_state(struct cia402 *cia402) {
    // Pointer arithmetic to return index into cia402_states[], which is a state enum value
    return cia402->smf_ctx.current - cia402_states;
}

static void cia402_unrecoverable_error(struct cia402 *cia402) {
    cia402->cb_oops(cia402);
}

// Sets and transmits the StatusWord
static void cia402_statusword(struct cia402* cia402, int statusword) {
    LOG_DBG("StatusWord = 0x%04X", statusword);
    if(OD_set_u16(OD_ENTRY_H6041_statusword, 0, statusword, true)) {
        LOG_ERR("Could not set Statusword");
        cia402_unrecoverable_error(cia402);
    }

    OD_requestTPDO(OD_getFlagsPDO(OD_ENTRY_H6041_statusword), 0);
}

// CIA402_NOT_READY_TO_SWITCH_ON: low power, disabled drive, brake enabled if exists. run self-test + init
static void enter_not_ready_to_switch_on(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state: NOT_READY_TO_SWITCH_ON");
    cia402_statusword(cia402, 0);
}

static void run_not_ready_to_switch_on(void *p) {
    struct cia402 *cia402 = p;
    cia402->cb_initialize(cia402);
    smf_set_state(&cia402->smf_ctx, &cia402_states[CIA402_SWITCH_ON_DISABLED]);
}


// CIA402_SWITCH_ON_DISABLED: self-test + init ok. idle powered-down state
static void enter_switch_on_disabled(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state: SWITCH_ON_DISABLED");
    cia402_statusword(cia402, CIA402_BIT_SWITCHONDISABLED);
}


// CIA402_READY_TO_SWITCH_ON: maybe high power, drive disabled. intermediary state during motor power-up
static void enter_ready_to_switch_on(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state: READY_TO_SWITCH_ON");
    cia402_statusword(cia402, CIA402_BIT_QUICKSTOP | CIA402_BIT_READYTOSWITCHON);
}


// CIA402_SWITCHED_ON: high power, drive disabled. intermediary state during motor power-up
static void enter_switched_on(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state: SWITCHED_ON");
    cia402_statusword(cia402, CIA402_BIT_QUICKSTOP | CIA402_BIT_READYTOSWITCHON | CIA402_BIT_SWITCHON);
    cia402->cb_start(cia402);
}


// CIA402_OPERATION_ENABLED: high power, drive enabled, motor actively communicating and moving
static void enter_operation_enabled(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state: OPERATION_ENABLED");
    cia402_statusword(cia402, CIA402_BIT_QUICKSTOP | CIA402_BIT_READYTOSWITCHON | CIA402_BIT_SWITCHON | CIA402_BIT_OPERATIONEN);
}


// CIA402_QUICK_STOP_ACTIVE: stop motor, then go to SWITCH_ON_DISABLED
static void enter_quick_stop_active(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state: QUICK_STOP_ACTIVE");
    cia402_statusword(cia402, CIA402_BIT_READYTOSWITCHON | CIA402_BIT_SWITCHON | CIA402_BIT_OPERATIONEN);
    cia402->cb_stop(cia402);
}

static void run_quick_stop_active(void *p) {
    struct cia402 *cia402 = p;
    if(cia402->cb_is_stopped(cia402)) {
        smf_set_state(&cia402->smf_ctx, &cia402_states[CIA402_SWITCH_ON_DISABLED]);
    }
}


// CIA402_FAULT_REACTION_ACTIVE: stop motor, then go to FAULT
static void enter_fault_reaction_active(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state: FAULT_REACTION_ACTIVE");
    cia402_statusword(cia402, CIA402_BIT_FAULT | CIA402_BIT_READYTOSWITCHON | CIA402_BIT_SWITCHON | CIA402_BIT_OPERATIONEN);
    cia402->cb_stop(cia402);
}

static void run_fault_reaction_active(void *p) {
    struct cia402 *cia402 = p;
    if(cia402->cb_is_stopped(cia402)) {
        smf_set_state(&cia402->smf_ctx, &cia402_states[CIA402_FAULT]);
    }
}


// CIA402_FAULT: stay in this state until explicitly taken out
static void enter_fault(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state: FAULT");
    cia402_statusword(cia402, CIA402_BIT_FAULT);
}

static void cia402_ensure_power_disabled(void *p) {
    struct cia402 *cia402 = p;
    if(!cia402->cb_is_stopped(cia402)) {
        LOG_WRN("Power was not disabled, as expected!");
        cia402->cb_stop(cia402);
    }
}

static void enter_group_power_disabled(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state group: power disabled");
    cia402_ensure_power_disabled(cia402);
}

static void enter_group_power_enabled(void *p) {
    __unused struct cia402 *cia402 = p;
    LOG_INF("Entering state group: power enabled");
}

static void enter_group_fault(void *p) {
    struct cia402 *cia402 = p;
    LOG_INF("Entering state group: fault");
    cia402_ensure_power_disabled(cia402);
}

#define CIA402_GROUP_POWER_DISABLED CIA402_FAULT+1
#define CIA402_GROUP_POWER_ENABLED  CIA402_FAULT+2
#define CIA402_GROUP_FAULT          CIA402_FAULT+3

static const struct smf_state cia402_states[] = {
    [CIA402_NOT_READY_TO_SWITCH_ON] = SMF_CREATE_STATE(enter_not_ready_to_switch_on, run_not_ready_to_switch_on, NULL, &cia402_states[CIA402_GROUP_POWER_DISABLED], NULL),
    [CIA402_SWITCH_ON_DISABLED]     = SMF_CREATE_STATE(enter_switch_on_disabled,     NULL,                       NULL, &cia402_states[CIA402_GROUP_POWER_DISABLED], NULL),
    [CIA402_READY_TO_SWITCH_ON]     = SMF_CREATE_STATE(enter_ready_to_switch_on,     NULL,                       NULL, &cia402_states[CIA402_GROUP_POWER_DISABLED], NULL),
    [CIA402_SWITCHED_ON]            = SMF_CREATE_STATE(enter_switched_on,            NULL,                       NULL, &cia402_states[CIA402_GROUP_POWER_ENABLED],  NULL),
    [CIA402_OPERATION_ENABLED]      = SMF_CREATE_STATE(enter_operation_enabled,      NULL,                       NULL, &cia402_states[CIA402_GROUP_POWER_ENABLED],  NULL),
    [CIA402_QUICK_STOP_ACTIVE]      = SMF_CREATE_STATE(enter_quick_stop_active,      run_quick_stop_active,      NULL, &cia402_states[CIA402_GROUP_POWER_ENABLED],  NULL),
    [CIA402_FAULT_REACTION_ACTIVE]  = SMF_CREATE_STATE(enter_fault_reaction_active,  run_fault_reaction_active,  NULL, &cia402_states[CIA402_GROUP_FAULT],          NULL),
    [CIA402_FAULT]                  = SMF_CREATE_STATE(enter_fault,                  NULL,                       NULL, &cia402_states[CIA402_GROUP_FAULT],          NULL),

    // Parent state machines for Power Enabled / Power Disabled / Fault "groups"
    [CIA402_GROUP_POWER_DISABLED] = SMF_CREATE_STATE(enter_group_power_disabled, NULL, NULL, NULL, NULL),
    [CIA402_GROUP_POWER_ENABLED]  = SMF_CREATE_STATE(enter_group_power_enabled, NULL, NULL, NULL, NULL),
    [CIA402_GROUP_FAULT]          = SMF_CREATE_STATE(enter_group_fault, NULL, NULL, NULL, NULL),
};

// Debug CAN error counters and bus state
static void can_debug_state(struct cia402 *cia402) {
    static int prev_tx_err_cnt = 0, prev_rx_err_cnt = 0;
    static enum can_state prev_state = CAN_STATE_STOPPED;
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	int err;

	err = can_get_state(cia402->co->can_dev, &state, &err_cnt);
	if (err != 0) {
		LOG_ERR("failed to get CAN controller state (err %d)", err);
		return;
	}
    if(prev_tx_err_cnt != err_cnt.tx_err_cnt || prev_rx_err_cnt != err_cnt.rx_err_cnt || prev_state != state) {
		LOG_DBG("CAN TEC = %d\tREC = %d\tstate = %s", err_cnt.tx_err_cnt, err_cnt.rx_err_cnt,
            state == CAN_STATE_ERROR_ACTIVE ? "ACTIVE" :
            state == CAN_STATE_ERROR_WARNING ? "WARNING" :
            state == CAN_STATE_ERROR_PASSIVE ? "PASSIVE" :
            state == CAN_STATE_BUS_OFF ? "BUS_OFF" :
            state == CAN_STATE_STOPPED ? "STOPPED" : "???"
        );
    }

    if(state == CAN_STATE_BUS_OFF) {
        // TODO rate limit, if it goes into bus off a lot, stop resetting and just leave it in the fault state
        LOG_ERR("Bus-off: killing motor");
        CO_NMT_sendInternalCommand(cia402->co->CO->NMT, CO_NMT_ENTER_STOPPED);
        cia402_enter_fault(cia402);
    }
    
    prev_tx_err_cnt = err_cnt.tx_err_cnt;
    prev_rx_err_cnt = err_cnt.rx_err_cnt;
    prev_state = state;
}

void cia402_thread_run(void *arg1, void *unused2, void *unused3) {
    struct cia402 *cia402 = arg1;

    ARG_UNUSED(unused2);
    ARG_UNUSED(unused3);

    while(k_yield(), !cia402->_unrecoverable_error_flag)
    {
        can_debug_state(cia402);

        // Check NMT state, CiA 402 logic only makes sense inside OPERATIONAL
        if(cia402->co->CO->NMT->operatingState == CO_NMT_STOPPED) {
            smf_set_state(&cia402->smf_ctx, &cia402_states[CIA402_FAULT_REACTION_ACTIVE]);

            LOG_ERR("NMT STOPPED");
            while(cia402->co->CO->NMT->operatingState == CO_NMT_STOPPED) { // No point in doing anything until the canopen thread takes us out of the stopped state
                k_msleep(1);
            }
            LOG_INF("NMT un-stopped");
            continue;
        }
        
        if(cia402->co->CO->NMT->operatingState != CO_NMT_OPERATIONAL) {
            k_msleep(1);
            continue;
        }

        // Run current state's logic one step, along with all code-triggered transitions (as opposed to external commands)
        smf_run_state(&cia402->smf_ctx);

        // Handle ControlWord transitions
        uint16_t controlword;
        int err = OD_get_u16(OD_ENTRY_H6040_controlword, 0, &controlword, false);
        if(err != ODR_OK) {
            LOG_ERR("Could not read ControlWord: %d", err);
            cia402_unrecoverable_error(cia402);
            break;
        }

        // FIXME: Use write extension to detect writes, not a sentinel value
        if(controlword == 0xffff) {
            // No write
            k_msleep(1);
            continue;
        }
        OD_set_u16(OD_ENTRY_H6040_controlword, 0, 0xFFFF, true);
        
        LOG_DBG("ControlWord = %04X", controlword);

        // Parse ControlWord bits (cia402 v2.0 10.3.1)
        const bool cw_halt             = (controlword >> 8) & 1; // TODO: How to properly handle? Should be something like enabling the motor but forcing it to brake.
        const bool cw_fault_reset      = (controlword >> 7) & 1;
        const bool cw_enable_operation = (controlword >> 3) & 1;
        const bool cw_quick_stop       = (controlword >> 2) & 1;
        const bool cw_enable_voltage   = (controlword >> 1) & 1;
        const bool cw_switch_on        = (controlword >> 0) & 1;

        // Compute commands (cia402 v2.0 10.3.1 table 4)
        const bool com_fault_reset       = cw_fault_reset;
        const bool com_shutdown          = !cw_fault_reset && cw_quick_stop && cw_enable_voltage && !cw_switch_on;
        const bool com_switch_on         = !cw_fault_reset && cw_quick_stop && cw_enable_voltage && cw_switch_on; // Enable operaion is "=0 OR =1" so an effective X
        const bool com_enable_operation  = !cw_fault_reset && cw_enable_operation && cw_quick_stop && cw_enable_voltage && cw_switch_on;
        const bool com_disable_operation = !cw_fault_reset && !cw_enable_operation && cw_quick_stop && cw_enable_voltage && cw_switch_on;
        const bool com_disable_voltage   = !cw_fault_reset && !cw_enable_voltage;
        const bool com_quick_stop        = !cw_fault_reset && !cw_quick_stop && cw_enable_voltage;

        const enum cia402_state state = cia402_current_state(cia402);
        enum cia402_state nextState = state;

        // Compute transitions (via402 v2.0 10.1.1.2)
        // Transitions 0, 1: automatic, already happened by now

        // Transitions 2, 3, 4: SWITCH_ON_DISABLED -> READY_TO_SWITCH_ON -> SWITCHED_ON -> OPERATION_ENABLE
        if(state == CIA402_SWITCH_ON_DISABLED && com_shutdown)                                          nextState = CIA402_READY_TO_SWITCH_ON;
        if(state == CIA402_READY_TO_SWITCH_ON && com_switch_on)                                         nextState = CIA402_SWITCHED_ON;
        if(state == CIA402_SWITCHED_ON        && com_enable_operation && cia402->cb_is_started(cia402)) nextState = CIA402_OPERATION_ENABLED;

        // Transitions 5, 6, 7: OPERATION_ENABLE -> SWITCHED_ON -> READY_TO_SWITCH_ON -> SWITCH_ON_DISABLED
        if(state == CIA402_OPERATION_ENABLED  && com_disable_operation)                   nextState = CIA402_SWITCHED_ON;
        if(state == CIA402_SWITCHED_ON        && com_shutdown)                            nextState = CIA402_READY_TO_SWITCH_ON;
        if(state == CIA402_READY_TO_SWITCH_ON && (com_quick_stop || com_disable_voltage)) nextState = CIA402_SWITCH_ON_DISABLED;

        // Transitions 8, 9: Power off from OPERATION_ENABLED
        if(state == CIA402_OPERATION_ENABLED && com_shutdown)        nextState = CIA402_READY_TO_SWITCH_ON;
        if(state == CIA402_OPERATION_ENABLED && com_disable_voltage) nextState = CIA402_SWITCH_ON_DISABLED;

        // Transitions 10, 11, 12: Quick stop
        if(state == CIA402_SWITCHED_ON && (com_quick_stop || com_disable_voltage)) nextState = CIA402_SWITCH_ON_DISABLED;
        if(state == CIA402_OPERATION_ENABLED && com_quick_stop) nextState = CIA402_QUICK_STOP_ACTIVE;
        if(state == CIA402_QUICK_STOP_ACTIVE && com_disable_voltage) nextState = CIA402_SWITCH_ON_DISABLED; // "Quick stop is completed" check handled in run_quick_stop_active

        // Transitions 13, 14 implemented by cia402_enter_fault, run_fault_reaction_active
        
        // Transition 15: FAULT -> SWITCH_ON_DISABLED 
        if(state == CIA402_FAULT && com_fault_reset) nextState = CIA402_SWITCH_ON_DISABLED;
        
        if(nextState != state) {
            smf_set_state(&cia402->smf_ctx, &cia402_states[nextState]);
        }

        k_msleep(1);
    }
}

void cia402_init(struct cia402* cia402) {
    cia402->statusword_extension.read = OD_readOriginal;
    cia402->statusword_extension.write = OD_writeOriginal;
    cia402->mod_extension.read = OD_readOriginal;
    cia402->mod_extension.write = OD_writeOriginal;
    cia402->vel_extension.read = OD_readOriginal;
    cia402->vel_extension.write = OD_writeOriginal;
    cia402->pos_extension.read = OD_readOriginal;
    cia402->pos_extension.write = OD_writeOriginal;

    OD_extension_init(OD_ENTRY_H6041_statusword, &cia402->statusword_extension);
    OD_extension_init(OD_ENTRY_H6061_modesOfOperationDisplay, &cia402->mod_extension);
    OD_extension_init(OD_ENTRY_H606C_velocityActualValue, &cia402->vel_extension);
    OD_extension_init(OD_ENTRY_H6064_positionActualValue, &cia402->pos_extension);

    cia402->_unrecoverable_error_flag = false;

    smf_set_initial(&cia402->smf_ctx, &cia402_states[CIA402_NOT_READY_TO_SWITCH_ON]);
}
