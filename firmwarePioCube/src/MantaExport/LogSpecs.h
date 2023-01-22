/*
 * LogSpecs.h
 *
 *  Created on: Jun 12, 2019
 *      Author: whzen
 *
 *  Encode & decode values to/from packet format
 *  Packet format is always float.
 *  enums are encoded into discrete values and decoded by range.
 *  bools are treated like enums
 *  ints go to floats
 *  the format specs provide a lookup for subject to type, and format string for numerics
 *
 */

#ifndef SRC_BOARD2BOARD_LOGSPECS_H_
#define SRC_BOARD2BOARD_LOGSPECS_H_

#include <cstdint>
#include <array>

namespace Manta {

class LogSpecs {
 public:
  LogSpecs();
  virtual ~LogSpecs() {
  }

  // if you add a Subject here, add an equivalent entry in the table (see cpp file)
  enum Subject {
    UNK,
    TEST,
    PIKEENC,
    GARENC,
    GARADC,
    GARADCRAW,
    TACH,
    PUMPSPD,
    PUMPSTATUS,
    DOORS,
    CPANEL,
    GARINTER,
    PIKEINTER,
    LEVELSW,
    GARAIR,
    LIGHTS,
    PIKEHEATTEMPS,    // actual temperature (degC)
    PIKEHEATSTATUS,   // open/shorted/untested/OK
    PIKEHEATOUT,      // output PWM (0 to 1)
    PIKEHEATSET,      // channel setpoint
    PIKEPWMSET,       // channel max PWM (0 to 1)
    UVLAMPTEMP,
    UVLAMPSTATUS,
    UVLAMPOUTPUT,
    GARINFO,        // ID (4 words), FW rev, encoder rev, gpio rev
    PIKEINFO,       // see above
    SERVICEPOSITION,
    SERVICESTATUS,
    SERVICESPEEDSET,  // fast speed
    SERVICEENABLESET,
    UVLAMPX_SET,
    AIRFANX_SET,
    PRINTMEASURE_SET,
    XSTATUS,      // x axis status (see eXSTATUS for channel definitions)
    FEATURE_ENABLE,  // non-volatile (saved in config) feature enables
    TUBE_ENABLE,
    TUBE_PWM,
    SERVICEWASHSET,   // wash pump speeds (0 to 1)
    SERVICEFASTSET,   // service fast retract position (as fraction)
    SERVICEWIPESET,   // wipe (slow) speed
    //
    NAUTLIGHTS,
    DISCUSLIGHTS,
    NAUT_ENABLESxxx,   // deprecated see NATUTEMPEN_TARG
    NAUT_UIDS,
    NAUTPUMPS,
    NAUTOVERFLOW,
    NAUTADCRAW,
    NAUTADCTEMPS,   // current readings
    NAUTTEMPDRIVE,  // current drive
    NAUTLEVELS,
    NAUTPRESSPID,
    NAUTTEMPMAXPWMxxx,  // deprecated see NAUTTEMPMAXPWM_TARG
    //
    DOORLOCKS,
    CAMERAMEASURE_SET,
    PROJECTOR_SET,
    //
    NAUTHTR_AMPS,
    SMPOWER_STATUS_1,
    SMPOWER_STATUS_2,
    SMPOWER_STATUS_3,
    SMPOWER_STATUS_4,
    NAUTPRESSIN_IDLE_TARG,
    NAUTPRESSIN_RUN_TARG,
    NAUTPRESSOUT_IDLE_TARG,
    NAUTPRESSOUT_RUN_TARG,
    NAUTPRESSPURGE_TARG,
    NAUTPRESSOVER_TARG,
    NAUTPRESSFLUID_TARG,
    NAUTSTREAMCONTROL_TARG,
    NAUTTEMPSET_TARG,
    NAUTTEMPBETA_TARG,
    NAUTTEMPMAXPWM_TARG,
    NAUTTEMPKI_TARG,
    NAUTTEMPKP_TARG,
    NAUTTEMPEN_TARG,
    IR_CAMERAMEASURE_SET,  // deprecated; from snapper see DARTER_TRIG series for current
    NAUT_PRINT_MODE,    // current print mode (aka FeedMode) info
    NAUT_HEATER_STATUS,  // derived power driver status pins
    NAUTTUBEPWM_TARG,
    UVSEGENABLE_LEFT,
    UVSEGENABLE_RIGHT,
    LOG_INTERVAL,
    SMPPOWER_ENABLE_TARG,   // turn on power for these by default
    //
    DARTER_TRIG0_PRINT,
    DARTER_TRIG1_M0,      // ^^^whz 20211108 these designitors are out of date
    DARTER_TRIG2_M1,
    DARTER_TRIG3_IR,
    DARTER_TRIG4_COOL,
    DARTER_TRIG5_PROJ,
    DARTER_TRIG6_UV,
    DARTER_TRIG7_PARK,
    DARTER_TRIG8_CAM0,
    DARTER_TRIG9_CAM1,
    DARTER_TRIGA_CAM2,
    DARTER_TRIGB_CAM3,
    DARTER_TRIGC_CAM4,
    DARTER_TRIGD_CAM5,
    DARTER_TRIGE_CAM6,
    DARTER_TRIGF_CAM7,
    DARTER_ENCODERS,      // reporting the current encoder counts
    DARTER_WINDOWS_UP,    // reporting the current window conditions (count up direction)
    DARTER_WINDOWS_DN,    // reporting the current window conditions (count down direction)
    //
    PROJECTOR_PWR,
    PROJECTOR_TIME,
    DARTER_FAN_MAXRPM,  // calibration for tach -> rpm conversion Build cooling fans +  cabinet fans
    DARTER_FAN_MAXREADING,  // calibration for tach -> rpm conversion
    DARTER_FAN_TACHRANGE,   // setting for fan controller's tach
    DARTER_COOL_PERSIST_TIME,  // fan trigger turns on for this much more (secs)
    DARTER_FAN_PWM_TARG,    // fan run speed (0-1)
    DAMSEL_FAN_MAXRPM,      // as above but for damselfish controller
    DAMSEL_FAN_MAXREADING,
    DAMSEL_FAN_TACHRANGE,
    DAMSEL_FAN_PWM_TARG,
    PROJECTOR_IDLE_PWR,  // projector idle power level, when it's not actively scanning
    PROJECTOR_ONOFF_MODE,  // projectors on/off or auto
    DAMSEL_FAN_PWM_OUT,    // fan pwm read back for profilometer assembly
    DAMSEL_FAN_RPM_OUT,    // fan rpm read back for profilometer assembly
    DARTER_FAN_PWM_OUT,    // fan pwm read out for cabinet & build cooling fans
    DARTER_FAN_RPM_OUT,    // fan rpm read out for cabinet & build cooling fans
    ORCA_TEMP,
    ORCA_TEMPBETA_TARG,
    ORCA_AMPS,            // 'current' seemed like a confusing name
    //
    ORCA_HEAT_TEMP_TARG,
    ORCA_HEAT_TEMP_IDLE,  // deprecated - alternate idle setpoints will be handled by Dolphin
    ORCA_HEAT_KI,
    ORCA_HEAT_KP,
    ORCA_HEAT_EN,
    ORCA_PLATFORM,
    ORCA_LEVELS_0,    // 1-12 channels
    ORCA_LEVELS_1,    // 13-24 channels
    ORCA_HEAT_PWM,
    //
    DARTER_INTERLOCK,  // orca interlock basically encapulates all the deprecated/unused subjects above
    OVERRIDE_PRESET,  // preset override setting for door safety calculation when in safety overridden operation
    //
    DAMSEL_AMBIENT_TEMP,    // PCB temperature from Damselfish assy
    //
    BULK_HEAT_TEMPERATURE0,  // bulk (materials feed side) reported temperatures wax channel
    BULK_HEAT_TEMPERATURE1,  // bulk (materials feed side) reported temperatures build materials channel
    BULK_HEAT_TEMPERATURE2,  // bulk (materials feed side) reported temperatures build materials channel
    BULK_HEAT_TEMPERATURE3,  // bulk (materials feed side) reported temperatures build materials channel
    BULK_HEAT_TARGET0,  // bulk (materials feed side) setpoint temperatures wax channel
    BULK_HEAT_TARGET1,  // bulk (materials feed side) setpoint temperatures materials
    BULK_HEAT_TARGET2,
    BULK_HEAT_TARGET3,
    BULK_HEAT_ENABLE0,  // bulk heat enables wax channel
    BULK_HEAT_ENABLE1,  // bulk heat enables materials channels
    BULK_HEAT_ENABLE2,
    BULK_HEAT_ENABLE3,
    BULK_HEAT_PWMOUT0,
    BULK_HEAT_PWMOUT1,
    BULK_HEAT_PWMOUT2,
    BULK_HEAT_PWMOUT3,
    //
    BONITO_FAN_PWM_TARG,
    BONITO_FAN_MAXRPM,      // as above but for bonito fan add on
    BONITO_FAN_MAXREADING,
    BONITO_FAN_TACHRANGE,
    BONITO_FAN_ADC,
    BONITO_FAN_RPM_OUT,
    BONITO_FAN_PWM_OUT,   // the target is a subset of the DARTER_FAN_... but it's easier to report separately
    DARTER_HOST_MAC, // used for Magic Packet wake up over lan
    DARTER_TRIG_ENABLES,  // trigger enables; report only see CMD_ENABLE_TRIGGERS for setting
    //
    UVLAMP_MIN_SPEED,   // minimum speed (mm/sec) below which the lamp & laser will not come on - to keep from roasting things or burning out the laser if build plate is just sitting
    DARTER_SMARTMOD_ENABLES,  // enable/disable the power and test/connect smart modules
    NOOK_DOOR,
    NOOK_FAN_SPEED,  // Nook fan % speed values
    NOOK_PUMP_SPEED,  // Nook pump % speed values
    NOOK_FAN_RPM,  // Nook fan tach
    NOOK_PUMP_RPM,  // Nook pump tach
    NAUTAMBIENT,    // smart module ambient temperatures
    NOOK_VOL_LITERS, // nook tank volume in liters
    NOOK_MAT_SPECG, // specific gravity of current loaded material
    //
    NAUTWATCH_P_MIN_VAL,    // pressure/feed loops values below this are 'out of bounds'
    NAUTWATCH_P_MAX_VAL,    // values above this (compensated for things like maxpwm) are 'out of bounds'
    NAUTWATCH_P_MAX_SECS,   // if out of bounds persists for more than secs, there will be an error
    NAUTWATCH_P_MIN_DRIVE,  // some readings, like tach feedback or current can't be checked below some minimum drive value
    NAUTWATCH_T_MIN_RAWVAL,  // for temperatures, we'll be looking at the raw millivolts inputs
    NAUTWATCH_T_MAX_RAWVAL,  // for temperatures, we'll be looking at the raw millivolts inputs
    NAUTWATCH_T_MAX_SECS,   // the expectation is this would be short, just enough to cover single measurement glitches eg.
    //
    MELTER_STATE, // reference Datagram.h eDgramMelterStates
    MELTWATCH_T_MIN_VAL,  // RKC temperatures target thresholds
    MELTWATCH_T_MAX_VAL,  // RKC temperatures target thresholds
    MELTWATCH_T_MAX_SECS,   // this will be long,
    //
    SERVICEFLUSHSECSSET,   // service station flush seconds
    //
    NOOK_MAT1_ID, // 16 byte material id
    NOOK_MAT2_ID,
    NOOK_MAT3_ID,
    NOOK_MAT4_ID,
    //
    WASTE_BAY_LVLS, // level sensor of each waste bay
    WASTE_BAY_PROXS, // proximity sensor of each waste bay
    //
    BIB_PUMP_STATES, // level sensor of each waste bay
    BIBWATCH_P_MIN_VAL,    // bib pump rpm values below this are 'out of bounds'
    BIBWATCH_P_MAX_VAL,    // values above this (compensated for things like maxpwm) are 'out of bounds'
    BIBWATCH_P_MAX_SECS,   // if out of bounds persists for more than secs, there will be an error
    BIBWATCH_P_MIN_DRIVE,  // some readings, like tach feedback or current can't be checked below some minimum drive value
    //
    SVC_CURR_SET, // set idle and rms currents (0-1.2A) 10x {idle0, rms0, idle1, ...}
    SVCWASHSET, // wash modulator period and duty cycles {period0, period1, period2 , duty0, duty1, duty2}
    //
    NAUTPRESSIN_SERVICE_TARG,   // feed mode inlet pressure target for new SERVICE mode
    NAUTPRESSOUT_SERVICE_TARG,  // feed mode outlet pressure target for new SERVICE mode
    //
    PSU1_STATUS,
    PSU2_STATUS,
    PSU3_STATUS,
    PSU4_STATUS,
    PSU5_STATUS,
    //
    BIB_SPEEDS, // specifies the empty, recirc, and valve transition speed setpoints
    BIB_TIMERS, // specifies the empty, recirc, and off transition speed setpoints
    /*
     * This enum is add only.
     */
    NUM_SUBJECTS
  };

};

} /* namespace Manta */

#endif /* SRC_BOARD2BOARD_LOGSPECS_H_ */

