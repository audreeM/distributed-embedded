/*
 * Cockpit (Pi) configuration: wheel calibration, button indices, pins, timing.
 * Replace the calibration numbers with what you recorded in Part 1.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* --- Wheel calibration (raw DIJOYSTATE2 values, Mac proxy defaults) ------ */
#define STEER_RAW_LEFT    (-32767) /* lX at full left  */
#define STEER_RAW_CENTER  0        /* lX centered      */
#define STEER_RAW_RIGHT   32767    /* lX at full right */

#define THR_RAW_RELEASED  0        /* lY  released      */
#define THR_RAW_PRESSED   32767    /* lY  fully pressed */
#define BRK_RAW_RELEASED  0        /* lRz released      */
#define BRK_RAW_PRESSED   32767    /* lRz fully pressed */

/* Deadbands, in output units */
#define STEER_DEADBAND    2 /* |steer| <= 2   -> 0 (servo dither near center) */
#define THR_DEADBAND      3 /* throttle < 3 % -> 0 (pedal creep) */
#define BRK_DEADBAND      3 /* brake < 3 %    -> 0 (brake > 0 means braking!) */

/* --- Buttons: raw rgbButtons[] indices from `wheel_monitor -r` / --probe --
 * -1 = not configured yet (that button always reads released).
 */
#define BTN_IDX_LEFT   9 /* LSB: left turn signal */
#define BTN_IDX_RIGHT  8 /* RSB: right turn signal */
#define BTN_IDX_TEST   1 /* B: self-test button */

/* --- Network / serial ---------------------------------------------------- */
#define UDP_PORT        8000
#define WHEEL_PKT_LEN   (4 + 272) /* counter + DIJOYSTATE2 */
#define SERIAL_DEV      "/dev/serial0"

/* --- Test points (BCM GPIO numbers) ---------------------------------------- */
#define TP_UDP_RX       23 /* header pin 16 */
#define TP_CMD_TX       24 /* header pin 18 */

/* --- Timing ---------------------------------------------------------------- */
#define TICK_MS          5   /* housekeeping timer */
#define CMD_REFRESH_MS   20  /* resend last command if nothing sent for this long */
#define UDP_STALE_MS     100 /* stop commanding if the wheel stream is this old */
#define STATUS_SILENT_MS 100 /* warn if no STM32 status for this long */
#define PRINT_MS         200 /* console print period */
#define INJECT_BAD_MS    1000 /* 'b' key: send out-of-range commands this long */
#define PARSER_GAP_US    10000 /* abandon a partial status frame after this gap */

#endif /* CONFIG_H */
