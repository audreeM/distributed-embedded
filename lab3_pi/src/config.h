// Copied from our lab 2 code + modified for taskset
/*
 * Cockpit zone (N1) configuration: every team-defined number the Pi uses.
 * Keep this in sync with the team-defined values table in the design doc.
 */
#ifndef CONFIG_H
#define CONFIG_H

/* --- Wheel calibration (raw DIJOYSTATE2 values, from Lab 2 Part 1) -------- */
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

/* Raw rgbButtons[] indices (wheel_monitor -r); -1 = not configured */
#define BTN_IDX_LEFT   9 /* LSB: left turn signal */
#define BTN_IDX_RIGHT  8 /* RSB: right turn signal */
#define BTN_IDX_TEST   1 /* B: cockpit self-test button */

/* --- Network -------------------------------------------------------------- */
#define UDP_PORT        8000      /* wheel state in, from the proxy */
#define FORCE_UDP_PORT  8001      /* force byte out, to the proxy's address */
#define WHEEL_PKT_LEN   (4 + 272) /* counter + DIJOYSTATE2 */

/* --- CAN ------------------------------------------------------------------ */
/* can_up.sh renames the interfaces so these always match the wiring. */
#define CAN_IF_A      "can0"    /* SPI0 CE0 module, bus A */
#define CAN_IF_B      "can1"    /* SPI0 CE1 module, bus B */

/* --- Timing (ms) ---------------------------------------------------------- */
#define TICK_MS           1    /* housekeeping timer */
#define HB_PERIOD_MS      20   /* R4.5: heartbeat every 20 ms +/- 10 % */
#define HB_MISS_LIMIT     3    /* R4.5: 3 missed on both buses = zone failed */
#define DEGRADED_MS       60   /* R4.7: bus silent this long (other alive) = degraded */
#define CTRL_REFRESH_MS   20   /* R4.4: resend CTRL if no UDP update for this long */
#define UDP_STALE_MS      100  /* Cockpit Zone spec: no UDP this long = cockpit failed */
#define BLINK_HALF_MS     500  /* R2.4: 1 Hz, 50 % duty */
#define HAZARD_HALF_MS    250  /* S4: 2 Hz hazard */
#define BLINK_MIN_GAP_MS  20   /* catalog: BLINK minimum interarrival */
#define FAULT_MIN_GAP_MS  20   /* catalog: FAULT minimum interarrival */
#define FORCE_UDP_MS      50   /* R2.3: UDP force message to the proxy every 50 ms */
#define SELFTEST_LOCKOUT_MS 50   /* R2.5: presses < 50 ms apart are one press */
#define SELFTEST_DOUBLE_MS  500  /* R3: two presses within 500 ms restore the zone */
#define PRINT_MS          200  /* console status line */

/* --- Turn-signal self-cancel (team-defined values, steer units) ----------- */
#define BLINK_ARM_STEER     20   /* |steer| >= this after the signal starts: armed */
#define BLINK_CANCEL_STEER  18   /* armed and back to |steer| <= this: cancel */

/* --- Test points and LEDs (BCM GPIO numbers, see the wiring page) -------- */
#define TP_UDP_RX    23 /* row 3: toggles on each UDP state update */
#define TP_CMD_TX    17 /* row 4: toggles when CTRL is queued on both buses */
#define TP_CAN_TX_A  27 /* row 5: toggles on every write to can0 */
#define TP_CAN_TX_B  22 /* row 6: toggles on every write to can1 */
#define LED_DEG_A    5  /* row 7: high while bus A is degraded */
#define LED_DEG_B    6  /* row 8: high while bus B is degraded */
#define LED_ERR      13 /* row 9: high while the cockpit is not in NORMAL */

#define RT_PRIORITY  80 /* SCHED_FIFO priority (needs sudo) */

#endif /* CONFIG_H */
