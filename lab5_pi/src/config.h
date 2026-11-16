/*
 * Cockpit zone (N1) configuration: every team-defined number the Pi uses.
 * Lab 3 values plus the lab 5 section at the bottom. Raft settings are in
 * common/raft/raft_config.h (shared with the STM32s).
 */
#ifndef CONFIG_H
#define CONFIG_H

/* --- Wheel calibration (raw DIJOYSTATE2 values, from Lab 2 Part 1) -------- */
#define STEER_RAW_LEFT (-32767) /* lX at full left  */
#define STEER_RAW_CENTER 0		/* lX centered      */
#define STEER_RAW_RIGHT 32767	/* lX at full right */

#define THR_RAW_RELEASED 0	  /* lY  released      */
#define THR_RAW_PRESSED 32767 /* lY  fully pressed */
#define BRK_RAW_RELEASED 0	  /* lRz released      */
#define BRK_RAW_PRESSED 32767 /* lRz fully pressed */

/* Deadbands, in output units */
#define STEER_DEADBAND 2 /* |steer| <= 2   -> 0 (servo dither near center) */
#define THR_DEADBAND 3	 /* throttle < 3 % -> 0 (pedal creep) */
#define BRK_DEADBAND 3	 /* brake < 3 %    -> 0 (brake > 0 means braking!) */

/* Raw rgbButtons[] indices (wheel_monitor -r); -1 = not configured */
#define BTN_IDX_LEFT 9	/* LSB: left turn signal */
#define BTN_IDX_RIGHT 8 /* RSB: right turn signal */
#define BTN_IDX_TEST 1	/* B: cockpit self-test button */

/* --- Network -------------------------------------------------------------- */
#define UDP_PORT 8000			/* wheel state in, from the proxy */
#define FORCE_UDP_PORT 8001		/* force byte out, to the proxy's address */
#define WHEEL_PKT_LEN (4 + 272) /* counter + DIJOYSTATE2 */

/* --- CAN ------------------------------------------------------------------ */
/* can_up.sh renames the interfaces so these always match the wiring. */
#define CAN_IF_A "can0" /* SPI0 CE0 module, bus A */
#define CAN_IF_B "can1" /* SPI0 CE1 module, bus B */

/* --- Timing (ms) ---------------------------------------------------------- */
#define TICK_MS 1			   /* housekeeping timer */
#define HB_PERIOD_MS 20		   /* R4.5: heartbeat every 20 ms +/- 10 % */
#define HB_MISS_LIMIT 3		   /* R4.5: 3 missed on both buses = zone failed */
#define DEGRADED_MS 60		   /* R4.7: bus silent this long (other alive) = degraded */
#define CTRL_REFRESH_MS 20	   /* R4.4: resend CTRL if no UDP update for this long */
#define UDP_STALE_MS 100	   /* Cockpit Zone spec: no UDP this long = cockpit failed */
#define BLINK_HALF_MS 500	   /* R2.4: 1 Hz, 50 % duty (left or right turn signal)*/
#define HAZARD_HALF_MS 250	   /* S4: 2 Hz hazard (all four lamps)*/
#define BLINK_MIN_GAP_MS 20	   /* catalog: BLINK minimum interarrival */
#define FAULT_MIN_GAP_MS 20	   /* catalog: FAULT minimum interarrival */
#define FAULT_HOLD_MS 100	   /* a received FAULT holds ERROR at least this long */
#define FORCE_UDP_MS 50		   /* R2.3: UDP force message to the proxy every 50 ms */
#define SELFTEST_LOCKOUT_MS 50 /* R2.5: presses < 50 ms apart are one press */
#define SELFTEST_DOUBLE_MS 500 /* R3: two presses within 500 ms restore the zone */
#define PRINT_MS 1000		   /* console status lines */

/* --- Turn-signal self-cancel (team-defined values, steer units) ----------- */
#define BLINK_ARM_STEER 20	  /* |steer| >= this after the signal starts: armed */
#define BLINK_CANCEL_STEER 18 /* armed and back to |steer| <= this: cancel */

/* --- Test points and LEDs (BCM GPIO numbers, see the wiring page) -------- */
#define TP_UDP_RX 23   /* row 3: toggles on each UDP state update */
#define TP_CMD_TX 17   /* row 4: toggles when CTRL is queued on both buses */
#define TP_CAN_TX_A 27 /* row 5: toggles on every write to can0 */
#define TP_CAN_TX_B 22 /* row 6: toggles on every write to can1 */
#define LED_DEG_A 5	   /* row 7: high while bus A is degraded */
#define LED_DEG_B 6	   /* row 8: high while bus B is degraded */
#define LED_ERR 13	   /* row 9: high while the cockpit is not in NORMAL */

#define RT_PRIORITY 80 /* SCHED_FIFO priority (needs sudo) */

/* --- Lab 5 ----------------------------------------------------------------- */
/* New LEDs / test points (BCM GPIO numbers) */
#define LED_LEADER 16 /* pin 36: on while the Pi is the raft leader */
#define LED_SYNC   20 /* pin 38: on while global time is synced (always, when leader) */
#define TP_REF_RX  21 /* pin 40: toggles on every new REF received */
#define TP_PPS     26 /* pin 37: rising edge every whole second of global time */
#define TP_HZ      19 /* pin 35: the hazard lamps as the Pi shows them, for the scope */

#define EDGE_RT_PRIORITY 90  /* PPS / HZ thread, above the main loop */
#define EDGE_SPIN_US     500 /* wake this early before an edge and busy-wait the rest */
#define PPS_HIGH_MS      100

#define HEALTH_FRESH_MS  60    /* a node's heartbeat health byte counts for this long */
#define CLIENT_MS        10    /* rebuild obs / routing this often */
#define PEERS_READY_MAX_MS 30000 /* LEADER=random: start elections after this even if a node is missing */
#define PROBE_MS         300   /* at startup, listen this long for a leader before deciding cold/warm start */
#define BLINK_LEAD_MS    20    /* a new blink pattern starts this far in the future, so every node starts it together */

#define RAFT_STATE_FILE "raft_state.txt" /* term + vote, so a restarted Pi never votes twice in a term */
#define RAFT_LOG_FILE   "raft_log.txt"   /* every committed entry, and how far we've displayed */

#endif /* CONFIG_H */
