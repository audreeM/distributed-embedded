CONFIG_GPIO=y
CONFIG_SPI=y
CONFIG_CAN=y
CONFIG_PWM=y
CONFIG_WATCHDOG=y
CONFIG_PRINTK=y
CONFIG_CBPRINTF_FP_SUPPORT=n

# Leave CONFIG_CAN_MANUAL_RECOVERY_MODE off: bus-off recovers automatically (R4.8).
# One RX filter per catalog ID we receive (CTRL, FAULT, HB_N1, HB_N3, BLINK) per
# module; the driver default (CAN_MCP2515_MAX_FILTERS = 16) is enough.
# MCP2515 INT thread is part of CAN_RX; keep it above every app thread (config.h).
CONFIG_CAN_MCP2515_INT_THREAD_PRIO=0

# main() only feeds the watchdog and prints status: lowest app priority.
CONFIG_MAIN_THREAD_PRIORITY=10

# Keep the debug port alive while idle, so west flash can reconnect (same as Lab 2)
CONFIG_STM32_ENABLE_DEBUG_SLEEP_STOP=y
