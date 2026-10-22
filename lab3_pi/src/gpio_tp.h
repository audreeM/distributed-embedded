// Copies from our lab 2 code + added tp_set
/* Scope test points on the Pi, driven through /dev/gpiomem (BCM2711). */
#ifndef GPIO_TP_H
#define GPIO_TP_H

#include <stdbool.h>

/* Returns 0 on success; on failure tp_toggle() and tp_set() become no-ops. */
int tp_init(const int *pins, int npins);
void tp_toggle(int pin);          /* edge marker for scope timing */
void tp_set(int pin, bool high);  /* level output: status LEDs, ERR */
void tp_release(void);

#endif /* GPIO_TP_H */
