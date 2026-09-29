/* Scope test points on the Pi, driven through /dev/gpiomem (BCM2711). */
#ifndef GPIO_TP_H
#define GPIO_TP_H

/* Returns 0 on success; on failure tp_toggle() becomes a no-op. */
int tp_init(const int *pins, int npins);
void tp_toggle(int pin);
void tp_release(void);

#endif /* GPIO_TP_H */
