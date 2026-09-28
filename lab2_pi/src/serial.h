#ifndef SERIAL_H
#define SERIAL_H

/* Opens dev raw 8N1 at 115200, non-blocking. Returns fd or -1. */
int serial_open(const char *dev);

#endif /* SERIAL_H */
