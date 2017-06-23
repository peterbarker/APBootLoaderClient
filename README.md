# APBootLoaderClient

## Introduction ##

APBootLoader client is an embeddable C implementation of the bootloader protocol commonly found on PixHawk and Cube autopilot systems.

A sample program is supplied which embeds the client.


## Embedding ##

Three functions must be defined by the embedding program:

```c
ssize_t bootloader_write(const uint8_t *bytes, const size_t bytecount, const bl_timeout_t timeout);
ssize_t bootloader_readbyte(uint8_t *byte, const bl_timeout_t timeout);
#ifdef __STDC__
void bootloader_debug(const char  *format, ...);
#else
void bootloader_debug(const char  *format, arg...);
```

`bootloader_readbyte` should return 1 if a byte was successfully returned in the `byte` parameter, 0 otherwise.

`bootloader_write` should return the number of bytes written, and -1 on error.

`bootloader_debug` is called with any debug which the bootloader might wish to supply.

If programming is desired, a callback can be supplied to `bootloader_program`; it will be called with an integer percentage indicating progress.
