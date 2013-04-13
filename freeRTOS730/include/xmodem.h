#ifndef XMODEM_H_INCLUDED
#define XMODEM_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif


int8_t xmodemReceive(uint8_t *dest, uint16_t dest_size);

int8_t xmodemTransmit(uint8_t *src, uint16_t src_size);


#ifdef __cplusplus
}
#endif

#endif
