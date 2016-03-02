/*
  ============================================================================
   File: G711.H
  ============================================================================

                            UGST/ITU-T G711 MODULE

                          GLOBAL FUNCTION PROTOTYPES

   History:
   10.Dec.91	v1.0	First version <hf@pkinbg.uucp>
   08.Feb.92	v1.1	Non-ANSI prototypes added <tdsimao@venus.cpqd.ansp.br>
   11.Jan.96    v1.2    Fixed misleading prototype parameter names in
                        alaw_expand() and ulaw_compress(); changed to
			smart prototypes <simao@ctd.comsat.com>,
			and <Volker.Springer@eedn.ericsson.se>
   31.Jan.2000  v3.01   [version no.aligned with g711.c] Updated list of
                        compilers for smart prototypes
  ============================================================================
*/
#ifndef G711_defined
#define G711_defined 301

/* Smart function prototypes: for [ag]cc, VaxC, and [tb]cc */
#if !defined(ARGS)
#if (defined(__STDC__) || defined(VMS) || defined(__DECC)  || defined(MSDOS) || defined(__MSDOS__)) || defined (__CYGWIN__) || defined (_MSC_VER)
#define ARGS(s) s
#else
#define ARGS(s) ()
#endif
#endif

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function prototypes */
void  alaw_compress ARGS((uint32_t lseg, int16_t *linbuf, uint8_t *logbuf));
void  alaw_expand ARGS((uint32_t lseg, uint8_t *logbuf, int16_t *linbuf));
void  ulaw_compress ARGS((uint32_t lseg, int16_t *linbuf, uint8_t *logbuf));
void  ulaw_expand ARGS((uint32_t lseg, uint8_t *logbuf, int16_t *linbuf));

void  alaw_compress1 ARGS((int16_t *linval, uint8_t *logval));
void  alaw_expand1 ARGS((uint8_t *logval, int16_t *linval));

#ifdef __cplusplus
}
#endif

#endif
/* .......................... End of G711.H ........................... */
