#ifndef _FT_GPU_H_
#define _FT_GPU_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Definitions used for FT800 co-processor command buffer */
#define FT_DL_SIZE   		 (0x2000)   // 8kB Display List buffer size
#define FT_CMD_FIFO_SIZE     (0x1000)	// 4kB co-processor FIFO size
#define FT_CMD_SIZE          (4)		// 4 byte per co-processor and Display List command of EVE
/* Definition used for FT800 co-processor Display List Buffer */


#define FT800_VERSION		 "1.9.0"

// Chapter numbers refer to the FT800 Programmers Guide.

// 1.4 Definitions of co-processor widget OPTIONS
#define OPT_3D               0x0000
#define OPT_RGB565           0x0000
#define OPT_MONO             0x0001
#define OPT_NODL             0x0002
#define OPT_FLAT             0x0100
#define OPT_SIGNED           0x0100
#define OPT_CENTERX          0x0200
#define OPT_CENTERY          0x0400
#define OPT_CENTER           0x0600
#define OPT_RIGHTX           0x0800
#define OPT_NOBACK           0x1000
#define OPT_NOTICKS          0x2000
#define OPT_NOHM             0x4000
#define OPT_NOPOINTER        0x4000
#define OPT_NOSECS           0x8000
#define OPT_NOHANDS          0xC000


// 4.4 ALPHA_FUNC test list
#define NEVER                0x00
#define LESS                 0x01
#define LEQUAL               0x02
#define GREATER              0x03
#define GEQUAL               0x04
#define EQUAL                0x05
#define NOTEQUAL             0x06
#define ALWAYS               0x07


// 4.5 BEGIN graphics primitives
#define BITMAPS              0x01
#define POINTS               0x02
#define LINES                0x03
#define LINE_STRIP           0x04
#define EDGE_STRIP_R         0x05
#define EDGE_STRIP_L         0x06
#define EDGE_STRIP_A         0x07
#define EDGE_STRIP_B         0x08
#define RECTS                0x09


// 4.7 BITMAP_LAYOUT format list
#define ARGB1555             0x00
#define L1                   0x01
#define L4                   0x02
#define L8                   0x03
#define RGB332               0x04
#define ARGB2                0x05
#define ARGB4                0x06
#define RGB565               0x07
#define PALETTED             0x08
#define TEXT8X8              0x09
#define TEXTVGA              0x0A
#define BARGRAPH             0x0B


// 4.8 BITMAP_SIZE definitions
#define NEAREST              0x00
#define BILINEAR             0x01
#define BORDER               0x00
#define REPEAT               0x01


// 4.16 BLEND_FUNC value definitions
#define ZERO                 0x00
#define ONE                  0x01
#define SRC_ALPHA            0x02
#define DST_ALPHA            0x03
#define ONE_MINUS_SRC_ALPHA  0x04
#define ONE_MINUS_DST_ALPHA  0x05


// 4.40 STENCIL_OP stencil test actions
#define ZERO                 0x00
#define KEEP                 0x01
#define REPLACE              0x02
#define INCR                 0x03
#define DECR                 0x04
#define INVERT               0x05


// Register 23 REG_DLSWAP Definition
#define DLSWAP_DONE          0x00
#define DLSWAP_LINE          0x01 // potential for frame tearing
#define DLSWAP_FRAME         0x02 // preferred


// Register 44 REG_TOUCH_MODE Definition
#define TOUCHMODE_OFF        0x00
#define TOUCHMODE_ONESHOT    0x01
#define TOUCHMODE_FRAME      0x02
#define TOUCHMODE_CONTINUOUS 0x03
#define NIL_TOUCH_Y         (0x8000)
#define NIL_TOUCH_XY        (0x80008000)

// Register 52 REG_PLAYBACK_FORMAT Definitions
#define LINEAR_SAMPLES       0x00
#define ULAW_SAMPLES         0x01
#define ADPCM_SAMPLES        0x02


// Register 63 & 65 REG_INT_MASK and REG_INT_FLAGS Definitions
#define INT_SWAP             0x01
#define INT_TOUCH            0x02
#define INT_TAG              0x04
#define INT_SOUND            0x08
#define INT_PLAYBACK         0x10
#define INT_CMDEMPTY         0x20
#define INT_CMDFLAG          0x40
#define INT_CONVCOMPLETE     0x80

/***************************************/

// Co-processor engine - Command Encoding

// These commands begin and finish the display list:
#define CMD_DLSTART          0xFFFFFF00 // 4294967040UL - start a new display list
#define CMD_SWAP             0xFFFFFF01 // 4294967041UL - swap the current display list


// Commands to draw graphics objects:
#define CMD_BGCOLOR          0xFFFFFF09 // 4294967049UL - set the background colour
#define CMD_FGCOLOR          0xFFFFFF0A // 4294967050UL - set the foreground colour
#define CMD_GRADIENT         0xFFFFFF0B // 4294967051UL - draw a smooth colour gradient
#define CMD_TEXT             0xFFFFFF0C // 4294967052UL // - draw text

#define CMD_BUTTON           0xFFFFFF0D // 4294967053UL // - draw a button
#define CMD_KEYS             0xFFFFFF0E // 4294967054UL // - draw a row of keys
#define CMD_GRADCOLOR        0xFFFFFF34 // 4294967092UL // - set the 3D effects for CMD_BUTTON and CMD_KEYS highlight colour

#define CMD_PROGRESS         0xFFFFFF0F // 4294967055UL // - draw a progress bar
#define CMD_SLIDER           0xFFFFFF10 // 4294967056UL // - draw a slider
#define CMD_SCROLLBAR        0xFFFFFF11 // 4294967057UL // - draw a scroll bar
#define CMD_TOGGLE           0xFFFFFF12 // 4294967058UL // - draw a toggle switch
#define CMD_GAUGE            0xFFFFFF13 // 4294967059UL // - draw a gauge
#define CMD_CLOCK            0xFFFFFF14 // 4294967060UL // - draw an analogue clock
#define CMD_DIAL             0xFFFFFF2D // 4294967085UL // - draw a rotary dial control
#define CMD_NUMBER           0xFFFFFF2E // 4294967086UL // - draw a decimal number


// Commands to operate on memory:
#define CMD_MEMCRC           0xFFFFFF18 // 4294967064UL // - compute a CRC-32 for memory
#define CMD_MEMWRITE         0xFFFFFF1A // 4294967066UL // - write bytes into memory
#define CMD_MEMSET           0xFFFFFF1B // 4294967067UL // - fill memory with a byte value
#define CMD_MEMZERO          0xFFFFFF1C // 4294967068UL // - write zero to a block of memory
#define CMD_MEMCPY           0xFFFFFF1D // 4294967069UL // - copy a block of memory
#define CMD_APPEND           0xFFFFFF1E // 4294967070UL // - append memory to display list


// Commands for loading image data into FT800 memory:
#define CMD_INFLATE          0xFFFFFF22 // 4294967074UL // - inflate data into memory
#define CMD_GETPTR           0xFFFFFF23 // 4294967075UL // - Get the end memory address of inflated data
#define CMD_LOADIMAGE        0xFFFFFF24 // 4294967076UL // - load a JPEG image


// Commands for setting the bitmap transform matrix:
#define CMD_BITMAP_TRANSFORM 0xFFFFFF21 // 4294967073UL // - bitmap transform A-F
#define CMD_LOADIDENTITY     0xFFFFFF26 // 4294967078UL // - set the current matrix to identity
#define CMD_TRANSLATE        0xFFFFFF27 // 4294967079UL // - apply a translation to the current matrix
#define CMD_SCALE            0xFFFFFF28 // 4294967080UL // - apply a scale to the current matrix
#define CMD_ROTATE           0xFFFFFF29 // 4294967081UL // - apply a rotation to the current matrix
#define CMD_SETMATRIX        0xFFFFFF2A // 4294967082UL // - write the current matrix as a bitmap transform
#define CMD_GETMATRIX        0xFFFFFF33 // 4294967091UL // - retrieves the current matrix coefficients


// Touch Commands commands:
#define CMD_CALIBRATE        0xFFFFFF15 // 4294967061UL // - execute the touch screen calibration routine
#define CMD_TOUCH_TRANSFORM  0xFFFFFF20 // 4294967072UL // - touch Transform A-F
#define CMD_TRACK            0xFFFFFF2C // 4294967084UL // - track touches for a graphics object


// Other commands:
#define CMD_LOGO             0xFFFFFF31 // 4294967089UL // - play device logo animation
#define CMD_COLDSTART        0xFFFFFF32 // 4294967090UL // - set co-processor engine state to default values
#define CMD_INTERRUPT        0xFFFFFF02 // 4294967042UL // - trigger interrupt INT_CMDFLAG

#define CMD_SPINNER          0xFFFFFF16 // 4294967062UL // - start an animated spinner
#define CMD_STOP             0xFFFFFF17 // 4294967063UL // - stop any spinner, screensaver or sketch
#define CMD_SCREENSAVER      0xFFFFFF2F // 4294967087UL // - start an animated screensaver
#define CMD_SKETCH           0xFFFFFF30 // 4294967088UL // - start a continuous sketch update

#define CMD_SNAPSHOT         0xFFFFFF1F // 4294967071UL // - take a snapshot of the current screen

#define CMD_SETFONT          0xFFFFFF2B // 4294967083UL // - set up a custom font

#define CMD_REGREAD          0xFFFFFF19 // 4294967065UL // - read a register value
#define CMD_GETPROPS         0xFFFFFF25 // 4294967077UL // - read a property value

#define CMD_CRC              0xFFFFFF03 // 4294967043UL // - seems undocumented
#define CMD_HAMMERAUX        0xFFFFFF04 // 4294967044UL // - seems undocumented
#define CMD_MARCH            0xFFFFFF05 // 4294967045UL // - seems undocumented
#define CMD_IDCT             0xFFFFFF06 // 4294967046UL // - seems undocumented
#define CMD_EXECUTE          0xFFFFFF07 // 4294967047UL // - seems undocumented
#define CMD_GETPOINT         0xFFFFFF08 // 4294967048UL // - seems undocumented


/* RAM Register locations */
#define RAM_G                0UL			// Main graphics RAM
#define ROM_FONT             0766524UL		// Font table and bitmap
#define ROM_CHIPID           0786432UL		// FT800 chip identification and revision information: Byte [0:1] Chip ID: “0800” Byte [2:3] Version ID: “0100”
#define ROM_FONT_ADDR        1048572UL		// Font table pointer address
#define RAM_DL               1048576UL		// Display List RAM
#define RAM_PAL              1056768UL		// Palette RAM
#define RAM_REG              1057792UL		// Registers commence
#define RAM_CMD              1081344UL		// Graphics Engine Command Buffer

#define REG_ANALOG           1058104UL
#define REG_ANA_COMP         1058160UL
#define REG_CLOCK            1057800UL
#define REG_CMD_READ         1058020UL
#define REG_CMD_WRITE        1058024UL
#define REG_CMD_DL           1058028UL
#define REG_CPURESET         1057820UL
#define REG_CRC              1058152UL
#define REG_CSPREAD          1057892UL
#define REG_CYA0             1058000UL
#define REG_CYA1             1058004UL
#define REG_CYA_TOUCH        1058100UL
#define REG_DATESTAMP        1058108UL
#define REG_DITHER           1057884UL
#define REG_DLSWAP           1057872UL
#define REG_FRAMES           1057796UL
#define REG_FREQUENCY        1057804UL
#define REG_GPIO             1057936UL
#define REG_GPIO_DIR         1057932UL
#define REG_HCYCLE           1057832UL
#define REG_HOFFSET          1057836UL
#define REG_HSIZE            1057840UL
#define REG_HSYNC0           1057844UL
#define REG_HSYNC1           1057848UL
#define REG_ID               1057792UL
#define REG_INT_EN           1057948UL
#define REG_INT_FLAGS        1057944UL
#define REG_INT_MASK         1057952UL
#define REG_MACRO_0          1057992UL
#define REG_MACRO_1          1057996UL
#define REG_OUTBITS          1057880UL
#define REG_PCLK             1057900UL
#define REG_PCLK_POL         1057896UL
#define REG_PLAY             1057928UL
#define REG_PLAYBACK_FORMAT  1057972UL
#define REG_PLAYBACK_FREQ    1057968UL
#define REG_PLAYBACK_LENGTH  1057960UL
#define REG_PLAYBACK_LOOP    1057976UL
#define REG_PLAYBACK_PLAY    1057980UL
#define REG_PLAYBACK_READPTR 1057964UL
#define REG_PLAYBACK_START   1057956UL
#define REG_PWM_DUTY         1057988UL
#define REG_PWM_HZ           1057984UL
#define REG_RENDERMODE       1057808UL
#define REG_ROMSUB_SEL       1058016UL
#define REG_ROTATE           1057876UL
#define REG_SNAPSHOT         1057816UL
#define REG_SNAPY            1057812UL
#define REG_SOUND            1057924UL
#define REG_SWIZZLE          1057888UL
#define REG_TAG              1057912UL
#define REG_TAG_X            1057904UL
#define REG_TAG_Y            1057908UL
#define REG_TAP_CRC          1057824UL
#define REG_TAP_MASK         1057828UL
#define REG_TOUCH_ADC_MODE   1058036UL
#define REG_TOUCH_CHARGE     1058040UL
#define REG_TOUCH_DIRECT_XY  1058164UL
#define REG_TOUCH_DIRECT_Z1Z2 1058168UL
#define REG_TOUCH_MODE       1058032UL
#define REG_TOUCH_OVERSAMPLE 1058048UL
#define REG_TOUCH_RAW_XY     1058056UL
#define REG_TOUCH_RZ         1058060UL
#define REG_TOUCH_RZTHRESH   1058052UL
#define REG_TOUCH_SCREEN_XY  1058064UL
#define REG_TOUCH_SETTLE     1058044UL
#define REG_TOUCH_TAG        1058072UL
#define REG_TOUCH_TAG_XY     1058068UL
#define REG_TOUCH_TRANSFORM_A 1058076UL
#define REG_TOUCH_TRANSFORM_B 1058080UL
#define REG_TOUCH_TRANSFORM_C 1058084UL
#define REG_TOUCH_TRANSFORM_D 1058088UL
#define REG_TOUCH_TRANSFORM_E 1058092UL
#define REG_TOUCH_TRANSFORM_F 1058096UL
#define REG_TRACKER          1085440UL
#define REG_TRIM             1058156UL
#define REG_VCYCLE           1057852UL
#define REG_VOFFSET          1057856UL
#define REG_VOL_PB           1057916UL
#define REG_VOL_SOUND        1057920UL
#define REG_VSIZE            1057860UL
#define REG_VSYNC0           1057864UL
#define REG_VSYNC1           1057868UL


/******** Command Groups ************/

// 4.3.1 Setting Graphics state
#define BITMAP_SOURCE(addr)         (((uint32_t)0x01<<24)|((uint32_t)(addr)&0x00FFFFFF))
#define CLEAR_COLOR_RGB(red,green,blue) (((uint32_t)0x02<<24)|((uint32_t)((red)&0xFF)<<16)|((uint16_t)((green)&0xFF)<<8)|((blue)&0xFF))
#define CLEAR_COLOR_X11(colour)     (((uint32_t)0x02<<24)|((uint32_t)(pgm_read_byte(&(colour)[0]))<<16)|((uint16_t)(pgm_read_byte(&(colour)[1]))<<8)|(pgm_read_byte(&(colour)[2])))
#define TAG(s)                      (((uint32_t)0x03<<24)|((s)&0xFF))
#define COLOR_RGB(red,green,blue)   (((uint32_t)0x04<<24)|((uint32_t)((red)&0xFF)<<16)|((uint16_t)((green)&0xFF)<<8)|((blue)&0xFF))
#define COLOR_X11(colour)           (((uint32_t)0x04<<24)|((uint32_t)(pgm_read_byte(&(colour)[0]))<<16)|((uint16_t)(pgm_read_byte(&(colour)[1]))<<8)|(pgm_read_byte(&(colour)[2])))
#define BITMAP_HANDLE(handle)       (((uint32_t)0x05<<24)|((handle)&0x1F))
#define CELL(cell)                  (((uint32_t)0x06<<24)|((cell)&0x7F))
#define BITMAP_LAYOUT(format,linestride,height)      (((uint32_t)0x07<<24)|((uint32_t)((format)&0x1F)<<19)|((uint32_t)((linestride)&0x03FF)<<9)|((height)&0x01FF))
#define BITMAP_SIZE(filter,wrapx,wrapy,width,height) (((uint32_t)0x08<<24)|((uint32_t)((filter)&0x01)<<20)|((uint32_t)((wrapx)&0x01)<<19)|((uint32_t)((wrapy)&0x01)<<18)|((uint32_t)((width)&0x01FF)<<9)|((height)&0x01FF))
#define ALPHA_FUNC(func,ref)        (((uint32_t)0x09<<24)|(((func)&0x0007)<<8)|((ref)&0xFF))
#define STENCIL_FUNC(func,ref,mask) (((uint32_t)0x0A<<24)|((uint32_t)((func)&0x0007)<<16)|(((ref)&0x00FF)<<8)|((mask)&0xFF))
#define BLEND_FUNC(src,dst)         (((uint32_t)0x0B<<24)|(((src)&0x07)<<3)|((dst)&0x07))
#define STENCIL_OP(sfail,spass)     (((uint32_t)0x0C<<24)|(((sfail)&0x07)<<3)|((spass)&0x07))
#define POINT_SIZE(size)            (((uint32_t)0x0D<<24)|((size)&0x1FFF))
#define LINE_WIDTH(width)           (((uint32_t)0x0E<<24)|((width)&0x0FFF))
#define CLEAR_COLOR_A(alpha)        (((uint32_t)0x0F<<24)|((alpha)&0xFF))
#define COLOR_A(alpha)              (((uint32_t)0x10<<24)|((alpha)&0xFF))
#define CLEAR_STENCIL(s)            (((uint32_t)0x11<<24)|((s)&0xFF))
#define CLEAR_TAG(s)                (((uint32_t)0x12<<24)|((s)&0xFF))
#define STENCIL_MASK(mask)          (((uint32_t)0x13<<24)|((mask)&0xFF))
#define TAG_MASK(mask)              (((uint32_t)0x14<<24)|((mask)&0x01))
#define BITMAP_TRANSFORM_A(a)       (((uint32_t)0x15<<24)|((uint32_t)(a)&0x0001FFFF))
#define BITMAP_TRANSFORM_B(b)       (((uint32_t)0x16<<24)|((uint32_t)(b)&0x0001FFFF))
#define BITMAP_TRANSFORM_C(c)       (((uint32_t)0x17<<24)|((uint32_t)(c)&0x00FFFFFF))
#define BITMAP_TRANSFORM_D(d)       (((uint32_t)0x18<<24)|((uint32_t)(d)&0x0001FFFF))
#define BITMAP_TRANSFORM_E(e)       (((uint32_t)0x19<<24)|((uint32_t)(e)&0x0001FFFF))
#define BITMAP_TRANSFORM_F(f)       (((uint32_t)0x1A<<24)|((uint32_t)(f)&0x00FFFFFF))
#define SCISSOR_XY(x,y)             (((uint32_t)0x1B<<24)|((uint32_t)((x)&0x01FF)<<9)|((y)&0x01FF))
#define SCISSOR_SIZE(width,height)  (((uint32_t)0x1C<<24)|((uint32_t)((width)&0x03FF)<<10)|((height)&0x03FF))

#define COLOR_MASK(r,g,b,a)         (((uint32_t)0x20<<24)|(((r)&0x01)<<3)|(((g)&0x01)<<2)|(((b)&0x01)<<1)|((a)&0x01))

#define SAVE_CONTEXT()               ((uint32_t)0x22<<24)
#define RESTORE_CONTEXT()            ((uint32_t)0x23<<24)

#define CLEAR(c,s,t)                (((uint32_t)0x26<<24)|(((c)&0x01)<<2)|(((s)&0x01)<<1)|((t)&0x01))

// 4.3.2 Drawing actions
#define BEGIN(prim)                 (((uint32_t)0x1F<<24)|((prim)&0x0F))
#define END()                        ((uint32_t)0x21<<24)
#define VERTEX2F(x,y)               (((uint32_t)0x40<<24)|((uint32_t)((x)&0xFFFF)<<15)|((y)&0xFFFF))
#define VERTEX2II(x,y,handle,cell)  (((uint32_t)0x80<<24)|((uint32_t)((x)&0x01FF)<<21)|((uint32_t)((y)&0x01FF)<<12)|((uint16_t)((handle)&0x1F)<<7)|((cell)&0x7F))

// 4.3.3 Execution control
#define DISPLAY()                    ((uint32_t)0x00<<24)
#define CALL(dest)                  (((uint32_t)0x1D<<24)|((dest)&0xFFFF))
#define JUMP(dest)                  (((uint32_t)0x1E<<24)|((dest)&0xFFFF))
#define RETURN()                     ((uint32_t)0x24<<24)
#define MACRO(m)                    (((uint32_t)0x25<<24)|((m)&0x01))


/* allow FT_GPU_CoCmd_*Color commands to use X11 colours stored in PROGMEM - only used colours are actually stored by linker */
#define X11(colour)					(((uint32_t)(pgm_read_byte(&(colour)[0]))<<16)|((uint16_t)(pgm_read_byte(&(colour)[1]))<<8)|(pgm_read_byte(&(colour)[2])))

#define NOTE(n, sharp)              (((n) - 'C') + ((sharp) * 128))


#define FT_GPU_NUMCHAR_PERFONT (128)
#define FT_GPU_FONT_TABLE_SIZE (148)

/* FT800 font table structure */
/* Font table address in ROM can be found by reading the address from 0xFFFFC location. */
/* 16 font tables are present at the address read from location 0xFFFFC */
typedef struct FT_GPU_Fonts
{
	/* All the values are in bytes */
	/* Width of each character font from 0 to 127 */
	ft_uint8_t	FontWidth[FT_GPU_NUMCHAR_PERFONT];
	/* Bitmap format of font wrt bitmap formats supported by FT800 - L1, L4, L8 */
	ft_uint32_t	FontBitmapFormat;
	/* Font line stride in FT800 ROM */
	ft_uint32_t	FontLineStride;
	/* Font width in pixels */
	ft_uint32_t	FontWidthInPixels;
	/* Font height in pixels */
	ft_uint32_t	FontHeightInPixels;
	/* Pointer to font graphics raw data */
	ft_uint32_t	PointerToFontGraphicsData;
}FT_GPU_Fonts_t;


#ifdef __cplusplus
}
#endif

#endif /* #ifndef _FT_GPU_H_ */

/* Nothing beyond this */
