#ifndef SOUND_H
#define SOUND_H

#include "sound_blargg.h"

#define SGCNT0_H 0x82
#define FIFOA_L 0xa0
#define FIFOA_H 0xa2
#define FIFOB_L 0xa4
#define FIFOB_H 0xa6

#define BLIP_BUFFER_ACCURACY 16
#define BLIP_PHASE_BITS 8
#define BLIP_WIDEST_IMPULSE_ 16
#define BLIP_BUFFER_EXTRA_ 18
#define BLIP_RES 256
#define BLIP_RES_MIN_ONE 255
#define BLIP_SAMPLE_BITS 30
#define BLIP_READER_DEFAULT_BASS 9
#define BLIP_DEFAULT_LENGTH 250		/* 1/4th of a second */

#define BUFS_SIZE 3
#define STEREO 2

#define	CLK_MUL	GB_APU_OVERCLOCK
#define CLK_MUL_MUL_2 8
#define CLK_MUL_MUL_4 16
#define CLK_MUL_MUL_6 24
#define CLK_MUL_MUL_8 32
#define CLK_MUL_MUL_15 60
#define CLK_MUL_MUL_32 128
#define DAC_BIAS 7

#define PERIOD_MASK 0x70
#define SHIFT_MASK 0x07

#define PERIOD2_MASK 0x1FFFF

#define BANK40_MASK 0x40
#define BANK_SIZE 32
#define BANK_SIZE_MIN_ONE 31
#define BANK_SIZE_DIV_TWO 16

/* 11-bit frequency in NRx3 and NRx4 (takes the struct pointer like the others above) */
#define GB_OSC_FREQUENCY(self) ((((self)->regs[4] & 7) << 8) + (self)->regs[3])

#define	WAVE_TYPE	0x100
#define NOISE_TYPE	0x200
#define MIXED_TYPE	WAVE_TYPE | NOISE_TYPE
#define TYPE_INDEX_MASK	0xFF


/* Declares the reader's local working variables.  Must be placed at the
 * top of the block (C89 forbids mixing declarations with statements);
 * the read pointer is NOT const because BLIP_READER_ADJ_ rebinds it. */
#define BLIP_READER_DECL( name ) \
        const int32_t * name##_reader_buf; \
        int32_t name##_reader_accum

/* Begins reading from buffer.  Issued after BLIP_READER_DECL, after any
 * statements that compute (blip_buffer).  Name should be unique to the
 * current block. */
#define BLIP_READER_BEGIN( name, blip_buffer ) do { \
        name##_reader_buf = (blip_buffer).buffer_; \
        name##_reader_accum = (blip_buffer).reader_accum_; \
        } while (0)

/* Advances to next sample*/
#define BLIP_READER_NEXT( name, bass ) \
        (void) (name##_reader_accum += *name##_reader_buf++ - (name##_reader_accum >> (bass)))

/* Ends reading samples from buffer. The number of samples read must now be removed
   using Blip_Buffer::remove_samples(). */
#define BLIP_READER_END( name, blip_buffer ) \
        (void) ((blip_buffer).reader_accum_ = name##_reader_accum)

#define BLIP_READER_ADJ_( name, offset ) (name##_reader_buf += offset)

#define BLIP_READER_NEXT_IDX_( name, idx ) {\
        name##_reader_accum -= name##_reader_accum >> BLIP_READER_DEFAULT_BASS;\
        name##_reader_accum += name##_reader_buf [(idx)];\
}

#define BLIP_READER_NEXT_RAW_IDX_( name, idx ) {\
        name##_reader_accum -= name##_reader_accum >> BLIP_READER_DEFAULT_BASS; \
        name##_reader_accum += *(int32_t const*) ((char const*) name##_reader_buf + (idx)); \
}

/* Portable on all targets. The previous non-x86 form `(int16_t)in != in`
 * relied on implementation-defined narrowing semantics; modern gcc/clang
 * compile both forms to the same code on every supported target. */
#define BLIP_CLAMP_( in ) ((in) < -0x8000 || 0x7FFF < (in))

/* Clamp sample to int16_t range */
#define BLIP_CLAMP( sample, out ) { if ( BLIP_CLAMP_( (sample) ) ) (out) = ((sample) >> 24) ^ 0x7FFF; }
/* Macros below took an implicit `this` in their original C++ form; the C
 * port passes the struct pointer explicitly. */
#define GB_ENV_DAC_ENABLED(self) ((self)->regs[2] & 0xF8)	/* Non-zero if DAC is enabled*/
#define GB_NOISE_PERIOD2_INDEX(self)	((self)->regs[3] >> 4)
#define GB_NOISE_PERIOD2(self, base)	((base) << GB_NOISE_PERIOD2_INDEX(self))
#define GB_NOISE_LFSR_MASK(self)	(((self)->regs[3] & 0x08) ? ~0x4040 : ~0x4000)
#define GB_WAVE_DAC_ENABLED(self) ((self)->regs[0] & 0x80)	/* Non-zero if DAC is enabled*/

#define reload_sweep_timer(self) \
        (self)->sweep_delay = ((self)->regs [0] & PERIOD_MASK) >> 4; \
        if ( !(self)->sweep_delay ) \
                (self)->sweep_delay = 8;

void soundSetVolume(float unused);

#ifdef __cplusplus
extern "C" {
#endif

/* Manages muting bitmask. The bits control the following channels: */
/* 0x001 Pulse 1 */
/* 0x002 Pulse 2 */
/* 0x004 Wave */
/* 0x008 Noise */
/* 0x100 PCM 1 */
/* 0x200 PCM 2 */
void soundPause (void);
void soundResume (void);
void soundSetSampleRate(long sampleRate);
void soundReset (void);
/* Frees the heap-allocated blip-buffer storage owned by the audio
 * pipeline (the 3 stereo_buffer entries each hold ~32 KB at 32 kHz /
 * 250 ms = ~96 KB total).  Call from retro_deinit so the storage is
 * returned to the allocator rather than relying on process exit -- the
 * latter accumulates across core load/unload cycles on dynamic-load
 * frontends (Android, RetroArch hot-swap).  Safe to call when buffers
 * are already destroyed; bufs_buffer[] is zeroed after teardown. */
void soundCleanUp (void);
void soundEvent_u8( int gb_addr, uint32_t addr, uint8_t  data );
void soundEvent_u8_parallel(int gb_addr[], uint32_t address[], uint8_t data[]);
void soundEvent_u16( uint32_t addr, uint16_t data );
void soundTimerOverflow( int which );
void process_sound_tick_fn (void);
void soundSaveGameMem(uint8_t **data);
void soundReadGameMem(const uint8_t **data, int version);

#ifdef __cplusplus
}
#endif

extern int SOUND_CLOCK_TICKS;   /* Number of 16.8 MHz clocks between calls to soundTick() */
extern int soundTicks;          /* Number of 16.8 MHz clocks until soundTick() will be called */

#endif /* SOUND_H */
