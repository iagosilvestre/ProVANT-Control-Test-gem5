#ifndef FRAME_H
#define FRAME_H

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

typedef	uint8_t	u8;
typedef	int8_t	i8;
typedef	uint16_t u16;
typedef	int16_t i16;
typedef int32_t	i32;
typedef uint32_t	u32;

typedef struct _frame {
	u8 header;
	u8 end;
	u8 escape;
	u8 data[100]; /* frame message data */
	i32 data_size; /* size of _data */
	// i32 data_alloc_size; #<{(| alloced size of buffer |)}>#
	u8 buffer[100]; /* final buffer to send the frame */
	i32 buffer_size; /* size of buffer */
	// i32 buffer_alloc_size; #<{(| alloced size of buffer |)}>#
	u16 cksum; /* calculated checksum */
	i32 current;
	u8 status; /* package verified and checksum ok? */
	u8 complete; /* is package done? has _end already been added? */
} Frame;


Frame frame_create();
void frame_setData(Frame *frame, u8 *data, i32 size);
u16 checksum(u8 *data, int count);
void frame_addEnd(Frame *frame);
void frame_addHeader(Frame *frame);
void frame_addByte(Frame *frame, u8 byte);
void frame_addBytes(Frame *frame, u8 *bytes, i32 size);
void frame_addBytes2Data(Frame *frame, u8 *bytes, i32 size);
void frame_addFloat(Frame *frame, float num);
float frame_getFloat(Frame *frame);
void frame_addDouble(Frame *frame, double num);
double frame_getDouble(Frame *frame);
void frame_addInt(Frame *frame, i32 num);
i32 frame_getInt(Frame *frame);
void frame_addData2Buffer(Frame *frame);
void frame_addBytes2Buffer(Frame *frame, u8 *bytes, i32 size);
void frame_addChecksum(Frame *frame);
void frame_build(Frame *frame);
void frame_build2(Frame *frame, u8 *data, i32 size);
i32 frame_insertEscape(Frame *frame);
i32 frame_insertEscape2(Frame *frame, i32 size);
u8 frame_isEscapable(Frame *frame, u8 byte);
i32 frame_removeEscape(Frame *frame);
i32 frame_removeEscape2(Frame *frame, i32 size);
u8 frame_unbuild(Frame *frame);
void frame_copyBuffer2data(Frame *frame);
void frame_clear(Frame *frame);

#endif 
