#include "c_common_frame.h"	
#include "c_common_ringbuffer.h"	
#define FALSE 0
#define TRUE 0

Frame frame_create()
{
	Frame frame;
	frame.header = 0x7E;
	frame.end = (0x7E);
	frame.escape = (0x7D);
	/* frame.data = NULL; */
	frame.data_size = 0;
	/* frame.data_alloc_size = 0; */
	/* frame.buffer = NULL; */
	frame.buffer_size = 0;
	/* frame.buffer_alloc_size = 0; */
	frame.cksum = (0);
	frame.current = 0;
	frame.status = FALSE;
	frame.complete = FALSE;

	return frame;
}
 
/*  changes current data buffer. DELETES old data. */
void frame_setData(Frame *frame, u8 *data, i32 size)
{
	memcpy(frame->data,data,size);
	frame->data_size = size;
}

/* fletcher-16 checksum implementation from wikipedia */
u16 checksum(u8 *data, int count)
{
	u16 sum1 = 0;
	u16 sum2 = 0;
	i32 index;

	for( index = 0; index < count; ++index )
	{
		sum1 = (sum1 + data[index]) % 255;
		sum2 = (sum2 + sum1) % 255;
	}

	return (sum2 << 8) | sum1;
}

void frame_addEnd(Frame *frame)
{
	frame_addBytes2Buffer(frame,&frame->end,1);
	frame->complete = TRUE;
}

void frame_addHeader(Frame *frame)
{
	frame_addBytes2Buffer(frame,&frame->header,1);
}

void frame_addByte(Frame *frame, u8 byte)
{
	frame_addBytes2Buffer(frame, &byte,1);
}

void frame_addBytes(Frame *frame, u8 *bytes, i32 size)
{
	frame_addBytes2Buffer(frame, bytes, size);
}

void frame_addBytes2Data(Frame *frame, u8 *bytes, i32 size)
{
	memcpy(frame->data + frame->data_size, bytes, size);
	frame->data_size += size;
}

void frame_addFloat(Frame *frame, float num) 
{
	u8 size = sizeof(float);
	u8 buf[4];
	memcpy(buf,&num,size);
	frame_addBytes2Data(frame,buf,size);
}

float frame_getFloat(Frame *frame)
{
	u8 size = sizeof(float);
	float num;
	memcpy(&num,frame->data + frame->current,size);
	frame->current += size;
	return num;
}

void frame_addDouble(Frame *frame, double num)
{
	u8 size = sizeof(double);
	u8 buf[8];
	memcpy(buf,&num,size);
	frame_addBytes2Data(frame,buf,size);
}

double frame_getDouble(Frame *frame)
{
	u8 size = sizeof(double);
	double num;
	memcpy(&num,frame->data + frame->current,size);
	frame->current += size;
	return num;
}

void frame_addInt(Frame *frame, i32 num)
{
	u8 size = sizeof(i32);
	u8 buf[64];
	memcpy(buf,&num,size);
	frame_addBytes2Data(frame,buf,size);
}

i32 frame_getInt(Frame *frame)
{
	u8 size = sizeof(i32);
	i32 num;
	memcpy(&num,frame->data + frame->current,size);
	frame->current += size;
	return num;
}

/* add data buffer bytes as is to the buffer */
void frame_addData2Buffer(Frame *frame)
{
	frame_addBytes2Buffer(frame,frame->data,frame->data_size);
}

/* add bytes to buffer without adding escape flags */
void frame_addBytes2Buffer(Frame *frame, u8 *bytes, i32 size)
{
	memcpy(frame->buffer + frame->buffer_size, bytes, size);
	frame->buffer_size += size;
}

/* calculate and add the checksum to the 'data' buffer */
void frame_addChecksum(Frame *frame)
{
	/* header and end of frame are not considered in checksum */
	frame->cksum = checksum(frame->data,frame->data_size);
	u8 buf[2] ;
	memcpy(buf,&frame->cksum,2);
	frame_addBytes2Data(frame,buf,2);
}

/* contruct the frame */
void frame_build(Frame *frame)
{
	frame_addHeader(frame);
	frame_addChecksum(frame);
	frame_insertEscape(frame);
	frame_addData2Buffer(frame);
	frame_addEnd(frame);
	frame->status = TRUE;
}

/* contruct the frame with the provided data */
void frame_build2(Frame *frame, u8 *data, i32 size)
{
	//assert(size > 0);
	//assert(data != NULL);
	/* to secure in case of static array passed */
	frame_setData(frame,data,size);
	frame_build(frame);
}

/* Calculates the frame checksum and check if matches the checksum in the
 * frame the buffer must be complete, a read with status = 1 is a requirement
 * to this function. This function assumes you already removed escape bytes
 * and checksum from the data(_data array).
 */
u8 frame_check(Frame *frame)
{
	return frame->status = frame->cksum == checksum(frame->data,frame->data_size);
}

i32 frame_insertEscape(Frame *frame)
{
	frame->data_size = frame_insertEscape2(frame, frame->data_size);
}

/* check and insert escapes on data if any bytes matches header or end */
i32 frame_insertEscape2(Frame *frame, i32 size)
{
	/* Create and initialize ring buffer */
	ring_buffer_t ring_buffer;
	ring_buffer_init(&ring_buffer);
	u8 *data = frame->data;

	for(i32 i = 0; i< size; i++){
		u8 c = data[i];
		u8 aux;
		if (frame_isEscapable(frame, c)) {
			aux = frame->escape;
			ring_buffer_queue(&ring_buffer, aux);
			aux = c^0x20;
			ring_buffer_queue(&ring_buffer, aux);
		} else {
			aux = c;
			ring_buffer_queue(&ring_buffer, aux);
		}
	}
	int n = ring_buffer_num_items(&ring_buffer);
	char aux[1];
	for (i32 i = 0; ring_buffer_dequeue(&ring_buffer, aux) > 0; i++)
		data[i] = *aux;

	return n;
}

u8 frame_isEscapable(Frame *frame, u8 byte)
{
	return (byte == frame->header || byte == frame->end || byte == frame->escape);
}

i32 frame_removeEscape(Frame *frame)
{
	frame->data_size = frame_removeEscape2(frame, frame->data_size);
}

/* check and insert escapes on data if any bytes matches header or end */
i32 frame_removeEscape2(Frame *frame, i32 size)
{
	/* Create and initialize ring buffer */
	ring_buffer_t ring_buffer;
	ring_buffer_init(&ring_buffer);

	u8 *data = frame->data;
	u8 aux[1];
	for(i32 i = 0; i < size - 1; i++) {
		if (data[i] == frame->escape) {
			i++; //jump the escape byte
			*aux = data[i]^0x20;
			ring_buffer_queue(&ring_buffer, *aux);
		} else {
			*aux = data[i];
			ring_buffer_queue(&ring_buffer, *aux);
		}
	}
	/* The for above checked until n-1, the last one is included in
	 * case n-2 was not an escape
	 */
	if (data[size-2] != frame->escape) {
		*aux = data[size-1];
		ring_buffer_queue(&ring_buffer, *aux);
	}

	int n = ring_buffer_num_items(&ring_buffer);
	for (i32 i = 0; ring_buffer_dequeue(&ring_buffer, (char*)aux) > 0; i++)
		frame->data[i] = *aux;

	return n;
}

u8 frame_unbuild(Frame *frame)
{
	//if(frame->complete==0) return 0;
	frame_copyBuffer2data(frame);
	frame_removeEscape(frame);
	memcpy(&frame->cksum,frame->data+frame->data_size-2,2);
	frame->data_size -= 2;
	frame->buffer_size = 0;

	return frame_check(frame);
}

/* Copy data from buffer with checksum. checksum might have escape bytes, so 
 * is necessary to remove escape bytes and then remove checksum to _cksum.
 */
void frame_copyBuffer2data(Frame *frame)
{
	frame->data_size = frame->buffer_size - 2;
	memcpy(frame->data, frame->buffer + 1, frame->data_size);
}

/* delete current frame and restarts the frame */
void frame_clear(Frame *frame)
{
	frame->status = FALSE;
	frame->complete = FALSE;
	frame->buffer_size = 0;
	frame->data_size = 0;
	frame->cksum = 0;
	frame->current = 0;
}

