#include "c_common_uart.h"
#include "c_io_protocolP2P.h"
#include "c_common_utils.h"

//#define BAUDRATE 921600//576000
#define TRUE 1
#define FALSE 0


void c_io_protocolP2P_init(USART_TypeDef* usartx, int baudrate)
{
	if (usartx == USART1)
		c_common_usart1_init(baudrate);
	else if (usartx == USART2)
		c_common_usart2_init(baudrate);
	else if (usartx == USART3)
		c_common_usart3_init(baudrate);
	else if (usartx == USART6)
		c_common_usart6_init(baudrate);
}

Frame c_io_protocolP2P_receive(USART_TypeDef* usartx)
{
	Frame frame = frame_create();
	u8 header = frame.header;
	u8 end = frame.end;
	i32  status = 0;
	u8 last = 'a';
	int fim  = FALSE;
	//unsigned long now = 0, timeOut;
	//now = c_common_utils_millis();
	//timeOut = 3 + now;
	while(!fim /*&& (long)(now - timeOut) <= 0*/)
	{
		//now = c_common_utils_millis();
		if(c_common_usart_available2(usartx))
		{
			char b[1];
			b[0] = c_common_usart_read(usartx);

			switch(status)
			{
				case 1:
					if (b[0] == last && last == header && header == end)
						break;
					if (b[0] != end) frame_addByte(&frame,b[0]);
					if (b[0] == end)
					{
							frame_addEnd(&frame);
							status = 0;
							fim = TRUE;
					}
					break;
				case 0:
					if (b[0] == header)
					{
						frame_addHeader(&frame);
						status = 1;
					}
					break;
			}
			last = b[0];
		}
	}
	//if((long)(now - timeOut) <= 0)	frame.complete = 0;
	return frame;
}


void c_io_protocolP2P_send(USART_TypeDef* usartx, Frame *frame)
{
	int i;
	for (i = 0; i < frame->buffer_size; ++i) 
		c_common_usart_putchar(usartx, frame->buffer[i]);
}

