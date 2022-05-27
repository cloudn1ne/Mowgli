/*
 * File      : ringbuffer.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2012, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2012-09-30     Bernard      first version.
 * 2013-05-08     Grissiom     reimplement
 */

#include "stm32f1xx_hal.h"
#include "ringbuffer.h"
#include <string.h>

#define ASSERT(EX)                                                         \

__inline enum ringbuffer_state  ringbuffer_status(struct  ringbuffer *rb)
{
	if (rb->read_index == rb->write_index) {
		if (rb->read_mirror == rb->write_mirror)
			return RT_RINGBUFFER_EMPTY;
		else
			return RT_RINGBUFFER_FULL;
	}
	return RT_RINGBUFFER_HALFFULL;
}

/** return the size of data in rb */
uint16_t  ringbuffer_data_len(struct  ringbuffer *rb)
{
	switch ( ringbuffer_status(rb)) {
	case RT_RINGBUFFER_EMPTY:
		return 0;
	case RT_RINGBUFFER_FULL:
		return rb->buffer_size;
	case RT_RINGBUFFER_HALFFULL:
	default:
		if (rb->write_index > rb->read_index)
			return rb->write_index - rb->read_index;
		else
			return rb->buffer_size - (rb->read_index - rb->write_index);
	};
}

/** return the free space size of rb */
uint16_t  ringbuffer_free_len(struct  ringbuffer *rb)
{
	return rb->buffer_size - ringbuffer_data_len(rb);
}

/**
 * put a block of data into ring buffer
 */
uint32_t  ringbuffer_put(struct  ringbuffer *rb,
                            const uint8_t     *ptr,
                            uint16_t           length)
{
    uint16_t size;

    ASSERT(rb != NULL);

    /* whether has enough space */
    size =  ringbuffer_empty_space(rb);

    /* no space */
    if (size == 0)
        return 0;

    /* drop some data */
    if (size < length)
        length = size;

    if (rb->buffer_size - rb->write_index > length)
    {
        /* read_index - write_index = empty space */
        memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rb->write_index += length;
        return length;
    }

    memcpy(&rb->buffer_ptr[rb->write_index],
           &ptr[0],
           rb->buffer_size - rb->write_index);
    memcpy(&rb->buffer_ptr[0],
           &ptr[rb->buffer_size - rb->write_index],
           length - (rb->buffer_size - rb->write_index));

    /* we are going into the other side of the mirror */
    rb->write_mirror = ~rb->write_mirror;
    rb->write_index = length - (rb->buffer_size - rb->write_index);

    return length;
}

/**
 * put a block of data into ring buffer
 *
 * When the buffer is full, it will discard the old data.
 */
uint32_t ringbuffer_put_force(struct ringbuffer *rb, const uint8_t *ptr, uint16_t length)
{
	enum ringbuffer_state old_state;

	ASSERT(rb != NULL);

	old_state = ringbuffer_status(rb);

	if (length > rb->buffer_size)
		length = rb->buffer_size;

	if (rb->buffer_size - rb->write_index > length) {
		/* read_index - write_index = empty space */
		memcpy(&rb->buffer_ptr[rb->write_index], ptr, length);
		/* this should not cause overflow because there is enough space for
		 * length of data in current mirror */
		rb->write_index += length;

		if (old_state == RT_RINGBUFFER_FULL)
			rb->read_index = rb->write_index;

		return length;
	}

	memcpy(&rb->buffer_ptr[rb->write_index], &ptr[0], rb->buffer_size - rb->write_index);
	memcpy(&rb->buffer_ptr[0], &ptr[rb->buffer_size - rb->write_index],
			length - (rb->buffer_size - rb->write_index));

	/* we are going into the other side of the mirror */
	rb->write_mirror = ~rb->write_mirror;
	rb->write_index = length - (rb->buffer_size - rb->write_index);

	if (old_state == RT_RINGBUFFER_FULL) {
		rb->read_mirror = ~rb->read_mirror;
		rb->read_index = rb->write_index;
	}

	return length;
}

/**
 *  get data from ring buffer
 */
uint32_t  ringbuffer_get(struct  ringbuffer *rb,
                            uint8_t           *ptr,
                            uint16_t           length)
{
    uint32_t size;

    ASSERT(rb != NULL);

    /* whether has enough data  */
    size =  ringbuffer_data_len(rb);

    /* no data */
    if (size == 0)
        return 0;

    /* less data */
    if (size < length)
        length = size;

    if (rb->buffer_size - rb->read_index > length)
    {
        /* copy all of data */
        memcpy(ptr, &rb->buffer_ptr[rb->read_index], length);
        /* this should not cause overflow because there is enough space for
         * length of data in current mirror */
        rb->read_index += length;
        return length;
    }

    memcpy(&ptr[0],
           &rb->buffer_ptr[rb->read_index],
           rb->buffer_size - rb->read_index);
    memcpy(&ptr[rb->buffer_size - rb->read_index],
           &rb->buffer_ptr[0],
           length - (rb->buffer_size - rb->read_index));

    /* we are going into the other side of the mirror */
    rb->read_mirror = ~rb->read_mirror;
    rb->read_index = length - (rb->buffer_size - rb->read_index);

    return length;
}

/**
 * put a character into ring buffer
 */
uint32_t  ringbuffer_putchar(struct  ringbuffer *rb, const uint8_t ch)
{
    ASSERT(rb != NULL);

    /* whether has enough space */
	if (! ringbuffer_empty_space(rb))
		return 0;

	rb->buffer_ptr[rb->write_index] = ch;

	/* flip mirror */
	if (rb->write_index == rb->buffer_size - 1) {
		rb->write_mirror = ~rb->write_mirror;
		rb->write_index = 0;
	} else {
		rb->write_index++;
	}

	return 1;
}

/**
 * put a character into ring buffer
 *
 * When the buffer is full, it will discard one old data.
 */
uint32_t  ringbuffer_putchar_force(struct  ringbuffer *rb, const uint8_t ch)
{
    enum  ringbuffer_state old_state;

    ASSERT(rb != NULL);

    old_state =  ringbuffer_status(rb);

    rb->buffer_ptr[rb->write_index] = ch;

    /* flip mirror */
    if (rb->write_index == rb->buffer_size-1)
    {
        rb->write_mirror = ~rb->write_mirror;
        rb->write_index = 0;
        if (old_state == RT_RINGBUFFER_FULL)
        {
            rb->read_mirror = ~rb->read_mirror;
            rb->read_index = rb->write_index;
        }
    }
    else
    {
        rb->write_index++;
        if (old_state == RT_RINGBUFFER_FULL)
            rb->read_index = rb->write_index;
    }

    return 1;
}

/**
 * get a character from a ringbuffer
 */
uint32_t  ringbuffer_getchar(struct  ringbuffer *rb, uint8_t *ch)
{
    ASSERT(rb != NULL);

    /* ringbuffer is empty */
    if (! ringbuffer_data_len(rb))
        return 0;

    /* put character */
    *ch = rb->buffer_ptr[rb->read_index];

    if (rb->read_index == rb->buffer_size-1)
    {
        rb->read_mirror = ~rb->read_mirror;
        rb->read_index = 0;
    }
    else
    {
        rb->read_index++;
    }

    return 1;
}

void  ringbuffer_flush(struct  ringbuffer *rb)
{
    ASSERT(rb != NULL);

    /* initialize read and write index */
    rb->read_mirror = rb->read_index = 0;
    rb->write_mirror = rb->write_index = 0;
}

void  ringbuffer_init(struct  ringbuffer *rb,
                        uint8_t           *pool,
                        int16_t            size)
{
    ASSERT(rb != NULL);
    ASSERT(size > 0);

    /* initialize read and write index */
    rb->read_mirror = rb->read_index = 0;
    rb->write_mirror = rb->write_index = 0;

    /* set buffer pool and size */
    rb->buffer_ptr = pool;
    rb->buffer_size = size; //ALIGN_DOWN(size, ALIGN_SIZE);
}

