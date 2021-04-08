/* Ring buffer header file. */

#pragma once

#include <inttypes.h>
#include <stdbool.h>

typedef struct {
	size_t ptr_b;    // pointer to begin (for 8 items 0..7)
	size_t ptr_e;    // pointer to end (for 8 items 0..7)
	uint8_t* data;   // data
	bool empty;      // wheter buffer is empty
	size_t size;     // data size
} ring_buffer;

/* ptr_b points to first byte
 * ptr_e points to byte after last byte
 * This specially implies that is it NOT POSSIBLE to differentiate empty and full buffer.
 * This is why buffer contains special \empty flag.
 */

/* Common situations:
 * full buffer: ptr_b == ptr_e && !empty
 * empty buffer: ptr_b == ptr_e && empty
 * Empty flag must be set when manipulating with ring buffer!
 */

static inline void ring_init(volatile ring_buffer* buf, uint8_t* data, size_t size) {
	buf->ptr_b = 0;
	buf->ptr_e = 0;
	buf->empty = true;
	buf->data = data;
	buf->size = size;
}

static inline bool ring_full(volatile ring_buffer* buf) {
	return (buf->ptr_b == buf->ptr_e) && (!buf->empty);
}

static inline size_t ring_length(volatile ring_buffer* buf) {
	return ((buf->ptr_e - buf->ptr_b) % buf->size) + (ring_full(buf) ? buf->size: 0);
}

static inline bool ring_empty(volatile ring_buffer* buf) {
	return (buf->ptr_b == buf->ptr_e) && (buf->empty);
}

static inline size_t ring_free_space(volatile ring_buffer* buf) {
	return buf->size - ring_length(buf);
}

static inline size_t ring_distance(volatile ring_buffer* buf, size_t first, size_t second) {
	return (second-first) % buf->size;
}

static inline void ring_clear(volatile ring_buffer* buf) {
	buf->ptr_b = buf->ptr_e;
	buf->empty = true;
}

static inline bool ring_add_byte(volatile ring_buffer* buf, uint8_t data) {
	if (ring_full(buf))
		return false;

	buf->data[buf->ptr_e] = data;
	buf->ptr_e = (buf->ptr_e + 1) % buf->size;
	buf->empty = false;
	return true;
}

static inline bool ring_add_bytes(volatile ring_buffer* buf, uint8_t* data, size_t size) {
	if (ring_free_space(buf) < size)
		return false;

	for (size_t i = 0; i < size; i++)
		buf->data[(buf->ptr_e+i) % buf->size] = data[i];
	buf->ptr_e = (buf->ptr_e + size) % buf->size;
	buf->empty = false;
	return true;
}

static inline uint8_t ring_remove_byte(volatile ring_buffer* buf) {
	uint8_t result;
	result = buf->data[buf->ptr_b];
	buf->ptr_b = (buf->ptr_b + 1) % buf->size;
	if (buf->ptr_b == buf->ptr_e)
		buf->empty = true;
	return result;
}

static inline uint8_t ring_get_byte_begin(volatile ring_buffer* buf, size_t offset) {
	return buf->data[(buf->ptr_b + offset) % buf->size];
}

static inline void ring_move_begin(volatile ring_buffer* buf, size_t count) {
	if (count > ring_length(buf))
		count = ring_length(buf);
	buf->ptr_b = (buf->ptr_b + count) % buf->size;
	if (buf->ptr_b == buf->ptr_e)
		buf->empty = true;
}

static inline void ring_serialize(volatile ring_buffer* buf, uint8_t* out, size_t start, size_t length) {
	for (size_t i = 0; i < length; i++)
		out[i] = buf->data[(start + i) % buf->size];
}

static inline void ring_remove_frame(volatile ring_buffer* buf, size_t count) {
	if (count > ring_length(buf))
		count = ring_length(buf);
	buf->ptr_b = (buf->ptr_b + count) % buf->size;
	if (buf->ptr_b == buf->ptr_e)
		buf->empty = true;
}
