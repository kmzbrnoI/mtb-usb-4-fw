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

inline void ring_init(ring_buffer* buf, uint8_t* data, size_t size) {
	buf->ptr_b = 0;
	buf->ptr_e = 0;
	buf->empty = true;
	buf->data = data;
	buf->size = size;
}

inline bool ring_full(ring_buffer* buf) {
	return (buf->ptr_b == buf->ptr_e) && (!buf->empty);
}

inline size_t ring_length(ring_buffer* buf) {
	return ((buf->ptr_e - buf->ptr_b) % buf->size) + (ring_full(buf) ? buf->size: 0);
}

inline bool ring_empty(ring_buffer* buf) {
	return (buf->ptr_b == buf->ptr_e) && (buf->empty);
}

inline size_t ring_free_space(ring_buffer* buf) {
	return buf->size - ring_length(buf);
}

inline size_t ring_distance(ring_buffer* buf, size_t first, size_t second) {
	return (second-first) % buf->size;
}

inline void ring_clear(ring_buffer* buf) {
	buf->ptr_b = buf->ptr_e;
	buf->empty = true;
}

inline bool ring_add_byte(ring_buffer* buf, uint8_t data) {
	if (ring_full(buf))
		return false;

	buf->data[buf->ptr_e] = data;
	buf->ptr_e = (buf->ptr_e + 1) % buf->size;
	buf->empty = false;
	return true;
}

inline bool ring_add_bytes(ring_buffer* buf, uint8_t* data, size_t size) {
	if (ring_free_space(buf) < size)
		return false;

	for (size_t i = 0; i < size; i++)
		buf->data[(buf->ptr_e+i) % buf->size] = data[i];
	buf->ptr_e = (buf->ptr_e + size) % buf->size;
	buf->empty = false;
	return true;
}

inline uint8_t ring_remove_byte(ring_buffer* buf) {
	uint8_t result;
	result = buf->data[buf->ptr_b];
	buf->ptr_b = (buf->ptr_b + 1) % buf->size;
	if (buf->ptr_b == buf->ptr_e)
		buf->empty = true;
	return result;
}

inline uint8_t ring_get_byte_begin(ring_buffer* buf, size_t offset) {
	return buf->data[(buf->ptr_b + offset) % buf->size];
}

inline void ring_move_begin(ring_buffer* buf, size_t count) {
	if (count > ring_length(buf))
		count = ring_length(buf);
	buf->ptr_b = (buf->ptr_b + count) % buf->size;
	if (buf->ptr_b == buf->ptr_e)
		buf->empty = true;
}

inline void ring_serialize(ring_buffer* buf, uint8_t* out, size_t start, size_t length) {
	for (size_t i = 0; i < length; i++)
		out[i] = buf->data[(start + i) % buf->size];
}

inline void ring_remove_frame(ring_buffer* buf, size_t count) {
	if (count > ring_length(buf))
		count = ring_length(buf);
	buf->ptr_b = (buf->ptr_b + count) % buf->size;
	if (buf->ptr_b == buf->ptr_e)
		buf->empty = true;
}
