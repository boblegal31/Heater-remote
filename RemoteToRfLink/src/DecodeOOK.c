/*
 * DecodeOOK.c
 *
 *  Created on: 28 nov. 2018
 *      Author: francois
 */

#include "lpc_types.h"

unsigned char total_bits_v1, bits_v1, flip_v1, state_v1, pos_v1, data_v1[25];
unsigned char total_bits_v2, bits_v2, flip_v2, state_v2, pos_v2, data_v2[25];
unsigned char total_bits_v3, bits_v3, flip_v3, state_v3, pos_v3, data_v3[25];
unsigned char * total_bits, * bits, * flip, * state, * pos, * data;
enum { UNKNOWN, T0, T1, T2, T3, OK, DONE };

void manchester (char value);
void resetDecoder ();
bool isDone ();
void done ();

signed char decode_oregonV1 (unsigned short width) {
        if (200 <= width && width < 1200) {
            unsigned char w = width >= 700;
            switch (*state) {
                case UNKNOWN:
                    if (w == 0)
                        ++(*flip);
                    else if (10 <= *flip && *flip <= 50) {
                        *flip = 1;
                        manchester(1);
                    } else
                        return -1;
                    break;
                case OK:
                    if (w == 0)
                        *state = T0;
                    else
                        manchester(1);
                    break;
                case T0:
                    if (w == 0)
                        manchester(0);
                    else
                        return -1;
                    break;
            }
            return 0;
        }
        if (width >= 2500 && *pos >= 9)
            return 1;
        return -1;
    }

signed char decode_oregonV2 (unsigned short width) {
        if (200 <= width && width < 1200) {
            unsigned char w = width >= 700;
            switch (*state) {
                case UNKNOWN:
                    if (w != 0) {
                        // Long pulse
                        ++(*flip);
                    } else if (w == 0 && 24 <= *flip) {
                        // Short pulse, start bit
                        *flip = 0;
                        *state = T0;
                    } else {
                      // Reset decoder
                        return -1;
                    }
                    break;
                case OK:
                    if (w == 0) {
                        // Short pulse
                        *state = T0;
                    } else {
                        // Long pulse
                        manchester(1);
                    }
                    break;
                case T0:
                    if (w == 0) {
                      // Second short pulse
                        manchester(0);
                    } else {
                        // Reset decoder
                        return -1;
                    }
                    break;
            }
        } else if (width >= 2500  && *pos >= 8) {
                return 1;
        } else {
            return -1;
        }
        return *total_bits == 160 ? 1: 0;
    }

signed char decode_oregonV3 (unsigned short width) {
        if (200 <= width && width < 1200) {
            unsigned char w = width >= 700;
            switch (*state) {
                case UNKNOWN:
                    if (w == 0)
                        ++(*flip);
                    else if (32 <= *flip) {
                        *flip = 1;
                        manchester(1);
                    } else
                        return -1;
                    break;
                case OK:
                    if (w == 0)
                        *state = T0;
                    else
                        manchester(1);
                    break;
                case T0:
                    if (w == 0)
                        manchester(0);
                    else
                        return -1;
                    break;
            }
        } else {
            return -1;
        }
        return  *total_bits == 80 ? 1: 0;
    }

void gotBit_oregonV1 (char value) {
    (*total_bits)++;
    unsigned char *ptr = data + *pos;
    *ptr = (*ptr >> 1) | (value << 7);

    if (++(*bits) >= 8) {
        *bits = 0;
        if (++(*pos) >= sizeof data_v1) {
            resetDecoder();
            return;
        }
    }
    *state = OK;
}

void gotBit_oregonV2 (char value) {
        if(!(*total_bits & 0x01))
        {
            data[*pos] = (data[*pos] >> 1) | (value ? 0x80 : 00);
        }
        (*total_bits)++;
        *pos = *total_bits >> 4;
        if (*pos >= sizeof data_v2) {
            resetDecoder();
            return;
        }
        *state = OK;
    }

void gotBit_oregonV3 (char value) {
        data[*pos] = (data[*pos] >> 1) | (value ? 0x80 : 00);
        (*total_bits)++;
        *pos = *total_bits >> 3;
        if (*pos >= sizeof data_v3) {
            resetDecoder();
            return;
        }
        *state = OK;
    }

void ((* gotBit) (char value)) = & gotBit_oregonV1;
signed char ((* decode) (unsigned short width)) = & decode_oregonV1;

void init_version (int version)
{
	if (version == 1)
	{
		gotBit = & gotBit_oregonV1;
		decode = & decode_oregonV1;
		total_bits = &total_bits_v1;
		bits = &bits_v1;
		flip = &flip_v1;
		state = &state_v1;
		pos = &pos_v1;
		data = data_v1;
	} else if (version == 2)
	{
		gotBit = & gotBit_oregonV2;
		decode = & decode_oregonV2;
		total_bits = &total_bits_v2;
		bits = &bits_v2;
		flip = &flip_v2;
		state = &state_v2;
		pos = &pos_v2;
		data = data_v2;
	} else if (version == 3)
	{
		gotBit = & gotBit_oregonV3;
		decode = & decode_oregonV3;
		total_bits = &total_bits_v3;
		bits = &bits_v3;
		flip = &flip_v3;
		state = &state_v3;
		pos = &pos_v3;
		data = data_v3;
	}
}

bool nextPulse (unsigned short width) {
    if (*state != DONE)

        switch ((*decode)(width)) {
            case -1: resetDecoder(); break;
            case 1:  done(); break;
        }
    return isDone();
}

bool isDone () { return *state == DONE; }

unsigned char* getData (unsigned char * count)
{
    *count = *pos;
    resetDecoder();
    return data;
}

void resetDecoder () {
    *total_bits = *bits = *pos = *flip = 0;
    *state = UNKNOWN;
}

// add one bit to the packet data buffer

// store a bit using Manchester encoding
void manchester (char value) {
    *flip ^= value; // manchester code, long pulse flips the bit
    (*gotBit)(*flip);
}

// move bits to the front so that all the bits are aligned to the end
void alignTail (unsigned char max) {
    // align bits
    if (*bits != 0) {
        data[*pos] >>= 8 - *bits;
        for (unsigned char i = 0; i < *pos; ++i)
            data[i] = (data[i] >> *bits) | (data[i+1] << (8 - *bits));
        *bits = 0;
    }
    // optionally shift unsigned chars down if there are too many of 'em
    if (max > 0 && *pos > max) {
        unsigned char n = *pos - max;
        *pos = max;
        for (unsigned char i = 0; i < *pos; ++i)
            data[i] = data[i+n];
    }
}

void reverseBits () {
    for (unsigned char i = 0; i < *pos; ++i) {
        unsigned char b = data[i];
        for (unsigned char j = 0; j < 8; ++j) {
            data[i] = (data[i] << 1) | (b & 1);
            b >>= 1;
        }
    }
}

void reverseNibbles () {
    for (unsigned char i = 0; i < *pos; ++i)
        data[i] = (data[i] << 4) | (data[i] >> 4);
}

void done () {
    while (*bits)
        (*gotBit)(0); // padding
    *state = DONE;
}
