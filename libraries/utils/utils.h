/*
 * File:   utils.h
 * Author: vincent
 *
 * Created on October 27, 2015, 10:55 AM
 */

#ifndef UTILS_H
#define	UTILS_H

#include <stdint.h>


#ifdef	__cplusplus
extern "C" {
#endif


float regFen(float val_, float b1_i, float b1_f, float b2_i, float b2_f) __attribute__ ((pure));

float regFenLim(float val_, float b1_i, float b1_f, float b2_i, float b2_f) __attribute__ ((pure));

void calculePos (const char *nom, float *lat, float *lon);

long unsigned int toBase10 (char *entree);

extern void loggerMsg(const char *msg_);

double radians(double value) __attribute__ ((pure));

double degrees(double value) __attribute__ ((pure));

uint32_t get_sec_jour(uint8_t hour_, uint8_t min_, uint8_t sec_) __attribute__ ((pure));

float compute2Complement(uint8_t msb, uint8_t lsb) __attribute__ ((pure));

float percentageBatt(float tensionValue, float current) __attribute__ ((pure));

void encode_uint16 (uint8_t* dest, uint16_t input);

void encode_uint32 (uint8_t* dest, uint32_t input);

uint16_t decode_uint16 (uint8_t* dest);

uint32_t decode_uint32 (uint8_t* dest);

void const_char_to_buffer(const char *str_, uint8_t *buff_, uint16_t max_size);

uint8_t calculate_crc(uint8_t input_a[], uint16_t length);

int floorSqrt(int x);

uint32_t date_to_timestamp(uint32_t sec_j, uint8_t day, uint8_t month, uint16_t year);

#ifdef	__cplusplus
}
#endif

#endif	/* UTILS_H */

