/*
  WString.cpp - String library for Wiring & Arduino
  ...mostly rewritten by Paul Stoffregen...
  Copyright (c) 2009-10 Hernando Barragan.  All rights reserved.
  Copyright 2011, Paul Stoffregen, paul@pjrc.com

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "WString.h"


/*********************************************/
/*  Constructors                             */
/*********************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define DTOA_UPPER 0x04
//#ifndef TDD
//#define isnanf(X)      __isnanf(X)
//#define isinff(X)      __isinff(X)
//char * fcvtf(float, int, int *, int *);
//#else
//#define isnanf(X)      false
//#define isinff(X)      false
//#endif


void reverse(char* begin, char* end) {
    char *is = begin;
    char *ie = end - 1;
    while(is < ie) {
        char tmp = *ie;
        *ie = *is;
        *is = tmp;
        ++is;
        --ie;
    }
}

char* utoa(unsigned value, char* result, int base) {
    if(base < 2 || base > 16) {
        *result = 0;
        return result;
    }

    char* out = result;
    unsigned quotient = value;

    do {
        const unsigned tmp = quotient / base;
        *out = "0123456789abcdef"[quotient - (tmp * base)];
        ++out;
        quotient = tmp;
    } while(quotient);

    reverse(result, out);
    *out = 0;
    return result;
}

char* itoa(int value, char* result, int base) {
    if(base < 2 || base > 16) {
        *result = 0;
        return result;
    }

    char* out = result;
    int quotient = abs(value);

    do {
        const int tmp = quotient / base;
        *out = "0123456789abcdef"[quotient - (tmp * base)];
        ++out;
        quotient = tmp;
    } while(quotient);

    // Apply negative sign
    if(value < 0)
        *out++ = '-';

    reverse(result, out);
    *out = 0;
    return result;
}

int atoi(const char* s) {
    return (int) atol(s);
}

long atol(const char* s) {
    char * tmp;
    return strtol(s, &tmp, 10);
}

double atof(const char* s) {
    char * tmp;
    return strtod(s, &tmp);
}

char * ultoa(unsigned long val, char *buf, int radix)
{
    unsigned digit;
    int i=0, j;
    char t;

    while (1) {
    	digit = val % radix;
    	buf[i] = ((digit < 10) ? '0' + digit : 'A' + digit - 10);
    	val /= radix;
    	if (val == 0) break;
    	i++;
    }
    buf[i + 1] = 0;
    for (j=0; j < i; j++, i--) {
    	t = buf[j];
    	buf[j] = buf[i];
    	buf[i] = t;
    }
    return buf;
}

char * ltoa(long val, char *buf, int radix)
{
    if (val >= 0) {
    	return ultoa(val, buf, radix);
    } else {
    	buf[0] = '-';
    	ultoa(-val, buf + 1, radix);
    	return buf;
    }
}


#ifdef TDD
char * dtostrf(float val, int width, unsigned int precision, char *buf)
{
//    int decpt, sign, reqd, pad;
//    const char *s, *e;
//    char *p;
//
//    int awidth = abs(width);
//    if (isnanf(val)) {
//    	int ndigs = (val<0) ? 4 : 3;
//    	awidth = (awidth > ndigs) ? awidth - ndigs : 0;
//    	if (width<0) {
//    		while (awidth) {
//    			*buf++ = ' ';
//    			awidth--;
//    		}
//    	}
//    	if (copysignf(1.0f, val)<0) *buf++ = '-';
//    	if (DTOA_UPPER) {
//    		*buf++ = 'N';  *buf++ = 'A';  *buf++ = 'N';
//    	} else {
//    		*buf++ = 'n';  *buf++ = 'a';  *buf++ = 'n';
//    	}
//    	while (awidth) {
//    		*buf++ = ' ';
//    		awidth--;
//    	}
//    	*buf = 0;
//    	return buf;
//    }
//    if (isinff(val)) {
//    	int ndigs = (val<0) ? 4 : 3;
//    	awidth = (awidth > ndigs) ? awidth - ndigs : 0;
//    	if (width<0) {
//    		while (awidth) {
//    			*buf++ = ' ';
//    			awidth--;
//    		}
//    	}
//    	if (val<0) *buf++ = '-';
//    	if (DTOA_UPPER) {
//    		*buf++ = 'I';  *buf++ = 'N';  *buf++ = 'F';
//    	} else {
//    		*buf++ = 'i';  *buf++ = 'n';  *buf++ = 'f';
//    	}
//    	while (awidth) {
//    		*buf++ = ' ';
//    		awidth--;
//    	}
//    	*buf = 0;
//    	return buf;
//    }
//
//    s = fcvtf(val, precision, &decpt, &sign);
//
//    // if only 1 digit in output
//    if (precision == 0 && decpt == 0) {
//    	// round and move decimal point
//    	s = (*s < '5') ? "0" : "1";
//    	decpt++;
//    }
//
//    // if all zeros, limit to precision
//    if (-decpt  > (int)precision) {
//    	s = "0";
//    	decpt = -precision;
//    }
//
//    reqd = strlen(s);
//
//    // add 1 for decimal point
//    if (reqd > decpt) reqd++;
//
//    // add 1 for zero in front of decimal point
//    if (decpt == 0) reqd++;
//
//    // if leading zeros after decimal point
//    if (decpt < 0 && precision > 0) {
//    	// ensure enough trailing zeros, add 2 for '0.'
//    	reqd = precision + 2;
//
//    	if (strlen(s) > precision + decpt) {
//    		// bug in fcvtf. e.g. 0.012, precision 2 should return 1 instead of 12.
//    		// However, 1.2, precision 0 returns correct value. So shift values so
//    		// that decimal point is after the first digit, then convert again
//
//    		int newPrecision = precision;
//    		int newDecimalPoint;
//
//    		// shift decimal point
//    		while (newPrecision > 0) {
//    			val *= 10.0;
//    			newPrecision--;
//    		}
//
//    		// round after accounting for leading 0's
//    		s = fcvtf(val, newPrecision, &newDecimalPoint, &sign);
//
//    		// if rounded up to new digit (e.g. 0.09 to 0.1), move decimal point
//    		if (newDecimalPoint - decpt == precision + 1) decpt++;
//    	}
//    }
//
//    // add 1 for sign if negative
//    if (sign) reqd++;
//
//    p = buf;
//    e = p + reqd;
//    pad = width - reqd;
//    if (pad > 0) {
//    	e += pad;
//    	while (pad-- > 0) *p++ = ' ';
//    }
//    if (sign) *p++ = '-';
//    if (decpt == 0 && precision > 0) {
//    	*p++ = '0';
//    	*p++ = '.';
//    }
//    else if (decpt < 0 && precision > 0) {
//    	*p++ = '0';
//    	*p++ = '.';
//    	// print leading zeros
//    	while ( decpt < 0 ) {
//    		decpt++;
//    		*p++ = '0';
//    	}
//    }
//    // print digits
//    while (p < e) {
//    	*p++ = *s++;
//    	if (p == e) break;
//    	if (--decpt == 0) *p++ = '.';
//    }
//    if (width < 0) {
//    	pad = (reqd + width) * -1;
//    	while (pad-- > 0) *p++ = ' ';
//    }
//    *p = 0;

    String res;

    int ent_val = (int) val;
    res = String(ent_val);

    if (width > 0) {
    	res += ".";
    	uint32_t dec_val = fabs(val - (float)ent_val) * pow(10, width);
    	res += dec_val;
    }

    res.toCharArray(buf, 512);

    return buf;
}
#endif

String::String(const char *cstr)
{
    init();
    if (cstr) copy(cstr, strlen(cstr));
}

String::String(const __FlashStringHelper *pgmstr)
{
    init();
    *this = pgmstr;
}

String::String(const String &value)
{
    init();
    *this = value;
}

#if __cplusplus >= 201103L || defined(__GXX_EXPERIMENTAL_CXX0X__)
String::String(String &&rval)
{
    init();
    move(rval);
}
String::String(StringSumHelper &&rval)
{
    init();
    move(rval);
}
#endif

String::String(char c)
{
    init();
    *this = c;
}

String::String(unsigned char c)
{
    init();
    char buf[4];
    utoa(c, buf, 10);
    *this = buf;
}

String::String(const int value, unsigned char base)
{
    init();
    char buf[18];
    itoa(value, buf, base);
    *this = buf;
}

String::String(unsigned int value, unsigned char base)
{
    init();
    char buf[17];
      utoa(value, buf, base);
    *this = buf;
}

String::String(long value, unsigned char base)
{
    init();
    char buf[34];
    ltoa(value, buf, base);
    *this = buf;
}

String::String(unsigned long value, unsigned char base)
{
    init();
    char buf[33];
    ultoa(value, buf, base);
    *this = buf;
}

String::String(float num, unsigned int digits)
{
    init();
    char buf[40];
    *this = dtostrf(num, digits + 2, digits, buf);
}

String::~String()
{
    free(buffer);
}

/*********************************************/
/*  Memory Management                        */
/*********************************************/

inline void String::init(void)
{
    buffer = NULL;
    capacity = 0;
    len = 0;
    flags = 0;
}

unsigned char String::reserve(unsigned int size)
{
    if (capacity >= size) return 1;
    if (changeBuffer(size)) {
    	if (len == 0) buffer[0] = 0;
    	return 1;
    }
    return 0;
}

unsigned char String::changeBuffer(unsigned int maxStrLen)
{
    char *newbuffer = (char *)realloc(buffer, maxStrLen + 1);
    if (newbuffer) {
    	buffer = newbuffer;
    	capacity = maxStrLen;
    	return 1;
    }
    return 0;
}

/*********************************************/
/*  Copy and Move                            */
/*********************************************/

String & String::copy(const char *cstr, unsigned int length)
{
    if (length == 0) {
    	if (buffer) buffer[0] = 0;
    	len = 0;
    	return *this;
    }
    if (!reserve(length)) {
    	if (buffer) {
    		free(buffer);
    		buffer = NULL;
    	}
    	len = capacity = 0;
    	return *this;
    }
    len = length;
    strcpy(buffer, cstr);
    return *this;
}

void String::move(String &rhs)
{
    if (buffer) {
    	if (capacity >= rhs.len) {
    		strcpy(buffer, rhs.buffer);
    		len = rhs.len;
    		rhs.len = 0;
    		return;
    	} else {
    		free(buffer);
    	}
    }
    buffer = rhs.buffer;
    capacity = rhs.capacity;
    len = rhs.len;
    rhs.buffer = NULL;
    rhs.capacity = 0;
    rhs.len = 0;
}

String & String::operator = (const String &rhs)
{
    if (this == &rhs) return *this;
    return copy(rhs.buffer, rhs.len);
}

#if __cplusplus >= 201103L || defined(__GXX_EXPERIMENTAL_CXX0X__)
String & String::operator = (String &&rval)
{
    if (this != &rval) move(rval);
    return *this;
}

String & String::operator = (StringSumHelper &&rval)
{
    if (this != &rval) move(rval);
    return *this;
}
#endif

String & String::operator = (const char *cstr)
{
    if (cstr) {
    	copy(cstr, strlen(cstr));
    } else {
    	len = 0;
    }
    return *this;
}

String & String::operator = (const __FlashStringHelper *pgmstr)
{
    copy(pgmstr);
    return *this;
}

String & String::operator = (char c)
{
    char buf[2];
    buf[0] = c;
    buf[1] = 0;
    return copy(buf, 1);
}

/*********************************************/
/*  Append                                   */
/*********************************************/

String & String::append(const String &s)
{
    return append(s.buffer, s.len);
}

String & String::append(const char *cstr, unsigned int length)
{
    unsigned int newlen = len + length;
    bool self = false;
    unsigned int buffer_offset; 
    if ( (cstr >= buffer) && (cstr < (buffer+len) ) ) {
    	self = true;
    	buffer_offset = (unsigned int)(cstr-buffer);
    }
    if (length == 0 || !reserve(newlen)) return *this;
    if ( self ) {
    	memcpy(buffer + len, buffer+buffer_offset, length);
    	buffer[newlen] = 0;
    	}
    else
    	strcpy(buffer + len, cstr);
    len = newlen;
    return *this;
}

String & String::append(const char *cstr)
{
    if (cstr) append(cstr, strlen(cstr));
    return *this;
}

String & String::append(char c)
{
    char buf[2];
    buf[0] = c;
    buf[1] = 0;
    append(buf, 1);
    return *this;
}

String & String::append(int num)
{
    char buf[12];
    ltoa((long)num, buf, 10);
    append(buf, strlen(buf));
    return *this;
}

String & String::append(unsigned int num)
{
    char buf[11];
    ultoa((unsigned long)num, buf, 10);
    append(buf, strlen(buf));
    return *this;
}

String & String::append(long num)
{
    char buf[12];
    ltoa(num, buf, 10);
    append(buf, strlen(buf));
    return *this;
}

String & String::append(unsigned long num)
{
    char buf[11];
    ultoa(num, buf, 10);
    append(buf, strlen(buf));
    return *this;
}

String & String::append(float num)
{
    char buf[30];
    dtostrf(num, 4, 2, buf); 
    append(buf, strlen(buf));
    return *this;
}


/*********************************************/
/*  Concatenate                              */
/*********************************************/


StringSumHelper & operator + (const StringSumHelper &lhs, const String &rhs)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append(rhs.buffer, rhs.len);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, const char *cstr)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    if (cstr) a.append(cstr, strlen(cstr));
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, const __FlashStringHelper *pgmstr)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append(pgmstr);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, char c)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append(c);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, unsigned char c)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append(c);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, int num)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append((long)num);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, unsigned int num)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append((unsigned long)num);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, long num)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append(num);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, unsigned long num)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append(num);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, float num)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append(num);
    return a;
}

StringSumHelper & operator + (const StringSumHelper &lhs, double num)
{
    StringSumHelper &a = const_cast<StringSumHelper&>(lhs);
    a.append(num);
    return a;
}

/*********************************************/
/*  Comparison                               */
/*********************************************/

int String::compareTo(const String &s) const
{
    if (!buffer || !s.buffer) {
    	if (s.buffer && s.len > 0) return 0 - *(unsigned char *)s.buffer;
    	if (buffer && len > 0) return *(unsigned char *)buffer;
    	return 0;
    }
    return strcmp(buffer, s.buffer);
}

unsigned char String::equals(const String &s2) const
{
    return (len == s2.len && compareTo(s2) == 0);
}

unsigned char String::equals(const char *cstr) const
{
    if (len == 0) return (cstr == NULL || *cstr == 0);
    if (cstr == NULL) return buffer[0] == 0;
    return strcmp(buffer, cstr) == 0;
}

unsigned char String::operator<(const String &rhs) const
{
    return compareTo(rhs) < 0;
}

unsigned char String::operator>(const String &rhs) const
{
    return compareTo(rhs) > 0;
}

unsigned char String::operator<=(const String &rhs) const
{
    return compareTo(rhs) <= 0;
}

unsigned char String::operator>=(const String &rhs) const
{
    return compareTo(rhs) >= 0;
}

unsigned char String::equalsIgnoreCase( const String &s2 ) const
{
    if (this == &s2) return 1;
    if (len != s2.len) return 0;
    if (len == 0) return 1;
    const char *p1 = buffer;
    const char *p2 = s2.buffer;
    while (*p1) {
    	if (tolower(*p1++) != tolower(*p2++)) return 0;
    } 
    return 1;
}

unsigned char String::startsWith( const String &s2 ) const
{
    if (len < s2.len) return 0;
    return startsWith(s2, 0);
}

unsigned char String::startsWith( const String &s2, unsigned int offset ) const
{
    if (offset > len - s2.len || !buffer || !s2.buffer) return 0;
    return strncmp( &buffer[offset], s2.buffer, s2.len ) == 0;
}

unsigned char String::endsWith( const String &s2 ) const
{
    if ( len < s2.len || !buffer || !s2.buffer) return 0;
    return strcmp(&buffer[len - s2.len], s2.buffer) == 0;
}

/*********************************************/
/*  Character Access                         */
/*********************************************/

char String::charAt(unsigned int loc) const
{
    return operator[](loc);
}

void String::setCharAt(unsigned int loc, char c) 
{
    if (loc < len) buffer[loc] = c;
}

char & String::operator[](unsigned int index)
{
    static char dummy_writable_char;
    if (index >= len || !buffer) {
    	dummy_writable_char = 0;
    	return dummy_writable_char;
    }
    return buffer[index];
}

char String::operator[]( unsigned int index ) const
{
    if (index >= len || !buffer) return 0;
    return buffer[index];
}

void String::getBytes(unsigned char *buf, unsigned int bufsize, unsigned int index) const
{
    if (!bufsize || !buf) return;
    if (index >= len) {
    	buf[0] = 0;
    	return;
    }
    unsigned int n = bufsize - 1;
    if (n > len - index) n = len - index;
    strncpy((char *)buf, buffer + index, n);
    buf[n] = 0;
}

/*********************************************/
/*  Search                                   */
/*********************************************/

int String::indexOf(char c) const
{
    return indexOf(c, 0);
}

int String::indexOf( char ch, unsigned int fromIndex ) const
{
    if (fromIndex >= len) return -1;
    const char* temp = strchr(buffer + fromIndex, ch);
    if (temp == NULL) return -1;
    return temp - buffer;
}

int String::indexOf(const String &s2) const
{
    return indexOf(s2, 0);
}

int String::indexOf(const String &s2, unsigned int fromIndex) const
{
    if (fromIndex >= len) return -1;
    const char *found = strstr(buffer + fromIndex, s2.buffer);
    if (found == NULL) return -1;
    return found - buffer;
}

int String::lastIndexOf( char theChar ) const
{
    return lastIndexOf(theChar, len - 1);
}

int String::lastIndexOf(char ch, unsigned int fromIndex) const
{
    if (fromIndex >= len || fromIndex < 0) return -1;
    char tempchar = buffer[fromIndex + 1];
    buffer[fromIndex + 1] = '\0';
    char* temp = strrchr( buffer, ch );
    buffer[fromIndex + 1] = tempchar;
    if (temp == NULL) return -1;
    return temp - buffer;
}

int String::lastIndexOf(const String &s2) const
{
    return lastIndexOf(s2, len - s2.len);
}

int String::lastIndexOf(const String &s2, unsigned int fromIndex) const
{
      if (s2.len == 0 || len == 0 || s2.len > len || fromIndex < 0) return -1;
    if (fromIndex >= len) fromIndex = len - 1;
    int found = -1;
    for (char *p = buffer; p <= buffer + fromIndex; p++) {
    	p = strstr(p, s2.buffer);
    	if (!p) break;
    	if ((unsigned int)(p - buffer) <= fromIndex) found = p - buffer;
    }
    return found;
}

String String::substring( unsigned int left ) const
{
    return substring(left, len);
}

String String::substring(unsigned int left, unsigned int right) const
{
    if (left > right) {
    	unsigned int temp = right;
    	right = left;
    	left = temp;
    }
    String out;
    if (left > len) return out;
    if (right > len) right = len;
    char temp = buffer[right];  // save the replaced character
    buffer[right] = '\0';	
    out = buffer + left;  // pointer arithmetic
    buffer[right] = temp;  //restore character
    return out;
}

/*********************************************/
/*  Modification                             */
/*********************************************/

String & String::replace(char find, char replace)
{
    if (!buffer) return *this;
    for (char *p = buffer; *p; p++) {
    	if (*p == find) *p = replace;
    }
    return *this;
}

String & String::replace(const String& find, const String& replace)
{
    if (len == 0 || find.len == 0) return *this;
    int diff = replace.len - find.len;
    char *readFrom = buffer;
    char *foundAt;
    if (diff == 0) {
    	while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
    		memcpy(foundAt, replace.buffer, replace.len);
    		readFrom = foundAt + replace.len;
    	}
    } else if (diff < 0) {
    	char *writeTo = buffer;
    	while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
    		unsigned int n = foundAt - readFrom;
    		memcpy(writeTo, readFrom, n);
    		writeTo += n;
    		memcpy(writeTo, replace.buffer, replace.len);
    		writeTo += replace.len;
    		readFrom = foundAt + find.len;
    		len += diff;
    	}
    	strcpy(writeTo, readFrom);
    } else {
    	unsigned int size = len; // compute size needed for result
    	while ((foundAt = strstr(readFrom, find.buffer)) != NULL) {
    		readFrom = foundAt + find.len;
    		size += diff;
    	}
    	if (size == len) return *this;
    	if (size > capacity && !changeBuffer(size)) return *this;
    	int index = len - 1;
    	while (index >= 0 && (index = lastIndexOf(find, index)) >= 0) {
    		readFrom = buffer + index + find.len;
    		memmove(readFrom + diff, readFrom, len - (readFrom - buffer));
    		len += diff;
    		buffer[len] = 0;
    		memcpy(buffer + index, replace.buffer, replace.len);
    		index--;
    	}
    }
    return *this;
}

String & String::remove(unsigned int index)
{
    if (index < len) {
    	len = index;
    	buffer[len] = 0;
    }
    return *this;
}

String & String::remove(unsigned int index, unsigned int count)
{
    if (index < len && count > 0) {
      	if (index + count > len) count = len - index;
    	len = len - count;
    	memmove(buffer + index, buffer + index + count, len - index);
    	buffer[len] = 0;
    }
    return *this;
}

String & String::toLowerCase(void)
{
    if (!buffer) return *this;
    for (char *p = buffer; *p; p++) {
    	*p = tolower(*p);
    }
    return *this;
}

String & String::toUpperCase(void)
{
    if (!buffer) return *this;
    for (char *p = buffer; *p; p++) {
    	*p = toupper(*p);
    }
    return *this;
}

String & String::trim(void)
{
    if (!buffer || len == 0) return *this;
    char *begin = buffer;
    while (isspace(*begin)) begin++;
    char *end = buffer + len - 1;
    while (isspace(*end) && end >= begin) end--;
    len = end + 1 - begin;
    if (begin > buffer) memcpy(buffer, begin, len);
    buffer[len] = 0;
    return *this;
}

/*********************************************/
/*  Parsing / Conversion                     */
/*********************************************/

long String::toInt(void) const
{
    if (buffer) return atol(buffer);
    return 0;
}

float String::toFloat(void) const
{
    if (buffer) return strtof(buffer, (char **)NULL);
    return 0.0;
}


