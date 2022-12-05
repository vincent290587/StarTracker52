//
// Created by vince on 28/11/2022.
//

#ifndef STARTRACKER_ULN2003_H
#define STARTRACKER_ULN2003_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void uln2003__init(void);

void uln2003__pause(uint8_t resume);

void uln2003__direction(uint8_t forward);

void uln2003__service(void);

#ifdef __cplusplus
}
#endif

#endif //STARTRACKER_ULN2003_H
