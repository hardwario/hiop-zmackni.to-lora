#ifndef _LIS2DH12_H
#define _LIS2DH12_H

#include <bc_common.h>

extern bool volatile lis2dh12_irq;

bool lis2dh12_init(void);
bool lis2dh12_clear_irq(void);

#endif
