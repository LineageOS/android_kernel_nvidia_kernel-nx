/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2020 Google, Inc.
 */
#ifndef _PMC_SMC_H
#define _PMC_SMC_H

#include <linux/types.h>

struct pmc_smc_ret8 {
	ulong r0;
	ulong r1;
	ulong r2;
	ulong r3;
	ulong r4;
	ulong r5;
	ulong r6;
	ulong r7;
};

struct pmc_smc_ret8 pmc_smc8(ulong r0, ulong r1, ulong r2, ulong r3,
			    ulong r4, ulong r5, ulong r6, ulong r7);

#endif /* _PMC_SMC_H */
