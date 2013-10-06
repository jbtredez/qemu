#ifndef ATLANTRONIC_CPU_H
#define ATLANTRONIC_CPU_H

#define LINUX
#undef FALSE
#undef TRUE
#undef bool
#define STM32F4XX
#include "kernel/cpu/cpu.h"
#undef LINUX

#define W_ACCESS(type, var, regname, val) \
		case offsetof(type, regname): \
			var.regname = val; \
			break

#define R_ACCESS(type, var, regname, val) \
		case offsetof(type, regname): \
			val = var.regname; \
			break

#endif
