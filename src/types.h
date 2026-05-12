/* VisualBoyAdvance - Nintendo Gameboy/GameboyAdvance (TM) emulator. */
/* Copyright (C) 2008 VBA-M development team */

/* This program is free software; you can redistribute it and/or modify */
/* it under the terms of the GNU General Public License as published by */
/* the Free Software Foundation; either version 2, or(at your option) */
/* any later version. */
/* */
/* This program is distributed in the hope that it will be useful, */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the */
/* GNU General Public License for more details. */
/* */
/* You should have received a copy of the GNU General Public License */
/* along with this program; if not, write to the Free Software Foundation, */
/* Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA. */

#ifndef __VBA_TYPES_H__
#define __VBA_TYPES_H__

#include <stdint.h>


/* C89 has no <stdbool.h>.  Provide bool / true / false as an enum so existing
 * source compiles unchanged.  Guarded against C++ (which has the builtin) and
 * C99+ (which has <stdbool.h>).  In strict-conforming C89 the typedef name
 * `bool` is in the user namespace, so this is portable. */
#if !defined(__cplusplus) && (!defined(__STDC_VERSION__) || __STDC_VERSION__ < 199901L)
typedef enum { false = 0, true = 1 } bool;
#endif

#endif /* __VBA_TYPES_H__ */
