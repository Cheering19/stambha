/* SPDX-License-Identifier: GPL-2.0-or-later */
/* What Linux's uapi ioctl header gives the DRM shims beside the request
 * encoding the image's C library has (asm/ioctl.h). Included first in the
 * shims' units of the image test only. */
#include <asm/ioctl.h>
#ifndef _IOC_NR
#define _IOC_NR(request) (((request) >> _IOC_NRSHIFT) & ((1 << _IOC_NRBITS) - 1))
#define _IOC_TYPE(request) (((request) >> _IOC_TYPESHIFT) & ((1 << _IOC_TYPEBITS) - 1))
#endif
