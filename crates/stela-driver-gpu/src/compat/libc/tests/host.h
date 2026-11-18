/* SPDX-License-Identifier: GPL-2.0-or-later */
#ifndef STELA_LIBC_TEST_HOST_H
#define STELA_LIBC_TEST_HOST_H
#include <stddef.h>
#include <stela/host.h>

/* The program: its result is the exit status. */
int main(void);
_Noreturn void leave(int status);

extern size_t live_blocks;
extern unsigned int current_core;
extern unsigned int waits, wakes;
/* What a wait changes, as another processor would. */
extern volatile unsigned int *waited_word;
extern unsigned int waited_value;
/* The host's render nodes, descriptors 3 onwards, and opens of any node. */
extern struct stela_libc_drm_device drm_devices[2];
extern unsigned int drm_device_count, drm_opens;
#endif
