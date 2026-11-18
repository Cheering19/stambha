/* SPDX-License-Identifier: GPL-2.0-or-later */
/* What the image test's program asks of its host (host.c). */
#ifndef IMAGE_HOST_H
#define IMAGE_HOST_H

#include <stdbool.h>

#include <stela/host.h>

/* The kernel drivers whose no-op shims answer the render nodes. */
enum image_kernel {
	IMAGE_AMDGPU,
	IMAGE_XE,
	IMAGE_I915,
	IMAGE_KERNELS,
};

/* The GPUs the program shows, at most two, and each one's kernel driver. */
void image_host_list(const struct stela_libc_drm_device *gpus, const enum image_kernel *kernels,
		     unsigned int count);
/* The driver ioctls each listed GPU's descriptors were sent. */
unsigned int image_host_driver_ioctls(unsigned int gpu);

#endif
