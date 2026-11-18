// SPDX-License-Identifier: GPL-2.0-or-later
/* The image's own Vulkan driver, C library and defines on the build host
 * (`cargo xtask gpu image-test`): this program links RADV's and ANV's
 * objects as the image compiles them, only built for Linux instead of the
 * image's COFF target, with the image's C library under them and host.c
 * under that, and runs them over render nodes Mesa's DRM shim answers, each
 * GPU's as its kernel driver would: RADV on amdgpu, ANV on xe or i915. It
 * makes each driver's instance and every physical device, then on each GPU in
 * turn
 * the device, host-visible memory it maps, writes and maps again, and one
 * submission of a fill of a buffer, then waits for it; each GPU's work must
 * reach that GPU's render node and no other. What differs from a Linux host
 * in the image (a probe define dropped, a C library call that fails or
 * answers a fixed value) shows here as it would on a GPU, short of what only
 * a GPU's work shows.
 *
 * Usage: image_device <GPU> [<GPU>], each GPU <kernel>:<device>: amdgpu and
 * its device name, or xe or i915 and the PCI device ID in hex. Two Intel
 * GPUs must be the same device, as the shims model one of each driver.
 * Prints "image device: ..." and "image ioctls: ..." and exits 0, or names
 * the step that failed and exits 1. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>

#include "image_host.h"

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL radv_icdGetInstanceProcAddr(VkInstance, const char *);
VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL anv_icdGetInstanceProcAddr(VkInstance, const char *);

#define CHECK(condition)                                                                       \
	do {                                                                                   \
		if (!(condition)) {                                                            \
			fprintf(stderr, "image device: %s:%d: %s\n", __FILE__, __LINE__,      \
				#condition);                                                   \
			return 1;                                                              \
		}                                                                              \
	} while (0)
#define VK(call)                                                                               \
	do {                                                                                   \
		VkResult result_ = (call);                                                     \
		if (result_ != VK_SUCCESS) {                                                   \
			fprintf(stderr, "image device: %s:%d: %s gave %d\n", __FILE__,        \
				__LINE__, #call, result_);                                     \
			return 1;                                                              \
		}                                                                              \
	} while (0)
#define LOAD(instance, name) name = (PFN_##name)entry(instance, #name)
#define DEVICE_LOAD(device, name)                                                              \
	PFN_##name name = (PFN_##name)vkGetDeviceProcAddr(device, #name)

#define BUFFER_BYTES (64u << 10)

/* The calls of the driver in use, which use_driver() sets. */
static PFN_vkGetInstanceProcAddr entry;
static PFN_vkCreateInstance vkCreateInstance;
static PFN_vkEnumeratePhysicalDevices vkEnumeratePhysicalDevices;
static PFN_vkGetPhysicalDeviceProperties vkGetPhysicalDeviceProperties;
static PFN_vkGetPhysicalDeviceQueueFamilyProperties vkGetPhysicalDeviceQueueFamilyProperties;
static PFN_vkGetPhysicalDeviceMemoryProperties vkGetPhysicalDeviceMemoryProperties;
static PFN_vkCreateDevice vkCreateDevice;
static PFN_vkGetDeviceProcAddr vkGetDeviceProcAddr;
static PFN_vkDestroyInstance vkDestroyInstance;

/* One Vulkan driver of the program: its entry, instance and calls. */
struct driver {
	const char *name;
	PFN_vkGetInstanceProcAddr entry;
	VkInstance instance;
	PFN_vkEnumeratePhysicalDevices enumerate;
	PFN_vkGetPhysicalDeviceProperties properties;
	PFN_vkGetPhysicalDeviceQueueFamilyProperties families;
	PFN_vkGetPhysicalDeviceMemoryProperties memory;
	PFN_vkCreateDevice create_device;
	PFN_vkGetDeviceProcAddr device_entry;
	PFN_vkDestroyInstance destroy;
};

static struct driver drivers[2] = {
	{.name = "radv", .entry = radv_icdGetInstanceProcAddr},
	{.name = "anv", .entry = anv_icdGetInstanceProcAddr},
};

static void use_driver(const struct driver *driver)
{
	entry = driver->entry;
	vkEnumeratePhysicalDevices = driver->enumerate;
	vkGetPhysicalDeviceProperties = driver->properties;
	vkGetPhysicalDeviceQueueFamilyProperties = driver->families;
	vkGetPhysicalDeviceMemoryProperties = driver->memory;
	vkCreateDevice = driver->create_device;
	vkGetDeviceProcAddr = driver->device_entry;
	vkDestroyInstance = driver->destroy;
}

/* The driver's instance and calls. */
static int start(struct driver *driver)
{
	entry = driver->entry;
	LOAD(NULL, vkCreateInstance);
	VK(vkCreateInstance(
		&(VkInstanceCreateInfo){
			.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
			.pApplicationInfo =
				&(VkApplicationInfo){
					.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
					.apiVersion = VK_API_VERSION_1_3,
				},
		},
		NULL, &driver->instance));
	VkInstance instance = driver->instance;
	driver->enumerate = (PFN_vkEnumeratePhysicalDevices)entry(instance, "vkEnumeratePhysicalDevices");
	driver->properties =
		(PFN_vkGetPhysicalDeviceProperties)entry(instance, "vkGetPhysicalDeviceProperties");
	driver->families = (PFN_vkGetPhysicalDeviceQueueFamilyProperties)entry(
		instance, "vkGetPhysicalDeviceQueueFamilyProperties");
	driver->memory = (PFN_vkGetPhysicalDeviceMemoryProperties)entry(
		instance, "vkGetPhysicalDeviceMemoryProperties");
	driver->create_device = (PFN_vkCreateDevice)entry(instance, "vkCreateDevice");
	driver->device_entry = (PFN_vkGetDeviceProcAddr)entry(instance, "vkGetDeviceProcAddr");
	driver->destroy = (PFN_vkDestroyInstance)entry(instance, "vkDestroyInstance");
	return 0;
}

/* A device of one physical device, with one queue of a compute family. */
static int open_device(VkPhysicalDevice physical, uint32_t *queue_family, VkDevice *device)
{
	uint32_t families = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(physical, &families, NULL);
	VkQueueFamilyProperties family_properties[8];
	CHECK(families && families <= 8);
	vkGetPhysicalDeviceQueueFamilyProperties(physical, &families, family_properties);
	uint32_t family = 0;
	while (family < families && !(family_properties[family].queueFlags & VK_QUEUE_COMPUTE_BIT))
		family++;
	CHECK(family < families);
	*queue_family = family;
	VK(vkCreateDevice(physical,
			  &(VkDeviceCreateInfo){
				  .sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO,
				  .queueCreateInfoCount = 1,
				  .pQueueCreateInfos =
					  &(VkDeviceQueueCreateInfo){
						  .sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO,
						  .queueFamilyIndex = family,
						  .queueCount = 1,
						  .pQueuePriorities = (const float[]){1.0f},
					  },
			  },
			  NULL, device));
	return 0;
}

/* One physical device's device, mapped memory and one submission. */
static int work(VkPhysicalDevice physical)
{
	uint32_t family;
	VkDevice device;
	if (open_device(physical, &family, &device))
		return 1;
	DEVICE_LOAD(device, vkGetDeviceQueue);
	DEVICE_LOAD(device, vkCreateBuffer);
	DEVICE_LOAD(device, vkGetBufferMemoryRequirements);
	DEVICE_LOAD(device, vkAllocateMemory);
	DEVICE_LOAD(device, vkBindBufferMemory);
	DEVICE_LOAD(device, vkMapMemory);
	DEVICE_LOAD(device, vkUnmapMemory);
	DEVICE_LOAD(device, vkCreateCommandPool);
	DEVICE_LOAD(device, vkAllocateCommandBuffers);
	DEVICE_LOAD(device, vkBeginCommandBuffer);
	DEVICE_LOAD(device, vkCmdFillBuffer);
	DEVICE_LOAD(device, vkEndCommandBuffer);
	DEVICE_LOAD(device, vkCreateFence);
	DEVICE_LOAD(device, vkQueueSubmit);
	DEVICE_LOAD(device, vkWaitForFences);
	DEVICE_LOAD(device, vkDestroyFence);
	DEVICE_LOAD(device, vkDestroyCommandPool);
	DEVICE_LOAD(device, vkDestroyBuffer);
	DEVICE_LOAD(device, vkFreeMemory);
	DEVICE_LOAD(device, vkDeviceWaitIdle);
	DEVICE_LOAD(device, vkDestroyDevice);
	VkQueue queue;
	vkGetDeviceQueue(device, family, 0, &queue);

	VkBuffer buffer;
	VK(vkCreateBuffer(device,
			  &(VkBufferCreateInfo){
				  .sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO,
				  .size = BUFFER_BYTES,
				  .usage = VK_BUFFER_USAGE_TRANSFER_DST_BIT |
					   VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
			  },
			  NULL, &buffer));
	VkMemoryRequirements requirements;
	vkGetBufferMemoryRequirements(device, buffer, &requirements);
	VkPhysicalDeviceMemoryProperties memory;
	vkGetPhysicalDeviceMemoryProperties(physical, &memory);
	uint32_t type = 0;
	const VkMemoryPropertyFlags wanted =
		VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;
	while (type < memory.memoryTypeCount &&
	       (!(requirements.memoryTypeBits & (1u << type)) ||
		(memory.memoryTypes[type].propertyFlags & wanted) != wanted))
		type++;
	CHECK(type < memory.memoryTypeCount);
	/* Small allocations, as most of the Metal device's are: drivers place
	 * them inside larger buffers of the kernel's and map them at an offset. */
	VkDeviceMemory allocations[4];
	for (unsigned int at = 0; at < 4; at++) {
		VK(vkAllocateMemory(device,
				    &(VkMemoryAllocateInfo){
					    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
					    .allocationSize = requirements.size,
					    .memoryTypeIndex = type,
				    },
				    NULL, &allocations[at]));
		uint32_t *words;
		VK(vkMapMemory(device, allocations[at], 0, VK_WHOLE_SIZE, 0, (void **)&words));
		for (unsigned int word = 0; word < BUFFER_BYTES / 4; word++)
			words[word] = word ^ at;
		vkUnmapMemory(device, allocations[at]);
		VK(vkMapMemory(device, allocations[at], 0, VK_WHOLE_SIZE, 0, (void **)&words));
		CHECK(words[BUFFER_BYTES / 4 - 1] == ((BUFFER_BYTES / 4 - 1) ^ at));
		vkUnmapMemory(device, allocations[at]);
	}
	VK(vkBindBufferMemory(device, buffer, allocations[3], 0));

	VkCommandPool pool;
	VK(vkCreateCommandPool(device,
			       &(VkCommandPoolCreateInfo){
				       .sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO,
				       .queueFamilyIndex = family,
			       },
			       NULL, &pool));
	VkCommandBuffer commands;
	VK(vkAllocateCommandBuffers(device,
				    &(VkCommandBufferAllocateInfo){
					    .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
					    .commandPool = pool,
					    .level = VK_COMMAND_BUFFER_LEVEL_PRIMARY,
					    .commandBufferCount = 1,
				    },
				    &commands));
	VK(vkBeginCommandBuffer(commands, &(VkCommandBufferBeginInfo){
						  .sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO,
					  }));
	vkCmdFillBuffer(commands, buffer, 0, VK_WHOLE_SIZE, 0x5a5a5a5a);
	VK(vkEndCommandBuffer(commands));
	VkFence fence;
	VK(vkCreateFence(device, &(VkFenceCreateInfo){.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO},
			 NULL, &fence));
	VK(vkQueueSubmit(queue, 1,
			 &(VkSubmitInfo){
				 .sType = VK_STRUCTURE_TYPE_SUBMIT_INFO,
				 .commandBufferCount = 1,
				 .pCommandBuffers = &commands,
			 },
			 fence));
	VK(vkWaitForFences(device, 1, &fence, VK_TRUE, 5000000000ull));
	VK(vkDeviceWaitIdle(device));

	vkDestroyFence(device, fence, NULL);
	vkDestroyCommandPool(device, pool, NULL);
	vkDestroyBuffer(device, buffer, NULL);
	for (unsigned int at = 0; at < 4; at++)
		vkFreeMemory(device, allocations[at], NULL);
	vkDestroyDevice(device, NULL);
	return 0;
}

int main(int argc, char **argv)
{
	unsigned int count = (unsigned int)argc - 1;
	CHECK(count == 1 || count == 2);
	/* The GPUs as Stela's GPU service lists them: one render node each, at
	 * its own PCI function; each kernel driver's shim answers its GPUs as
	 * the device named. */
	struct stela_libc_drm_device gpus[2];
	enum image_kernel kernels[2];
	struct driver *driver_of[2];
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		const char *argument = argv[gpu + 1];
		const char *device = strchr(argument, ':');
		CHECK(device);
		size_t length = (size_t)(device - argument);
		device++;
		int intel;
		if (length == 6 && !strncmp(argument, "amdgpu", 6)) {
			kernels[gpu] = IMAGE_AMDGPU;
			intel = 0;
		} else if (length == 2 && !strncmp(argument, "xe", 2)) {
			kernels[gpu] = IMAGE_XE;
			intel = 1;
		} else {
			CHECK(length == 4 && !strncmp(argument, "i915", 4));
			kernels[gpu] = IMAGE_I915;
			intel = 1;
		}
		driver_of[gpu] = &drivers[intel];
		memset(&gpus[gpu], 0, sizeof(gpus[gpu]));
		gpus[gpu].vendor = intel ? 0x8086 : 0x1002;
		gpus[gpu].device = intel ? (uint16_t)strtoul(device, NULL, 16) : 0x15e7;
		gpus[gpu].bus = (uint8_t)(4 + gpu);
		gpus[gpu].minor = gpu;
	}
	image_host_list(gpus, kernels, count);

	/* Each driver's instance finds its own vendor's GPUs, in list order. */
	VkPhysicalDevice physicals[2];
	for (unsigned int index = 0; index < 2; index++) {
		struct driver *driver = &drivers[index];
		unsigned int wanted = 0;
		for (unsigned int gpu = 0; gpu < count; gpu++)
			wanted += driver_of[gpu] == driver;
		if (!wanted)
			continue;
		if (start(driver))
			return 1;
		use_driver(driver);
		uint32_t found = 0;
		VK(vkEnumeratePhysicalDevices(driver->instance, &found, NULL));
		CHECK(found == wanted);
		VkPhysicalDevice list[2];
		VK(vkEnumeratePhysicalDevices(driver->instance, &found, list));
		unsigned int next = 0;
		for (unsigned int gpu = 0; gpu < count; gpu++) {
			if (driver_of[gpu] != driver)
				continue;
			physicals[gpu] = list[next++];
			VkPhysicalDeviceProperties properties;
			vkGetPhysicalDeviceProperties(physicals[gpu], &properties);
			CHECK(properties.vendorID == gpus[gpu].vendor);
		}
	}
	/* A device held open on every GPU, as the GPU service holds them: a
	 * library that took a second GPU's descriptor for a GPU it already
	 * has open sends the second GPU's work to the first. */
	VkDevice held[2];
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		uint32_t family;
		use_driver(driver_of[gpu]);
		if (open_device(physicals[gpu], &family, &held[gpu]))
			return 1;
	}
	/* Each GPU's device, memory and submission reach its own render node. */
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		unsigned int before[2] = {image_host_driver_ioctls(0), image_host_driver_ioctls(1)};
		use_driver(driver_of[gpu]);
		if (work(physicals[gpu]))
			return 1;
		for (unsigned int other = 0; other < count; other++) {
			unsigned int sent = image_host_driver_ioctls(other) - before[other];
			CHECK(other == gpu ? sent > 0 : sent == 0);
		}
	}
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		use_driver(driver_of[gpu]);
		DEVICE_LOAD(held[gpu], vkDestroyDevice);
		vkDestroyDevice(held[gpu], NULL);
	}
	printf("image device:");
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		VkPhysicalDeviceProperties properties;
		use_driver(driver_of[gpu]);
		vkGetPhysicalDeviceProperties(physicals[gpu], &properties);
		printf("%s %s on %s: %s, API %u.%u, pipeline cache UUID %02x%02x%02x%02x...",
		       gpu ? ";" : "", driver_of[gpu]->name, argv[gpu + 1], properties.deviceName,
		       VK_API_VERSION_MAJOR(properties.apiVersion),
		       VK_API_VERSION_MINOR(properties.apiVersion), properties.pipelineCacheUUID[0],
		       properties.pipelineCacheUUID[1], properties.pipelineCacheUUID[2],
		       properties.pipelineCacheUUID[3]);
	}
	printf("; one submission on each waited for\n");
	for (unsigned int index = 0; index < 2; index++) {
		if (drivers[index].instance) {
			use_driver(&drivers[index]);
			vkDestroyInstance(drivers[index].instance, NULL);
		}
	}
	printf("image ioctls:");
	for (unsigned int gpu = 0; gpu < count; gpu++)
		printf(" gpu%u=%u", gpu, image_host_driver_ioctls(gpu));
	printf("\n");
	return 0;
}
