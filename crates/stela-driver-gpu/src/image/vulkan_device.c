// SPDX-License-Identifier: GPL-2.0-or-later
/* The image's own Vulkan driver, C library and defines on the build host
 * (`cargo xtask gpu image-test`): this program links RADV's and ANV's
 * objects as the image compiles them, only built for Linux instead of the
 * image's COFF target, with the image's C library under them and host.c
 * under that, and runs one driver over render nodes Mesa's DRM shim answers.
 * It makes the instance and every physical device, then on each GPU in turn
 * the device, host-visible memory it maps, writes and maps again, and one
 * submission of a fill of a buffer, then waits for it; each GPU's work must
 * reach that GPU's render node and no other. What differs from a Linux host
 * in the image (a probe define dropped, a C library call that fails or
 * answers a fixed value) shows here as it would on a GPU, short of what only
 * a GPU's work shows.
 *
 * Usage: image_device radv <amdgpu device name> <GPUs: 1 or 2>
 *        image_device anv <PCI device ID, hex> 1
 * Prints "image device: ..." and "image ioctls: ..." and exits 0, or names
 * the step that failed and exits 1. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define VK_NO_PROTOTYPES
#include <vulkan/vulkan.h>

#include <stela/host.h>

VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL radv_icdGetInstanceProcAddr(VkInstance, const char *);
VKAPI_ATTR PFN_vkVoidFunction VKAPI_CALL anv_icdGetInstanceProcAddr(VkInstance, const char *);
void image_host_list(const struct stela_libc_drm_device *gpus, unsigned int count, int intel);
unsigned int image_host_driver_ioctls(unsigned int gpu);

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

static PFN_vkGetInstanceProcAddr entry;
static PFN_vkCreateInstance vkCreateInstance;
static PFN_vkEnumeratePhysicalDevices vkEnumeratePhysicalDevices;
static PFN_vkGetPhysicalDeviceProperties vkGetPhysicalDeviceProperties;
static PFN_vkGetPhysicalDeviceQueueFamilyProperties vkGetPhysicalDeviceQueueFamilyProperties;
static PFN_vkGetPhysicalDeviceMemoryProperties vkGetPhysicalDeviceMemoryProperties;
static PFN_vkCreateDevice vkCreateDevice;
static PFN_vkGetDeviceProcAddr vkGetDeviceProcAddr;
static PFN_vkDestroyInstance vkDestroyInstance;

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
	CHECK(argc == 4);
	int intel = !strcmp(argv[1], "anv");
	CHECK(intel || !strcmp(argv[1], "radv"));
	unsigned int count = (unsigned int)strtoul(argv[3], NULL, 10);
	CHECK(count == 1 || count == 2);
	/* The GPUs as Stela's GPU service lists them: one render node each, at
	 * its own PCI function; the shim answers each as the device named. */
	struct stela_libc_drm_device gpus[2];
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		memset(&gpus[gpu], 0, sizeof(gpus[gpu]));
		gpus[gpu].vendor = intel ? 0x8086 : 0x1002;
		gpus[gpu].device = intel ? (uint16_t)strtoul(argv[2], NULL, 16) : 0x15e7;
		gpus[gpu].bus = (uint8_t)(intel ? 0 : 4 + gpu);
		gpus[gpu].slot = (uint8_t)(intel ? 2 : 0);
		gpus[gpu].minor = gpu;
	}
	image_host_list(gpus, count, intel);
	entry = intel ? anv_icdGetInstanceProcAddr : radv_icdGetInstanceProcAddr;

	LOAD(NULL, vkCreateInstance);
	VkInstance instance;
	VK(vkCreateInstance(
		&(VkInstanceCreateInfo){
			.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO,
			.pApplicationInfo =
				&(VkApplicationInfo){
					.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO,
					.apiVersion = VK_API_VERSION_1_3,
				},
		},
		NULL, &instance));
	LOAD(instance, vkEnumeratePhysicalDevices);
	LOAD(instance, vkGetPhysicalDeviceProperties);
	LOAD(instance, vkGetPhysicalDeviceQueueFamilyProperties);
	LOAD(instance, vkGetPhysicalDeviceMemoryProperties);
	LOAD(instance, vkCreateDevice);
	LOAD(instance, vkGetDeviceProcAddr);
	LOAD(instance, vkDestroyInstance);

	uint32_t physical_count = 0;
	VK(vkEnumeratePhysicalDevices(instance, &physical_count, NULL));
	CHECK(physical_count == count);
	VkPhysicalDevice physicals[2];
	VK(vkEnumeratePhysicalDevices(instance, &physical_count, physicals));
	VkPhysicalDeviceProperties properties;
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		vkGetPhysicalDeviceProperties(physicals[gpu], &properties);
		CHECK(properties.vendorID == gpus[gpu].vendor);
	}
	/* A device held open on every GPU, as the GPU service holds them: a
	 * library that took a second GPU's descriptor for a GPU it already
	 * has open sends the second GPU's work to the first. */
	VkDevice held[2];
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		uint32_t family;
		if (open_device(physicals[gpu], &family, &held[gpu]))
			return 1;
	}
	/* Each GPU's device, memory and submission reach its own render node. */
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		unsigned int before[2] = {image_host_driver_ioctls(0), image_host_driver_ioctls(1)};
		if (work(physicals[gpu]))
			return 1;
		for (unsigned int other = 0; other < count; other++) {
			unsigned int sent = image_host_driver_ioctls(other) - before[other];
			CHECK(other == gpu ? sent > 0 : sent == 0);
		}
	}
	for (unsigned int gpu = 0; gpu < count; gpu++) {
		DEVICE_LOAD(held[gpu], vkDestroyDevice);
		vkDestroyDevice(held[gpu], NULL);
	}
	vkGetPhysicalDeviceProperties(physicals[0], &properties);
	vkDestroyInstance(instance, NULL);

	printf("image device: %s on %u GPU%s: %s, API %u.%u, pipeline cache UUID "
	       "%02x%02x%02x%02x..., one submission on each waited for\n",
	       argv[1], count, count == 1 ? "" : "s", properties.deviceName,
	       VK_API_VERSION_MAJOR(properties.apiVersion),
	       VK_API_VERSION_MINOR(properties.apiVersion), properties.pipelineCacheUUID[0],
	       properties.pipelineCacheUUID[1], properties.pipelineCacheUUID[2],
	       properties.pipelineCacheUUID[3]);
	printf("image ioctls:");
	for (unsigned int gpu = 0; gpu < count; gpu++)
		printf(" gpu%u=%u", gpu, image_host_driver_ioctls(gpu));
	printf("\n");
	return 0;
}
