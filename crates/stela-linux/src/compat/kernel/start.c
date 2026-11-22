// SPDX-License-Identifier: GPL-2.0-or-later
/* Driver initcalls run after the original Initial resource bootstrap.
 * Stop at the first failed stage and preserve its error. The command line
 * is the kernel's: `length` bytes of space-separated `module.parameter=value`
 * words without a NUL, which the registry copies and parses into the
 * built-in modules' parameters before any initcall, as start_kernel does. A
 * word no module declares fails the start, as does a value its parameter
 * refuses; the word, or the refusing parameter's name, is then copied into
 * `rejected` (`rejected_size` bytes, NUL-terminated, cut to fit). */
#include <linux/errno.h>
#include <linux/string.h>
#include <stela/registry.h>

struct rejection {
	char *bytes;
	size_t size;
};

static void name_rejected(struct rejection *rejection, const char *word)
{
	if (rejection->bytes && rejection->size && !rejection->bytes[0])
		strscpy(rejection->bytes, word, rejection->size);
}

static int unknown_word(void *context, char *complete_option)
{
	name_rejected(context, complete_option);
	return -ENOENT;
}

/* The parameter whose own setter refused its value: the registry's status
 * of each parameter record, which follow the init and exit records. */
static void refused_parameter(const struct stela_linux_registry_image *image,
			      const struct stela_linux_registry *registry,
			      struct rejection *rejection)
{
	size_t first = image->init_count + image->exit_count;

	for (size_t i = 0; i < image->parameter_count; i++) {
		struct stela_linux_registration_status status;

		if (!stela_linux_registry_status(registry, first + i, &status) &&
		    status.state == STELA_LINUX_RECORD_NEGATIVE) {
			name_rejected(rejection, image->parameters[i]->label);
			return;
		}
	}
}

int stela_linux_start(const struct stela_linux_registry_image *image,
		      const char *command_line, size_t length,
		      struct stela_linux_registry **result, char *rejected,
		      size_t rejected_size)
{
	struct rejection rejection = {.bytes = rejected, .size = rejected_size};
	const struct stela_linux_registry_unknown_options words = {
		.unhandled_option = unknown_word,
		.context = &rejection,
	};
	int error;

	if (rejected && rejected_size)
		rejected[0] = 0;
	error = stela_linux_registry_begin(image, command_line, length, result);
	if (!error)
		error = stela_linux_registry_early_parameters(*result, NULL);
	if (!error) {
		error = stela_linux_registry_initial_parameters(*result, &words, NULL);
		if (error)
			refused_parameter(image, *result, &rejection);
	}
	for (unsigned int stage = 0; !error && stage < STELA_LINUX_STAGE_DONE; stage++)
		error = stela_linux_registry_run_stage(*result, stage, NULL);
	return error;
}
