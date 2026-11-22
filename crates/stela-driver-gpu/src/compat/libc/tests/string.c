// SPDX-License-Identifier: GPL-2.0-only
/* The string functions the C library's users call that `stela.efi` takes from
 * what shares its image: Linux's lib/string.c, and the UEFI crates' wcslen.
 * A program on Linux that links the C library links these beside its host. */
#include <ctype.h>
#include <string.h>
#include <wchar.h>

void *memcpy(void *destination, const void *source, size_t size)
{
	__asm__ volatile("rep movsb" : "+D"(destination), "+S"(source), "+c"(size) : : "memory");
	return destination;
}

void *memset(void *destination, int value, size_t size)
{
	unsigned char *bytes = destination;
	while (size--)
		*bytes++ = (unsigned char)value;
	return destination;
}

void *memmove(void *destination, const void *source, size_t size)
{
	unsigned char *to = destination;
	const unsigned char *from = source;
	if (to < from) {
		while (size--)
			*to++ = *from++;
	} else {
		while (size--)
			to[size] = from[size];
	}
	return destination;
}

int memcmp(const void *a, const void *b, size_t size)
{
	const unsigned char *left = a, *right = b;
	for (size_t at = 0; at < size; at++) {
		if (left[at] != right[at])
			return left[at] - right[at];
	}
	return 0;
}

void *memchr(const void *from, int value, size_t size)
{
	const unsigned char *at = from;
	for (; size; at++, size--) {
		if (*at == (unsigned char)value)
			return (void *)at;
	}
	return NULL;
}

/* What compilers turn a memcmp's equality into. */
int bcmp(const void *left, const void *right, size_t size)
{
	return memcmp(left, right, size);
}

size_t strlen(const char *text)
{
	const char *at = text;
	while (*at)
		at++;
	return (size_t)(at - text);
}

size_t strnlen(const char *text, size_t limit)
{
	size_t size = 0;
	while (size < limit && text[size])
		size++;
	return size;
}

char *stpcpy(char *to, const char *from)
{
	while ((*to = *from++))
		to++;
	return to;
}

char *strcpy(char *to, const char *from)
{
	stpcpy(to, from);
	return to;
}

char *strcat(char *to, const char *from)
{
	strcpy(to + strlen(to), from);
	return to;
}

char *strncat(char *to, const char *from, size_t size)
{
	char *end = to + strlen(to);
	size_t length = strnlen(from, size);
	memcpy(end, from, length);
	end[length] = 0;
	return to;
}

int strcmp(const char *a, const char *b)
{
	for (; *a && *a == *b; a++, b++)
		;
	return (unsigned char)*a - (unsigned char)*b;
}

int strncmp(const char *a, const char *b, size_t size)
{
	for (; size && *a && *a == *b; a++, b++, size--)
		;
	return size ? (unsigned char)*a - (unsigned char)*b : 0;
}

int strcasecmp(const char *a, const char *b)
{
	for (; *a && tolower((unsigned char)*a) == tolower((unsigned char)*b); a++, b++)
		;
	return tolower((unsigned char)*a) - tolower((unsigned char)*b);
}

int strncasecmp(const char *a, const char *b, size_t size)
{
	for (; size && *a && tolower((unsigned char)*a) == tolower((unsigned char)*b);
	     a++, b++, size--)
		;
	return size ? tolower((unsigned char)*a) - tolower((unsigned char)*b) : 0;
}

char *strchr(const char *text, int c)
{
	for (;; text++) {
		if (*text == (char)c)
			return (char *)text;
		if (!*text)
			return NULL;
	}
}

char *strrchr(const char *text, int c)
{
	const char *found = NULL;
	do {
		if (*text == (char)c)
			found = text;
	} while (*text++);
	return (char *)found;
}

char *strstr(const char *text, const char *needle)
{
	return memmem(text, strlen(text), needle, strlen(needle));
}

size_t strspn(const char *text, const char *accept)
{
	size_t size = 0;
	while (text[size] && strchr(accept, text[size]))
		size++;
	return size;
}

size_t strcspn(const char *text, const char *reject)
{
	size_t size = 0;
	while (text[size] && !strchr(reject, text[size]))
		size++;
	return size;
}

char *strpbrk(const char *text, const char *accept)
{
	text += strcspn(text, accept);
	return *text ? (char *)text : NULL;
}

size_t wcslen(const wchar_t *text)
{
	size_t length = 0;
	while (text[length])
		length++;
	return length;
}
