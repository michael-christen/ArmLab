#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <regex.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>

#include "string_util.h"
#include "zarray.h"

struct string_buffer
{
    char *s;
    int alloc;
    size_t size; // as if strlen() was called; not counting terminating \0
};

struct string_feeder
{
    char *s;
    size_t len;
    size_t pos;

    int line, col;
};

#define MIN_PRINTF_ALLOC 16

/** Allocate a string and call sprintf, resizing if necessary and calling
 *  sprintf again.  Returns a char* that must be freed by the caller.
 */
char *sprintf_alloc(const char *fmt, ...)
{
    assert(fmt != NULL);

    int size = MIN_PRINTF_ALLOC;
    char *buf = malloc(size * sizeof(char));

    int returnsize;
    va_list args;

    va_start(args,fmt);
    returnsize = vsnprintf(buf, size, fmt, args);
    va_end(args);

    // it was successful
    if (returnsize < size) {
        return buf;
    }

    // otherwise, we should try again
    free(buf);
    size = returnsize + 1;
    buf = malloc(size * sizeof(char));

    va_start(args,fmt);
    returnsize = vsnprintf(buf, size, fmt, args);
    va_end(args);

    return buf;
}

char *_str_concat_private(const char *first, ...)
{
    size_t len = 0;

    // get the total length (for the allocation)
    {
        va_list args;
        va_start(args, first);
        const char *arg = first;
        while(arg != NULL) {
            len += strlen(arg);
            arg = va_arg(args, const char *);
        }
        va_end(args);
    }

    // write the string
    char *str = malloc(len*sizeof(char) + 1);
    char *ptr = str;
    {
        va_list args;
        va_start(args, first);
        const char *arg = first;
        while(arg != NULL) {
            while(*arg)
                *ptr++ = *arg++;
            arg = va_arg(args, const char *);
        }
        *ptr = '\0';
        va_end(args);
    }

    return str;
}

zarray_t *str_split(const char *str, const char *delim)
{
    assert(str != NULL);
    assert(delim != NULL);

    zarray_t *parts = zarray_create(sizeof(char*));
    string_buffer_t *sb = string_buffer_create();

    size_t delim_len = strlen(delim);
    size_t len = strlen(str);
    size_t pos = 0;

    while (pos < len) {
        if (str_starts_with(&str[pos], delim) && delim_len > 0) {
            pos += delim_len;
            // never add empty strings (repeated tokens)
            if (string_buffer_size(sb) > 0) {
                char *part = string_buffer_to_string(sb);
                zarray_add(parts, &part);
            }
            string_buffer_reset(sb);
        } else {
            string_buffer_append(sb, str[pos]);
            pos++;
        }
    }

    if (string_buffer_size(sb) > 0) {
        char *part = string_buffer_to_string(sb);
        zarray_add(parts, &part);
    }

    string_buffer_destroy(sb);
    return parts;
}

zarray_t *str_match_regex(const char *str, const char *regex)
{
    assert(str != NULL);
    assert(regex != NULL);

    regex_t regex_comp;
    if (regcomp(&regex_comp, regex, REG_EXTENDED) != 0)
        return NULL;

    zarray_t *matches = zarray_create(sizeof(char*));

    void *ptr = (void*) str;
    regmatch_t match;
    while (regexec(&regex_comp, ptr, 1, &match, 0) != REG_NOMATCH)
    {
        if (match.rm_so == -1)
            break;

        if (match.rm_so == match.rm_eo) // ambiguous regex
            break;

        int   length = match.rm_eo - match.rm_so;
        char *substr = malloc(length+1);
        bzero(substr, length+1);

        memcpy(substr, ptr+match.rm_so, length);

        ptr = ptr + match.rm_eo;

        zarray_add(matches, &substr);
    }

    regfree(&regex_comp);
    return matches;
}

zarray_t *str_split_regex(const char *str, const char *regex)
{
    assert(str != NULL);
    assert(regex != NULL);

    regex_t regex_comp;
    if (regcomp(&regex_comp, regex, REG_EXTENDED) != 0)
        return NULL;

    zarray_t *matches = zarray_create(sizeof(char*));

    int origlen = strlen(str);
    void *end = ((void*) str) + origlen;
    void *ptr = (void*) str;

    regmatch_t match;
    while (regexec(&regex_comp, ptr, 1, &match, 0) != REG_NOMATCH)
    {
        //printf("so %3d eo %3d\n", match.rm_so, match.rm_eo);

        if (match.rm_so == -1)
            break;

        if (match.rm_so == match.rm_eo) // ambiguous regex
            break;

        void *so = ((void*) ptr) + match.rm_so;

        int length = so - ptr;

        if (length > 0) {
            char *substr = malloc(length+1);
            bzero(substr, length+1);
            memcpy(substr, ptr, length);

            //printf("match '%s'\n", substr);
            //fflush(stdout);

            zarray_add(matches, &substr);
        }

        ptr = ptr + match.rm_eo;
    }

    if (ptr - (void*) str != origlen) {
        int length = end - ptr;
        char *substr = malloc(length+1);
        bzero(substr, length+1);

        memcpy(substr, ptr, length);

        zarray_add(matches, &substr);
    }


    regfree(&regex_comp);
    return matches;
}

char *str_trim(char *str)
{
    assert(str != NULL);

    return str_lstrip(str_rstrip(str));
}

char *str_lstrip(char *str)
{
    assert(str != NULL);

    char *ptr = str;
    char *end = str + strlen(str);
    for(; ptr != end && isspace(*ptr); ptr++);
    // shift the string to the left so the original pointer still works
    memmove(str, ptr, strlen(ptr)+1);
    return str;
}

char *str_rstrip(char *str)
{
    assert(str != NULL);

    char *ptr = str + strlen(str) - 1;
    for(; ptr+1 != str && isspace(*ptr); ptr--);
    *(ptr+1) = '\0';
    return str;
}

int str_indexof(const char *haystack, const char *needle)
{
	assert(haystack != NULL);
	assert(needle != NULL);

    int hlen = strlen(haystack);
    int nlen = strlen(needle);

    for (int i = 0; i <= hlen - nlen; i++) {
        if (!strncmp(&haystack[i], needle, nlen))
            return i;
    }

    return -1;
}

// in-place modification.
char *str_tolowercase(char *s)
{
	assert(s != NULL);

    int slen = strlen(s);
    for (int i = 0; i < slen; i++) {
        if (s[i] >= 'A' && s[i] <= 'Z')
            s[i] = s[i] + 'a' - 'A';
    }

    return s;
}

string_buffer_t* string_buffer_create()
{
    string_buffer_t *sb = (string_buffer_t*) calloc(1, sizeof(string_buffer_t));
    assert(sb != NULL);
    sb->alloc = 32;
    sb->s = calloc(sb->alloc, 1);
    return sb;
}

void string_buffer_destroy(string_buffer_t *sb)
{
    if (sb == NULL)
        return;

    if (sb->s)
        free(sb->s);

    free(sb);
}

void string_buffer_append(string_buffer_t *sb, char c)
{
    assert(sb != NULL);

    if (sb->size+2 >= sb->alloc) {
        sb->alloc *= 2;
        sb->s = realloc(sb->s, sb->alloc);
    }

    sb->s[sb->size++] = c;
    sb->s[sb->size] = 0;
}

void string_buffer_appendf(string_buffer_t *sb, const char *fmt, ...)
{
    assert(sb != NULL);
    assert(fmt != NULL);

    int size = MIN_PRINTF_ALLOC;
    char *buf = malloc(size * sizeof(char));

    int returnsize;
    va_list args;

    va_start(args,fmt);
    returnsize = vsnprintf(buf, size, fmt, args);
    va_end(args);

    if (returnsize >= size) {
        // otherwise, we should try again
        free(buf);
        size = returnsize + 1;
        buf = malloc(size * sizeof(char));

        va_start(args, fmt);
        returnsize = vsnprintf(buf, size, fmt, args);
        va_end(args);
    }

    string_buffer_append_string(sb, buf);
    free(buf);
}

void string_buffer_append_string(string_buffer_t *sb, const char *str)
{
    assert(sb != NULL);
    assert(str != NULL);

    size_t len = strlen(str);

    while (sb->size+len + 1 >= sb->alloc) {
        sb->alloc *= 2;
        sb->s = realloc(sb->s, sb->alloc);
    }

    memcpy(&sb->s[sb->size], str, len);
    sb->size += len;
    sb->s[sb->size] = 0;
}

int string_buffer_ends_with(string_buffer_t *sb, const char *str)
{
    assert(sb != NULL);
    assert(str != NULL);

    return str_ends_with(sb->s, str);
}

char *string_buffer_to_string(string_buffer_t *sb)
{
    assert(sb != NULL);

    return strdup(sb->s);
}

// returns length of string (not counting \0)
size_t string_buffer_size(string_buffer_t *sb)
{
    assert(sb != NULL);

    return sb->size;
}

void string_buffer_reset(string_buffer_t *sb)
{
    assert(sb != NULL);

    sb->s[0] = 0;
    sb->size = 0;
}

string_feeder_t *string_feeder_create(const char *str)
{
    assert(str != NULL);

    string_feeder_t *sf = (string_feeder_t*) calloc(1, sizeof(string_feeder_t));
    sf->s = strdup(str);
    sf->len = strlen(sf->s);
    sf->line = 1;
    sf->col = 0;
    sf->pos = 0;
    return sf;
}

int string_feeder_get_line(string_feeder_t *sf)
{
    assert(sf != NULL);
    return sf->line;
}

int string_feeder_get_column(string_feeder_t *sf)
{
    assert(sf != NULL);
    return sf->col;
}

void string_feeder_destroy(string_feeder_t *sf)
{
    if (sf == NULL)
        return;

    free(sf->s);
    free(sf);
}

int string_feeder_has_next(string_feeder_t *sf)
{
    assert(sf != NULL);

    return sf->s[sf->pos] != 0 && sf->pos <= sf->len;
}

char string_feeder_next(string_feeder_t *sf)
{
    assert(sf != NULL);
    assert(sf->pos <= sf->len);

    char c = sf->s[sf->pos++];
    if (c == '\n') {
        sf->line++;
        sf->col = 0;
    } else {
        sf->col++;
    }

    return c;
}

char *string_feeder_next_length(string_feeder_t *sf, int length)
{
    assert(sf != NULL);
    assert(length >= 0);
    assert(sf->pos <= sf->len);

    if (sf->pos + length > sf->len)
        length = sf->len - sf->pos;

    char *substr = calloc(length+1, sizeof(char));
    for (int i = 0 ; i < length ; i++)
        substr[i] = string_feeder_next(sf);
    return substr;
}

char string_feeder_peek(string_feeder_t *sf)
{
    assert(sf != NULL);
    assert(sf->pos <= sf->len);

    return sf->s[sf->pos];
}

char *string_feeder_peek_length(string_feeder_t *sf, int length)
{
    assert(sf != NULL);
    assert(length >= 0);
    assert(sf->pos <= sf->len);

    if (sf->pos + length > sf->len)
        length = sf->len - sf->pos;

    char *substr = calloc(length+1, sizeof(char));
    memcpy(substr, &sf->s[sf->pos], length*sizeof(char));
    return substr;
}

int string_feeder_starts_with(string_feeder_t *sf, const char *str)
{
    assert(sf != NULL);
    assert(str != NULL);
    assert(sf->pos <= sf->len);

    return str_starts_with(&sf->s[sf->pos], str);
}

void string_feeder_require(string_feeder_t *sf, const char *str)
{
    assert(sf != NULL);
    assert(str != NULL);
    assert(sf->pos <= sf->len);

    int len = strlen(str);

    for (int i = 0; i < len; i++) {
        char c = string_feeder_next(sf);
        assert(c == str[i]);
    }
}

////////////////////////////////////////////
int str_ends_with(const char *haystack, const char *needle)
{
    assert(haystack != NULL);
    assert(needle != NULL);

    size_t lens = strlen(haystack);
    size_t lenneedle = strlen(needle);

    if (lenneedle > lens)
        return 0;

    return !strncmp(&haystack[lens - lenneedle], needle, lenneedle);
}

int str_starts_with(const char *haystack, const char *needle)
{
    assert(haystack != NULL);
    assert(needle != NULL);

    size_t lenneedle = strlen(needle);

    return !strncmp(haystack, needle, lenneedle);
}

char *str_substring(const char *str, size_t startidx, long endidx)
{
    assert(str != NULL);
    assert(startidx >= 0 && startidx <= strlen(str)+1);
    assert(endidx < 0 || endidx >= startidx);
    assert(endidx < 0 || endidx <= strlen(str)+1);

    if (endidx < 0)
        endidx = (long) strlen(str);

    size_t blen = endidx - startidx; // not counting \0
    char *b = malloc(blen + 1);
    memcpy(b, &str[startidx], blen);
    b[blen] = 0;
    return b;
}

char *str_replace(const char *haystack, const char *needle, const char *replacement)
{
    assert(haystack != NULL);
    assert(needle != NULL);
    assert(replacement != NULL);

    string_buffer_t *sb = string_buffer_create();
    size_t haystack_len = strlen(haystack);
    size_t needle_len = strlen(needle);

    int pos = 0;
    while (pos < haystack_len) {
        if (needle_len > 0 && str_starts_with(&haystack[pos], needle)) {
            string_buffer_append_string(sb, replacement);
            pos += needle_len;
        } else {
            string_buffer_append(sb, haystack[pos]);
            pos++;
        }
    }
    if (needle_len == 0 && haystack_len == 0)
        string_buffer_append_string(sb, replacement);

    char *res = string_buffer_to_string(sb);
    string_buffer_destroy(sb);
    return res;
}
