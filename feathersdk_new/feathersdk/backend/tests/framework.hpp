// Some basic things to make testing C code easier
#pragma once

#define _DEFAULT_SOURCE 1  // Need to define this to get DT_REG file types. Leave this at top of file

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <dirent.h>
#include <ftw.h>
#include <signal.h>
#include <limits.h>

#define TR_EM_SIZE 1024
#define TEST_ERR_MSG_TMP_SIZE 16384
#define UNIT_TEST_MAX 128

typedef struct {
    char err_message[TR_EM_SIZE];   // The reason a test failed
    const char* func;               // The test function name
    const char* file;               // The test file name
    int32_t line;                   // The line number of the assert statement
    bool success;                   // 1 if test was successful, 0 otherwise
} test_result_t;

typedef test_result_t (*unit_test_fn_t)(void);

#define TEST_SUCCESS \
    /* The test_result_t to return on test success */ \
    ((test_result_t){.func=__FUNCTION__, .file=__FILE__, .line=__LINE__, .success=1})

// There was an error with the test itself
#define TEST_TESTING_ERROR(message) ((test_result_t){.func=__FUNCTION__, .file=__FILE__, .line=__LINE__, .success=1})

#define _TEST_ASSERT_ERR_STR "In file %s:%d\n\t\t\tFunction %s(): \n\t\t\tFailed Condition: ```%s```  [errno: %d, errno_message: '%s']\n\t\t\tmessage: "

#define TEST_ASSERT_ONERR_NOERR(condition, onerr, noerr, message, ...)  { \
    /* Do a TEST_ASSERT. On an error, run the `onerr` code. On no error, run the `noerr` code. */ \
    if (!(condition)) { \
        test_result_t __ret_trt__ = ((test_result_t){.func=__FUNCTION__, .file=__FILE__, .line=__LINE__, .success=0}); \
        char tmp_message[TEST_ERR_MSG_TMP_SIZE]; \
        snprintf(tmp_message, TEST_ERR_MSG_TMP_SIZE, _TEST_ASSERT_ERR_STR message, __FILE__, __LINE__, __FUNCTION__, #condition, errno, strerror(errno), ##__VA_ARGS__); \
        strncpy(&tmp_message[TEST_ERR_MSG_TMP_SIZE - 4], "...", 3); \
        strncpy(__ret_trt__.err_message, tmp_message, TR_EM_SIZE); \
        onerr; return __ret_trt__; \
    } else { \
        noerr; \
    } \
}

#define TEST_ASSERT_ONERR(condition, onerr, message, ...) \
    /* Do a TEST_ASSERT, but on an error, run the `onerr` code. Useful for closing files on error, etc. */ \
    TEST_ASSERT_ONERR_NOERR(condition, onerr, ;, message, ##__VA_ARGS__);

#define TEST_ASSERT_FINALLY(condition, finally, message, ...) \
    /* Do a TEST_ASSERT, and no matter whether there is an error or not, run the `finally` code before returning. */ \
    TEST_ASSERT_ONERR_NOERR(condition, finally, finally, message, ##__VA_ARGS__);

#define TEST_ASSERT(condition, message, ...) \
    /* Check that the given condition is true. If false, return an error message test result (whose message can be formatted) */ \
    TEST_ASSERT_FINALLY(condition, ;, message, ##__VA_ARGS__);

#define TEST_ASSERT_STACKED(test_result, message, ...) \
    /* Do a TEST_ASSERT on a test_result_t, checking that it is a success. If not, will do a simple stacktrace */ \
    TEST_ASSERT(test_result.success == 1, message "\nError caused by: %s", ##__VA_ARGS__, test_result.err_message)


///////////////////////
// Registering Tests //
///////////////////////


static unit_test_fn_t _unit_test_fns[UNIT_TEST_MAX];
static const char* _unit_test_names[UNIT_TEST_MAX];
static size_t _unit_test_count = 0;

static inline void _unit_test_register(unit_test_fn_t fn, const char* name) {
    if (_unit_test_count >= UNIT_TEST_MAX) {
        fprintf(stderr, "Too many unit tests (max %d)\n", UNIT_TEST_MAX);
        abort();
    }
    _unit_test_fns[_unit_test_count++] = fn;
    _unit_test_names[_unit_test_count - 1] = name;
}

/**
 * Auto-registering test. Use as: DECLARE_TEST(my_test) { ...; return TEST_SUCCESS; }
 * Requires GCC/Clang. Call RUN_TESTS() once at the end of the file.
 */
#define DECLARE_TEST(name)                                                  \
    static test_result_t name(void);                                        \
    static void __attribute__((constructor)) _unit_test_reg_##name(void) {  \
        _unit_test_register(name, #name);                                   \
    }                                                                       \
    static test_result_t name(void)


////////////////////////////
// Running Unit Test File //
////////////////////////////


/**
 * Write into `store` (at least 1024 bytes) the path of `file_path` relative to the
 * parent of this header's directory (the backend root).
 */
 static inline void _get_file_relative_path(const char* file_path, char* store) {
    char file_abs[PATH_MAX];
    char fw_abs[PATH_MAX];
    char anchor[PATH_MAX];

    if (realpath(file_path, file_abs) == NULL) {
        snprintf(store, 1024, "%s", file_path);
        return;
    }

    // Resolve this header to an absolute path. __FILE__ is often a relative include
    // path (e.g. "../../framework.hpp"), so join it with the test file's directory.
    if (realpath(__FILE__, fw_abs) == NULL) {
        char dir[PATH_MAX];
        snprintf(dir, sizeof(dir), "%s", file_abs);
        char* slash = strrchr(dir, '/');
        if (slash != NULL) {
            *slash = '\0';
        }
        char joined[PATH_MAX];
        if (__FILE__[0] == '/') {
            snprintf(joined, sizeof(joined), "%s", __FILE__);
        } else {
            snprintf(joined, sizeof(joined), "%s/%s", dir, __FILE__);
        }
        if (realpath(joined, fw_abs) == NULL) {
            snprintf(store, 1024, "%s", file_abs);
            return;
        }
    }

    // anchor = dirname(dirname(framework.hpp)) → parent of the tests/ directory
    snprintf(anchor, sizeof(anchor), "%s", fw_abs);
    char* slash = strrchr(anchor, '/');
    if (slash != NULL) {
        *slash = '\0';
    }
    slash = strrchr(anchor, '/');
    if (slash != NULL) {
        *slash = '\0';
    }

    size_t anchor_len = strlen(anchor);
    if (strncmp(file_abs, anchor, anchor_len) == 0 && file_abs[anchor_len] == '/') {
        snprintf(store, 1024, "%s", file_abs + anchor_len + 1);
    } else {
        snprintf(store, 1024, "%s", file_abs);
    }
}

static inline int run_tests(unit_test_fn_t* functions, size_t num_functions) {
    // Find the maximum length of the test names to rjust the PASSED messages
    uint32_t max_name_length = 0;
    for (size_t i = 0; i < num_functions; i++) {
        size_t name_length = strlen(_unit_test_names[i]);
        max_name_length = name_length > max_name_length ? name_length : max_name_length;
    }

    bool all_passed = true;
    for (size_t i = 0; i < num_functions; i++) {
        printf("\t%s...", _unit_test_names[i]);
        size_t name_length = strlen(_unit_test_names[i]);
        test_result_t result = functions[i]();

        if (!result.success) {
            all_passed = false;
            printf("\n\t\t[FAILURE] %s\n\n", result.err_message);
        } else {
            printf("%*sPASSED\n", (int)(max_name_length - name_length + 6), "");
        }
    }
    return all_passed ? 0 : 1;
}

#define RETURN_NONZERO_EXIT_CODE_ON_ERROR true

static inline int _unit_test_main(const char* test_file) {
    char relative_path[1024];
    _get_file_relative_path(test_file, relative_path);
    printf("UNIT TEST - %s (%d tests)\n", relative_path, (int)_unit_test_count);
    int result = run_tests(_unit_test_fns, _unit_test_count);
    return RETURN_NONZERO_EXIT_CODE_ON_ERROR ? result : 0;
}

#define RUN_TESTS()                                                  \
    /* Runs all unit tests. Put at the end of any unit test file. */ \
    int main(void) { return _unit_test_main(__FILE__); }
