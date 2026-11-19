// Some basic things to make testing easier
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

#include "../src/logging.hpp"
#include "../src/comms/ethernet.hpp"

// Compound literals are rvalues in C++, so the default config has to be a real object.
#define SET_DEFAULT_COMMS_CONFIG() \
    do { \
        CommsConfig _default_comms_config = DEFAULT_COMMS_CONFIG; \
        set_comms_config(&_default_comms_config); \
    } while (0)

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

#define TEST_ASSERT_NO_ERROR_MESSAGES(func_name) TEST_ASSERT(get_local_error_msg() == NULL && get_global_error_msg() == NULL, \
    /* Assert that both the local and global error messages are clear.*/ \
    "Function `%s()` ended with error message(s): \n\t\t\t\tLocal err msg: \"\"\"%s\"\"\"\n\t\t\t\tglobal err msg: \"\"\"%s\"\"\"", \
    func_name, get_local_error_msg(), get_global_error_msg())

#define TEST_ASSERT_LOCAL_ERROR_MESSAGE(err_msg) \
    /* Assert that the global error message is clear, and the local error message contains the given string. Clears the local error message after.*/ \
    TEST_ASSERT(get_global_error_msg() == NULL, "global error message was produced: \"%s\"", get_global_error_msg()); \
    TEST_ASSERT(get_local_error_msg() != NULL, "No local error message was produced"); \
    TEST_ASSERT(strstr(get_local_error_msg(), err_msg) != NULL, "Expected local error containing: \"%s\"\n\t\t\t\tGot: \"%s\"", err_msg, get_local_error_msg()); \
    clear_local_error_msg(); \
    TEST_ASSERT(get_local_error_msg() == NULL, "Local error message wasn't cleared"); \
    TEST_ASSERT(get_global_error_msg() == NULL, "global error message wasn't cleared"); \

#define TEST_ASSERT_GLOBAL_ERROR_MESSAGE(err_msg) \
    /* Assert that the local error message is clear, and the global error message contains the given string. Clears the global error message after.*/ \
    TEST_ASSERT(get_local_error_msg() == NULL, "local error message was produced: \"%s\"", get_local_error_msg()); \
    TEST_ASSERT(get_global_error_msg() != NULL, "No global error message was produced"); \
    TEST_ASSERT(strstr(get_global_error_msg(), err_msg) != NULL, "Expected global error containing: \"%s\"\n\t\t\t\tGot: \"%s\"", err_msg, get_global_error_msg()); \
    clear_global_error_msg(); \
    TEST_ASSERT(get_local_error_msg() == NULL, "local error message wasn't cleared"); \
    TEST_ASSERT(get_global_error_msg() == NULL, "global error message wasn't cleared");


////////////////////////////////////
// Ethernet Write Buffer Checking //
////////////////////////////////////


extern uint8_t eth_write_buf[ETH_WRITE_BUF_SIZE];
extern uint32_t eth_write_buf_start;
extern uint32_t eth_write_buf_end;

/**
 * Check that the Ethernet write buffer contains the expected data. Automatically inserts the header before the
 * expected data.
 *
 * @param msg_id The message ID of the expected data.
 * @param opcode The opcode of the expected data.
 * @param expected The expected data.
 * @param expected_size The size of the expected data.
 * @return A test_result_t indicating whether the Ethernet write buffer contains the expected data.
 */
static inline test_result_t check_eth_write_buf_eq(
    uint32_t msg_id, uint32_t opcode, const void* expected, uint32_t expected_size
) {
    TEST_ASSERT(expected_size < ETH_WRITE_BUF_SIZE - sizeof(ClientCommandHeader),
        "`check_eth_write_buf_eq` expected_size must be < %u", (uint32_t)(ETH_WRITE_BUF_SIZE - sizeof(ClientCommandHeader)));
    TEST_ASSERT(expected_size == 0 || expected != NULL, "`check_eth_write_buf_eq` expected must be non-NULL when expected_size > 0");

    ClientCommandHeader hdr = {
        .msg_id = msg_id, .opcode = (TeensyCommandOpcode)opcode, .payload_size = expected_size
    };
    uint32_t total_size = (uint32_t)sizeof(hdr) + expected_size;
    uint8_t full_expected[ETH_WRITE_BUF_SIZE];
    memcpy(full_expected, &hdr, sizeof(hdr));
    if (expected_size > 0) {
        memcpy(full_expected + sizeof(hdr), expected, expected_size);
    }

    uint32_t eth_buf_size = (eth_write_buf_end - eth_write_buf_start + ETH_WRITE_BUF_SIZE) % ETH_WRITE_BUF_SIZE;
    TEST_ASSERT(eth_buf_size == total_size, "eth write buf size was %u, expected %u", eth_buf_size, total_size);

    // Copy the ring buffer into a contiguous buffer
    uint8_t curr_buf[ETH_WRITE_BUF_SIZE];
    if (eth_write_buf_end >= eth_write_buf_start) {
        memcpy(curr_buf, eth_write_buf + eth_write_buf_start, eth_write_buf_end - eth_write_buf_start);
    } else {
        memcpy(curr_buf, eth_write_buf + eth_write_buf_start, ETH_WRITE_BUF_SIZE - eth_write_buf_start);
        memcpy(curr_buf + ETH_WRITE_BUF_SIZE - eth_write_buf_start, eth_write_buf, eth_write_buf_end);
    }

    // Convert the expected and got buffers to strings of hex values for error messages.
    char expected_str[total_size * 3 + 1];
    char got_str[total_size * 3 + 1];
    for (uint32_t i = 0; i < total_size; i++) {
        sprintf(expected_str + i * 3, "%02X ", full_expected[i]);
        sprintf(got_str + i * 3, "%02X ", curr_buf[i]);
    }

    TEST_ASSERT(memcmp(curr_buf, full_expected, total_size) == 0, "Expected: %s\n\t\t\t\t      Got: %s", expected_str, got_str);
    return TEST_SUCCESS;
}


// Reset the start/end indices of the ethernet write buffer to 0
static void eth_write_buf_clear(void) {
    eth_write_buf_start = eth_write_buf_end = 0;
}

// Pretend to fill the ethernet write buffer, leaving exactly ``remaining`` free bytes in the eth write ring. Sets
// eth_write_buf_start to 0 and eth_write_buf_end to appropriate size
static void eth_write_buf_leave_space(uint32_t remaining) {
    eth_write_buf_start = 0;
    eth_write_buf_end = ETH_WRITE_BUF_SIZE - 1u - remaining;
}


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

static inline void _unit_test_prelude(void) {
    clear_local_error_msg();
    clear_global_error_msg();
    set_stdout_print_enabled(false);
}

/**
 * Auto-registering test. Use as: DECLARE_TEST(my_test) { ...; return TEST_SUCCESS; }
 * Requires GCC/Clang. Call RUN_TESTS() once at the end of the file.
 *
 * Automatically disables stdout printing during the test and checks for error messages after the test.
 */
#define DECLARE_TEST(name)                                                  \
    static test_result_t name(void);                                        \
    static void __attribute__((constructor)) _unit_test_reg_##name(void) {  \
        _unit_test_register(name, #name);                                   \
    }                                                                       \
    static test_result_t __unit_test_helper_##name(void);                   \
    static test_result_t name(void) {                                       \
        _unit_test_prelude();                                               \
        test_result_t result = __unit_test_helper_##name();                 \
        if (!result.success) return result;                                 \
        TEST_ASSERT_NO_ERROR_MESSAGES(#name);                               \
                                                                            \
        return TEST_SUCCESS;                                                \
    }                                                                       \
    static test_result_t __unit_test_helper_##name(void)


////////////////////////////
// Running Unit Test File //
////////////////////////////


/**
 * Write into `store` (at least 1024 bytes) the path of `file_path` relative to the
 * parent of framework.h's directory (i.e. the teensy project root: .../teensy/).
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
    // path (e.g. "../../framework.h"), so join it with the test file's directory.
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

    // anchor = dirname(dirname(framework.h)) → parent of the tests/ directory
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

static bool testing_print_enabled = true;
