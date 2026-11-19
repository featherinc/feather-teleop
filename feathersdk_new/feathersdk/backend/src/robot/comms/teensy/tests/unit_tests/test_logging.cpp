#include "../../src/logging.cpp"
#include "../framework.hpp"
#include <fcntl.h>


DECLARE_TEST(test_logging_enable_flags) {
    set_logging_enabled(false);
    TEST_ASSERT(!logging_enabled, "logging_enabled should be false");
    set_logging_enabled(true);
    TEST_ASSERT(logging_enabled, "logging_enabled should be true");

    set_stdout_print_enabled(true);
    TEST_ASSERT(stdout_print_enabled, "stdout_print_enabled should be true");
    set_stdout_print_enabled(false);
    TEST_ASSERT(!stdout_print_enabled, "stdout_print_enabled should be false");

    // feather_log is a no-op when logging disabled; always_log ignores stdout_print_enabled
    set_logging_enabled(false);
    feather_log("should not print: %d", 1);
    {
        // Flush the "test_name..." line before remapping the stdout fd.
        fflush(stdout);
        int saved = dup(STDOUT_FILENO);
        int nullfd = open("/dev/null", O_WRONLY);
        TEST_ASSERT(saved >= 0 && nullfd >= 0, "failed to open stdout redirect fds");
        TEST_ASSERT(dup2(nullfd, STDOUT_FILENO) >= 0, "failed to redirect stdout");
        close(nullfd);
        feather_always_log("always log ok: %d", 2);
        fflush(stdout);
        TEST_ASSERT(dup2(saved, STDOUT_FILENO) >= 0, "failed to restore stdout");
        close(saved);
    }
    set_logging_enabled(true);

    return TEST_SUCCESS;
}

DECLARE_TEST(test_logging_local_error_msg) {
    TEST_ASSERT(get_local_error_msg() == NULL, "local err should start clear");

    set_local_error_msg("local %d %s", 7, "x");
    TEST_ASSERT(get_local_error_msg() != NULL, "local err should be set");
    TEST_ASSERT(strstr(get_local_error_msg(), "local 7 x") != NULL, "got: %s", get_local_error_msg());

    clear_local_error_msg();
    TEST_ASSERT(get_local_error_msg() == NULL, "local err should clear");

    // Empty first byte means "no message" even if rest is garbage
    set_local_error_msg("tmp");
    clear_local_error_msg();
    TEST_ASSERT(get_local_error_msg() == NULL, "cleared local err must be NULL");

    return TEST_SUCCESS;
}

DECLARE_TEST(test_logging_global_error_msg) {
    TEST_ASSERT(get_global_error_msg() == NULL, "global err should start clear");

    set_global_error_msg("global %u", 99u);
    TEST_ASSERT(get_global_error_msg() != NULL, "global err should be set");
    TEST_ASSERT(strstr(get_global_error_msg(), "global 99") != NULL, "got: %s", get_global_error_msg());

    clear_global_error_msg();
    TEST_ASSERT(get_global_error_msg() == NULL, "global err should clear");

    return TEST_SUCCESS;
}

DECLARE_TEST(test_logging_set_error_msg_scope) {
    set_error_msg(ERROR_SCOPE_LOCAL, "via scope local %d", 1);
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("via scope local 1");

    set_error_msg(ERROR_SCOPE_GLOBAL, "via scope global %d", 2);
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("via scope global 2");

    set_error_msg((ErrorScope)99, "ignored");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Invalid error scope");

    return TEST_SUCCESS;
}

static int _assert_local_ok(void) {
    ASSERT_LOCAL(true, -1, "should not fire");
    return 42;
}

static int _assert_local_fail(void) {
    ASSERT_LOCAL(false, -1, "local assert detail %d", 3);
    return 42;
}

static int _assert_global_ok(void) {
    ASSERT_GLOBAL(1 == 1, -1, "should not fire");
    return 42;
}

static int _assert_global_fail(void) {
    ASSERT_GLOBAL(1 == 0, -1, "global assert detail %d", 4);
    return 42;
}

DECLARE_TEST(test_logging_assert_macros) {
    TEST_ASSERT(_assert_local_ok() == 42, "ASSERT_LOCAL should not return on success");
    TEST_ASSERT(get_local_error_msg() == NULL, "ASSERT_LOCAL success must not set error");

    TEST_ASSERT(_assert_local_fail() == -1, "ASSERT_LOCAL should return retval on fail");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("Failed condition");
    // Re-trigger to check formatted detail (cleared by previous macro)
    TEST_ASSERT(_assert_local_fail() == -1, "ASSERT_LOCAL fail again");
    TEST_ASSERT_LOCAL_ERROR_MESSAGE("local assert detail 3");

    TEST_ASSERT(_assert_global_ok() == 42, "ASSERT_GLOBAL should not return on success");
    TEST_ASSERT(get_global_error_msg() == NULL, "ASSERT_GLOBAL success must not set error");

    TEST_ASSERT(_assert_global_fail() == -1, "ASSERT_GLOBAL should return retval on fail");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("Failed condition");
    TEST_ASSERT(_assert_global_fail() == -1, "ASSERT_GLOBAL fail again");
    TEST_ASSERT_GLOBAL_ERROR_MESSAGE("global assert detail 4");

    return TEST_SUCCESS;
}

RUN_TESTS();
