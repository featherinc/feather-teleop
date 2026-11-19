#include "logging.hpp"

#include <stdio.h>
#include "system.hpp"

#define LOG_BUFFER_SIZE 256
#define MAX_ERR_MSG_SIZE 256

static char err_msg[MAX_ERR_MSG_SIZE] = {0};
runtime_static bool logging_enabled = true;
runtime_static bool stdout_print_enabled = true;
static char global_err_msg[MAX_ERR_MSG_SIZE] = {0};

void feather_log(const char* message, ...) {
    if (!logging_enabled) return;

    char buffer[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, message);
    vsnprintf(buffer, sizeof(buffer), message, args);
    va_end(args);

    if (stdout_print_enabled) {
        PRINT_STRBUF(buffer);
    }
}

void feather_always_log(const char* message, ...) {
    char buffer[LOG_BUFFER_SIZE];
    va_list args;
    va_start(args, message);
    vsnprintf(buffer, sizeof(buffer), message, args);
    va_end(args);

    PRINT_STRBUF(buffer);
}

void set_logging_enabled(bool enabled) {
    logging_enabled = enabled;
}

static void vset_local_error_msg(const char* message, va_list args) {
    vsnprintf(err_msg, MAX_ERR_MSG_SIZE, message, args);
    feather_log("Set local error message: %s", err_msg);
}

static void vset_global_error_msg(const char* message, va_list args) {
    vsnprintf(global_err_msg, MAX_ERR_MSG_SIZE, message, args);
    feather_log("Set global error message: %s", global_err_msg);
}

void set_local_error_msg(const char* message, ...) {
    va_list args;
    va_start(args, message);
    vset_local_error_msg(message, args);
    va_end(args);
}

const char* get_local_error_msg(void) {
    return err_msg[0] != 0 ? err_msg : NULL;
}

void clear_local_error_msg(void) {
    err_msg[0] = 0;
}

void set_global_error_msg(const char* message, ...) {
    va_list args;
    va_start(args, message);
    vset_global_error_msg(message, args);
    va_end(args);
}

void clear_global_error_msg(void) {
    global_err_msg[0] = 0;
}

const char* get_global_error_msg(void) {
    return global_err_msg[0] != 0 ? global_err_msg : NULL;
}

void set_stdout_print_enabled(bool enabled) {
    stdout_print_enabled = enabled;
}

void set_error_msg(ErrorScope scope, const char* message, ...) {
    va_list args;
    va_start(args, message);

    if (scope == ERROR_SCOPE_GLOBAL) {
        vset_global_error_msg(message, args);
    } else if (scope == ERROR_SCOPE_LOCAL) {
        vset_local_error_msg(message, args);
    } else {
        set_global_error_msg("Invalid error scope: %u", scope);
    }
    
    va_end(args);
}
