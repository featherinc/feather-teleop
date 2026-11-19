#pragma once

#include <stdarg.h>
#include <stdbool.h>

/**
 * Sets whether logging is enabled.
 * 
 * @param enabled Whether logging is enabled.
 */
void set_logging_enabled(bool enabled);

/**
 * Logs a message. Currently just prints to the serial port.
 * 
 * @param message The message to log.
 * @param ... The arguments to the message.
 */
void feather_log(const char* message, ...);

/**
 * Always logs a message. Currently just prints to the serial port.
 * 
 * @param message The message to log.
 * @param ... The arguments to the message.
 */
void feather_always_log(const char* message, ...);

/**
 * Sets the most recent local error message. This is an error message for the currently-executing ethernet command.
 * 
 * @param message The message to set.
 * @param ... The arguments to format the message.
 */
void set_local_error_msg(const char* message, ...);

/**
 * @return The most recent local error message as char*, or NULL if no local error message is set.
 */
const char* get_local_error_msg(void);

/**
 * Clears the current local error message.
 */
void clear_local_error_msg(void);

/**
 * Sets the most recent global error message. This is an error in the system, either from outside messages on the
 * CAN bus, or from system-side logic problems.
 * 
 * @param message The message to set.
 * @param ... The arguments to format the message.
 */
void set_global_error_msg(const char* message, ...);

/**
 * @return The most recent global error message as char*, or NULL if no global error message is set.
 */
const char* get_global_error_msg(void);

/**
 * Clears the current global error message.
 */
void clear_global_error_msg(void);

/**
 * Sets whether stdout printing is enabled.
 * 
 * @param enabled Whether stdout printing is enabled.
 */
void set_stdout_print_enabled(bool enabled);

// Designates whether an error message should be set to the global or local error message.
typedef enum ErrorScope {
    ERROR_SCOPE_GLOBAL,
    ERROR_SCOPE_LOCAL,
} ErrorScope;

/**
 * Sets the error message for either local or global depending on the scope parameter.
 * 
 * @param scope The scope of the calling function for error messages.
 * @param message The message to set
 * @param ... The arguments to format the message
 */
void set_error_msg(ErrorScope scope, const char* message, ...);


#define ASSERT(err_func, condition, retval, ...) { \
    /* Assert a condition, setting err_func on fail, returning retval. Must pass error message at minimum. */ \
    if (!(condition)) { \
        err_func("Failed condition ``" #condition "``: " __VA_ARGS__); \
        return retval; \
    } \
}

#define ASSERT_LOCAL(condition, retval, ...) \
    /* Assert a condition, setting local error msg on fail, returning retval. Must pass error message at minimum. */ \
    ASSERT(set_local_error_msg, condition, retval, __VA_ARGS__)
#define ASSERT_GLOBAL(condition, retval, ...) \
    /* Assert a condition, setting global error msg on fail, returning retval. Must pass error message at minimum. */ \
    ASSERT(set_global_error_msg, condition, retval, __VA_ARGS__)
