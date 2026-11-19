#pragma once

// Same layout as the Python logger:
//   [DD_HH:MM:SS.uuuuuu] SEVERITY (pid:tid:file(line)):
//   	message
//
// Call init once before any log. A later init is ignored.
// DEBUG/INFO/WARNING/ERROR/CRITICAL/NOTICE take a printf format string.
// A call below both the console level and the file level is not formatted.

#include <cstdint>
#include <cstdio>
#include <string>

namespace feather::log {

enum class Level : int {
    Debug = 10,
    Info = 20,
    Warning = 30,
    Error = 40,
    Critical = 50,
    Notice = 100,
};

struct Config {
    bool console_enabled = true;
    bool file_enabled = false;
    Level console_level = Level::Info;
    Level file_level = Level::Debug;
    std::string file_path;
};

namespace detail {

struct State {
    Config config{};
    std::FILE* file = nullptr;
    bool configured = false;
    int pid = -1;
    std::uint64_t start = 0;

    State();
    ~State();
};

State& state();

// Test-only. Closes an open log file and clears init so init() can run again.
void reset_for_tests();

// True when this level would be written to the console or the file.
bool would_log(int level, const char* fmt) noexcept;

// loc is the compile-time "file(line)" string. Caller has checked would_log.
[[gnu::format(printf, 3, 4)]]
void write(Level level, const char* loc, const char* fmt, ...);

}  // namespace detail

// Returns false if init already succeeded, or file output was requested and the file did not open.
bool init(const Config& next);

}  // namespace feather::log

#define FEATHER_STR_(x) #x
#define FEATHER_STR(x) FEATHER_STR_(x)
#define FEATHER_LOC __FILE__ "(" FEATHER_STR(__LINE__) ")"

#define FEATHER_LOG(level, fmt, ...)                                                    \
    /* Log the message if the level is enabled. This does not evaluate */               \
    /* any arguments if the level is not enabled. */                                    \
    do {                                                                                \
        if (::feather::log::detail::would_log(static_cast<int>(level), fmt)) {          \
            ::feather::log::detail::write(level, FEATHER_LOC, fmt, ##__VA_ARGS__);      \
        }                                                                               \
    } while (0)

#define DEBUG(fmt, ...) \
    /* Log a debug message. This does not evaluate any arguments if the DEBUG level is not enabled. */      \
    FEATHER_LOG(::feather::log::Level::Debug, fmt, ##__VA_ARGS__)
#define INFO(fmt, ...) \
    /* Log an info message. This does not evaluate any arguments if the INFO level is not enabled. */      \
    FEATHER_LOG(::feather::log::Level::Info, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) \
    /* Log a warning message. This does not evaluate any arguments if the WARNING level is not enabled. */      \
    FEATHER_LOG(::feather::log::Level::Warning, fmt, ##__VA_ARGS__)
#define ERROR(fmt, ...) \
    /* Log an error message. This does not evaluate any arguments if the ERROR level is not enabled. */      \
    FEATHER_LOG(::feather::log::Level::Error, fmt, ##__VA_ARGS__)
#define CRITICAL(fmt, ...) \
    /* Log a critical message. This does not evaluate any arguments if the CRITICAL level is not enabled. */      \
    FEATHER_LOG(::feather::log::Level::Critical, fmt, ##__VA_ARGS__)
#define NOTICE(fmt, ...) \
    /* Log a notice message. This does not evaluate any arguments if the NOTICE level is not enabled. */      \
    FEATHER_LOG(::feather::log::Level::Notice, fmt, ##__VA_ARGS__)
