#include "logging.hpp"

#include "system.hpp"

#include <cstdarg>
#include <cstring>
#include <unistd.h>

namespace feather::log {
namespace {

const char* level_name(Level level) noexcept {
    switch (level) {
        case Level::Debug: return "   DEBUG";
        case Level::Info: return "    INFO";
        case Level::Warning: return " WARNING";
        case Level::Error: return "   ERROR";
        case Level::Critical: return "CRITICAL";
        case Level::Notice: return "  NOTICE";
        default: return "       ?";
    }
}

char* put2(char* p, unsigned v) noexcept {
    *p++ = static_cast<char>('0' + v / 10);
    *p++ = static_cast<char>('0' + v % 10);
    return p;
}

char* put6(char* p, unsigned v) noexcept {
    p = put2(p, v / 10000);
    p = put2(p, (v / 100) % 100);
    return put2(p, v % 100);
}

char* put_uint(char* p, unsigned long long v) noexcept {
    char tmp[20];
    char* const end = tmp + sizeof tmp;
    char* t = end;
    do {
        *--t = static_cast<char>('0' + static_cast<unsigned>(v % 10));
        v /= 10;
    } while (v != 0);
    const auto n = static_cast<std::size_t>(end - t);
    std::memcpy(p, t, n);
    return p + n;
}

}  // namespace

namespace detail {

State::State() : start(feather::time::now()) {}

State::~State() {
    if (file != nullptr) std::fclose(file);
}

State& state() {
    static State s;
    return s;
}

void reset_for_tests() {
    State& s = state();
    if (s.file != nullptr) {
        std::fclose(s.file);
        s.file = nullptr;
    }
    s.config = Config{};
    s.configured = false;
    s.pid = -1;
    s.start = feather::time::now();
}

bool would_log(int level, const char* fmt) noexcept {
    const State& s = state();
    const bool console = s.config.console_enabled && level >= static_cast<int>(s.config.console_level);
    const bool file = s.config.file_enabled && level >= static_cast<int>(s.config.file_level);
    return fmt != nullptr && (console || file) && s.configured;
}

void write(Level level, const char* loc, const char* fmt, ...) {
    State& s = state();
    const int level_value = static_cast<int>(level);

    const auto us = feather::time::elapsed(s.start) / 1000;
    const auto total_s = us / 1000000;
    const auto total_m = total_s / 60;
    const auto total_h = total_m / 60;

    char linebuf[8192];
    const std::size_t loc_len = std::strlen(loc);

    char* p = linebuf;
    *p++ = '[';
    p = put2(p, static_cast<unsigned>((total_h / 24) % 100));
    *p++ = '_';
    p = put2(p, static_cast<unsigned>(total_h % 24));
    *p++ = ':';
    p = put2(p, static_cast<unsigned>(total_m % 60));
    *p++ = ':';
    p = put2(p, static_cast<unsigned>(total_s % 60));
    *p++ = '.';
    p = put6(p, static_cast<unsigned>(us % 1000000));
    *p++ = ']';
    *p++ = ' ';
    std::memcpy(p, level_name(level), 8);
    p += 8;
    *p++ = ' ';
    *p++ = '(';
    p = put_uint(p, static_cast<unsigned>(s.pid));
    *p++ = ':';
    p = put_uint(p, static_cast<unsigned>(feather::system::thread_id()));
    *p++ = ':';
    std::memcpy(p, loc, loc_len);
    p += loc_len;
    *p++ = ':';
    *p++ = '\n';
    *p++ = '\t';

    char* const end = linebuf + sizeof linebuf;
    const std::size_t remain = static_cast<std::size_t>(end - p);
    va_list args;
    va_start(args, fmt);
    const int body_len = std::vsnprintf(p, remain, fmt, args);
    va_end(args);

    static constexpr char err[] = "[ERROR: Failed to format message]";
    constexpr std::size_t err_len = sizeof err - 1;
    std::size_t n = 0;
    if (body_len < 0) {
        if (remain > err_len) {
            std::memcpy(p, err, err_len);
            n = static_cast<std::size_t>(p - linebuf) + err_len;
        } else {
            std::memcpy(linebuf, err, err_len);
            n = err_len;
        }
    } else if (static_cast<std::size_t>(body_len) >= remain) {
        n = static_cast<std::size_t>(p - linebuf) + (remain - 1);
    } else {
        n = static_cast<std::size_t>(p - linebuf) + static_cast<std::size_t>(body_len);
    }
    if (n < sizeof linebuf) linebuf[n++] = '\n';
    else linebuf[sizeof linebuf - 1] = '\n';

    const Config& config = s.config;
    if (config.console_enabled && level_value >= static_cast<int>(config.console_level)) {
        std::fwrite(linebuf, 1, n, stdout);
    }
    if (config.file_enabled && s.file != nullptr && level_value >= static_cast<int>(config.file_level)) {
        std::fwrite(linebuf, 1, n, s.file);
    }
}

}  // namespace detail

bool init(const Config& next) {
    detail::State& s = detail::state();
    if (s.configured) return false;

    std::FILE* opened = nullptr;
    if (next.file_enabled) {
        if (next.file_path.empty()) return false;
        opened = std::fopen(next.file_path.c_str(), "a");
        if (opened == nullptr) return false;
    }

    s.config = next;
    s.file = opened;
    s.pid = static_cast<int>(::getpid());
    s.configured = true;
    return true;
}

}  // namespace feather::log
