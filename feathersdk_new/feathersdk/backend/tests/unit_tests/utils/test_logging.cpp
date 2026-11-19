#include "../../framework.hpp"
#include "../../../src/utils/logging.hpp"
#include "../../../src/utils/system.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unistd.h>

using namespace feather::log;

static int g_evals = 0;
static int counted() { return ++g_evals; }

static std::string read_all(const char* path) {
    std::FILE* f = std::fopen(path, "rb");
    if (f == nullptr) return {};
    std::string out;
    char buf[1024];
    std::size_t n = 0;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) out.append(buf, n);
    std::fclose(f);
    return out;
}

static std::string log_text(const char* path) {
    if (detail::state().file != nullptr) std::fflush(detail::state().file);
    return read_all(path);
}

static void rewind_log() {
    std::FILE* f = detail::state().file;
    std::fflush(f);
    if (ftruncate(fileno(f), 0) == 0) std::rewind(f);
}

static bool eat(const std::string& s, std::size_t& i, const char* lit) {
    const std::size_t n = std::strlen(lit);
    if (s.compare(i, n, lit) != 0) return false;
    i += n;
    return true;
}

static bool eat_digits(const std::string& s, std::size_t& i, std::size_t n) {
    if (i + n > s.size()) return false;
    for (std::size_t k = 0; k < n; ++k) {
        if (s[i + k] < '0' || s[i + k] > '9') return false;
    }
    i += n;
    return true;
}

// [days_HH:MM:SS.uuuuuu] <level> (pid:tid:file(line):\n\tbody\n
static bool check_record(const std::string& line, const char* level, const char* body) {
    std::size_t i = 0;
    if (!eat(line, i, "[")) return false;
    const std::size_t days = i;
    while (i < line.size() && line[i] >= '0' && line[i] <= '9') ++i;
    if (i - days < 2 || !eat(line, i, "_")) return false;
    if (!eat_digits(line, i, 2) || !eat(line, i, ":") || !eat_digits(line, i, 2) || !eat(line, i, ":")
        || !eat_digits(line, i, 2) || !eat(line, i, ".") || !eat_digits(line, i, 6) || !eat(line, i, "] ")) {
        return false;
    }
    if (!eat(line, i, level) || !eat(line, i, " (")) return false;

    const std::string id = std::to_string(::getpid()) + ":" + std::to_string(feather::system::thread_id()) + ":";
    if (!eat(line, i, id.c_str())) return false;
    const std::size_t colon = line.find(":\n\t", i);
    if (colon == std::string::npos) return false;

    const std::string where = line.substr(i, colon - i);
    const std::size_t paren = where.rfind("test_logging.cpp(");
    if (paren == std::string::npos || where.back() != ')') return false;
    for (std::size_t k = paren + sizeof("test_logging.cpp(") - 1; k + 1 < where.size(); ++k) {
        if (where[k] < '0' || where[k] > '9') return false;
    }
    const std::string expect = std::string(body) + '\n';
    return line.compare(colon + 3, std::string::npos, expect) == 0;
}

static test_result_t expect_log(const char* path, const char* level, const char* body, const char* prefix = nullptr) {
    const std::string got = log_text(path);
    if (prefix != nullptr) TEST_ASSERT(got.compare(0, std::strlen(prefix), prefix) == 0, "%s", got.c_str());
    TEST_ASSERT(check_record(got, level, body), "%s", got.c_str());
    rewind_log();
    return TEST_SUCCESS;
}

// Restores stdout on every exit path, including TEST_ASSERT.
struct StdoutFile {
    int saved = -1;
    char path[64] = "/tmp/feather-log-out-XXXXXX";

    StdoutFile() {
        const int fd = mkstemp(path);
        if (fd < 0) {
            path[0] = 0;
            return;
        }
        std::fflush(stdout);
        saved = dup(STDOUT_FILENO);
        dup2(fd, STDOUT_FILENO);
        close(fd);
    }

    ~StdoutFile() {
        if (saved >= 0) {
            std::fflush(stdout);
            dup2(saved, STDOUT_FILENO);
            close(saved);
        }
        if (path[0] != 0) unlink(path);
    }

    std::string contents() const {
        std::fflush(stdout);
        return read_all(path);
    }

    void clear() const {
        std::fflush(stdout);
        if (ftruncate(STDOUT_FILENO, 0) == 0) std::rewind(stdout);
    }
};

struct TempPath {
    char path[64] = "/tmp/feather-log-XXXXXX";
    TempPath() {
        const int fd = mkstemp(path);
        if (fd < 0) path[0] = 0;
        else close(fd);
    }
    ~TempPath() { if (path[0] != 0) unlink(path); }
};

DECLARE_TEST(test_init_and_reset) {
    detail::reset_for_tests();
    g_evals = 0;

    StdoutFile out;
    TEST_ASSERT(out.saved >= 0, "stdout redirect failed");
    INFO("before %d", counted());
    TEST_ASSERT(out.contents().empty(), "logged before init");
    TEST_ASSERT(g_evals == 0, "evaluated arguments before init");

    Config bad;
    bad.console_enabled = false;
    bad.file_enabled = true;
    TEST_ASSERT(!init(bad), "empty log path should fail");
    bad.file_path = "/tmp/feather-log-missing-dir/nope";
    TEST_ASSERT(!init(bad), "missing directory should fail");

    TempPath file;
    TEST_ASSERT(file.path[0] != 0, "temp file");
    Config first;
    first.console_enabled = false;
    first.file_enabled = true;
    first.file_level = Level::Warning;
    first.file_path = file.path;
    TEST_ASSERT(init(first), "init");
    TEST_ASSERT(!init(first), "second init should fail");

    Config other;
    other.console_enabled = true;
    other.console_level = Level::Debug;
    other.file_enabled = false;
    TEST_ASSERT(!init(other), "a later init should not replace the first");
    g_evals = 0;
    DEBUG("no %d", counted());
    TEST_ASSERT(g_evals == 0, "debug was evaluated under a warning file level");
    TEST_ASSERT(out.contents().empty(), "console was enabled by the rejected init");

    WARNING("kept");
    detail::reset_for_tests();
    const std::string kept = read_all(file.path);
    TEST_ASSERT(check_record(kept, " WARNING", "kept"), "%s", kept.c_str());

    TEST_ASSERT(init(other), "init after reset");
    DEBUG("after");
    const std::string console = out.contents();
    TEST_ASSERT(check_record(console, "   DEBUG", "after"), "%s", console.c_str());
    return TEST_SUCCESS;
}

DECLARE_TEST(test_routing) {
    detail::reset_for_tests();
    TempPath file;
    TEST_ASSERT(file.path[0] != 0, "temp file");
    Config cfg;
    cfg.console_enabled = true;
    cfg.file_enabled = true;
    cfg.console_level = Level::Warning;
    cfg.file_level = Level::Error;
    cfg.file_path = file.path;
    TEST_ASSERT(init(cfg), "init");

    StdoutFile out;
    TEST_ASSERT(out.saved >= 0, "stdout redirect failed");

    g_evals = 0;
    INFO("skip %d", counted());
    TEST_ASSERT(g_evals == 0, "filtered call evaluated its arguments");
    TEST_ASSERT(out.contents().empty(), "info reached the console");
    TEST_ASSERT(log_text(file.path).empty(), "info reached the file");
    TEST_ASSERT(!detail::would_log(static_cast<int>(Level::Error), nullptr), "null format should not log");

    WARNING("console-only");
    const std::string console_only = out.contents();
    TEST_ASSERT(check_record(console_only, " WARNING", "console-only"), "%s", console_only.c_str());
    TEST_ASSERT(log_text(file.path).empty(), "warning reached the file");
    out.clear();

    g_evals = 0;
    ERROR("both %d", counted());
    TEST_ASSERT(g_evals == 1, "logged call did not evaluate its arguments");
    const std::string console = out.contents();
    TEST_ASSERT(check_record(console, "   ERROR", "both 1"), "%s", console.c_str());
    TEST_ASSERT_STACKED(expect_log(file.path, "   ERROR", "both 1"), "file");

    ERROR("one");
    ERROR("two");
    const std::string appended = log_text(file.path);
    const std::size_t first = appended.find("\tone");
    const std::size_t second = appended.find("\ttwo");
    TEST_ASSERT(first != std::string::npos && second != std::string::npos && first < second,
        "file was not appended: %s", appended.c_str());

    detail::reset_for_tests();
    cfg.console_enabled = false;
    cfg.file_enabled = false;
    TEST_ASSERT(init(cfg), "reinit with logging off");
    out.clear();
    g_evals = 0;
    CRITICAL("no %d", counted());
    TEST_ASSERT(g_evals == 0, "disabled logger evaluated its arguments");
    TEST_ASSERT(out.contents().empty(), "disabled logger wrote to the console");
    return TEST_SUCCESS;
}

DECLARE_TEST(test_format) {
    detail::reset_for_tests();
    TempPath file;
    TEST_ASSERT(file.path[0] != 0, "temp file");
    Config cfg;
    cfg.console_enabled = false;
    cfg.file_enabled = true;
    cfg.file_level = Level::Debug;
    cfg.file_path = file.path;
    TEST_ASSERT(init(cfg), "init");

    DEBUG("debug");
    TEST_ASSERT_STACKED(expect_log(file.path, "   DEBUG", "debug", "[00_00:00:00."), "debug");
    INFO("n=%d %s", 7, "ok");
    TEST_ASSERT_STACKED(expect_log(file.path, "    INFO", "n=7 ok"), "info");
    WARNING("plain");
    TEST_ASSERT_STACKED(expect_log(file.path, " WARNING", "plain"), "warning");
    ERROR("err");
    TEST_ASSERT_STACKED(expect_log(file.path, "   ERROR", "err"), "error");
    CRITICAL("crit");
    TEST_ASSERT_STACKED(expect_log(file.path, "CRITICAL", "crit"), "critical");
    NOTICE("note");
    TEST_ASSERT_STACKED(expect_log(file.path, "  NOTICE", "note"), "notice");

    const std::string big(9000, 'A');
    INFO("%s", big.c_str());
    const std::string trunc = log_text(file.path);
    TEST_ASSERT(trunc.size() == 8192, "truncated record is %zu bytes", trunc.size());
    const std::size_t body = trunc.find("\n\t");
    TEST_ASSERT(body != std::string::npos && trunc.back() == '\n'
        && trunc.find_first_not_of('A', body + 2) == trunc.size() - 1,
        "truncated body is not the message prefix");
    return TEST_SUCCESS;
}

RUN_TESTS()
