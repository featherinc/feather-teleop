#pragma once

#include <string>
#include <vector>

/**
 * Runs the given command in a new process. Throws if it fails to start, times out, or is killed by a signal.
 * @param args The command and its arguments.
 * @param timeout_s Maximum seconds to wait. Zero or less waits forever.
 * @param cwd Working directory, or null to keep the current one.
 * @param captured_out If not null, stdout is written here. Otherwise it is discarded.
 * @param captured_err If not null, stderr is written here. Otherwise it is discarded.
 * @return The exit code of the command.
 */
int run(
    const std::vector<std::string>& args,
    double timeout_s,
    const char* cwd = nullptr,
    std::string* captured_out = nullptr,
    std::string* captured_err = nullptr
);
