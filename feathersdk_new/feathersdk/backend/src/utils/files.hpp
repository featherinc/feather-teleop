#pragma once

#include <string>
#include <filesystem>
#include <thread>
#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include "system.hpp"
#include "logging.hpp"


namespace feather::files {

struct FileLock {
    int fd = -1;
    bool held = false;

    FileLock(const std::string& path, uint64_t timeout_ms) noexcept {
        fd = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0666);
        ENSURE(fd >= 0, , "FileLock failed to open %s", path.c_str());

        if (fchmod(fd, 0666) != 0) {
            ERROR("FileLock failed to set permissions for %s", path.c_str());
            close(fd);
            fd = -1;
            return;
        }

        const feather_time_t deadline = time::now() + time::ms2time(timeout_ms);
        while (flock(fd, LOCK_EX | LOCK_NB) != 0) {
            if (errno != EWOULDBLOCK && errno != EAGAIN) {
                ERROR("FileLock failed to lock %s", path.c_str());
                close(fd);
                fd = -1;
                return;
            }
            if (time::now() >= deadline) {
                close(fd);
                fd = -1;
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        held = true;
    }

    ~FileLock() {
        if (fd >= 0) {
            flock(fd, LOCK_UN);
            close(fd);
        }
    }

    FileLock(const FileLock&) = delete;
    FileLock& operator=(const FileLock&) = delete;
};

}  // namespace feather::files