#include "run.hpp"
#include "system.hpp"

#include <cerrno>
#include <stdexcept>
#include <vector>

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>


// Helper function called by the child process to execute the command.
void _child_exec(const std::vector<std::string>& args, const char* cwd, int pipefd_out, int pipefd_err) {
    // Own process group, so a timeout can kill this command and anything it starts.
    setpgid(0, 0);

    // Change to the working directory, if provided.
    if (cwd != nullptr && chdir(cwd) != 0) _exit(127);

    // Redirect stdout/stderr to the pipes, if provided.
    if (pipefd_out >= 0) dup2(pipefd_out, STDOUT_FILENO);
    else dup2(open("/dev/null", O_WRONLY), STDOUT_FILENO);
    if (pipefd_err >= 0) dup2(pipefd_err, STDERR_FILENO);
    else dup2(open("/dev/null", O_WRONLY), STDERR_FILENO);

    // Create the argv array and execute the command.
    std::vector<char*> argv;
    for (const std::string& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);
    execvp(argv[0], argv.data());

    _exit(127);
}


int run(
    const std::vector<std::string>& args, 
    double timeout_s, 
    const char* cwd, 
    std::string* captured_out, 
    std::string* captured_err
) {
    // Read ends live until run() returns. Write ends live only in the block below, so the parent
    // drops them before draining and sees EOF when the child exits. fork copies the child's fds,
    // so closing here does not close the child's copies.
    feather::system::Socket out_read(-1);
    feather::system::Socket err_read(-1);
    pid_t pid = -1;
    {
        int raw_out[2] = {-1, -1};
        int raw_err[2] = {-1, -1};
        if (captured_out != nullptr && pipe2(raw_out, O_CLOEXEC) != 0) throw std::runtime_error("failed to open pipes");
        out_read.fd = raw_out[0];
        feather::system::Socket out_write(raw_out[1]);
        if (captured_err != nullptr && pipe2(raw_err, O_CLOEXEC) != 0) throw std::runtime_error("failed to open pipes");
        err_read.fd = raw_err[0];
        feather::system::Socket err_write(raw_err[1]);

        pid = fork();
        if (pid < 0) throw std::runtime_error("fork failed");
        if (pid == 0) _child_exec(args, cwd, out_write.fd, err_write.fd);  // No return from child process.

        // Same setpgid as the child. Whichever side runs first creates the group.
        setpgid(pid, pid);
    }

    // Drain both pipes while the child runs, so a full pipe cannot stall it.
    std::string out, err;
    int out_fd = out_read.fd;
    int err_fd = err_read.fd;
    int status = 0;
    bool exited = false;
    const feather_time_t start = feather::time::now();
    auto drain = [](int& fd, std::string& dest) {
        char buf[4096];
        const ssize_t n = read(fd, buf, sizeof buf);
        if (n > 0) dest.append(buf, static_cast<size_t>(n));
        else if (n == 0 || errno != EINTR) fd = -1;  // Stop polling. The Socket still owns the fd.
    };

    while (!exited || out_fd >= 0 || err_fd >= 0) {
        // poll ignores a negative fd, so a stream we are not capturing is skipped.
        pollfd pfd[2] = {{out_fd, POLLIN, 0}, {err_fd, POLLIN, 0}};
        poll(pfd, 2, 50);
        if (out_fd >= 0 && (pfd[0].revents & (POLLIN | POLLHUP | POLLERR))) drain(out_fd, out);
        if (err_fd >= 0 && (pfd[1].revents & (POLLIN | POLLHUP | POLLERR))) drain(err_fd, err);

        // Check if the command has finished
        if (!exited) {
            const pid_t got = waitpid(pid, &status, WNOHANG);
            if (got == pid) exited = true;
            else if (got < 0 && errno != EINTR) {
                kill(-pid, SIGKILL);
                waitpid(pid, &status, 0);
                throw std::runtime_error(args[0] + " failed");
            }
        }

        // If the command timed out, kill the process group and wait for it to exit.
        if (!exited && timeout_s > 0 && feather::time::elapsed(start) >= feather::time::from_seconds(timeout_s)) {
            kill(-pid, SIGKILL);  // Negative pid signals the whole process group.
            waitpid(pid, &status, 0);
            throw std::runtime_error(args[0] + " timed out");
        }
    }

    if (captured_out != nullptr) *captured_out = std::move(out);
    if (captured_err != nullptr) *captured_err = std::move(err);
    if (!WIFEXITED(status)) throw std::runtime_error(args[0] + " failed");
    return WEXITSTATUS(status);
}
