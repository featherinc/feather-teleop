#include "em_ethernet.hpp"
#include "../src/comms/comms.hpp"
#include "../src/comms/ethernet.hpp"
#include "../src/system.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

/////////////////////
// EthernetClient //
/////////////////////

static bool _eth_client_valid(void) {
    return ethernet_client.fd >= 0;
}

static bool eth_client_connected(void) {
    if (!_eth_client_valid()) return false;
    char c;
    ssize_t n = recv(ethernet_client.fd, &c, 1, MSG_PEEK | MSG_DONTWAIT);
    if (n == 0) return false;  /* peer closed */
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;
    return true;
}

static int eth_client_available(void) {
    if (!_eth_client_valid()) return 0;
    int bytes = 0;
    if (ioctl(ethernet_client.fd, FIONREAD, &bytes) < 0) return 0;
    return bytes;
}

static int eth_client_available_for_write(void) {
    if (!_eth_client_valid()) return 0;
    int unsent = 0;
#ifdef SIOCOUTQ
    if (ioctl(ethernet_client.fd, SIOCOUTQ, &unsent) < 0) unsent = 0;
#endif
    /* Approximate free TX space. Cap so callers don't try to write huge chunks at once. */
    int free_bytes = 65536 - unsent;
    return free_bytes > 0 ? free_bytes : 0;
}

static int eth_client_read(uint8_t* buf, size_t size) {
    if (!_eth_client_valid() || size == 0) return 0;
    ssize_t n = recv(ethernet_client.fd, buf, size, MSG_DONTWAIT);
    if (n < 0) {
        if (ERRNO_IS_EAGAIN_OR_EWOULDBLOCK(errno)) return 0;
        return -1;
    }
    return (int)n;
}

static int eth_client_write(const uint8_t* buf, size_t size) {
    if (!_eth_client_valid() || size == 0) return 0;
    ssize_t n = send(ethernet_client.fd, buf, size, MSG_NOSIGNAL | MSG_DONTWAIT);
    if (n < 0) {
        if (ERRNO_IS_EAGAIN_OR_EWOULDBLOCK(errno)) return 0;
        return -1;
    }
    return (int)n;
}

static void eth_client_flush(void) {/* TCP has no user-space flush; nothing to do. */}

static bool eth_client_set_no_delay(bool flag) {
    if (!_eth_client_valid()) return false;
    int one = flag ? 1 : 0;
    return setsockopt(ethernet_client.fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) == 0;
}

static void eth_client_stop(void) {
    if (!_eth_client_valid()) return;
    if (ethernet_client.fd >= 0) {
        shutdown(ethernet_client.fd, SHUT_RDWR);
        close(ethernet_client.fd);
        ethernet_client.fd = -1;
    }
    ethernet_client.remote_port = 0;
    ethernet_client.remote_ip.addr = 0;
}

static void _eth_client_attach(int fd, IPAddress remote_ip, uint16_t remote_port) {
    if (ethernet_client.fd >= 0 && ethernet_client.fd != fd) close(ethernet_client.fd);
    ethernet_client.fd = fd;
    ethernet_client.remote_ip = remote_ip;
    ethernet_client.remote_port = remote_port;
}

EthernetClient ethernet_client = {
    .fd = -1,
    .remote_ip = { .addr = 0 },
    .remote_port = 0,
    .connected = eth_client_connected,
    .available = eth_client_available,
    .availableForWrite = eth_client_available_for_write,
    .read = eth_client_read,
    .write = eth_client_write,
    .flush = eth_client_flush,
    .setNoDelay = eth_client_set_no_delay,
    .stop = eth_client_stop,
};

/////////////////////
// EthernetServer //
/////////////////////

static void eth_server_close(void) {
    if (ethernet_server.listen_fd >= 0) {
        close(ethernet_server.listen_fd);
        ethernet_server.listen_fd = -1;
    }
}

static void eth_server_begin(void) {
    if (ethernet_server.listen_fd >= 0) return;

    /* Hold the fd in a local until listen() succeeds. -fanalyzer assumes an
     * external call such as perror() may overwrite globals, so an fd that
     * lives only in ethernet_server.listen_fd looks leaked across perror. */
    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("EthernetServer: socket");
        return;
    }

    int yes = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;  /* accept on all interfaces (incl. 127.0.0.1) */
    addr.sin_port = htons(ethernet_server.port);

    if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("EthernetServer: bind");
        close(fd);
        return;
    }
    if (listen(fd, 1) < 0) {
        perror("EthernetServer: listen");
        close(fd);
        return;
    }
    ethernet_server.listen_fd = fd;
}

static bool eth_server_available(EthernetClient* out_client) {
    if (out_client != &ethernet_client) return false;
    if (ethernet_server.listen_fd < 0) return false;

    struct sockaddr_in peer;
    memset(&peer, 0, sizeof(peer));
    socklen_t peer_len = sizeof(peer);
    /* accept4, not accept plus fcntl. -fanalyzer treats fcntl on the new fd as
     * losing the listening socket. SOCK_NONBLOCK is the same flag fcntl set. */
    int fd = accept4(ethernet_server.listen_fd, (struct sockaddr*)&peer, &peer_len, SOCK_NONBLOCK);
    if (fd < 0) return false;  /* EAGAIN / nothing waiting */

    /* Disable Nagle so small command/response packets go out promptly. */
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    _eth_client_attach(fd, BUILD_IP_ADDR(peer.sin_addr.s_addr), ntohs(peer.sin_port));
    return true;
}

EthernetServer ethernet_server = {
    .port = IP_PORT,
    .listen_fd = -1,
    .begin = eth_server_begin,
    .close = eth_server_close,
    .available = eth_server_available,
};

////////////////////
// EthernetClass  //
////////////////////

/* QNEthernet-compatible: configure addressing and return immediately.
 * On Linux we don't reconfigure a NIC; we just record the intended IP
 * and let EthernetServer bind INADDR_ANY so localhost works without root. */
static bool ethernet_begin(IPAddress ip, IPAddress subnet, IPAddress gateway) {
    (void)subnet;
    (void)gateway;
    Ethernet._local_ip = ip;
    Ethernet._link_up = true;
    return true;
}

static bool ethernet_link_state(void) {return Ethernet._link_up;}
static void ethernet_set_mac_address(const uint8_t mac[6]) {memcpy(Ethernet._mac, mac, 6);}

EthernetClass Ethernet = {
    ._local_ip = { .addr = 0 },
    ._mac = {0},
    ._link_up = false,
    .setMACAddress = ethernet_set_mac_address,
    .begin = ethernet_begin,
    .linkState = ethernet_link_state,
};
