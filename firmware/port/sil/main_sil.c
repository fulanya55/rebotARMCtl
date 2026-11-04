/*
 * Software-in-the-loop controller: the firmware core plus simulated arms,
 * exposed on a TCP socket in place of USB CDC. The byte stream is identical to
 * what the real board sends, so the Controller Service cannot tell them apart
 * except through the RAC_CAP_SIL capability it reports.
 *
 *   rac_sil [--port 5760] [--models rs,rs,hd,hd] [--time-scale 1.0]
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <mmsystem.h>
typedef SOCKET sock_t;
#define SOCK_INVALID INVALID_SOCKET
#define sock_close closesocket
#define SOCK_WOULD_BLOCK() (WSAGetLastError() == WSAEWOULDBLOCK)
#else
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
typedef int sock_t;
#define SOCK_INVALID (-1)
#define sock_close close
#define SOCK_WOULD_BLOCK() (errno == EWOULDBLOCK || errno == EAGAIN)
#endif

#include "rac_core.h"
#include "sim_plant.h"

#define OUT_CAPACITY (256u * 1024u)

static sock_t client = SOCK_INVALID;
static uint8_t out_buf[OUT_CAPACITY];
static size_t out_len;

static uint64_t wall_us(void)
{
#ifdef _WIN32
    static LARGE_INTEGER freq;
    LARGE_INTEGER now;
    if (freq.QuadPart == 0) QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return (uint64_t)((double)now.QuadPart * 1e6 / (double)freq.QuadPart);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000u + (uint64_t)ts.tv_nsec / 1000u;
#endif
}

static void set_nonblocking(sock_t s)
{
#ifdef _WIN32
    u_long on = 1;
    ioctlsocket(s, FIONBIO, &on);
#else
    fcntl(s, F_SETFL, fcntl(s, F_GETFL, 0) | O_NONBLOCK);
#endif
}

static void host_sink(const uint8_t *data, size_t len)
{
    if (client == SOCK_INVALID) return;
    if (out_len + len > OUT_CAPACITY) return; /* a stalled client loses samples, not the plant */
    memcpy(out_buf + out_len, data, len);
    out_len += len;
}

static void drop_client(void)
{
    if (client != SOCK_INVALID) {
        sock_close(client);
        client = SOCK_INVALID;
    }
    out_len = 0;
}

static void flush_out(void)
{
    if (client == SOCK_INVALID || out_len == 0) return;
    int sent = send(client, (const char *)out_buf, (int)out_len, 0);
    if (sent > 0) {
        memmove(out_buf, out_buf + sent, out_len - (size_t)sent);
        out_len -= (size_t)sent;
    } else if (sent < 0 && !SOCK_WOULD_BLOCK()) {
        drop_client();
    }
}

static uint8_t parse_model(const char *name)
{
    if (!strcmp(name, "rs")) return RAC_MODEL_REBOT_B601_RS;
    if (!strcmp(name, "fl")) return RAC_MODEL_STAR_102_FL;
    if (!strcmp(name, "hd")) return RAC_MODEL_STAR_102_HD;
    if (!strcmp(name, "ld")) return RAC_MODEL_STAR_102_LD;
    return RAC_MODEL_NONE;
}

int main(int argc, char **argv)
{
    int port = 5760;
    double time_scale = 1.0;
    const char *models = "rs,rs,hd,hd";

    for (int i = 1; i + 1 < argc; i += 2) {
        if (!strcmp(argv[i], "--port")) port = atoi(argv[i + 1]);
        else if (!strcmp(argv[i], "--models")) models = argv[i + 1];
        else if (!strcmp(argv[i], "--time-scale")) time_scale = atof(argv[i + 1]);
    }

#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    timeBeginPeriod(1);
#endif

    sock_t listener = socket(AF_INET, SOCK_STREAM, 0);
    int yes = 1;
    setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, (const char *)&yes, sizeof(yes));
    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK); /* loopback only: this drives simulated arms */
    addr.sin_port = htons((uint16_t)port);
    if (bind(listener, (struct sockaddr *)&addr, sizeof(addr)) != 0 || listen(listener, 1) != 0) {
        fprintf(stderr, "rac_sil: cannot listen on 127.0.0.1:%d\n", port);
        return 1;
    }
    set_nonblocking(listener);

    sim_init();
    sim_set_host_sink(host_sink);

    /* Comma separated, one per port in the order CAN1, CAN2, UART1, UART2. */
    {
        char copy[64];
        strncpy(copy, models, sizeof(copy) - 1);
        copy[sizeof(copy) - 1] = 0;
        uint8_t index = 0;
        for (char *tok = strtok(copy, ","); tok && index < RAC_MAX_PORTS; tok = strtok(NULL, ",")) {
            sim_set_model(index++, parse_model(tok));
        }
    }

    rac_core_init();

    printf("rac_sil listening on 127.0.0.1:%d models=%s\n", port, models);
    fflush(stdout);

    uint64_t wall_start = wall_us();
    uint64_t virtual_now = 0;

    for (;;) {
        uint64_t elapsed = wall_us() - wall_start;
        uint64_t target = (uint64_t)((double)elapsed * time_scale);

        /* Never fall more than a moment behind: after a debugger pause or a
         * laptop sleep, resume from now instead of replaying the gap. */
        if (target > virtual_now + 100000u) {
            virtual_now = target - 100000u;
        }
        while (virtual_now + RAC_TICK_US <= target) {
            virtual_now += RAC_TICK_US;
            sim_advance(virtual_now);
            rac_core_tick(virtual_now);
        }
        flush_out();

        if (client == SOCK_INVALID) {
            sock_t c = accept(listener, NULL, NULL);
            if (c != SOCK_INVALID) {
                set_nonblocking(c);
                setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char *)&yes, sizeof(yes));
                client = c;
            }
        }

        fd_set readable;
        FD_ZERO(&readable);
        struct timeval tv = {0, 200};
        if (client != SOCK_INVALID) {
            FD_SET(client, &readable);
            int nfds = (int)client + 1;
            if (select(nfds, &readable, NULL, NULL, &tv) > 0) {
                uint8_t buf[4096];
                int n = recv(client, (char *)buf, sizeof(buf), 0);
                if (n > 0) {
                    rac_core_on_host_bytes(buf, (size_t)n, virtual_now);
                } else if (n == 0 || !SOCK_WOULD_BLOCK()) {
                    drop_client();
                }
            }
        } else {
#ifdef _WIN32
            Sleep(1);
#else
            struct timespec ts = {0, 500000};
            nanosleep(&ts, NULL);
#endif
        }
    }
}
