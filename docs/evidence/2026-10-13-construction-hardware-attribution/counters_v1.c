#define _GNU_SOURCE
#include <errno.h>
#include <linux/perf_event.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

struct counter_snapshot {
    uint64_t enabled_ns;
    uint64_t running_ns;
    uint64_t cycles;
    uint64_t instructions;
};

static int open_counter(uint64_t config, int group_fd) {
    struct perf_event_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.type = PERF_TYPE_HARDWARE;
    attr.size = PERF_ATTR_SIZE_VER0;
    attr.config = config;
    attr.disabled = group_fd == -1;
    attr.pinned = group_fd == -1;
    attr.exclude_kernel = 1;
    attr.exclude_hv = 1;
    attr.read_format = PERF_FORMAT_GROUP | PERF_FORMAT_ID |
                       PERF_FORMAT_TOTAL_TIME_ENABLED |
                       PERF_FORMAT_TOTAL_TIME_RUNNING;
    return (int)syscall(SYS_perf_event_open, &attr, 0, -1, group_fd,
                        PERF_FLAG_FD_CLOEXEC);
}

int counter_open(int fds[2], uint64_t ids[2]) {
    int leader = open_counter(PERF_COUNT_HW_CPU_CYCLES, -1);
    if (leader == -1) {
        return -1;
    }
    int member = open_counter(PERF_COUNT_HW_INSTRUCTIONS, leader);
    if (member == -1) {
        int saved = errno;
        close(leader);
        errno = saved;
        return -1;
    }
    if (ioctl(leader, PERF_EVENT_IOC_ID, &ids[0]) == -1 ||
        ioctl(member, PERF_EVENT_IOC_ID, &ids[1]) == -1) {
        int saved = errno;
        close(member);
        close(leader);
        errno = saved;
        return -1;
    }
    fds[0] = leader;
    fds[1] = member;
    return 0;
}

int counter_reset(int fd) {
    return ioctl(fd, PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP);
}

int counter_enable(int fd) {
    return ioctl(fd, PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP);
}

int counter_disable(int fd) {
    return ioctl(fd, PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP);
}

int counter_read(int fd, const uint64_t ids[2], struct counter_snapshot *out) {
    /* nr, enabled, running, then two (value, id) entries. No scaled values. */
    uint64_t data[7];
    ssize_t bytes;
    do {
        bytes = read(fd, data, sizeof(data));
    } while (bytes == -1 && errno == EINTR);
    if (bytes == -1) {
        return -1;
    }
    if (bytes != (ssize_t)sizeof(data) || data[0] != 2 || ids[0] == ids[1]) {
        errno = EIO;
        return -1;
    }
    struct counter_snapshot value = {.enabled_ns = data[1],
                                     .running_ns = data[2],
                                     .cycles = 0,
                                     .instructions = 0};
    unsigned seen = 0;
    for (unsigned index = 0; index < 2; ++index) {
        uint64_t count = data[3 + index * 2];
        uint64_t id = data[4 + index * 2];
        if (id == ids[0] && !(seen & 1)) {
            value.cycles = count;
            seen |= 1;
        } else if (id == ids[1] && !(seen & 2)) {
            value.instructions = count;
            seen |= 2;
        } else {
            errno = EIO;
            return -1;
        }
    }
    if (seen != 3) {
        errno = EIO;
        return -1;
    }
    *out = value;
    return 0;
}

int counter_thread_cpu_ns(uint64_t *out) {
    struct timespec value;
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) == -1) {
        return -1;
    }
    if (value.tv_sec < 0 || value.tv_nsec < 0 || value.tv_nsec >= 1000000000) {
        errno = EIO;
        return -1;
    }
    *out = (uint64_t)value.tv_sec * UINT64_C(1000000000) + (uint64_t)value.tv_nsec;
    return 0;
}

uint64_t counter_snapshot_size(void) {
    return sizeof(struct counter_snapshot);
}
