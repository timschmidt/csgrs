#include <stdlib.h>
int ireal_sum_oracle(int dependent, unsigned long n, unsigned long seed,
                     const char *lower, const char *upper);
int main(int argc, char **argv) {
    if (argc != 6) return 3;
    return ireal_sum_oracle(atoi(argv[1]), strtoul(argv[2], 0, 10),
        strtoul(argv[3], 0, 10), argv[4], argv[5]);
}
