#include <sys/shm.h>
#include <unistd.h>
#include <string.h>
#include <sys/ipc.h>
#include <stdio.h>
#include <time.h>

int main()
{
    key_t key = ftok("/dev/shm/myshm1", 0);
    int shm_id = shmget(key, 0x400000, IPC_CREAT | 0666);
    double *p = (double *)shmat(shm_id, NULL, 0);

    //int a;scanf("%d",&a);
    struct timespec tv;
    for (size_t i = 0; i < 100; i++)
    {
        p[0] = 1.0 * i;
        clock_gettime(CLOCK_MONOTONIC, &tv);
        printf("%lf : %ld.%ld\n", p[0], tv.tv_sec, tv.tv_nsec);
        sleep(1);
    }

    //memset(p, 1, sizeof(int));
    //printf("%d %d %d %d .\n", p[0], p[1], p[2], p[3]);
    shmdt(p);

    return 0;
}