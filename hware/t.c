#include <time.h>


clock_t start, end;
double cpu_time_used;

start = clock();
/* Do the work. */
end = clock();
cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

printf("%f", cpu_time_used);