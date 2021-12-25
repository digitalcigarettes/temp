#include <chrono>


struct clock
{
    typedef unsigned long long                 rep;
    typedef std::ratio<1, 2'800'000'000>       period; // My machine is 2.8 GHz
    typedef std::chrono::duration<rep, period> duration;
    typedef std::chrono::time_point<clock>     time_point;
    static const bool is_steady =              true;

    static time_point now() noexcept
    {
        unsigned lo, hi;
        asm volatile("rdtsc" : "=a" (lo), "=d" (hi));
        return time_point(duration(static_cast<rep>(hi) << 32 | lo));
    }

};
