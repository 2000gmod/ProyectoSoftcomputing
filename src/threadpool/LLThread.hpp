#include <chrono>
#include <mutex>

namespace ll {
    // Executes function inside a static mutex lock.
    // Prefer to use a lambda as the function when possible, as it allows the mutex lock to be unique to that invocation.
    template <class F, class... Args>
    void Synchronize(const F& func, const Args&... args) {
        static std::mutex funcMut;
        std::lock_guard lock(funcMut);
        func(args...);
    }

    // Measures time for a function execution.
    template <class F, class... Args>
    auto TimeFunc(const F& func, const Args&... args) {
        using namespace std::chrono;
        auto start = high_resolution_clock().now();
        func(args...);
        auto end = high_resolution_clock().now();
        return duration_cast<milliseconds>(end - start);
    }
}