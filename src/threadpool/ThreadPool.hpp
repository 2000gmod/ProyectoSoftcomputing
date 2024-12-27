#pragma once

#include <type_traits>
#include <vector>
#include <queue>
#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <unordered_map>
#include <future>
#include <memory>


namespace ll {
    class ThreadPool;
    class TaskGroupFuture {
        private:
            friend class ThreadPool;
            std::vector<std::shared_future<void>> futures;
        public:
            // Blocks until all futures are valid. Rethrows any exception.
            void Get();
            // Blocks until all futures are valid. Ignores any exception thrown inside the task group.
            void Wait();

            // Returns a copy of the future list (`std::vector<std::shared_future<void>>`).
            auto GetFutureList() {
                return futures;
            }
    };

    class ThreadPool {
        public:
            // Constructs ThreadPool with specified thread number.
            explicit ThreadPool(size_t numThreads = std::thread::hardware_concurrency());

            ThreadPool(const ThreadPool&) = delete;
            ThreadPool(ThreadPool &&) = delete;

            ThreadPool& operator= (const ThreadPool&) = delete;
            ThreadPool& operator= (ThreadPool&&) = delete;

            ~ThreadPool();

        private:
            using Task = std::function<void()>;

            std::vector<std::jthread> threads;
            std::queue<Task> tasks;
            std::condition_variable cv, doneCV;
            mutable std::mutex queueMutex, workerMutex, doneMutex;
            bool stopped;
            bool paused;
            bool* workflags;
            std::unordered_map<std::thread::id, int> idMap;

            void WorkerFunc(int tid);

            bool AllThreadsDone();
            void Start();
            void JoinThreads();
            void Stop();

        public:
            // Pauses execution of queued tasks. This will not affect any currently executing tasks.
            // If the pool is paused when it reaches the end of its lifetime, any task still in the queue will not be executed.
            void Pause();
            // Resumes execution of tasks.
            void Resume();
            // Removes all queued tasks. Any currently executing task will not be affected.
            void Clear();
            // Blocks the caller thread until all tasks finish executing (either the queue is emptied or the pool gets paused).
            void WaitUntilEmpty();
            // Returns the number of remaining tasks in the queue.
            unsigned int GetRemainingTasks() const;
            // Returns the index in the internal list of threads of the caller thread.
            // Will return `-1` if caller thread is not managed by the pool.
            int GetThreadIndex() const;

            // Submits a task to be executed (a function object and optional parameters) and returns an `std::future`.
            // Parameters to the task will be passed by copy. Consider using a reference capture if this is not acceptable.
            template <class F, class... Args>
            requires std::invocable<F, Args...>
            auto Submit(const F& func, Args&&... args) -> std::future<decltype(func(args...))> {
                using Res = decltype(func(args...));
                auto taskPromise = std::make_shared<std::promise<Res>>();
                auto future = taskPromise->get_future();

                auto taskFunc = [&func, args...] () {
                    return func(args...);
                };

                Push([taskFunc, taskPromise] {
                    try {
                        if constexpr (std::is_void_v<Res>) {
                            taskFunc();
                            taskPromise->set_value();
                        } else {
                            taskPromise->set_value(taskFunc());
                        }
                    }
                    catch (...) {
                        try {
                            taskPromise->set_exception(std::current_exception());
                        }
                        catch (...) { }
                    }
                });
                return future;
            }


            // Submits a task to be executed (a function object) and returns an `std::future`.
            template <class F>
            requires std::invocable<F>
            auto Submit(const F& func) -> std::future<decltype(func())> {
                using Res = decltype(func());
                auto taskPromise = std::make_shared<std::promise<Res>>();
                auto future = taskPromise->get_future();

                Push([taskFunc = func, taskPromise] {
                    try {
                        if constexpr (std::is_void_v<Res>) {
                            taskFunc();
                            taskPromise->set_value();
                        } else {
                            taskPromise->set_value(taskFunc());
                        }
                    }
                    catch (...) {
                        try {
                            taskPromise->set_exception(std::current_exception());
                        }
                        catch (...) { }
                    }
                });
                return future;
            }

            // Submit a member function call as a task. Returns an `std::future` with the corresponding return type.
            // Arguments will be passed by copy, prefer to submit a lambda with reference capture instead if this is not acceptable.
            template <class T, class R, class... Args>
            requires requires (T t, R (T::*f)(Args...), Args... args) {
                (t.*f)(args...);
            }
            auto Submit(R (T::*f)(Args...), T& obj, Args&&... args) {
                return Submit([&obj, f, args...] {
                    (obj.*f)(args...);
                });
            }

            // Executes a for loop in parallel. Runs task as `func(i)` where `i` is of type `int` and goes from [min, max).
            // The task is required to take a single argument of type `int`.
            template <class F>
            requires std::invocable<F, int> && requires (F f, int i) {
                {f(i)} -> std::same_as<void>;
            }
            TaskGroupFuture For(int min, int max, const F& func) {
                TaskGroupFuture result;
                for (int i = min; i < max; i++) {
                    auto fut = Submit(func, i);
                    result.futures.push_back(std::move(fut));
                }
                return result;
            }

            // Executes a for loop in parallel. Runs task as `func(i)` where `i` is of type `T` and goes from [min, max) with specified step size.
            // The task is required to take a single argument of type `T`.
            template <class T = int, class F>
            requires std::invocable<F, T> && std::is_arithmetic_v<T> && requires (F f, T t) {
                {f(t)} -> std::same_as<void>;
            }
            TaskGroupFuture For(T min, T max, T step, const F& func) {
                TaskGroupFuture result;
                for (T i = min; i < max; i += step) {
                    auto fut = Submit(func, i);
                    result.futures.push_back(std::move(fut));
                }
                return result;
            }

            // Repeats a task in parallel. Runs task as `func()` a number of times, specified with the parameter `n`.
            template <class F>
            requires std::invocable<F> && requires (F f) {
                {f()} -> std::same_as<void>;
            }
            TaskGroupFuture Repeat(int n, const F& func) {
                TaskGroupFuture result;
                for (int i = 0; i < n; i++) {
                    auto fut = Submit(func);
                    result.futures.push_back(std::move(fut));
                }
                return result;
            }


            // Executes a ranged-based for loop in parallel. Runs function as `func(elem)` where `elem` is of type of the element of the sequence.
            // The function is required to take a single argument of the type of element in the sequence.
            template <class F, class Container>
            requires requires (F f, Container cont) {
                cont.begin();
                cont.end();
                {f(*cont.begin())} -> std::same_as<void>;
            }
            TaskGroupFuture ForEach(Container& cont, const F& func) {
                TaskGroupFuture result;
                for (auto& elem : cont) {
                    auto fut = Submit([&func, &elem] {
                        func(elem);
                    });
                    result.futures.push_back(std::move(fut));
                }
                return result;
            }

            
            /* Executes a ranged-based for loop in parallel. Runs function as `func(elem, i)`
            where `elem` is of type of the element of the sequence and `i` is of type `int`.
            The function is required to have an argument containing a reference to the accessed element, 
            and a second parameter of type 'int' that indicates the position in which the element was added to the task queue. */
            template <class F, class Container>
            requires requires (F f, int i, Container cont) {
                cont.begin();
                cont.end();
                {f(*cont.begin(), i)} -> std::same_as<void>;
            }
            TaskGroupFuture ForEachIndexed(Container& cont, const F& func) {
                TaskGroupFuture result;
                int i = 0;
                for (auto& elem : cont) {
                    auto fut = Submit([&func, &elem, i] {
                        func(elem, i);
                    });
                    i++;
                    result.futures.push_back(std::move(fut));
                }
                return result;
            }

            // Creates task that executes only while the condition returns true.
            template <class F, class C>
            requires std::invocable<F> && std::invocable<C> && requires (C c, F f) {
                {c()} -> std::convertible_to<bool>;
                {f()} -> std::same_as<void>;
            }
            auto While(const C& condition, const F& func) {
                return Submit([condition, func] {
                    while (condition()) {
                        func();
                    } 
                });
            }

            // Creates task that executes only while the condition returns true, executes the task at least once.
            template <class F, class C>
            requires std::invocable<F> && std::invocable<C> && requires (C c, F f) {
                {c()} -> std::convertible_to<bool>;
                {f()} -> std::same_as<void>;
            }
            auto DoWhile(const C& condition, const F& func) {
                return Submit([condition, func] {
                    do {
                        func();
                    } while (condition());
                });
            }

            // Creates task that executes until the condition is true. Equivalent to While with `!condition()`.
            template <class F, class C>
            requires std::invocable<F> && std::invocable<C> && requires (C c, F f) {
                {c()} -> std::convertible_to<bool>;
                {f()} -> std::same_as<void>;
            }
            auto Until(const C& condition, const F& func) {
                return Submit([condition, func] {
                    while (!condition()) {
                        func();
                    } 
                });
            }

        private:
            template <class F, class... Args>
            requires std::invocable<F, Args...> && requires (F f, Args... args) {
                {f(args...)} -> std::same_as<void>;
            }
            void Push(const F& func, Args&&... args) {
                std::lock_guard lock(queueMutex);
                auto f = [func, args...] {
                    func(args...);
                };
                tasks.push(f);
                cv.notify_one();
            }

            template <class F>
            requires std::invocable<F> && requires (F f) {
                {f()} -> std::same_as<void>;
            }
            void Push(const F& func) {
                std::lock_guard lock(queueMutex);
                tasks.push(func);
                cv.notify_one();
            }
    };

}