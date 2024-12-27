#include "ThreadPool.hpp"
#include <mutex>
#include <thread>

using namespace ll;

void TaskGroupFuture::Get() {
    for (auto& fut : futures) {
        fut.get();
    }
}

void TaskGroupFuture::Wait() {
    for (auto& fut : futures) {
        fut.wait();
    }
}

ThreadPool::ThreadPool(const size_t numThreads) {
    if (numThreads == 0) {
        throw std::invalid_argument("Tried to create ThreadPool with 0 threads.");
    }
    stopped = false;
    paused = true;
    workflags = new bool[numThreads];
    threads.reserve(numThreads);
    for (size_t i = 0; i < numThreads; i++) {
        workflags[i] = false;
        threads.emplace_back(&ThreadPool::WorkerFunc, this, i);
        idMap.insert_or_assign(threads[i].get_id(), i);
    }
    Start();
}

ThreadPool::~ThreadPool() {
    if (stopped || paused || tasks.empty()) {
        stopped = true;
        cv.notify_all();
        JoinThreads();
        delete[] workflags;
        return;
    }
    WaitUntilEmpty();
    Stop();
    JoinThreads();
    delete[] workflags;
}

void ThreadPool::Start() {
    stopped = false;
    paused = false;
    cv.notify_all();
}

void ThreadPool::JoinThreads() {
    for (auto &t : threads) {
        t.join();
    }
}

void ThreadPool::Stop() {
    stopped = true;
    cv.notify_all();
}

void ThreadPool::Pause() {
    paused = true;
}

void ThreadPool::WorkerFunc(int tid) {
    while (true) {
        std::function<void()> job;
        {
            std::unique_lock lock(workerMutex);
            cv.wait(lock, [this] { return (!tasks.empty() && !paused) || stopped; });
            if (stopped) {
                return;
            }

            std::lock_guard _(queueMutex);
            job = std::move(tasks.front());
            tasks.pop();

            workflags[tid] = true;
        }
        job();
        {
            std::lock_guard _(workerMutex);
            workflags[tid] = false;
        }
        cv.notify_one();
        if (tasks.empty()) doneCV.notify_all();
    }
}

void ThreadPool::Clear() {
    std::lock_guard lock(queueMutex);
    tasks = {};
}

void ThreadPool::WaitUntilEmpty() {
    std::unique_lock lock(doneMutex);
    doneCV.wait(lock, [this] {return (tasks.empty() || paused) && AllThreadsDone();});
}

void ThreadPool::Resume() {
    paused = false;
    cv.notify_all();
}

bool ThreadPool::AllThreadsDone() {
    std::lock_guard _(workerMutex);
    for (size_t i = 0; i < this->threads.size(); i++) {
        bool b = workflags[i];
        if (b) return false;
    }
    return true;
}

unsigned int ThreadPool::GetRemainingTasks() const {
    std::lock_guard lock(queueMutex);
    return tasks.size();
}

int ThreadPool::GetThreadIndex() const {
    auto thisId = std::this_thread::get_id();
    if (!idMap.contains(thisId)) return -1;
    return idMap.at(thisId);
}
