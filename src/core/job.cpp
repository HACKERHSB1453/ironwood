// SPDX-License-Identifier: GPL-3.0-or-later
#include "core/job.hpp"

#include <array>
#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>

#include <oneapi/tbb/task_arena.h>

namespace iw {

// Scheduling model: every submit pushes its job onto the deque for its priority and posts one
// "drain" task to the arena. A drain task pops the highest-priority job queued at the moment it runs.
// Each successfully posted drain consumes exactly one job, so jobs are never stranded. If posting
// fails (bad_alloc inside TBB), the submitting thread runs the highest-priority job itself so the
// count still balances.
struct JobSystem::Impl {
    explicit Impl(unsigned threads) : arena(threads == 0 ? tbb::task_arena::automatic : int(threads)) {}

    tbb::task_arena arena;

    std::mutex mutex;
    std::condition_variable idle;
    // Indexed by Priority; the enum order (High, Normal, Low) is the drain order.
    std::array<std::deque<std::move_only_function<void()>>, 3> queues; // guarded by mutex
    std::size_t outstanding = 0; // submitted and not yet finished; guarded by mutex

    /// Pops the highest-priority queued job and runs it. Never holds the mutex while running a job,
    /// so jobs may submit further jobs.
    void runOne() {
        std::move_only_function<void()> task;
        {
            std::lock_guard lock(mutex);
            for (auto& q : queues) {
                if (!q.empty()) {
                    task = std::move(q.front());
                    q.pop_front();
                    break;
                }
            }
        }
        if (!task) return;
        try {
            task();
        } catch (...) {
            // JobSystem::submit's wrapper converts every exception into a Result, so this only fires
            // if completing a handle itself failed. The job is still accounted for below.
        }
        std::lock_guard lock(mutex);
        if (--outstanding == 0) idle.notify_all();
    }
};

JobSystem::JobSystem(unsigned threads) : impl_(std::make_unique<Impl>(threads)) {}

JobSystem::~JobSystem() { waitAll(); }

void JobSystem::enqueue(Priority p, std::move_only_function<void()> task) {
    Impl& im = *impl_;
    {
        std::lock_guard lock(im.mutex);
        im.queues[static_cast<std::size_t>(p)].push_back(std::move(task));
        ++im.outstanding;
    }
    try {
        im.arena.enqueue([&im] { im.runOne(); });
    } catch (...) {
        // Could not post a drain task for this job: run one queued job on this thread instead.
        im.runOne();
    }
}

void JobSystem::waitAll() {
    std::unique_lock lock(impl_->mutex);
    impl_->idle.wait(lock, [this] { return impl_->outstanding == 0; });
}

} // namespace iw
