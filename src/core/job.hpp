// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <type_traits>
#include <utility>

#include "core/cancel.hpp"
#include "core/result.hpp"

namespace iw {

/// Scheduling hint. P0 contract: `High` jobs (interactive previews) should be started before `Normal`
/// (recompute) before `Low` (background caches). The skeleton ignores it; the implementation must honour it.
enum class Priority { High, Normal, Low };

namespace detail {
template <class T> struct JobState {
    CancelSource cancel;
    std::mutex mutex;
    std::condition_variable cv;
    std::optional<Result<T>> result;

    void finish(Result<T> r) {
        {
            std::lock_guard lock(mutex);
            result.emplace(std::move(r));
        }
        cv.notify_all();
    }
};
} // namespace detail

/// Handle to a submitted job producing `Result<T>`. Copyable (shared state); thread-safe.
/// Dropping all handles does NOT cancel the job. A job cancelled before it starts completes with
/// `ErrorCode::Cancelled` without running. A job that throws completes with `ErrorCode::Internal`.
template <class T> class JobHandle {
public:
    JobHandle() = default;
    explicit JobHandle(std::shared_ptr<detail::JobState<T>> s) : state_(std::move(s)) {}

    /// False for a default-constructed handle.
    bool valid() const noexcept { return state_ != nullptr; }
    /// True once the result is available (non-blocking). False for an invalid handle.
    bool ready() const {
        if (!state_) return false;
        std::lock_guard lock(state_->mutex);
        return state_->result.has_value();
    }
    /// Blocks until done; returns the result. Reference is valid while any handle lives.
    /// Must not be called from inside the job itself. On an invalid handle returns InvalidArgument.
    const Result<T>& wait() const {
        static const Result<T> invalid = makeError(ErrorCode::InvalidArgument, "invalid JobHandle");
        if (!state_) return invalid;
        std::unique_lock lock(state_->mutex);
        state_->cv.wait(lock, [&] { return state_->result.has_value(); });
        return *state_->result;
    }
    /// Requests cooperative cancellation (idempotent; no-op on an invalid handle). The job may still
    /// complete successfully.
    void cancel() const noexcept {
        if (state_) state_->cancel.cancel();
    }
    bool cancelRequested() const noexcept { return state_ && state_->cancel.cancelled(); }

private:
    std::shared_ptr<detail::JobState<T>> state_;
};

/// Task scheduler over a oneTBB arena (TBB is a private dependency of the .cpp; this header is light).
/// Thread-safe. Destruction blocks until all submitted jobs finish.
class JobSystem {
public:
    /// `threads == 0` -> hardware concurrency.
    explicit JobSystem(unsigned threads = 0);
    ~JobSystem();
    JobSystem(const JobSystem&) = delete;
    JobSystem& operator=(const JobSystem&) = delete;

    /// Submits `fn(CancelToken) -> Result<T>`; returns a handle to its result.
    /// `fn` is run on a worker thread; it should poll the token. Never blocks the caller.
    template <class F> auto submit(Priority p, F&& fn) {
        using R = std::invoke_result_t<F&, CancelToken>;
        using T = typename R::value_type;
        auto state = std::make_shared<detail::JobState<T>>();
        enqueue(p, [state, f = std::forward<F>(fn)]() mutable {
            if (state->cancel.cancelled()) {
                state->finish(makeError(ErrorCode::Cancelled, "cancelled before start"));
                return;
            }
            try {
                state->finish(f(state->cancel.token()));
            } catch (const std::exception& e) {
                state->finish(makeError(ErrorCode::Internal, e.what()));
            } catch (...) {
                state->finish(makeError(ErrorCode::Internal, "unknown exception"));
            }
        });
        return JobHandle<T>(std::move(state));
    }

    /// Blocks until every submitted job has finished (including jobs submitted while waiting).
    /// Must not be called from inside a job (would deadlock the worker it runs on).
    void waitAll();

private:
    void enqueue(Priority p, std::move_only_function<void()> task);
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// Latest-wins helper for speculative previews (drag a slider -> many requests, only the newest matters).
/// `submit()` cancels the previous in-flight job and starts a new one; `latest()` is the newest handle.
/// Thread-safe. Stale results are simply never consumed by callers who always read `latest()`.
/// The job is submitted WITHOUT holding the internal mutex: if JobSystem runs it inline (its enqueue fallback
/// on a posting failure), the job may itself call latest()/submit()/cancel() without self-deadlocking.
template <class T> class LatestWins {
public:
    explicit LatestWins(JobSystem& jobs) : jobs_(jobs) {}

    /// The previous job is cancelled only after the new one has been accepted, so a throwing submit
    /// leaves the previous job as `latest()`, not a cancelled one. Submissions are ordered by a ticket taken
    /// under the lock: a slower concurrent submit that finishes after a newer one is cancelled, never installed.
    template <class F> JobHandle<T> submit(Priority p, F&& fn) {
        std::uint64_t ticket = 0;
        {
            std::lock_guard lock(mutex_);
            ticket = ++nextTicket_;
        }
        JobHandle<T> next = jobs_.submit(p, std::forward<F>(fn)); // mutex NOT held: see class comment
        JobHandle<T> superseded;
        {
            std::lock_guard lock(mutex_);
            if (ticket > latestTicket_) {
                superseded = latest_;
                latest_ = next;
                latestTicket_ = ticket;
            } else {
                next.cancel(); // a newer submit already won the race
            }
        }
        if (superseded.valid()) superseded.cancel();
        return next;
    }
    /// Newest handle (invalid before first submit).
    JobHandle<T> latest() const {
        std::lock_guard lock(mutex_);
        return latest_;
    }
    /// Cancels the newest job without replacing it.
    void cancel() {
        std::lock_guard lock(mutex_);
        if (latest_.valid()) latest_.cancel();
    }

private:
    JobSystem& jobs_;
    mutable std::mutex mutex_;
    JobHandle<T> latest_;
    std::uint64_t nextTicket_ = 0;   ///< last ticket handed out (guarded by mutex_)
    std::uint64_t latestTicket_ = 0; ///< ticket of latest_ (guarded by mutex_)
};

} // namespace iw
