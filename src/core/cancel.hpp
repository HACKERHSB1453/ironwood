// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <atomic>
#include <memory>

namespace iw {

/// Cheap, copyable, read-only view of a cancellation flag. Long-running operations take a
/// `CancelToken` and poll `cancelled()` at safe points, returning `ErrorCode::Cancelled` early.
/// A default-constructed token is never cancelled. Thread-safe (lock-free reads).
class CancelToken {
public:
    CancelToken() noexcept = default;
    /// True once the owning CancelSource::cancel() was called. Monotonic: never goes back to false.
    bool cancelled() const noexcept { return flag_ && flag_->load(std::memory_order_relaxed); }

private:
    friend class CancelSource;
    explicit CancelToken(std::shared_ptr<std::atomic<bool>> f) noexcept : flag_(std::move(f)) {}
    std::shared_ptr<std::atomic<bool>> flag_;
};

/// Owner side of cancellation. Copies share the same flag. Tokens stay valid after the source dies.
/// Thread-safe.
class CancelSource {
public:
    CancelSource() : flag_(std::make_shared<std::atomic<bool>>(false)) {}
    /// Requests cancellation (idempotent).
    void cancel() noexcept { flag_->store(true, std::memory_order_relaxed); }
    bool cancelled() const noexcept { return flag_->load(std::memory_order_relaxed); }
    CancelToken token() const noexcept { return CancelToken(flag_); }

private:
    std::shared_ptr<std::atomic<bool>> flag_;
};

} // namespace iw
