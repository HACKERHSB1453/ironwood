// SPDX-License-Identifier: GPL-3.0-or-later
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/cancel.hpp"
#include "core/ids.hpp"
#include "core/job.hpp"
#include "core/result.hpp"
#include "core/undo.hpp"
#include "core/units.hpp"

using namespace iw;

TEST(Result, ErrorPropagation) {
    Result<int> ok = 5;
    EXPECT_TRUE(ok);
    Result<int> bad = notImplemented("x");
    ASSERT_FALSE(bad);
    EXPECT_EQ(bad.error().code, ErrorCode::NotImplemented);
    EXPECT_STREQ(errorCodeName(bad.error().code), "not_implemented");
}

TEST(Ids, StrongAndHashable) {
    FeatureId a, b{3};
    EXPECT_FALSE(a.valid());
    EXPECT_TRUE(b.valid());
    EXPECT_LT(a, b);
    EXPECT_EQ(std::hash<FeatureId>{}(b), std::hash<FeatureId>{}(FeatureId{3}));
    static_assert(!std::is_convertible_v<FeatureId, BodyId>);
    static_assert(!std::is_convertible_v<std::uint64_t, FeatureId>);
}

TEST(Units, Conversions) {
    EXPECT_DOUBLE_EQ(toMillimeters(1, LengthUnit::Inch), 25.4);
    EXPECT_DOUBLE_EQ(fromMillimeters(1000, LengthUnit::Meter), 1.0);
    EXPECT_NEAR(convertLength(12, LengthUnit::Inch, LengthUnit::Foot), 1.0, 1e-12);
    EXPECT_EQ(parseLengthUnit("in"), LengthUnit::Inch);
    EXPECT_FALSE(parseLengthUnit("parsec"));
    EXPECT_EQ(unitSymbol(LengthUnit::Centimeter), "cm");
    EXPECT_NEAR(degreesToRadians(180), 3.14159265358979, 1e-12);
}

TEST(Cancel, TokenSeesSource) {
    CancelToken never;
    EXPECT_FALSE(never.cancelled());
    CancelSource src;
    CancelToken t = src.token(), copy = t;
    EXPECT_FALSE(copy.cancelled());
    src.cancel();
    EXPECT_TRUE(t.cancelled());
    EXPECT_TRUE(copy.cancelled());
}

namespace {
struct AddCmd : Command {
    AddCmd(int& v, int d, bool mergeable = false) : v_(v), d_(d), mergeable_(mergeable) {}
    Result<void> execute() override {
        v_ += d_;
        return {};
    }
    void undo() override { v_ -= d_; }
    std::string text() const override { return "add " + std::to_string(d_); }
    int mergeId() const override { return mergeable_ ? 7 : -1; }
    bool mergeWith(const Command& n) override {
        d_ += static_cast<const AddCmd&>(n).d_;
        return true;
    }
    int& v_;
    int d_;
    bool mergeable_;
};
struct FailCmd : Command {
    Result<void> execute() override { return makeError(ErrorCode::Internal, "no"); }
    void undo() override {}
    std::string text() const override { return "fail"; }
};
} // namespace

TEST(UndoStack, PushUndoRedo) {
    int v = 0;
    UndoStack s;
    ASSERT_TRUE(s.push(std::make_unique<AddCmd>(v, 1)));
    ASSERT_TRUE(s.push(std::make_unique<AddCmd>(v, 10)));
    EXPECT_EQ(v, 11);
    EXPECT_EQ(s.undoText(), "add 10");
    ASSERT_TRUE(s.undo());
    EXPECT_EQ(v, 1);
    EXPECT_EQ(s.redoText(), "add 10");
    ASSERT_TRUE(s.redo());
    EXPECT_EQ(v, 11);
    ASSERT_TRUE(s.undo());
    ASSERT_TRUE(s.push(std::make_unique<AddCmd>(v, 100))); // discards redo tail
    EXPECT_FALSE(s.canRedo());
    EXPECT_EQ(v, 101);
    ASSERT_TRUE(s.undo());
    ASSERT_TRUE(s.undo());
    EXPECT_EQ(v, 0);
    EXPECT_FALSE(s.undo());
}

TEST(UndoStack, FailedCommandNotRecorded) {
    UndoStack s;
    EXPECT_FALSE(s.push(std::make_unique<FailCmd>()));
    EXPECT_EQ(s.size(), 0u);
    EXPECT_FALSE(s.push(nullptr));
}

TEST(UndoStack, MergeAndClean) {
    int v = 0;
    UndoStack s;
    EXPECT_TRUE(s.push(std::make_unique<AddCmd>(v, 1, true)));
    EXPECT_TRUE(s.push(std::make_unique<AddCmd>(v, 2, true)));
    EXPECT_EQ(s.size(), 1u);
    EXPECT_EQ(v, 3);
    EXPECT_TRUE(s.undo());
    EXPECT_EQ(v, 0);
    EXPECT_TRUE(s.redo());
    s.setClean();
    EXPECT_TRUE(s.isClean());
    EXPECT_TRUE(s.push(std::make_unique<AddCmd>(v, 4, true))); // merges into saved command -> no longer clean
    EXPECT_FALSE(s.isClean());
    EXPECT_TRUE(s.undo());
    EXPECT_EQ(v, 0);
    EXPECT_FALSE(s.isClean());
}

TEST(JobSystem, SubmitWaitAndCancel) {
    JobSystem js(2);
    auto h = js.submit(Priority::Normal, [](CancelToken) -> Result<int> { return 42; });
    ASSERT_TRUE(h.wait());
    EXPECT_EQ(*h.wait(), 42);
    EXPECT_TRUE(h.ready());

    auto thrower = js.submit(Priority::Low, [](CancelToken) -> Result<int> { throw std::runtime_error("boom"); });
    EXPECT_EQ(thrower.wait().error().code, ErrorCode::Internal);

    std::atomic<bool> release{false};
    auto spin = js.submit(Priority::High, [&](CancelToken t) -> Result<int> {
        while (!release && !t.cancelled()) std::this_thread::yield();
        if (t.cancelled()) return makeError(ErrorCode::Cancelled, "c");
        return 1;
    });
    spin.cancel();
    EXPECT_EQ(spin.wait().error().code, ErrorCode::Cancelled);
}

TEST(JobSystem, LatestWinsCancelsPrevious) {
    JobSystem js(2);
    LatestWins<int> lw(js);
    auto first = lw.submit(Priority::High, [](CancelToken t) -> Result<int> {
        while (!t.cancelled()) std::this_thread::yield();
        return makeError(ErrorCode::Cancelled, "superseded");
    });
    auto second = lw.submit(Priority::High, [](CancelToken) -> Result<int> { return 2; });
    EXPECT_EQ(first.wait().error().code, ErrorCode::Cancelled);
    EXPECT_EQ(*second.wait(), 2);
    EXPECT_EQ(*lw.latest().wait(), 2);
}

namespace {
// Occupies the single worker of a JobSystem(1) until `release` is set.
struct WorkerGate {
    std::atomic<bool> started{false};
    std::atomic<bool> release{false};
    JobHandle<int> handle;
    explicit WorkerGate(JobSystem& js) {
        handle = js.submit(Priority::Normal, [this](CancelToken) -> Result<int> {
            started = true;
            while (!release) std::this_thread::yield();
            return 0;
        });
        while (!started) std::this_thread::yield();
    }
};
} // namespace

TEST(JobSystem, PriorityOrderSingleWorker) {
    JobSystem js(1);
    WorkerGate gate(js);
    std::mutex m;
    std::vector<std::string> order;
    auto record = [&](std::string tag) {
        return [&, tag](CancelToken) -> Result<int> {
            std::lock_guard lock(m);
            order.push_back(tag);
            return 0;
        };
    };
    (void)js.submit(Priority::Low, record("low1"));
    (void)js.submit(Priority::Normal, record("normal1"));
    (void)js.submit(Priority::High, record("high1"));
    (void)js.submit(Priority::Low, record("low2"));
    (void)js.submit(Priority::High, record("high2"));
    gate.release = true;
    js.waitAll();
    EXPECT_EQ(order, (std::vector<std::string>{"high1", "high2", "normal1", "low1", "low2"}));
}

TEST(JobSystem, StressTenThousandJobs) {
    JobSystem js(0);
    std::atomic<int> ran{0};
    std::vector<JobHandle<int>> handles;
    handles.reserve(10000);
    for (int i = 0; i < 10000; ++i) {
        const auto p = static_cast<Priority>(i % 3);
        handles.push_back(js.submit(p, [&ran, i](CancelToken) -> Result<int> {
            ran.fetch_add(1, std::memory_order_relaxed);
            return i;
        }));
    }
    js.waitAll();
    EXPECT_EQ(ran.load(), 10000);
    for (int i = 0; i < 10000; ++i) {
        ASSERT_TRUE(handles[i].ready());
        EXPECT_EQ(*handles[i].wait(), i);
    }
}

TEST(JobSystem, WaitAllBlocksUntilSlowJobFinishes) {
    JobSystem js(2);
    std::atomic<bool> done{false};
    auto h = js.submit(Priority::Normal, [&done](CancelToken) -> Result<int> {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        done = true;
        return 7;
    });
    js.waitAll();
    EXPECT_TRUE(done);
    EXPECT_TRUE(h.ready());
    EXPECT_EQ(*h.wait(), 7);
}

TEST(JobSystem, CancelBeforeStartNeverRunsBody) {
    JobSystem js(1);
    WorkerGate gate(js);
    std::atomic<bool> bodyRan{false};
    auto victim = js.submit(Priority::High, [&bodyRan](CancelToken) -> Result<int> {
        bodyRan = true;
        return 1;
    });
    victim.cancel();
    gate.release = true;
    js.waitAll();
    EXPECT_FALSE(bodyRan.load());
    EXPECT_TRUE(victim.ready());
    EXPECT_EQ(victim.wait().error().code, ErrorCode::Cancelled);
}

TEST(JobSystem, NonStdExceptionBecomesInternal) {
    JobSystem js(2);
    auto h = js.submit(Priority::Normal, [](CancelToken) -> Result<int> { throw 42; });
    EXPECT_EQ(h.wait().error().code, ErrorCode::Internal);
    // The system stays usable after a throwing job.
    auto ok = js.submit(Priority::Normal, [](CancelToken) -> Result<int> { return 3; });
    EXPECT_EQ(*ok.wait(), 3);
}

TEST(JobSystem, DestructorRunsPendingJobs) {
    std::atomic<int> ran{0};
    {
        JobSystem js(2);
        for (int i = 0; i < 500; ++i) {
            (void)js.submit(Priority::Low, [&ran](CancelToken) -> Result<int> {
                ran.fetch_add(1, std::memory_order_relaxed);
                return 0;
            });
        }
    }
    EXPECT_EQ(ran.load(), 500);
}

TEST(JobHandle, InvalidHandleIsSafe) {
    JobHandle<int> h;
    EXPECT_FALSE(h.valid());
    EXPECT_FALSE(h.ready());
    h.cancel();
    EXPECT_FALSE(h.cancelRequested());
    EXPECT_EQ(h.wait().error().code, ErrorCode::InvalidArgument);
}

TEST(LatestWins, ThousandRapidSubmitsEndWithNewest) {
    JobSystem js(4);
    LatestWins<int> lw(js);
    std::vector<JobHandle<int>> handles;
    handles.reserve(1000);
    for (int i = 0; i < 1000; ++i) {
        handles.push_back(lw.submit(Priority::High, [i](CancelToken t) -> Result<int> {
            if (t.cancelled()) return makeError(ErrorCode::Cancelled, "superseded");
            return i;
        }));
    }
    js.waitAll();
    ASSERT_TRUE(lw.latest().ready());
    EXPECT_EQ(*lw.latest().wait(), 999);
    for (int i = 0; i < 999; ++i) {
        const auto& r = handles[i].wait();
        if (r) {
            EXPECT_EQ(*r, i);
        } else {
            EXPECT_EQ(r.error().code, ErrorCode::Cancelled);
        }
    }
}
