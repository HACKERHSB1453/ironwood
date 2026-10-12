// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
// PRIVATE to iw_kernel: bridges iw::CancelToken to OCCT's progress/user-break mechanism.

#include <Message_ProgressIndicator.hxx>
#include <Message_ProgressRange.hxx>

#include "core/cancel.hpp"

namespace iw::kernel {

class CancelProgress final : public Message_ProgressIndicator {
public:
    explicit CancelProgress(CancelToken t) : token_(std::move(t)) {}
    void Show(const Message_ProgressScope&, const Standard_Boolean) override {}
    Standard_Boolean UserBreak() override { return token_.cancelled(); }

private:
    CancelToken token_;
};

} // namespace iw::kernel
