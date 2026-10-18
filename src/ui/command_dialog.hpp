// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <QWidget>

#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/cancel.hpp"
#include "core/expr.hpp"
#include "core/job.hpp"
#include "core/params.hpp"
#include "core/result.hpp"

#include <array>
#include <memory>

class QCheckBox;
class QComboBox;
class QEvent;
class QLabel;
class QLineEdit;
class QListWidget;
class QPushButton;
class QTimer;

namespace iw::tess {
struct MeshData;
}

namespace iw::ui {

enum class InputKind { Selection, Value, Dropdown, Checkbox };

/// One field of a command dialog.
struct InputSpec {
    std::string name;                  ///< Key in the snapshot.
    QString label;
    InputKind kind = InputKind::Value;
    Dimension dim = Dimension::Length; ///< Value: the dimension the expression must have.
    std::string defaultText;           ///< Value: expression; Dropdown: option; Checkbox: "1"/"0".
    std::vector<QString> options;      ///< Dropdown choices (the value the snapshot keeps); Selection candidates (by label).
    /// Dropdown: text shown for each option (same size as `options`; empty = the option itself). The option stays the
    /// item data, so the value is independent of what the user reads.
    std::vector<QString> optionLabels = {};
};

/// Qt-free state of a command dialog: the raw text of every field, validation through a parameter table, and the
/// typed snapshot a command acts on. Value fields accept expressions such as `10 mm + d1`. A bare number is read in
/// the field's dimension (mm for lengths, degrees for angles).
class DialogModel {
public:
    struct Snapshot {
        std::map<std::string, Quantity, std::less<>> values;
        std::map<std::string, std::string, std::less<>> choices;               ///< Dropdown text, Checkbox "1"/"0".
        std::map<std::string, std::vector<std::string>, std::less<>> selections;
    };

    DialogModel(std::vector<InputSpec> specs, const ParameterTable* params);

    const std::vector<InputSpec>& specs() const noexcept { return specs_; }
    /// Sets the raw text of a field (expression, dropdown option, "1"/"0" for checkboxes).
    void setText(std::string_view name, std::string text);
    std::string text(std::string_view name) const;
    /// Adds the item when absent, removes it when present.
    void toggleSelection(std::string_view name, const std::string& item);
    std::vector<std::string> selection(std::string_view name) const;
    void setSelection(std::string_view name, std::vector<std::string> items);

    /// Evaluated value of a Value field. Errors: InvalidArgument (empty, dimension mismatch), ParseError, NotFound.
    Result<Quantity> valueOf(std::string_view name) const;
    /// The error of a Value field, nullopt when valid (or when the field is not a Value field).
    std::optional<Error> errorFor(std::string_view name) const;
    bool canAccept() const;
    /// Every field evaluated; the first failing field's error when any is invalid.
    Result<Snapshot> snapshot() const;

private:
    const InputSpec* spec(std::string_view name) const;

    std::vector<InputSpec> specs_;
    const ParameterTable* params_;
    std::map<std::string, std::string, std::less<>> texts_;
    std::map<std::string, std::vector<std::string>, std::less<>> selections_;
};

/// Evaluates expression text as a value of dimension `expected`. Bare numbers take the expected unit (mm, degrees).
Result<Quantity> evaluateValueText(std::string_view text, Dimension expected, const ParameterTable* params);

/// Geometry a preview produced besides its text (the owner draws it over the viewport). World-space triangles.
struct PreviewMesh {
    std::shared_ptr<const tess::MeshData> mesh;   ///< nullptr: no geometry.
    std::array<float, 4> rgba{0.2f, 0.85f, 0.3f, 0.45f};   ///< RGBA 0..1, straight alpha.
};

struct PreviewResult {
    std::uint64_t seq = 0;   ///< Request number, increasing.
    std::string text;        ///< Preview text, or the failure message when `failed`.
    bool failed = false;
    PreviewMesh geometry;    ///< Empty unless the preview produced geometry (never set when `failed`).
};
using PreviewFn = std::function<Result<std::string>(const DialogModel::Snapshot&, CancelToken)>;
/// A preview that also produces geometry (see PreviewMesh).
struct PreviewOutput {
    std::string text;
    PreviewMesh geometry;
};
using GeometryPreviewFn = std::function<Result<PreviewOutput>(const DialogModel::Snapshot&, CancelToken)>;

/// Live preview on a latest-wins job: every request supersedes the previous one (which is cancelled), and `poll()`
/// (GUI thread, never blocks) delivers only the newest finished preview, once.
class LivePreview {
public:
    LivePreview(JobSystem& jobs, PreviewFn fn, std::function<void(const PreviewResult&)> onResult);
    LivePreview(JobSystem& jobs, GeometryPreviewFn fn, std::function<void(const PreviewResult&)> onResult);

    /// Starts a preview for `snap`. Never blocks.
    void request(DialogModel::Snapshot snap);
    /// Delivers the newest finished preview if it has not been delivered yet. Returns true when the callback ran.
    bool poll();
    /// Number of requests made so far.
    std::uint64_t requests() const noexcept { return seq_; }

private:
    GeometryPreviewFn fn_;
    std::function<void(const PreviewResult&)> onResult_;
    LatestWins<PreviewResult> latest_;
    std::uint64_t seq_ = 0;
    std::uint64_t delivered_ = 0;
};

/// A command dialog: its fields, the preview function and what OK does.
struct CommandDialogSpec {
    QString title;
    std::vector<InputSpec> inputs;
    PreviewFn preview;   ///< Optional.
    /// Optional, used instead of `preview` when set: a preview that also produces geometry.
    GeometryPreviewFn geometryPreview;
    std::function<Result<void>(const DialogModel::Snapshot&)> accept; ///< Called on OK with a valid snapshot.
    /// Optional: called after a field's text changed (typed, picked, or set with setText) and its preview requested.
    std::function<void(const std::string& field)> fieldChanged;
    /// Optional: called on the GUI thread with every preview result delivered (failures included).
    std::function<void(const PreviewResult&)> previewDelivered;
    /// Optional: a dropdown option under the mouse changed (`option` = its value; empty when none is under the mouse).
    std::function<void(const std::string& field, const std::string& option)> optionHovered;
};

/// Right-docked command panel: a form built from the spec, inline errors per field, a live preview line, and
/// OK/Cancel. Editing a field never waits for the preview; OK is enabled only while every field is valid.
class CommandPanel : public QWidget {
    Q_OBJECT
public:
    CommandPanel(CommandDialogSpec spec, const ParameterTable* params, JobSystem& jobs, QWidget* parent = nullptr);

    DialogModel& model() noexcept { return model_; }
    const DialogModel& model() const noexcept { return model_; }
    /// Sets a field's text from code (e.g. a manipulator drag) and updates the widget and the preview.
    void setText(std::string_view name, const QString& text);
    QString previewText() const;
    QString errorText() const;   ///< Form-level error (e.g. a failed accept), empty otherwise.
    bool okEnabled() const;
    /// Pumps the preview (the panel also does this on a timer).
    void pollPreview();
    /// Runs OK: validates, calls accept, emits finished(true) on success.
    void accept();
    /// Emits finished(false).
    void reject();

signals:
    void finished(bool accepted);

private:
    void buildForm();
    void onFieldChanged(const std::string& name);
    void refreshValidation();
    void requestPreview();
    bool eventFilter(QObject* watched, QEvent* event) override;

    CommandDialogSpec spec_;
    DialogModel model_;
    LivePreview preview_;
    QTimer* pollTimer_ = nullptr;
    QLabel* previewLabel_ = nullptr;
    QLabel* errorLabel_ = nullptr;
    QPushButton* okButton_ = nullptr;
    std::map<std::string, QLineEdit*, std::less<>> valueEdits_;
    std::map<std::string, QLabel*, std::less<>> fieldErrors_;
    std::map<std::string, QComboBox*, std::less<>> dropdowns_;
    std::map<std::string, QCheckBox*, std::less<>> checks_;
    std::map<std::string, QListWidget*, std::less<>> selectionLists_;
    std::map<QObject*, std::string, std::less<>> hoverFields_;   ///< Dropdown views that report hovered options.
};

} // namespace iw::ui
