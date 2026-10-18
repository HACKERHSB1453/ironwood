// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/icons.hpp"

#include <QByteArray>
#include <QFile>
#include <QPainter>
#include <QRectF>
#include <QPalette>
#include <QPixmap>
#include <QString>
#include <QSvgRenderer>
#include <QtGlobal>

/// Links the icon resources into the static library (a static archive would otherwise drop the object file).
static void ensureIconResources() {
    static const bool loaded = [] {
        Q_INIT_RESOURCE(icons);
        return true;
    }();
    (void)loaded;
}

namespace iw::ui {

namespace {

constexpr const char* kMissing = "missing";
constexpr const char* kStrokeLight = "#2b2f36"; // stroke on a light UI
constexpr const char* kStrokeDark = "#e8eaed";  // stroke on a dark UI

QString resourcePath(std::string_view name) {
    ensureIconResources();
    return QStringLiteral(":/icons/") + QString::fromUtf8(name.data(), static_cast<qsizetype>(name.size())) +
           QStringLiteral(".svg");
}

/// Reads the SVG source for `name`, or for the fallback when it does not exist. Empty when even that is missing.
QByteArray svgSource(std::string_view name) {
    QFile f(resourcePath(name));
    if (!name.empty() && f.open(QIODevice::ReadOnly)) return f.readAll();
    QFile fallback(resourcePath(kMissing));
    return fallback.open(QIODevice::ReadOnly) ? fallback.readAll() : QByteArray();
}

} // namespace

IconTheme iconThemeFor(const QPalette& palette) {
    return palette.color(QPalette::Window).lightness() < 128 ? IconTheme::Dark : IconTheme::Light;
}

bool iconExists(std::string_view name) {
    return !name.empty() && QFile::exists(resourcePath(name));
}

QImage renderIcon(std::string_view name, int size, IconTheme theme) {
    if (size <= 0) return {};
    QByteArray svg = svgSource(name);
    if (svg.isEmpty()) return {};
    svg.replace("currentColor", theme == IconTheme::Dark ? kStrokeDark : kStrokeLight);

    // Rasterised by QSvgRenderer (linked, Qt6::Svg): no imageformats plugin is needed at run time.
    QSvgRenderer renderer(svg);
    if (!renderer.isValid()) return {};
    QImage image(size, size, QImage::Format_ARGB32_Premultiplied);
    image.fill(Qt::transparent);
    QPainter painter(&image);
    renderer.render(&painter, QRectF(0, 0, size, size));
    painter.end();
    return image;
}

QIcon icon(std::string_view name, int size, IconTheme theme) {
    QImage image = renderIcon(name, size, theme);
    if (image.isNull()) image = renderIcon(kMissing, size, theme);
    QIcon result;
    if (!image.isNull()) result.addPixmap(QPixmap::fromImage(image));
    return result;
}

} // namespace iw::ui
