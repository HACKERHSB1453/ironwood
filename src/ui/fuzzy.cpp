// SPDX-License-Identifier: GPL-3.0-or-later
#include "ui/fuzzy.hpp"

#include <algorithm>
#include <cstddef>

namespace iw::ui {
namespace {

constexpr int kRecentBonusMax = 5;

bool isWordBoundary(QChar prev, QChar cur) {
    if (!prev.isLetterOrNumber()) return true;
    return prev.isLower() && cur.isUpper(); // camelCase hump
}

} // namespace

int fuzzyScore(QStringView query, QStringView text) {
    const QString q = query.toString().simplified().remove(QLatin1Char(' ')).toLower();
    if (q.isEmpty() || text.isEmpty()) return 0;
    const QString t = text.toString();
    const QString tl = t.toLower();

    int score = 0;
    qsizetype pos = 0;            // next text index a query character may match at
    qsizetype prevMatch = -2;     // index of the previous matched character
    for (qsizetype qi = 0; qi < q.size(); ++qi) {
        const qsizetype j = tl.indexOf(q.at(qi), pos);
        if (j < 0) return 0;
        int pts = 10;
        const bool boundary = j == 0 || isWordBoundary(t.at(j - 1), t.at(j));
        if (boundary) pts += 12;
        if (j == prevMatch + 1) pts += 8;
        if (j == 0) pts += 10;
        pts -= static_cast<int>(std::min<qsizetype>(j - pos, 5)); // gap cost
        score += pts;
        prevMatch = j;
        pos = j + 1;
    }
    score -= static_cast<int>(t.size()) / 4; // prefer shorter names on equal matches
    return std::max(score, 1);
}

std::vector<const Command*> rankCommands(const CommandRegistry& registry, const QString& query,
                                         const std::vector<std::string>& recent) {
    auto recentRank = [&](const std::string& id) -> int {
        auto it = std::find(recent.begin(), recent.end(), id);
        return it == recent.end() ? -1 : static_cast<int>(it - recent.begin());
    };
    auto listable = [](const Command& c) { return c.id != "sbox"; };

    std::vector<const Command*> out;
    const QString q = query.simplified();
    if (q.isEmpty()) {
        for (const std::string& id : recent) {
            const Command* c = registry.find(id);
            if (c && listable(*c)) out.push_back(c);
        }
        for (const Command& c : registry.all()) {
            if (listable(c) && recentRank(c.id) < 0) out.push_back(&c);
        }
        return out;
    }

    struct Scored {
        const Command* cmd;
        int score;
    };
    std::vector<Scored> scored;
    for (const Command& c : registry.all()) {
        if (!listable(c)) continue;
        const int s = std::max(fuzzyScore(q, c.label), fuzzyScore(q, QString::fromStdString(c.id)));
        if (s == 0) continue;
        int bonus = 0;
        if (const int r = recentRank(c.id); r >= 0) bonus += kRecentBonusMax - std::min(r, kRecentBonusMax - 1);
        if (c.implemented()) bonus += 2;
        scored.push_back({&c, s + bonus});
    }
    std::stable_sort(scored.begin(), scored.end(), [](const Scored& a, const Scored& b) { return a.score > b.score; });
    for (const Scored& s : scored) out.push_back(s.cmd);
    return out;
}

} // namespace iw::ui
