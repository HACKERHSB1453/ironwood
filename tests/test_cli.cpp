// SPDX-License-Identifier: GPL-3.0-or-later
// ironwood-cli end-to-end: the test generates its own STEP/STL fixtures with the kernel, runs the built CLI binary
// (path injected by tests/cli.cmake) and asserts the printed counts. A box has 6 faces, 12 edges, 8 vertices and
// 12 triangles (2 per face at the CLI's 0.1 mm deflection).
#include <gtest/gtest.h>

#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <optional>
#include <string>

#include "kernel/exchange.hpp"
#include "kernel/ops.hpp"

#ifndef IRONWOOD_CLI_PATH
#error "IRONWOOD_CLI_PATH must be defined by tests/cli.cmake"
#endif

namespace fs = std::filesystem;
namespace kernel = iw::kernel;

namespace {

struct CliRun {
    int exitCode = -1;
    std::string output; ///< stdout and stderr, interleaved in write order
};

CliRun runCli(const std::string& args) {
    CliRun r;
    const std::string cmd = std::string("'") + IRONWOOD_CLI_PATH + "' " + args + " 2>&1";
    FILE* pipe = ::popen(cmd.c_str(), "r");
    if (!pipe) return r;
    char buf[4096];
    while (std::size_t n = std::fread(buf, 1, sizeof buf, pipe)) r.output.append(buf, n);
    const int status = ::pclose(pipe);
    r.exitCode = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    return r;
}

/// Value of a "key: <number>" line in the --stats output, or nullopt when absent.
std::optional<long long> statValue(const std::string& output, const std::string& key) {
    const std::string needle = "\n" + key + ": ";
    const std::string text = "\n" + output;
    const std::size_t at = text.find(needle);
    if (at == std::string::npos) return std::nullopt;
    return std::stoll(text.substr(at + needle.size()));
}

kernel::Shape box(double dx, double dy, double dz) {
    auto r = kernel::makeBox({0, 0, 0}, dx, dy, dz);
    EXPECT_TRUE(r.has_value());
    return r ? r->shape : kernel::Shape{};
}

} // namespace

class CliTest : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = fs::temp_directory_path() / ("ironwood-cli-test-" + std::to_string(::getpid()));
        fs::create_directories(dir_);
    }
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }
    fs::path dir_;
};

TEST_F(CliTest, VersionPrintsNameAndOcctAndExitsZero) {
    const CliRun r = runCli("--version");
    EXPECT_EQ(r.exitCode, 0);
    EXPECT_NE(r.output.find("ironwood-cli "), std::string::npos) << r.output;
    EXPECT_NE(r.output.find("(OCCT "), std::string::npos) << r.output;
}

TEST_F(CliTest, NoArgumentsIsUsageError) {
    const CliRun r = runCli("");
    EXPECT_EQ(r.exitCode, 64);
    EXPECT_NE(r.output.find("usage:"), std::string::npos);
}

TEST_F(CliTest, UnsupportedExtensionIsUsageError) {
    const fs::path file = dir_ / "part.dwg";
    const CliRun r = runCli("import '" + file.string() + "' --stats");
    EXPECT_EQ(r.exitCode, 64);
    EXPECT_NE(r.output.find("unsupported file extension"), std::string::npos);
}

TEST_F(CliTest, MissingStepFileIsRuntimeError) {
    const fs::path file = dir_ / "missing.step";
    const CliRun r = runCli("import '" + file.string() + "' --stats");
    EXPECT_EQ(r.exitCode, 2);
    EXPECT_NE(r.output.find("error ["), std::string::npos) << r.output;
}

TEST_F(CliTest, StatsOnGeneratedStepBoxMatchesTopology) {
    const fs::path file = dir_ / "box.step";
    auto w = kernel::exportStep(file, box(10, 20, 30));
    ASSERT_TRUE(w.has_value()) << w.error().message;

    const CliRun r = runCli("import '" + file.string() + "' --stats");
    ASSERT_EQ(r.exitCode, 0) << r.output;
    EXPECT_EQ(statValue(r.output, "solids"), 1);
    EXPECT_EQ(statValue(r.output, "faces"), 6);
    EXPECT_EQ(statValue(r.output, "edges"), 12);
    EXPECT_EQ(statValue(r.output, "vertices"), 8);
    EXPECT_EQ(statValue(r.output, "triangles"), 12);
    EXPECT_EQ(statValue(r.output, "nodes"), 1);
    EXPECT_EQ(statValue(r.output, "roots"), 1);
    EXPECT_NE(r.output.find("length unit in file: mm"), std::string::npos) << r.output;
    EXPECT_NE(r.output.find("peak RSS:"), std::string::npos);
}

TEST_F(CliTest, StatsOnGeneratedStlCountsFacets) {
    const fs::path file = dir_ / "box.stl";
    auto w = kernel::exportStl(file, box(10, 20, 30), {});
    ASSERT_TRUE(w.has_value()) << w.error().message;

    const CliRun r = runCli("import '" + file.string() + "' --stats");
    ASSERT_EQ(r.exitCode, 0) << r.output;
    // One planar triangle face per facet: 12 facets for a box, each also one triangle.
    EXPECT_EQ(statValue(r.output, "faces"), 12);
    EXPECT_EQ(statValue(r.output, "triangles"), 12);
    EXPECT_EQ(statValue(r.output, "solids"), 1);
}

TEST_F(CliTest, StatsOnCurvedBodyReportsTriangles) {
    auto cyl = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, 10.0, 20.0);
    ASSERT_TRUE(cyl.has_value());
    const fs::path file = dir_ / "cylinder.step";
    ASSERT_TRUE(kernel::exportStep(file, cyl->shape).has_value());

    const CliRun r = runCli("import '" + file.string() + "' --stats");
    ASSERT_EQ(r.exitCode, 0) << r.output;
    EXPECT_EQ(statValue(r.output, "faces"), 3);     // lateral, top, bottom
    EXPECT_EQ(statValue(r.output, "solids"), 1);
    const auto triangles = statValue(r.output, "triangles");
    ASSERT_TRUE(triangles.has_value());
    EXPECT_GT(*triangles, 12); // curved faces need more than the box's two triangles per face
}

TEST_F(CliTest, ImportWithoutStatsIsSilentOnSuccess) {
    const fs::path file = dir_ / "quiet.step";
    ASSERT_TRUE(kernel::exportStep(file, box(1, 2, 3)).has_value());
    const CliRun r = runCli("import '" + file.string() + "'");
    EXPECT_EQ(r.exitCode, 0) << r.output;
    EXPECT_TRUE(r.output.empty()) << r.output;
}
