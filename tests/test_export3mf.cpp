// SPDX-License-Identifier: GPL-3.0-or-later
// 3MF export: the package is read back with an independent ZIP reader (central directory, CRC-32) and the model
// XML is checked for schema-shaped structure. Counts are cross-checked against tess::tessellate with the same
// deflections, and the signed volume of each object proves the triangle winding is outward.
#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <unistd.h>
#include <vector>

#include "core/cancel.hpp"
#include "core/math.hpp"
#include "kernel/exchange.hpp"
#include "kernel/ops.hpp"
#include "tess/tessellate.hpp"

namespace fs = std::filesystem;
namespace kernel = iw::kernel;

namespace {

std::uint32_t rd16(const std::string& b, std::size_t o) {
    return static_cast<std::uint8_t>(b[o]) | (static_cast<std::uint32_t>(static_cast<std::uint8_t>(b[o + 1])) << 8);
}
std::uint32_t rd32(const std::string& b, std::size_t o) { return rd16(b, o) | (rd16(b, o + 2) << 16); }

std::uint32_t crc32Ref(const std::string& data) {
    std::uint32_t crc = 0xFFFFFFFFu;
    for (const char ch : data) {
        crc ^= static_cast<std::uint8_t>(ch);
        for (int k = 0; k < 8; ++k) crc = (crc & 1u) ? (crc >> 1) ^ 0xEDB88320u : crc >> 1;
    }
    return crc ^ 0xFFFFFFFFu;
}

/// Reads a STORED-only ZIP through its central directory. Returns entries in archive order; `ok` is false on any
/// structural problem (bad signature, method != 0, CRC mismatch, out-of-range offsets).
struct ZipView {
    bool ok = false;
    std::vector<std::string> names;
    std::map<std::string, std::string> data;
};

ZipView readZip(const std::string& bytes) {
    ZipView z;
    if (bytes.size() < 22) return z;
    std::size_t eocd = std::string::npos;
    for (std::size_t p = bytes.size() - 22 + 1; p-- > 0;) {
        if (rd32(bytes, p) == 0x06054b50u) {
            eocd = p;
            break;
        }
        if (bytes.size() - p > 22 + 65535) break;
    }
    if (eocd == std::string::npos) return z;
    const std::uint32_t count = rd16(bytes, eocd + 10);
    const std::uint32_t cdSize = rd32(bytes, eocd + 12);
    const std::uint32_t cdOffset = rd32(bytes, eocd + 16);
    if (static_cast<std::size_t>(cdOffset) + cdSize > bytes.size()) return z;

    std::size_t p = cdOffset;
    for (std::uint32_t i = 0; i < count; ++i) {
        if (p + 46 > bytes.size() || rd32(bytes, p) != 0x02014b50u) return z;
        const std::uint32_t method = rd16(bytes, p + 10);
        const std::uint32_t crc = rd32(bytes, p + 16);
        const std::uint32_t csize = rd32(bytes, p + 20);
        const std::uint32_t usize = rd32(bytes, p + 24);
        const std::uint32_t nameLen = rd16(bytes, p + 28);
        const std::uint32_t extraLen = rd16(bytes, p + 30);
        const std::uint32_t commentLen = rd16(bytes, p + 32);
        const std::uint32_t localOff = rd32(bytes, p + 42);
        if (p + 46 + nameLen > bytes.size()) return z;
        const std::string name = bytes.substr(p + 46, nameLen);
        p += 46 + nameLen + extraLen + commentLen;

        if (method != 0 || csize != usize) return z;
        if (static_cast<std::size_t>(localOff) + 30 > bytes.size()) return z;
        if (rd32(bytes, localOff) != 0x04034b50u) return z;
        const std::uint32_t lNameLen = rd16(bytes, localOff + 26);
        const std::uint32_t lExtraLen = rd16(bytes, localOff + 28);
        const std::size_t dataOff = localOff + 30 + lNameLen + lExtraLen;
        if (dataOff + csize > bytes.size()) return z;
        const std::string body = bytes.substr(dataOff, csize);
        if (crc32Ref(body) != crc) return z;
        z.names.push_back(name);
        z.data[name] = body;
    }
    z.ok = true;
    return z;
}

struct ParsedObject {
    std::string id;
    std::string pid;
    std::string pindex;
    std::vector<std::array<double, 3>> vertices;
    std::vector<std::array<std::int64_t, 3>> triangles;
};

std::optional<std::string> attr(const std::string& tag, const std::string& key) {
    const std::string needle = " " + key + "=\"";
    const std::size_t at = tag.find(needle);
    if (at == std::string::npos) return std::nullopt;
    const std::size_t start = at + needle.size();
    const std::size_t end = tag.find('"', start);
    if (end == std::string::npos) return std::nullopt;
    return tag.substr(start, end - start);
}

/// Splits the <resources> objects out of the model XML by scanning tags; enough for the exporter's known layout.
std::vector<ParsedObject> parseObjects(const std::string& xml) {
    std::vector<ParsedObject> out;
    std::size_t pos = 0;
    while (true) {
        const std::size_t objAt = xml.find("<object ", pos);
        if (objAt == std::string::npos) break;
        const std::size_t objEnd = xml.find("</object>", objAt);
        const std::string obj = xml.substr(objAt, objEnd - objAt);
        pos = objEnd + 1;

        ParsedObject po;
        const std::size_t headEnd = obj.find('>');
        const std::string head = obj.substr(0, headEnd);
        po.id = attr(head, "id").value_or("");
        po.pid = attr(head, "pid").value_or("");
        po.pindex = attr(head, "pindex").value_or("");

        std::size_t v = 0;
        while ((v = obj.find("<vertex ", v)) != std::string::npos) {
            const std::size_t e = obj.find("/>", v);
            const std::string tag = obj.substr(v, e - v);
            po.vertices.push_back({std::stod(attr(tag, "x").value_or("nan")), std::stod(attr(tag, "y").value_or("nan")),
                                   std::stod(attr(tag, "z").value_or("nan"))});
            v = e;
        }
        std::size_t t = 0;
        while ((t = obj.find("<triangle ", t)) != std::string::npos) {
            const std::size_t e = obj.find("/>", t);
            const std::string tag = obj.substr(t, e - t);
            po.triangles.push_back({std::stoll(attr(tag, "v1").value_or("-1")), std::stoll(attr(tag, "v2").value_or("-1")),
                                    std::stoll(attr(tag, "v3").value_or("-1"))});
            t = e;
        }
        out.push_back(std::move(po));
    }
    return out;
}

/// Signed volume by the divergence theorem: positive when every triangle is wound outward (CCW from outside).
double signedVolume(const ParsedObject& o) {
    double vol = 0;
    for (const auto& t : o.triangles) {
        const auto& a = o.vertices.at(static_cast<std::size_t>(t[0]));
        const auto& b = o.vertices.at(static_cast<std::size_t>(t[1]));
        const auto& c = o.vertices.at(static_cast<std::size_t>(t[2]));
        vol += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) + a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
    }
    return vol;
}

std::string readAll(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

kernel::Shape boxShape(double dx, double dy, double dz) {
    auto r = kernel::makeBox({0, 0, 0}, dx, dy, dz);
    EXPECT_TRUE(r.has_value()) << (r ? "" : r.error().message);
    return r ? r->shape : kernel::Shape{};
}

} // namespace

class Export3mf : public ::testing::Test {
protected:
    void SetUp() override {
        dir_ = fs::temp_directory_path() / ("ironwood-3mf-test-" + std::to_string(::getpid()));
        fs::create_directories(dir_);
    }
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }
    fs::path dir_;
};

TEST_F(Export3mf, PackageHasOpcPartsAndStoredEntries) {
    const fs::path file = dir_ / "box.3mf";
    auto w = kernel::export3mf(file, {kernel::MeshBody{"Box", boxShape(10, 20, 30), std::nullopt}});
    ASSERT_TRUE(w.has_value()) << w.error().message;

    const ZipView z = readZip(readAll(file));
    ASSERT_TRUE(z.ok) << "zip structure or CRC invalid";
    EXPECT_EQ(z.names, (std::vector<std::string>{"[Content_Types].xml", "_rels/.rels", "3D/3dmodel.model"}));

    const std::string& ct = z.data.at("[Content_Types].xml");
    EXPECT_NE(ct.find("Extension=\"model\" ContentType=\"application/vnd.ms-package.3dmanufacturing-3dmodel+xml\""),
              std::string::npos);
    EXPECT_NE(ct.find("Extension=\"rels\""), std::string::npos);

    const std::string& rels = z.data.at("_rels/.rels");
    EXPECT_NE(rels.find("Target=\"/3D/3dmodel.model\""), std::string::npos);
    EXPECT_NE(rels.find("http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"), std::string::npos);
}

TEST_F(Export3mf, ModelXmlHasMillimetreUnitAndOneObjectPerBody) {
    const fs::path file = dir_ / "two.3mf";
    std::vector<kernel::MeshBody> bodies{
        {"Left", boxShape(10, 10, 10), kernel::Color{1, 0, 0, 1}},
        {"Right & <Co>", boxShape(5, 5, 5), std::nullopt},
    };
    ASSERT_TRUE(kernel::export3mf(file, bodies).has_value());

    const ZipView z = readZip(readAll(file));
    ASSERT_TRUE(z.ok);
    const std::string& xml = z.data.at("3D/3dmodel.model");
    EXPECT_NE(xml.find("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"), std::string::npos);
    EXPECT_NE(xml.find("unit=\"millimeter\""), std::string::npos);
    EXPECT_NE(xml.find("xmlns=\"http://schemas.microsoft.com/3dmanufacturing/core/2015/02\""), std::string::npos);

    // Colours: one base per body in one basematerials group; red for the first, neutral grey default for the second.
    EXPECT_NE(xml.find("<m:basematerials id=\"1\">"), std::string::npos);
    EXPECT_NE(xml.find("displaycolor=\"#FF0000FF\""), std::string::npos);
    EXPECT_NE(xml.find("displaycolor=\"#CCCCCCFF\""), std::string::npos);
    EXPECT_NE(xml.find("name=\"Right &amp; &lt;Co&gt;\""), std::string::npos) << "names must be XML-escaped";

    const auto objects = parseObjects(xml);
    ASSERT_EQ(objects.size(), 2u);
    EXPECT_EQ(objects[0].id, "2");
    EXPECT_EQ(objects[1].id, "3");
    EXPECT_EQ(objects[0].pid, "1");
    EXPECT_EQ(objects[0].pindex, "0");
    EXPECT_EQ(objects[1].pindex, "1");

    // <build> has one item per object.
    std::size_t items = 0;
    for (std::size_t p = xml.find("<item "); p != std::string::npos; p = xml.find("<item ", p + 1)) ++items;
    EXPECT_EQ(items, 2u);
}

TEST_F(Export3mf, CountsMatchTessellationAndIndicesAreInRange) {
    const kernel::Shape box = boxShape(10, 20, 30);
    auto cyl = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, 8.0, 25.0);
    ASSERT_TRUE(cyl.has_value());

    const fs::path file = dir_ / "counts.3mf";
    ASSERT_TRUE(kernel::export3mf(file, {{"Box", box, std::nullopt}, {"Cyl", cyl->shape, std::nullopt}}).has_value());
    const ZipView z = readZip(readAll(file));
    ASSERT_TRUE(z.ok);
    const auto objects = parseObjects(z.data.at("3D/3dmodel.model"));
    ASSERT_EQ(objects.size(), 2u);

    // Same deflections as the exporter defaults: the kernel's tessellation is the reference.
    const std::vector<const kernel::Shape*> shapes{&box, &cyl->shape};
    for (std::size_t i = 0; i < shapes.size(); ++i) {
        auto mesh = iw::tess::tessellate(*shapes[i], {});
        ASSERT_TRUE(mesh.has_value()) << mesh.error().message;
        EXPECT_EQ(objects[i].vertices.size(), mesh->positions.size() / 3) << "object " << i;
        EXPECT_EQ(objects[i].triangles.size(), mesh->indices.size() / 3) << "object " << i;
        for (const auto& t : objects[i].triangles)
            for (const auto idx : t) {
                ASSERT_GE(idx, 0);
                ASSERT_LT(static_cast<std::size_t>(idx), objects[i].vertices.size());
            }
    }
    // A box: 6 faces x 2 triangles = 12 triangles, 6 x 4 = 24 per-face vertices.
    EXPECT_EQ(objects[0].triangles.size(), 12u);
    EXPECT_EQ(objects[0].vertices.size(), 24u);
}

TEST_F(Export3mf, GeometryIsOutwardWoundAndMillimetre) {
    const fs::path file = dir_ / "volume.3mf";
    ASSERT_TRUE(kernel::export3mf(file, {{"Box", boxShape(10, 20, 30), std::nullopt}}).has_value());
    const ZipView z = readZip(readAll(file));
    ASSERT_TRUE(z.ok);
    const auto objects = parseObjects(z.data.at("3D/3dmodel.model"));
    ASSERT_EQ(objects.size(), 1u);
    // 10 x 20 x 30 mm box: +6000 mm^3 only if every face is wound outward and coordinates are millimetres.
    EXPECT_NEAR(signedVolume(objects[0]), 6000.0, 1e-6);
}

TEST_F(Export3mf, CylinderVolumeWithinTessellationError) {
    auto cyl = kernel::makeCylinder({0, 0, 0}, {0, 0, 1}, 8.0, 25.0);
    ASSERT_TRUE(cyl.has_value());
    const fs::path file = dir_ / "cyl.3mf";
    ASSERT_TRUE(kernel::export3mf(file, {{"Cyl", cyl->shape, std::nullopt}}).has_value());
    const ZipView z = readZip(readAll(file));
    ASSERT_TRUE(z.ok);
    const auto objects = parseObjects(z.data.at("3D/3dmodel.model"));
    // Inscribed polygon: area factor < 1, bounded by the 0.1 mm linear deflection. Exact cylinder = pi r^2 h.
    const double exact = 3.14159265358979323846 * 8.0 * 8.0 * 25.0;
    const double v = signedVolume(objects[0]);
    EXPECT_GT(v, 0.0);
    EXPECT_NEAR(v, exact, exact * 0.01);
}

TEST_F(Export3mf, UnzipAcceptsPackageWhenToolAvailable) {
    if (std::system("command -v unzip >/dev/null 2>&1") != 0) GTEST_SKIP() << "unzip not installed";
    const fs::path file = dir_ / "unzip.3mf";
    ASSERT_TRUE(kernel::export3mf(file, {{"Box", boxShape(1, 2, 3), std::nullopt}}).has_value());
    const std::string cmd = "unzip -tq '" + file.string() + "' >/dev/null 2>&1";
    EXPECT_EQ(std::system(cmd.c_str()), 0);
}

TEST_F(Export3mf, RejectsInvalidInput) {
    const fs::path file = dir_ / "bad.3mf";
    auto noBodies = kernel::export3mf(file, {});
    ASSERT_FALSE(noBodies.has_value());
    EXPECT_EQ(noBodies.error().code, iw::ErrorCode::InvalidArgument);

    auto nullBody = kernel::export3mf(file, {{"Null", kernel::Shape{}, std::nullopt}});
    ASSERT_FALSE(nullBody.has_value());
    EXPECT_EQ(nullBody.error().code, iw::ErrorCode::InvalidArgument);

    auto emptyPath = kernel::export3mf({}, {{"Box", boxShape(1, 1, 1), std::nullopt}});
    ASSERT_FALSE(emptyPath.has_value());
    EXPECT_EQ(emptyPath.error().code, iw::ErrorCode::InvalidArgument);

    kernel::StlOptions bad;
    bad.linearDeflection = 0.0;
    auto badDeflection = kernel::export3mf(file, {{"Box", boxShape(1, 1, 1), std::nullopt}}, bad);
    ASSERT_FALSE(badDeflection.has_value());
    EXPECT_EQ(badDeflection.error().code, iw::ErrorCode::InvalidArgument);

    EXPECT_FALSE(fs::exists(file)) << "a rejected export must not create the file";
}

TEST_F(Export3mf, HonoursCancellation) {
    iw::CancelSource source;
    source.cancel();
    const fs::path file = dir_ / "cancelled.3mf";
    auto r = kernel::export3mf(file, {{"Box", boxShape(1, 1, 1), std::nullopt}}, {}, source.token());
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, iw::ErrorCode::Cancelled);
    EXPECT_FALSE(fs::exists(file));
}

TEST_F(Export3mf, ReportsIoErrorForMissingDirectory) {
    const fs::path file = dir_ / "no-such-dir" / "x.3mf";
    auto r = kernel::export3mf(file, {{"Box", boxShape(1, 1, 1), std::nullopt}});
    ASSERT_FALSE(r.has_value());
    EXPECT_EQ(r.error().code, iw::ErrorCode::IoError);
}
