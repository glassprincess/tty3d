// tty3d - a software 3D renderer that draws into a terminal character grid.
// Fork of the tri3d project (renamed to tty3d to not squat on the original name).
// Fork changes: Windows console support (VT input via ReadFile, Ctrl handler),
// input logging (--log) + --debug HUD, non-Latin layout key bindings.
//
// Pure CPU, no OpenGL / DirectX / Vulkan / GUI / third-party libraries.
// Only the C++ standard library plus the OS terminal API (termios on
// Linux/macOS, the console API on Windows).
//
// Build:  g++ -O3 main.cpp -o tty3d
//
// Pipeline (see README.md):
//   1. Math engine     Vec3 / Vec4 / Mat4 / Quat, Model-View-Projection
//   2. Rasterizer      near-plane clipping, barycentric fill, Z-buffer (1/w)
//   3. Shading         Lambert N.L -> character ramp " .:-=+*#%@" (+ TrueColor)
//   4. Terminal I/O    raw mode, SGR mouse, double buffer + diff, one write()/frame

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#ifndef ENABLE_VIRTUAL_TERMINAL_PROCESSING
#define ENABLE_VIRTUAL_TERMINAL_PROCESSING 0x0004
#endif
#ifndef DISABLE_NEWLINE_AUTO_RETURN
#define DISABLE_NEWLINE_AUTO_RETURN 0x0008
#endif
#ifndef ENABLE_VIRTUAL_TERMINAL_INPUT
#define ENABLE_VIRTUAL_TERMINAL_INPUT 0x0200
#endif
#else
#include <cerrno>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace {

const float PI = 3.14159265358979323846f;

// ============================================================================
// 1. Math engine
// ============================================================================

struct Vec3 {
    float x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
};
inline Vec3 operator+(Vec3 a, Vec3 b) { return Vec3(a.x + b.x, a.y + b.y, a.z + b.z); }
inline Vec3 operator-(Vec3 a, Vec3 b) { return Vec3(a.x - b.x, a.y - b.y, a.z - b.z); }
inline Vec3 operator-(Vec3 a) { return Vec3(-a.x, -a.y, -a.z); }
inline Vec3 operator*(Vec3 a, float s) { return Vec3(a.x * s, a.y * s, a.z * s); }
inline float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return Vec3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
inline Vec3 normalize(Vec3 a) {
    float l = length(a);
    return l > 1e-20f ? a * (1.0f / l) : Vec3();
}

struct Vec4 {
    float x, y, z, w;
    Vec4() : x(0), y(0), z(0), w(0) {}
    Vec4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_), w(w_) {}
};

// Row-major 4x4 matrix, column-vector convention: v' = M * v.
struct Mat4 {
    float m[4][4];
    Mat4() {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) m[i][j] = 0.0f;
    }
    static Mat4 identity() {
        Mat4 r;
        for (int i = 0; i < 4; ++i) r.m[i][i] = 1.0f;
        return r;
    }
};
inline Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            float s = 0.0f;
            for (int k = 0; k < 4; ++k) s += a.m[i][k] * b.m[k][j];
            r.m[i][j] = s;
        }
    return r;
}
inline Vec4 operator*(const Mat4& a, const Vec4& v) {
    return Vec4(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z + a.m[0][3] * v.w,
                a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z + a.m[1][3] * v.w,
                a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z + a.m[2][3] * v.w,
                a.m[3][0] * v.x + a.m[3][1] * v.y + a.m[3][2] * v.z + a.m[3][3] * v.w);
}
// Transform a direction (w = 0): only the upper-left 3x3 block is used.
inline Vec3 transformDir(const Mat4& a, Vec3 v) {
    return Vec3(a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
                a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
                a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z);
}
inline Mat4 translation(Vec3 t) {
    Mat4 r = Mat4::identity();
    r.m[0][3] = t.x;
    r.m[1][3] = t.y;
    r.m[2][3] = t.z;
    return r;
}
inline Mat4 scaling(float s) {
    Mat4 r = Mat4::identity();
    r.m[0][0] = r.m[1][1] = r.m[2][2] = s;
    return r;
}
// View matrix. Right-handed, camera looks down its local -Z.
inline Mat4 lookAt(Vec3 eye, Vec3 target, Vec3 up) {
    Vec3 f = normalize(target - eye);
    Vec3 s = normalize(cross(f, up));
    Vec3 u = cross(s, f);
    Mat4 r = Mat4::identity();
    r.m[0][0] = s.x; r.m[0][1] = s.y; r.m[0][2] = s.z; r.m[0][3] = -dot(s, eye);
    r.m[1][0] = u.x; r.m[1][1] = u.y; r.m[1][2] = u.z; r.m[1][3] = -dot(u, eye);
    r.m[2][0] = -f.x; r.m[2][1] = -f.y; r.m[2][2] = -f.z; r.m[2][3] = dot(f, eye);
    return r;
}
// Perspective projection (OpenGL clip space: -w <= x,y,z <= w). clip.w = -z_view.
inline Mat4 perspective(float fovy, float aspect, float zNear, float zFar) {
    float f = 1.0f / std::tan(fovy * 0.5f);
    Mat4 r;
    r.m[0][0] = f / aspect;
    r.m[1][1] = f;
    r.m[2][2] = (zFar + zNear) / (zNear - zFar);
    r.m[2][3] = 2.0f * zFar * zNear / (zNear - zFar);
    r.m[3][2] = -1.0f;
    return r;
}

struct Quat {
    float w, x, y, z;
    Quat() : w(1), x(0), y(0), z(0) {}
    Quat(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}
};
inline Quat operator*(const Quat& a, const Quat& b) {
    return Quat(a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
                a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w);
}
inline Quat normalize(const Quat& q) {
    float l = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
    if (l < 1e-12f) return Quat();
    float k = 1.0f / l;
    return Quat(q.w * k, q.x * k, q.y * k, q.z * k);
}
inline Quat quatAxisAngle(Vec3 axis, float angle) {
    axis = normalize(axis);
    float s = std::sin(angle * 0.5f);
    return Quat(std::cos(angle * 0.5f), axis.x * s, axis.y * s, axis.z * s);
}
// Rotation matrix of a unit quaternion.
inline Mat4 quatToMat4(const Quat& q) {
    float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    Mat4 r = Mat4::identity();
    r.m[0][0] = 1 - 2 * (yy + zz); r.m[0][1] = 2 * (xy - wz);     r.m[0][2] = 2 * (xz + wy);
    r.m[1][0] = 2 * (xy + wz);     r.m[1][1] = 1 - 2 * (xx + zz); r.m[1][2] = 2 * (yz - wx);
    r.m[2][0] = 2 * (xz - wy);     r.m[2][1] = 2 * (yz + wx);     r.m[2][2] = 1 - 2 * (xx + yy);
    return r;
}

// ============================================================================
// Meshes: builders and OBJ loader
// ============================================================================

struct Tri {
    int a, b, c;
    Tri() : a(0), b(0), c(0) {}
    Tri(int a_, int b_, int c_) : a(a_), b(b_), c(c_) {}
};

struct Mesh {
    std::string name;
    std::vector<Vec3> v;   // vertex positions (model space)
    std::vector<Tri> t;    // triangles (CCW = front face)
    std::vector<Vec3> vn;  // smooth vertex normals (area weighted), filled by finalize()
    Vec3 center;           // bounding-box center, moved to the origin by the model matrix
    float scale;           // uniform scale that fits the model into a unit sphere
    float tilt;            // initial tilt around X (radians), so the model looks good at start
    Mesh() : scale(1.0f), tilt(0.3f) {}

    bool finalize(std::string& err) {
        if (v.empty() || t.empty()) {
            err = "model has no triangles";
            return false;
        }
        Vec3 lo = v[0], hi = v[0];
        for (size_t i = 0; i < v.size(); ++i) {
            const Vec3& p = v[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
                err = "model contains non-finite vertex coordinates";
                return false;
            }
            lo = Vec3(std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z));
            hi = Vec3(std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z));
        }
        center = (lo + hi) * 0.5f;
        float r = 0.0f;
        for (size_t i = 0; i < v.size(); ++i) r = std::max(r, length(v[i] - center));
        if (r < 1e-12f) {
            err = "model is degenerate (all vertices coincide)";
            return false;
        }
        scale = 1.0f / r;
        vn.assign(v.size(), Vec3());
        for (size_t i = 0; i < t.size(); ++i) {
            const Tri& f = t[i];
            Vec3 n = cross(v[f.b] - v[f.a], v[f.c] - v[f.a]);  // length = 2 * area
            vn[f.a] = vn[f.a] + n;
            vn[f.b] = vn[f.b] + n;
            vn[f.c] = vn[f.c] + n;
        }
        for (size_t i = 0; i < vn.size(); ++i) vn[i] = normalize(vn[i]);
        return true;
    }
};

// Signed volume of a closed mesh; > 0 when triangles are wound CCW seen from outside.
float signedVolume(const Mesh& m) {
    double vol = 0.0;
    for (size_t i = 0; i < m.t.size(); ++i) {
        const Vec3& a = m.v[m.t[i].a];
        const Vec3& b = m.v[m.t[i].b];
        const Vec3& c = m.v[m.t[i].c];
        vol += dot(a, cross(b, c));
    }
    return (float)(vol / 6.0);
}
void flipAll(Mesh& m) {
    for (size_t i = 0; i < m.t.size(); ++i) std::swap(m.t[i].b, m.t[i].c);
}
// For convex meshes containing the origin: make every face point away from the origin.
void orientConvex(Mesh& m) {
    for (size_t i = 0; i < m.t.size(); ++i) {
        Tri& f = m.t[i];
        Vec3 n = cross(m.v[f.b] - m.v[f.a], m.v[f.c] - m.v[f.a]);
        Vec3 c = (m.v[f.a] + m.v[f.b] + m.v[f.c]) * (1.0f / 3.0f);
        if (dot(n, c) < 0.0f) std::swap(f.b, f.c);
    }
}

Mesh makeTorus() {
    const int N = 48, M = 24;
    const float R = 1.0f, r = 0.42f;
    Mesh m;
    m.name = "donut";
    m.tilt = 1.0f;
    for (int i = 0; i < N; ++i) {
        float a = 2.0f * PI * i / N;
        for (int j = 0; j < M; ++j) {
            float b = 2.0f * PI * j / M;
            float rad = R + r * std::cos(b);
            m.v.push_back(Vec3(rad * std::cos(a), r * std::sin(b), rad * std::sin(a)));
        }
    }
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j) {
            int i1 = (i + 1) % N, j1 = (j + 1) % M;
            int v00 = i * M + j, v01 = i * M + j1, v10 = i1 * M + j, v11 = i1 * M + j1;
            m.t.push_back(Tri(v00, v01, v11));
            m.t.push_back(Tri(v00, v11, v10));
        }
    if (signedVolume(m) < 0.0f) flipAll(m);
    return m;
}

Mesh makeCube() {
    Mesh m;
    m.name = "cube";
    m.tilt = 0.5f;
    for (int axis = 0; axis < 3; ++axis)
        for (int s = -1; s <= 1; s += 2) {
            Vec3 e[3] = {Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1)};
            Vec3 n = e[axis] * (float)s, u = e[(axis + 1) % 3], w = e[(axis + 2) % 3];
            int base = (int)m.v.size();  // 4 unique vertices per face: flat corners even when smooth
            m.v.push_back(n - u - w);
            m.v.push_back(n + u - w);
            m.v.push_back(n + u + w);
            m.v.push_back(n - u + w);
            m.t.push_back(Tri(base, base + 1, base + 2));
            m.t.push_back(Tri(base, base + 2, base + 3));
        }
    orientConvex(m);
    return m;
}

Mesh makeSphere() {
    const int rings = 20, seg = 32;
    Mesh m;
    m.name = "sphere";
    m.tilt = 0.4f;
    m.v.push_back(Vec3(0, 1, 0));  // north pole: 0
    for (int k = 1; k < rings; ++k) {
        float phi = PI * k / rings;
        for (int j = 0; j < seg; ++j) {
            float th = 2.0f * PI * j / seg;
            m.v.push_back(Vec3(std::sin(phi) * std::cos(th), std::cos(phi), std::sin(phi) * std::sin(th)));
        }
    }
    int south = (int)m.v.size();
    m.v.push_back(Vec3(0, -1, 0));
    for (int j = 0; j < seg; ++j) {
        int j1 = (j + 1) % seg;
        m.t.push_back(Tri(0, 1 + j, 1 + j1));
    }
    for (int k = 0; k < rings - 2; ++k)
        for (int j = 0; j < seg; ++j) {
            int j1 = (j + 1) % seg;
            int a = 1 + k * seg + j, b = 1 + k * seg + j1;
            int c = 1 + (k + 1) * seg + j, d = 1 + (k + 1) * seg + j1;
            m.t.push_back(Tri(a, c, d));
            m.t.push_back(Tri(a, d, b));
        }
    int last = 1 + (rings - 2) * seg;
    for (int j = 0; j < seg; ++j) {
        int j1 = (j + 1) % seg;
        m.t.push_back(Tri(south, last + j1, last + j));
    }
    orientConvex(m);
    return m;
}

Mesh makeCone() {
    const int seg = 40;
    Mesh m;
    m.name = "cone";
    m.tilt = 0.5f;
    const float radius = 0.8f;
    int apex = 0;
    m.v.push_back(Vec3(0, 1, 0));
    int ringSide = (int)m.v.size();  // ring used by the side surface
    for (int j = 0; j < seg; ++j) {
        float th = 2.0f * PI * j / seg;
        m.v.push_back(Vec3(radius * std::cos(th), -1, radius * std::sin(th)));
    }
    int center = (int)m.v.size();  // base cap has its own vertices (sharp edge when smooth-shaded)
    m.v.push_back(Vec3(0, -1, 0));
    int ringBase = (int)m.v.size();
    for (int j = 0; j < seg; ++j) {
        float th = 2.0f * PI * j / seg;
        m.v.push_back(Vec3(radius * std::cos(th), -1, radius * std::sin(th)));
    }
    for (int j = 0; j < seg; ++j) {
        int j1 = (j + 1) % seg;
        m.t.push_back(Tri(apex, ringSide + j, ringSide + j1));
        m.t.push_back(Tri(center, ringBase + j, ringBase + j1));
    }
    orientConvex(m);
    return m;
}

// Trefoil torus knot (p = 2, q = 3) swept with a circular tube.
Mesh makeKnot() {
    const int N = 240, M = 16;
    const float tube = 0.32f;
    const int p = 2, q = 3;
    Mesh m;
    m.name = "knot";
    m.tilt = 0.5f;
    for (int i = 0; i < N; ++i) {
        float t = 2.0f * PI * i / N;
        float rr = std::cos(q * t) + 2.0f;
        Vec3 c(rr * std::cos(p * t), rr * std::sin(p * t), -std::sin(q * t));
        float dr = -q * std::sin(q * t);
        Vec3 d(dr * std::cos(p * t) - p * rr * std::sin(p * t),
               dr * std::sin(p * t) + p * rr * std::cos(p * t),
               -q * std::cos(q * t));
        Vec3 T = normalize(d);
        Vec3 U(std::cos(p * t), std::sin(p * t), 0.0f);  // smooth, closed reference direction
        Vec3 Nn = normalize(U - T * dot(U, T));
        Vec3 B = cross(T, Nn);
        for (int j = 0; j < M; ++j) {
            float s = 2.0f * PI * j / M;
            m.v.push_back(c + (Nn * std::cos(s) + B * std::sin(s)) * tube);
        }
    }
    for (int i = 0; i < N; ++i)
        for (int j = 0; j < M; ++j) {
            int i1 = (i + 1) % N, j1 = (j + 1) % M;
            int v00 = i * M + j, v01 = i * M + j1, v10 = i1 * M + j, v11 = i1 * M + j1;
            m.t.push_back(Tri(v00, v01, v11));
            m.t.push_back(Tri(v00, v11, v10));
        }
    if (signedVolume(m) < 0.0f) flipAll(m);
    return m;
}

// Wavefront OBJ: v, f (any polygon, triangulated as a fan), negative indices,
// v/vt/vn tokens. Everything else is ignored. Invalid faces are skipped and counted.
bool parseObj(const std::string& text, Mesh& mesh, std::string& err, int* skippedFaces) {
    std::vector<int> faceIdx;    // flattened, 0-based (may still be out of range)
    std::vector<int> faceSizes;
    long badFaces = 0;
    long badVerts = 0;
    std::string line;
    size_t pos = 0;
    while (pos < text.size()) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string::npos) eol = text.size();
        line.assign(text, pos, eol - pos);
        pos = eol + 1;
        const char* s = line.c_str();
        while (*s == ' ' || *s == '\t') ++s;
        if (s[0] == 'v' && (s[1] == ' ' || s[1] == '\t')) {
            ++s;
            float c[3];
            bool ok = true;
            for (int k = 0; k < 3 && ok; ++k) {
                char* e = NULL;
                c[k] = std::strtof(s, &e);
                if (e == s) ok = false;
                s = e;
            }
            if (ok) mesh.v.push_back(Vec3(c[0], c[1], c[2]));
            else ++badVerts;
        } else if (s[0] == 'f' && (s[1] == ' ' || s[1] == '\t')) {
            ++s;
            long nv = (long)mesh.v.size();
            std::vector<int> idx;
            bool bad = false;
            for (;;) {
                while (*s == ' ' || *s == '\t' || *s == '\r') ++s;
                if (*s == '\0') break;
                char* e = NULL;
                long i = std::strtol(s, &e, 10);
                if (e == s || i == 0) { bad = true; break; }
                s = e;
                while (*s != '\0' && *s != ' ' && *s != '\t' && *s != '\r') ++s;  // skip /vt/vn
                long k = i > 0 ? i - 1 : nv + i;  // negative = relative to the current end
                if (k < 0 || k > 2000000000L) { bad = true; break; }
                idx.push_back((int)k);
            }
            if (bad || idx.size() < 3) {
                ++badFaces;
            } else {
                faceSizes.push_back((int)idx.size());
                faceIdx.insert(faceIdx.end(), idx.begin(), idx.end());
            }
        }
    }
    int nv = (int)mesh.v.size();
    size_t off = 0;
    for (size_t f = 0; f < faceSizes.size(); ++f) {
        int n = faceSizes[f];
        bool ok = true;
        for (int k = 0; k < n; ++k)
            if (faceIdx[off + k] >= nv) ok = false;
        if (ok) {
            for (int k = 1; k + 1 < n; ++k)
                mesh.t.push_back(Tri(faceIdx[off], faceIdx[off + k], faceIdx[off + k + 1]));
        } else {
            ++badFaces;
        }
        off += n;
    }
    if (skippedFaces) *skippedFaces = (int)badFaces;
    (void)badVerts;
    if (mesh.v.empty() || mesh.t.empty()) {
        err = "no usable geometry (need 'v' and 'f' lines)";
        return false;
    }
    return true;
}

bool loadObjFile(const char* path, Mesh& mesh, std::string& err, int* skipped) {
    FILE* fp = std::fopen(path, "rb");
    if (!fp) {
        err = std::string("cannot open file: ") + path;
        return false;
    }
    std::string text;
    char buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, fp)) > 0) text.append(buf, n);
    std::fclose(fp);
    if (!parseObj(text, mesh, err, skipped)) return false;
    std::string base = path;
    size_t slash = base.find_last_of("/\\");
    if (slash != std::string::npos) base = base.substr(slash + 1);
    mesh.name = base;
    mesh.tilt = 0.2f;
    return mesh.finalize(err);
}

// ============================================================================
// 2. Rasterizer: clipping, barycentric fill, Z-buffer
// ============================================================================

struct Framebuffer {
    int w, h;
    std::vector<float> depth;  // 1/w of the nearest surface; 0 = nothing drawn (infinitely far)
    std::vector<float> lum;    // light intensity 0..1
    Framebuffer() : w(0), h(0) {}
    void resize(int W, int H) {
        w = W;
        h = H;
        depth.assign((size_t)W * H, 0.0f);
        lum.assign((size_t)W * H, 0.0f);
    }
    void clear() { std::fill(depth.begin(), depth.end(), 0.0f); }
};

struct Settings {
    bool smooth;  // false: flat (per-face normal), true: interpolated vertex normals
    bool cull;    // back-face culling; when off, back faces are lit two-sided
    Vec3 light;   // unit vector towards the light, in view space
    float ambient;
    Settings() : smooth(false), cull(true), light(normalize(Vec3(-0.45f, 0.55f, 0.70f))), ambient(0.10f) {}
};

struct FrameStats {
    int total, drawn;
    FrameStats() : total(0), drawn(0) {}
};

struct ClipVert {
    Vec4 c;  // clip-space position
    Vec3 n;  // view-space normal (only used for smooth shading)
};
struct ScrVert {
    float x, y, iw;  // screen position and 1/w
    Vec3 nw;         // normal / w, for perspective-correct interpolation
};

inline float lambert(const Settings& st, Vec3 n) {
    float d = dot(n, st.light);
    if (d < 0.0f) d = 0.0f;
    return st.ambient + (1.0f - st.ambient) * d;
}

// Fill one screen-space triangle. Pixel centres are at (x + 0.5, y + 0.5).
void rasterTriangle(Framebuffer& fb, const Settings& st, const ScrVert& a, const ScrVert& b,
                    const ScrVert& c, bool flip, float flatLum) {
    float area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    if (!(std::fabs(area) > 1e-12f)) return;  // degenerate (also rejects NaN)
    float minX = std::min(a.x, std::min(b.x, c.x)), maxX = std::max(a.x, std::max(b.x, c.x));
    float minY = std::min(a.y, std::min(b.y, c.y)), maxY = std::max(a.y, std::max(b.y, c.y));
    // clamp as floats first: huge coordinates must not overflow the int conversion
    float fx0 = std::min(std::max(0.0f, std::floor(minX - 0.5f)), (float)fb.w);
    float fx1 = std::max(-1.0f, std::min(std::ceil(maxX - 0.5f), (float)(fb.w - 1)));
    float fy0 = std::min(std::max(0.0f, std::floor(minY - 0.5f)), (float)fb.h);
    float fy1 = std::max(-1.0f, std::min(std::ceil(maxY - 0.5f), (float)(fb.h - 1)));
    int x0 = (int)fx0, x1 = (int)fx1, y0 = (int)fy0, y1 = (int)fy1;
    if (x0 > x1 || y0 > y1) return;
    const float invArea = 1.0f / area;
    const float eps = -1e-5f;  // tolerance so shared edges leave no gaps
    for (int y = y0; y <= y1; ++y) {
        float py = y + 0.5f;
        for (int x = x0; x <= x1; ++x) {
            float px = x + 0.5f;
            // barycentric coordinates via edge functions
            float w0 = ((b.x - px) * (c.y - py) - (c.x - px) * (b.y - py)) * invArea;
            float w1 = ((c.x - px) * (a.y - py) - (a.x - px) * (c.y - py)) * invArea;
            float w2 = 1.0f - w0 - w1;
            if (w0 < eps || w1 < eps || w2 < eps) continue;
            float iw = w0 * a.iw + w1 * b.iw + w2 * c.iw;
            size_t idx = (size_t)y * fb.w + x;
            if (!(iw > fb.depth[idx])) continue;  // Z-test: 1/w is larger for nearer points
            fb.depth[idx] = iw;
            if (st.smooth) {
                Vec3 n = (a.nw * w0 + b.nw * w1 + c.nw * w2) * (1.0f / iw);
                n = normalize(n);
                if (flip) n = -n;
                fb.lum[idx] = lambert(st, n);
            } else {
                fb.lum[idx] = flatLum;
            }
        }
    }
}

class Renderer {
public:
    void render(const Mesh& mesh, const Mat4& model, const Mat4& view, const Mat4& proj,
                const Settings& st, Framebuffer& fb, FrameStats& stats) {
        const size_t nv = mesh.v.size();
        pv_.resize(nv);
        pc_.resize(nv);
        code_.resize(nv);
        if (st.smooth) pn_.resize(nv);
        Mat4 mv = view * model;
        for (size_t i = 0; i < nv; ++i) {
            Vec4 vv = mv * Vec4(mesh.v[i].x, mesh.v[i].y, mesh.v[i].z, 1.0f);
            pv_[i] = Vec3(vv.x, vv.y, vv.z);
            Vec4 cc = proj * vv;
            pc_[i] = cc;
            int k = 0;
            if (cc.x < -cc.w) k |= 1;
            if (cc.x > cc.w) k |= 2;
            if (cc.y < -cc.w) k |= 4;
            if (cc.y > cc.w) k |= 8;
            if (cc.z < -cc.w) k |= 16;  // behind the near plane
            if (cc.z > cc.w) k |= 32;
            code_[i] = k;
            if (st.smooth) pn_[i] = normalize(transformDir(mv, mesh.vn[i]));
        }
        stats.total = (int)mesh.t.size();
        stats.drawn = 0;
        for (size_t ti = 0; ti < mesh.t.size(); ++ti) {
            const Tri& f = mesh.t[ti];
            int ia = f.a, ib = f.b, ic = f.c;
            if ((code_[ia] & code_[ib] & code_[ic]) != 0) continue;  // fully outside one plane
            // Face normal in view space; the camera sits at the origin.
            Vec3 n = cross(pv_[ib] - pv_[ia], pv_[ic] - pv_[ia]);
            float n2 = dot(n, n);
            if (!(n2 > 1e-24f)) continue;
            bool front = dot(n, pv_[ia]) < 0.0f;
            if (!front && st.cull) continue;
            float flatLum = 0.0f;
            if (!st.smooth) {
                Vec3 nn = n * (1.0f / std::sqrt(n2));
                if (!front) nn = -nn;
                flatLum = lambert(st, nn);
            }
            ClipVert poly[4];
            int cnt = 3;
            poly[0].c = pc_[ia]; poly[1].c = pc_[ib]; poly[2].c = pc_[ic];
            if (st.smooth) { poly[0].n = pn_[ia]; poly[1].n = pn_[ib]; poly[2].n = pn_[ic]; }
            if ((code_[ia] | code_[ib] | code_[ic]) & 16) cnt = clipNear(poly);
            if (cnt < 3) continue;
            ScrVert sv[4];
            for (int k = 0; k < cnt; ++k) {
                float iw = 1.0f / poly[k].c.w;  // w >= near > 0 after clipping
                float ndcX = poly[k].c.x * iw, ndcY = poly[k].c.y * iw;
                sv[k].x = (ndcX * 0.5f + 0.5f) * fb.w;
                sv[k].y = (0.5f - ndcY * 0.5f) * fb.h;
                sv[k].iw = iw;
                sv[k].nw = poly[k].n * iw;
            }
            ++stats.drawn;
            for (int k = 1; k + 1 < cnt; ++k)
                rasterTriangle(fb, st, sv[0], sv[k], sv[k + 1], !front, flatLum);
        }
    }

private:
    // Sutherland-Hodgman against the near plane z + w >= 0. Input: 3 vertices, output: 0..4.
    static int clipNear(ClipVert* poly) {
        ClipVert out[4];
        int m = 0;
        for (int i = 0; i < 3; ++i) {
            const ClipVert& cur = poly[i];
            const ClipVert& nxt = poly[(i + 1) % 3];
            float dc = cur.c.z + cur.c.w, dn = nxt.c.z + nxt.c.w;
            if (dc >= 0.0f) out[m++] = cur;
            if ((dc >= 0.0f) != (dn >= 0.0f)) {
                float t = dc / (dc - dn);
                ClipVert v;
                v.c = Vec4(cur.c.x + (nxt.c.x - cur.c.x) * t, cur.c.y + (nxt.c.y - cur.c.y) * t,
                           cur.c.z + (nxt.c.z - cur.c.z) * t, cur.c.w + (nxt.c.w - cur.c.w) * t);
                v.n = cur.n + (nxt.n - cur.n) * t;
                out[m++] = v;
            }
        }
        for (int i = 0; i < m; ++i) poly[i] = out[i];
        return m;
    }
    std::vector<Vec3> pv_, pn_;
    std::vector<Vec4> pc_;
    std::vector<int> code_;
};

// ============================================================================
// 3. Shading: luminance -> character (+ optional TrueColor)
// ============================================================================

const char* const RAMP = " .:-=+*#%@";
const int RAMP_N = 10;

struct Palette {
    const char* name;
    int r, g, b;
};
const Palette PALETTES[] = {
    {"off", 0, 0, 0}, {"amber", 255, 176, 0}, {"cyan", 0, 210, 255},
    {"green", 70, 255, 110}, {"magenta", 255, 90, 220},
};
const int PALETTE_N = (int)(sizeof(PALETTES) / sizeof(PALETTES[0]));

inline uint32_t packColor(int r, int g, int b) {
    return 0x01000000u | ((uint32_t)r << 16) | ((uint32_t)g << 8) | (uint32_t)b;
}

// Any covered pixel gets at least ramp index 1, so silhouettes never vanish into the background.
inline int rampIndex(float lum) {
    if (!(lum > 0.0f)) lum = 0.0f;
    if (lum > 1.0f) lum = 1.0f;
    int idx = (int)(lum * (RAMP_N - 1) + 0.5f);
    return std::max(1, std::min(RAMP_N - 1, idx));
}

// ============================================================================
// 4. Screen: character grid, double buffering, diff output
// ============================================================================

struct Cell {
    char ch;
    uint32_t color;  // 0 = terminal default foreground
    Cell() : ch(' '), color(0) {}
    bool operator==(const Cell& o) const { return ch == o.ch && color == o.color; }
    bool operator!=(const Cell& o) const { return !(*this == o); }
};

struct Screen {
    int w, h;
    std::vector<Cell> cur, prev;
    bool full;  // next frame must be sent completely (start, resize)
    Screen() : w(0), h(0), full(true) {}
    void resize(int W, int H) {
        w = W;
        h = H;
        cur.assign((size_t)W * H, Cell());
        prev.assign((size_t)W * H, Cell());
        full = true;
    }
    void commit() {  // the finished frame becomes "previous"
        cur.swap(prev);
        full = false;
    }
};

void resolve(const Framebuffer& fb, Screen& scr, int palette) {
    const Palette& pal = PALETTES[palette];
    size_t n = (size_t)fb.w * fb.h;
    for (size_t i = 0; i < n; ++i) {
        Cell& c = scr.cur[i];
        if (fb.depth[i] > 0.0f) {
            float l = fb.lum[i];
            c.ch = RAMP[rampIndex(l)];
            if (palette == 0) {
                c.color = 0;
            } else {
                if (!(l > 0.0f)) l = 0.0f;
                if (l > 1.0f) l = 1.0f;
                // brightness quantised to 32 steps: far fewer colour changes to send
                int q = (int)std::lround((0.2f + 0.8f * l) * 31.0f);
                float f = q / 31.0f;
                c.color = packColor((int)std::lround(pal.r * f), (int)std::lround(pal.g * f),
                                    (int)std::lround(pal.b * f));
            }
        } else {
            c = Cell();
        }
    }
}

void drawText(Screen& scr, int x, int y, const std::string& text, uint32_t color) {
    if (y < 0 || y >= scr.h) return;
    for (size_t i = 0; i < text.size(); ++i) {
        int xx = x + (int)i;
        if (xx < 0 || xx >= scr.w) continue;
        Cell& c = scr.cur[(size_t)y * scr.w + xx];
        c.ch = text[i];
        c.color = color;
    }
}

void appendInt(std::string& s, int v) {
    char buf[16];
    int n = std::snprintf(buf, sizeof buf, "%d", v);
    s.append(buf, n);
}

// Build the escape-sequence stream for this frame: one string, sent with one write().
// Only cells that changed since the previous frame are emitted.
void buildFrame(const Screen& scr, std::string& out) {
    out.clear();
    uint32_t curColor = 0;         // invariant: the foreground is the default at every frame boundary
    int curRow = -1, curCol = -1;  // where the terminal cursor is, when known
    if (scr.full) {
        out += "\x1b[2J\x1b[H";
        curRow = 0;
        curCol = 0;
    }
    for (int y = 0; y < scr.h; ++y) {
        for (int x = 0; x < scr.w; ++x) {
            size_t i = (size_t)y * scr.w + x;
            const Cell& c = scr.cur[i];
            if (!scr.full && c == scr.prev[i]) continue;
            if (curRow != y || curCol != x) {
                out += "\x1b[";
                appendInt(out, y + 1);
                out += ';';
                appendInt(out, x + 1);
                out += 'H';
            }
            if (c.color != curColor) {
                if (c.color == 0) {
                    out += "\x1b[39m";
                } else {
                    out += "\x1b[38;2;";
                    appendInt(out, (int)((c.color >> 16) & 0xFF));
                    out += ';';
                    appendInt(out, (int)((c.color >> 8) & 0xFF));
                    out += ';';
                    appendInt(out, (int)(c.color & 0xFF));
                    out += 'm';
                }
                curColor = c.color;
            }
            out += c.ch;
            curRow = y;
            curCol = x + 1;
        }
    }
    if (curColor != 0) out += "\x1b[39m";
}

// ============================================================================
// Input: escape sequence parser (keys + SGR mouse)
// ============================================================================

enum { K_UP = 1001, K_DOWN, K_RIGHT, K_LEFT };

struct Event {
    enum Type { Key, MouseDown, MouseUp, MouseMove, Wheel } type;
    int key;   // Key: character code or K_*; Wheel: +1 up, -1 down
    int btn;   // mouse button 0 left, 1 middle, 2 right, 3 none
    int x, y;  // 1-based cell coordinates
    Event() : type(Key), key(0), btn(0), x(0), y(0) {}
};

// Decode one UTF-8 sequence starting at buf[i]. Returns the Unicode code point
// (0..0x10FFFF) and sets len to the sequence length, or -1 when more bytes are
// needed / the sequence is invalid. Needed so non-Latin layouts (e.g. Russian
// Russian-letter keys) arrive as single Key events instead of stray byte-keys.
inline int decodeUtf8(const std::string& buf, size_t i, size_t& len) {
    unsigned char c0 = (unsigned char)buf[i];
    if (c0 < 0x80) { len = 1; return c0; }
    size_t need = 0;
    int cp = 0;
    if ((c0 & 0xE0) == 0xC0) { need = 2; cp = c0 & 0x1F; }
    else if ((c0 & 0xF0) == 0xE0) { need = 3; cp = c0 & 0x0F; }
    else if ((c0 & 0xF8) == 0xF0) { need = 4; cp = c0 & 0x07; }
    else { len = 1; return -1; }
    if (buf.size() - i < need) { len = 0; return -1; }  // split between reads: wait
    for (size_t k = 1; k < need; ++k) {
        unsigned char cc = (unsigned char)buf[i + k];
        if ((cc & 0xC0) != 0x80) { len = 1; return -1; }
        cp = (cp << 6) | (cc & 0x3F);
    }
    len = need;
    return cp;
}

class InputParser {
public:
    InputParser() : lastFeed_(0.0) {}

    void feed(const char* data, int n, double now, std::vector<Event>& out) {
        buf_.append(data, n);
        lastFeed_ = now;
        parse(out);
    }
    // A lone ESC (no continuation within 80 ms) is the Escape key; other leftovers are dropped.
    void tick(double now, std::vector<Event>& out) {
        if (buf_.empty() || now - lastFeed_ < 0.08) return;
        if (buf_ == "\x1b") {
            Event e;
            e.type = Event::Key;
            e.key = 27;
            out.push_back(e);
        }
        buf_.clear();
    }

private:
    void parse(std::vector<Event>& out) {
        size_t i = 0;
        while (i < buf_.size()) {
            unsigned char c = (unsigned char)buf_[i];
            if (c != 0x1b) {
                if (c < 0x80) {
                    Event e;
                    e.type = Event::Key;
                    e.key = c;
                    out.push_back(e);
                    ++i;
                    continue;
                }
                // Non-ASCII: decode UTF-8 so other layouts yield one Key event.
                size_t len = 0;
                int cp = decodeUtf8(buf_, i, len);
                if (len == 0) break;  // sequence split between reads: wait for more
                if (cp >= 0) {
                    Event e;
                    e.type = Event::Key;
                    e.key = cp;
                    out.push_back(e);
                }
                i += len ? len : 1;
                continue;
            }
            if (i + 1 >= buf_.size()) break;  // incomplete
            unsigned char c1 = (unsigned char)buf_[i + 1];
            if (c1 == '[') {
                size_t j = i + 2;
                bool malformed = false;
                while (j < buf_.size()) {
                    unsigned char d = (unsigned char)buf_[j];
                    if (d >= 0x40 && d <= 0x7e) break;   // final byte
                    if (d < 0x20 || d > 0x3f) { malformed = true; break; }  // not parameter/intermediate
                    ++j;
                }
                if (malformed) { i = j; continue; }
                if (j >= buf_.size()) {
                    if (buf_.size() - i > 64) { i = buf_.size(); }
                    break;  // incomplete
                }
                char fin = buf_[j];
                std::string params = buf_.substr(i + 2, j - i - 2);
                handleCsi(params, fin, out);
                i = j + 1;
            } else if (c1 == 'O') {
                if (i + 2 >= buf_.size()) break;
                arrow(buf_[i + 2], out);
                i += 3;
            } else {
                ++i;  // ESC + key (Alt+key): drop the ESC, the key is handled next
            }
        }
        buf_.erase(0, i);
    }
    static void arrow(char fin, std::vector<Event>& out) {
        int k = 0;
        if (fin == 'A') k = K_UP;
        else if (fin == 'B') k = K_DOWN;
        else if (fin == 'C') k = K_RIGHT;
        else if (fin == 'D') k = K_LEFT;
        if (!k) return;
        Event e;
        e.type = Event::Key;
        e.key = k;
        out.push_back(e);
    }
    static void handleCsi(const std::string& params, char fin, std::vector<Event>& out) {
        if (!params.empty() && params[0] == '<' && (fin == 'M' || fin == 'm')) {
            int b = 0, x = 0, y = 0;
            if (std::sscanf(params.c_str() + 1, "%d;%d;%d", &b, &x, &y) != 3) return;
            Event e;
            e.x = x;
            e.y = y;
            e.btn = b & 3;
            if (b & 64) {  // wheel
                if (fin != 'M' || e.btn > 1) return;
                e.type = Event::Wheel;
                e.key = e.btn == 0 ? +1 : -1;
            } else if (b & 32) {
                e.type = Event::MouseMove;
            } else {
                e.type = fin == 'M' ? Event::MouseDown : Event::MouseUp;
            }
            out.push_back(e);
            return;
        }
        arrow(fin, out);
    }
    std::string buf_;
    double lastFeed_;
};

// ============================================================================
// Logging (--log FILE) + debug HUD (--debug)
// ============================================================================
// The rendered frame owns stdout/the console, so diagnostics must go to a file,
// never to stdout. Every raw input chunk is logged escaped, then each parsed
// event plus the action it triggered. When binds "don't work", the log shows
// whether the terminal sent anything at all (mouse tracking off? wrong
// encoding?) and what the parser made of it.

struct Logger {
    FILE* f;
    Logger() : f(NULL) {}
    bool open(const char* path, std::string& err) {
        f = std::fopen(path, "a");
        if (!f) { err = std::string("cannot open log file: ") + path; return false; }
        return true;
    }
    void close() { if (f) { std::fclose(f); f = NULL; } }
    void write(const std::string& s) {
        if (f) { std::fwrite(s.data(), 1, s.size(), f); std::fflush(f); }
    }
};

inline void appendEscaped(std::string& out, const char* data, int n) {
    char tmp[8];
    for (int i = 0; i < n; ++i) {
        unsigned char c = (unsigned char)data[i];
        if (c == 0x1b) out += "<ESC>";
        else if (c == '\r') out += "<CR>";
        else if (c == '\n') out += "<LF>";
        else if (c < 0x20 || c == 0x7f) {
            std::snprintf(tmp, sizeof tmp, "<%02X>", c);
            out += tmp;
        } else if (c < 0x80) out += (char)c;
        else {
            std::snprintf(tmp, sizeof tmp, "<%02X>", c);
            out += tmp;
        }
    }
}

inline std::string keyName(int k) {
    if (k == 27) return "esc";
    if (k == 3) return "ctrl-c";
    if (k == 32) return "space";
    if (k == K_UP) return "up";
    if (k == K_DOWN) return "down";
    if (k == K_RIGHT) return "right";
    if (k == K_LEFT) return "left";
    if (k >= 32 && k < 127) return std::string(1, (char)k);
    char tmp[16];
    std::snprintf(tmp, sizeof tmp, "U+%04X", k);
    return tmp;
}

inline std::string eventToString(const Event& e) {
    char tmp[64];
    switch (e.type) {
    case Event::Key: return "key=" + keyName(e.key);
    case Event::MouseDown:
        std::snprintf(tmp, sizeof tmp, "mouse-down btn=%d x=%d y=%d", e.btn, e.x, e.y);
        return tmp;
    case Event::MouseUp:
        std::snprintf(tmp, sizeof tmp, "mouse-up btn=%d x=%d y=%d", e.btn, e.x, e.y);
        return tmp;
    case Event::MouseMove:
        std::snprintf(tmp, sizeof tmp, "mouse-move btn=%d x=%d y=%d", e.btn, e.x, e.y);
        return tmp;
    case Event::Wheel:
        std::snprintf(tmp, sizeof tmp, "wheel %s", e.key > 0 ? "up" : "down");
        return tmp;
    }
    return "?";
}

// ============================================================================
// Application state
// ============================================================================

struct App {
    std::vector<Mesh> meshes;
    size_t cur;
    Quat orient;
    float zoom;
    bool spin, dragging, hud, lightOrbit, quit;
    int lastX, lastY;
    int palette;
    float lightAngle;
    float cellAspect;  // cell height / cell width
    Settings st;
    std::string lastEvent;   // last parsed input, for --debug HUD and --log
    std::string lastAction;  // what the last input did ("ignored" when nothing)
    App()
        : cur(0), zoom(1.0f), spin(true), dragging(false), hud(true), lightOrbit(false), quit(false),
          lastX(0), lastY(0), palette(0), lightAngle(0.0f), cellAspect(2.0f) {}

    void resetView() {
        orient = quatAxisAngle(Vec3(1, 0, 0), meshes[cur].tilt);
        zoom = 1.0f;
    }
    void rotateView(float yaw, float pitch) {
        Quat dq = quatAxisAngle(Vec3(0, 1, 0), yaw) * quatAxisAngle(Vec3(1, 0, 0), pitch);
        orient = normalize(dq * orient);  // rotation about the fixed view axes
    }
    void zoomBy(float f) { zoom = std::max(0.2f, std::min(6.0f, zoom * f)); }

    void handle(const Event& e) {
        lastEvent = eventToString(e);
        char tmp[96];
        switch (e.type) {
        case Event::Key: handleKey(e.key); break;
        case Event::MouseDown:
            if (e.btn == 0) {
                dragging = true; lastX = e.x; lastY = e.y;
                lastAction = "drag start";
            } else {
                std::snprintf(tmp, sizeof tmp, "ignored mouse button %d", e.btn);
                lastAction = tmp;
            }
            break;
        case Event::MouseUp:
            if (e.btn == 0) { dragging = false; lastAction = "drag end"; }
            else lastAction = "ignored";
            break;
        case Event::MouseMove:
            if (e.btn == 0 && dragging) {
                // a cell is cellAspect times taller than wide: equalise the physical distance
                const float k = 0.05f;
                rotateView((e.x - lastX) * k, (e.y - lastY) * k * cellAspect);
                std::snprintf(tmp, sizeof tmp, "rotate dx=%d dy=%d", e.x - lastX, e.y - lastY);
                lastAction = tmp;
                lastX = e.x;
                lastY = e.y;
            } else if (e.btn == 3) {
                dragging = false;
                lastAction = "drag cancelled";
            } else lastAction = "ignored (not dragging)";
            break;
        case Event::Wheel:
            zoomBy(e.key > 0 ? 0.9f : 1.1f);
            std::snprintf(tmp, sizeof tmp, "zoom=%.2f", zoom);
            lastAction = tmp;
            break;
        }
    }
    void handleKey(int k) {
        char tmp[64];
        switch (k) {
        // Russian layout equivalents: U+0419/0439=q, U+0421/0441=c, U+042B/044B=s,
        // U+0422/0442=n, U+0420/0440=h, U+0414/0434=l, U+041A/043A=k.
        case 'q': case 'Q': case 0x439: case 0x419: case 27: case 3: quit = true; lastAction = "quit"; break;
        case ' ': spin = !spin; lastAction = spin ? "spin on" : "spin off"; break;
        case 'c': case 'C': case 0x441: case 0x421:
            palette = (palette + 1) % PALETTE_N;
            std::snprintf(tmp, sizeof tmp, "color=%s", PALETTES[palette].name);
            lastAction = tmp;
            break;
        case 's': case 'S': case 0x44B: case 0x42B:
            st.smooth = !st.smooth; lastAction = st.smooth ? "smooth on" : "flat on"; break;
        case 'n': case 'N': case 0x442: case 0x422:
            cur = (cur + 1) % meshes.size(); resetView();
            lastAction = std::string("model=") + meshes[cur].name;
            break;
        case 'h': case 'H': case 0x440: case 0x420: hud = !hud; lastAction = hud ? "hud on" : "hud off"; break;
        case 'l': case 'L': case 0x434: case 0x414:
            lightOrbit = !lightOrbit; lastAction = lightOrbit ? "light orbit on" : "light orbit off"; break;
        case 'r': case 'R': case 0x43A: case 0x41A: resetView(); lastAction = "view reset"; break;
        case '+': case '=': zoomBy(0.9f);
            std::snprintf(tmp, sizeof tmp, "zoom=%.2f", zoom); lastAction = tmp; break;
        case '-': case '_': zoomBy(1.1f);
            std::snprintf(tmp, sizeof tmp, "zoom=%.2f", zoom); lastAction = tmp; break;
        case K_LEFT: rotateView(-0.1f, 0.0f); lastAction = "rotate left"; break;
        case K_RIGHT: rotateView(0.1f, 0.0f); lastAction = "rotate right"; break;
        case K_UP: rotateView(0.0f, -0.1f); lastAction = "rotate up"; break;
        case K_DOWN: rotateView(0.0f, 0.1f); lastAction = "rotate down"; break;
        default: lastAction = std::string("ignored ") + keyName(k); break;
        }
    }
    void update(float dt) {
        if (spin && !dragging) orient = normalize(quatAxisAngle(Vec3(0, 1, 0), 0.7f * dt) * orient);
        if (lightOrbit) lightAngle += 0.8f * dt;
    }

    void renderScene(Renderer& rdr, Framebuffer& fb, FrameStats& stats) {
        const Mesh& m = meshes[cur];
        const float fov = 45.0f * PI / 180.0f;
        // the terminal cell is not square: correct the aspect ratio
        float aspect = (float)fb.w / ((float)fb.h * cellAspect);
        float tv = std::tan(fov * 0.5f), th = tv * aspect;
        float half = std::atan(std::min(tv, th));
        float dist = 1.15f / std::sin(half) * zoom;  // model fits a unit sphere
        Mat4 model = quatToMat4(orient) * scaling(m.scale) * translation(-m.center);
        Mat4 view = lookAt(Vec3(0, 0, dist), Vec3(0, 0, 0), Vec3(0, 1, 0));
        Mat4 proj = perspective(fov, aspect, 0.02f, 1000.0f);
        Settings s = st;
        if (lightOrbit) {
            float ca = std::cos(lightAngle), sa = std::sin(lightAngle);
            Vec3 l = st.light;
            s.light = Vec3(l.x * ca + l.z * sa, l.y, -l.x * sa + l.z * ca);
        }
        rdr.render(m, model, view, proj, s, fb, stats);
    }
};

// ============================================================================
// Terminal I/O (raw mode, alternate screen, mouse)
// ============================================================================

volatile std::sig_atomic_t g_signalQuit = 0;
extern "C" void onSignal(int) { g_signalQuit = 1; }

const char* const TERM_ENTER =
    "\x1b[0m"      // known attribute state: default foreground
    "\x1b[?1049h"  // alternate screen
    "\x1b[?25l"    // hide cursor
    "\x1b[?7l"     // no auto-wrap (safe to write the bottom-right cell)
    "\x1b[?1002h"  // mouse: press / release / drag
    "\x1b[?1006h"  // SGR mouse encoding
    "\x1b[2J\x1b[H";
const char* const TERM_LEAVE =
    "\x1b[0m"
    "\x1b[?1006l\x1b[?1003l\x1b[?1002l"
    "\x1b[?7h\x1b[?25h"
    "\x1b[?1049l";

#ifdef _WIN32

// Closing the window / Ctrl+C / logoff must also restore the console:
// the alternate screen, cursor, wrapping and mouse tracking from TERM_LEAVE.
static BOOL WINAPI onConsoleCtrl(DWORD evt) {
    if (evt == CTRL_C_EVENT || evt == CTRL_BREAK_EVENT || evt == CTRL_CLOSE_EVENT ||
        evt == CTRL_LOGOFF_EVENT || evt == CTRL_SHUTDOWN_EVENT) {
        g_signalQuit = 1;
        return TRUE;
    }
    return FALSE;
}

class Terminal {
public:
    Terminal() : active_(false), hIn_(NULL), hOut_(NULL), inMode_(0), outMode_(0), inCp_(0), outCp_(0) {}
    ~Terminal() { restore(); }
    bool init(std::string& err) {
        hIn_ = GetStdHandle(STD_INPUT_HANDLE);
        hOut_ = GetStdHandle(STD_OUTPUT_HANDLE);
        if (hIn_ == NULL || hIn_ == INVALID_HANDLE_VALUE || hOut_ == NULL ||
            hOut_ == INVALID_HANDLE_VALUE) {
            err = "stdin/stdout must be a console (use --bench for headless mode)";
            return false;
        }
        if (!GetConsoleMode(hIn_, &inMode_) || !GetConsoleMode(hOut_, &outMode_)) {
            err = "stdin/stdout must be a console (use --bench for headless mode)";
            return false;
        }
        DWORD in = (inMode_ | ENABLE_EXTENDED_FLAGS | ENABLE_VIRTUAL_TERMINAL_INPUT);
        // VT input: mouse/arrows arrive as ANSI bytes via ReadFile, so the legacy
        // INPUT_RECORD mouse/window events must stay disabled.
        in &= ~(DWORD)(ENABLE_ECHO_INPUT | ENABLE_LINE_INPUT | ENABLE_PROCESSED_INPUT |
                       ENABLE_QUICK_EDIT_MODE | ENABLE_MOUSE_INPUT | ENABLE_WINDOW_INPUT);
        DWORD out = outMode_ | ENABLE_PROCESSED_OUTPUT | ENABLE_VIRTUAL_TERMINAL_PROCESSING |
                    DISABLE_NEWLINE_AUTO_RETURN;
        if (!SetConsoleMode(hOut_, out)) {
            err = "this console does not support ANSI escape sequences (need Windows 10+ / Windows Terminal)";
            return false;
        }
        if (!SetConsoleMode(hIn_, in)) {
            SetConsoleMode(hOut_, outMode_);
            err = "cannot switch the console input to raw mode";
            return false;
        }
        // Cyrillic and other non-ASCII keys must arrive as UTF-8 so the input
        // parser sees proper multibyte sequences (legacy consoles use CP866/1251,
        // which would deliver single undecodable bytes). Best effort: keep going
        // even if the codepage switch fails.
        inCp_ = GetConsoleCP();
        outCp_ = GetConsoleOutputCP();
        SetConsoleCP(CP_UTF8);
        SetConsoleOutputCP(CP_UTF8);
        active_ = true;
        SetConsoleCtrlHandler(onConsoleCtrl, TRUE);
        write(TERM_ENTER);
        return true;
    }
    void restore() {
        if (!active_) return;
        active_ = false;
        write(TERM_LEAVE);
        SetConsoleCtrlHandler(onConsoleCtrl, FALSE);
        if (inCp_) SetConsoleCP(inCp_);
        if (outCp_) SetConsoleOutputCP(outCp_);
        SetConsoleMode(hIn_, inMode_);
        SetConsoleMode(hOut_, outMode_);
    }
    void size(int& cols, int& rows) const {
        CONSOLE_SCREEN_BUFFER_INFO info;
        cols = 80;
        rows = 24;
        if (GetConsoleScreenBufferInfo(hOut_, &info)) {
            cols = info.srWindow.Right - info.srWindow.Left + 1;
            rows = info.srWindow.Bottom - info.srWindow.Top + 1;
        }
    }
    bool write(const std::string& s) {
        size_t off = 0;
        while (off < s.size()) {
            DWORD written = 0;
            DWORD chunk = (DWORD)std::min<size_t>(s.size() - off, 1u << 16);
            if (!WriteFile(hOut_, s.data() + off, chunk, &written, NULL) || written == 0) return false;
            off += written;
        }
        return true;
    }
    // Returns bytes read (>0), 0 on timeout, -1 on error.
    // NOTE: with ENABLE_VIRTUAL_TERMINAL_INPUT the console delivers keys, arrows
    // and SGR mouse (ESC[<b;x;yM/m) as plain ANSI bytes, so we must use ReadFile
    // (not ReadConsoleInputW, which would drop arrows/wheel/drag).
    int read(char* buf, int cap, int timeoutMs) {
        if (cap <= 0) return 0;
        DWORD r = WaitForSingleObject(hIn_, (DWORD)std::max(0, timeoutMs));
        if (r == WAIT_TIMEOUT) return 0;
        if (r != WAIT_OBJECT_0) return -1;
        // Drop focus/menu/resize records that would make ReadFile return 0 bytes.
        for (;;) {
            DWORD avail = 0;
            if (!GetNumberOfConsoleInputEvents(hIn_, &avail)) return -1;
            if (avail == 0) return 0;
            INPUT_RECORD rec;
            DWORD nn = 0;
            if (!PeekConsoleInputW(hIn_, &rec, 1, &nn) || nn == 0) return 0;
            if (rec.EventType == WINDOW_BUFFER_SIZE_EVENT || rec.EventType == MENU_EVENT ||
                rec.EventType == FOCUS_EVENT) {
                if (!ReadConsoleInputW(hIn_, &rec, 1, &nn)) return -1;
                continue;
            }
            break;
        }
        DWORD n = 0;
        if (!ReadFile(hIn_, buf, (DWORD)cap, &n, NULL)) return -1;
        return (int)n;  // may be 0: caller treats it as "no input yet"
    }

private:
    bool active_;
    HANDLE hIn_, hOut_;
    DWORD inMode_, outMode_;
    UINT inCp_, outCp_;  // console codepages to restore on exit (0 = unknown)
};

#else

class Terminal {
public:
    Terminal() : active_(false) {}
    ~Terminal() { restore(); }
    bool init(std::string& err) {
        if (!isatty(STDIN_FILENO) || !isatty(STDOUT_FILENO)) {
            err = "stdin/stdout must be a terminal (use --bench for headless mode)";
            return false;
        }
        if (tcgetattr(STDIN_FILENO, &orig_) != 0) {
            err = "tcgetattr failed";
            return false;
        }
        termios raw = orig_;
        raw.c_iflag &= ~(tcflag_t)(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
        raw.c_oflag &= ~(tcflag_t)(OPOST);
        raw.c_cflag |= CS8;
        raw.c_lflag &= ~(tcflag_t)(ECHO | ICANON | IEXTEN | ISIG);  // Ctrl+C arrives as byte 3
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw) != 0) {
            err = "cannot switch the terminal to raw mode";
            return false;
        }
        active_ = true;
        write(TERM_ENTER);
        return true;
    }
    void restore() {
        if (!active_) return;
        active_ = false;
        write(TERM_LEAVE);
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_);
    }
    void size(int& cols, int& rows) const {
        cols = 0;
        rows = 0;
        struct winsize ws;
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0 && ws.ws_row > 0) {
            cols = ws.ws_col;
            rows = ws.ws_row;
        }
        if (cols <= 0) cols = 80;
        if (rows <= 0) rows = 24;
    }
    bool write(const std::string& s) { return write(s.data(), s.size()); }
    bool write(const char* p) { return write(p, std::strlen(p)); }
    // Returns bytes read (>0), 0 on timeout/interrupt, -1 on EOF or error.
    // select() rather than poll(): poll() is unreliable on terminal devices on some macOS versions.
    int read(char* buf, int cap, int timeoutMs) {
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(STDIN_FILENO, &rfds);
        struct timeval tv;
        if (timeoutMs < 0) timeoutMs = 0;
        tv.tv_sec = timeoutMs / 1000;
        tv.tv_usec = (timeoutMs % 1000) * 1000;
        int r = select(STDIN_FILENO + 1, &rfds, NULL, NULL, &tv);
        if (r < 0) return errno == EINTR ? 0 : -1;
        if (r == 0) return 0;
        ssize_t n = ::read(STDIN_FILENO, buf, (size_t)cap);
        if (n > 0) return (int)n;
        if (n == 0) return -1;  // EOF: the terminal went away
        return (errno == EINTR || errno == EAGAIN) ? 0 : -1;
    }

private:
    bool write(const char* p, size_t n) {
        size_t off = 0;
        while (off < n) {
            ssize_t w = ::write(STDOUT_FILENO, p + off, n - off);
            if (w < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN) {
                    fd_set wfds;
                    FD_ZERO(&wfds);
                    FD_SET(STDOUT_FILENO, &wfds);
                    struct timeval tv;
                    tv.tv_sec = 0;
                    tv.tv_usec = 100000;
                    select(STDOUT_FILENO + 1, NULL, &wfds, NULL, &tv);
                    continue;
                }
                return false;
            }
            off += (size_t)w;
        }
        return true;
    }
    bool active_;
    termios orig_;
};

#endif

// ============================================================================
// Main loops
// ============================================================================

double nowSec() {
    using namespace std::chrono;
    static const steady_clock::time_point t0 = steady_clock::now();
    return duration<double>(steady_clock::now() - t0).count();
}

void drawHud(Screen& scr, const App& app, const FrameStats& stats, double fps, double ms, bool debug) {
    uint32_t col = app.palette ? packColor(255, 255, 255) : 0;
    char line[256];
    char fpsBuf[32];
    if (fps > 0.0) std::snprintf(fpsBuf, sizeof fpsBuf, "%.1f", fps);
    else std::snprintf(fpsBuf, sizeof fpsBuf, "--");
    // Tokens "FPS:" and "polygons:" are part of the interface (tests/pty_test.py reads them).
    std::snprintf(line, sizeof line,
                  " FPS: %s | frame: %.2f ms | polygons: %d (drawn: %d) | verts: %d | %s | zoom: %.2fx | %s%s%s%s | %dx%d ",
                  fpsBuf, ms, stats.total, stats.drawn, (int)app.meshes[app.cur].v.size(),
                  app.meshes[app.cur].name.c_str(), app.zoom, app.spin ? "spin " : "",
                  PALETTES[app.palette].name, app.st.smooth ? " smooth" : " flat",
                  app.st.cull ? "" : " 2-sided", scr.w, scr.h);
    drawText(scr, 0, 0, line, col);
    if (debug && scr.h > 2) {  // live input inspector: what arrived and what it did
        std::string dbg = " in: " + app.lastEvent + " => " + app.lastAction;
        if ((int)dbg.size() > scr.w) dbg.resize(scr.w);
        drawText(scr, 0, 1, dbg, col);
    }
    drawText(scr, 0, scr.h - 1,
             " drag: rotate  wheel/+-: zoom  arrows: rotate  space: spin  n: model  c: color  "
             "s: smooth  l: light  h: hud  r: reset  q: quit ",
             col);
}

int runInteractive(App& app, int fps, Logger& log, bool debug) {
    Terminal term;
    std::string err;
    if (!term.init(err)) {
        std::fprintf(stderr, "tty3d: %s\n", err.c_str());
        return 1;
    }
    if (log.f) log.write("tty3d log started\n");
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
#ifdef SIGHUP
    std::signal(SIGHUP, onSignal);
#endif
#ifdef SIGPIPE
    std::signal(SIGPIPE, SIG_IGN);
#endif

    Framebuffer fb;
    Screen scr;
    Renderer rdr;
    InputParser parser;
    std::vector<Event> events;
    std::string out;
    const double frameDur = fps > 0 ? 1.0 / fps : 0.0;  // fps == 0: uncapped
    double next = nowSec(), last = next, fpsT = next;
    int fpsFrames = 0, cols = 0, rows = 0;
    double fpsShown = 0.0, msShown = 0.0;
    FrameStats stats;

    while (!app.quit && !g_signalQuit) {
        double t = nowSec();
        float dt = (float)std::min(0.1, t - last);
        last = t;
        int c, r;
        term.size(c, r);
        c = std::min(c, 1000);
        r = std::min(r, 500);
        if (c != cols || r != rows) {
            cols = c;
            rows = r;
            fb.resize(cols, rows);
            scr.resize(cols, rows);
            if (log.f) {
                char tmp[64];
                std::snprintf(tmp, sizeof tmp, "resize %dx%d\n", cols, rows);
                log.write(tmp);
            }
        }
        app.update(dt);
        double t0 = nowSec();
        fb.clear();
        app.renderScene(rdr, fb, stats);
        resolve(fb, scr, app.palette);
        if (app.hud) drawHud(scr, app, stats, fpsShown, msShown, debug);
        buildFrame(scr, out);
        scr.commit();
        if (!out.empty() && !term.write(out)) break;
        double ms = (nowSec() - t0) * 1000.0;
        msShown = msShown == 0.0 ? ms : msShown * 0.9 + ms * 0.1;
        ++fpsFrames;
        if (t - fpsT >= 0.5) {
            fpsShown = fpsFrames / (t - fpsT);
            fpsFrames = 0;
            fpsT = t;
        }

        // One non-blocking input poll; the timed branch below calls it repeatedly
        // while waiting for the next frame. fps == 0 means uncapped: render
        // back-to-back, polling input once per frame.
        auto pollInput = [&](int timeoutMs) {
            char buf[512];
            int n = term.read(buf, (int)sizeof buf, timeoutMs);
            if (n < 0) {
                if (log.f) log.write("input: EOF/error, quitting\n");
                app.quit = true;
                return;
            }
            if (n > 0) {
                if (log.f) {
                    std::string raw = "raw (" + std::to_string(n) + "b): ";
                    appendEscaped(raw, buf, n);
                    raw += "\n";
                    log.write(raw);
                }
                events.clear();
                parser.feed(buf, n, nowSec(), events);
                for (size_t i = 0; i < events.size(); ++i) {
                    app.handle(events[i]);
                    if (log.f) log.write("event: " + app.lastEvent + " => " + app.lastAction + "\n");
                }
                if (log.f && events.empty()) log.write("event: (no complete sequence yet, buffered)\n");
            }
        };
        if (fps > 0) {
            // Wait for the next frame; process input while waiting.
            next += frameDur;
            double now = nowSec();
            if (now > next + frameDur) next = now;  // we fell behind: do not try to catch up
            for (;;) {
                now = nowSec();
                double rem = next - now;
                if (rem <= 0.0 || g_signalQuit || app.quit) break;
                pollInput((int)std::ceil(rem * 1000.0));
            }
        } else {
            if (!g_signalQuit) pollInput(0);
        }
        events.clear();
        parser.tick(nowSec(), events);
        for (size_t i = 0; i < events.size(); ++i) {
            app.handle(events[i]);
            if (log.f) log.write("event: " + app.lastEvent + " => " + app.lastAction + "\n");
        }
    }
    if (log.f) log.write("tty3d log finished\n");
    term.restore();
    log.close();
    return 0;
}

// Headless benchmark: renders `frames` frames, prints the last one as plain text to stdout
// and timing (measured, not estimated) to stderr.
int runBench(App& app, int frames, int W, int H) {
    Framebuffer fb;
    Screen scr;
    Renderer rdr;
    fb.resize(W, H);
    scr.resize(W, H);
    FrameStats stats;
    double total = 0.0;
    for (int i = 0; i < frames; ++i) {
        app.update(1.0f / 60.0f);
        double t0 = nowSec();
        fb.clear();
        app.renderScene(rdr, fb, stats);
        resolve(fb, scr, 0);
        total += nowSec() - t0;
    }
    for (int y = 0; y < H; ++y) {
        std::string row;
        for (int x = 0; x < W; ++x) row += scr.cur[(size_t)y * W + x].ch;
        std::printf("%s\n", row.c_str());
    }
    double avg = total / frames * 1000.0;
    std::fprintf(stderr,
                 "model=%s size=%dx%d frames=%d polygons=%d drawn=%d total=%.2f ms avg=%.4f ms/frame "
                 "(%.1f frames/s, render + resolve, single thread)\n",
                 app.meshes[app.cur].name.c_str(), W, H, frames, stats.total, stats.drawn, total * 1000.0,
                 avg, avg > 0.0 ? 1000.0 / avg : 0.0);
    return 0;
}

// ============================================================================
// Self-test (tty3d --selftest)
// ============================================================================

int g_checks = 0, g_fails = 0;
void check(bool ok, const char* what) {
    ++g_checks;
    if (!ok) {
        ++g_fails;
        std::fprintf(stderr, "FAIL: %s\n", what);
    }
}
bool approx(float a, float b, float eps = 1e-4f) { return std::fabs(a - b) <= eps; }

Mesh quadMesh(float z, float half) {  // camera-facing quad in the XY plane at depth z
    Mesh m;
    m.name = "quad";
    m.v.push_back(Vec3(-half, -half, z));
    m.v.push_back(Vec3(half, -half, z));
    m.v.push_back(Vec3(half, half, z));
    m.v.push_back(Vec3(-half, half, z));
    m.t.push_back(Tri(0, 1, 2));
    m.t.push_back(Tri(0, 2, 3));
    std::string e;
    m.finalize(e);
    return m;
}

int runSelfTest() {
    // --- matrices, quaternions
    {
        Mat4 t = translation(Vec3(1, 2, 3)) * scaling(2.0f);
        Vec4 p = t * Vec4(1, 1, 1, 1);
        check(approx(p.x, 3) && approx(p.y, 4) && approx(p.z, 5) && approx(p.w, 1), "translate*scale");
        Quat q = quatAxisAngle(Vec3(0, 1, 0), PI * 0.5f);
        Vec4 r = quatToMat4(q) * Vec4(0, 0, 1, 1);
        check(approx(r.x, 1) && approx(r.y, 0) && approx(r.z, 0), "quat +90deg about Y maps +Z to +X");
        Quat q2 = quatAxisAngle(Vec3(1, 0, 0), 0.7f) * quatAxisAngle(Vec3(0, 1, 0), 0.3f);
        Mat4 ma = quatToMat4(q2);
        Mat4 mb = quatToMat4(quatAxisAngle(Vec3(1, 0, 0), 0.7f)) * quatToMat4(quatAxisAngle(Vec3(0, 1, 0), 0.3f));
        bool same = true;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j) same = same && approx(ma.m[i][j], mb.m[i][j]);
        check(same, "quaternion product equals matrix product");
    }
    // --- projection
    {
        float fov = 60.0f * PI / 180.0f, asp = 2.0f, zn = 0.1f, zf = 50.0f;
        Mat4 P = perspective(fov, asp, zn, zf);
        Vec4 a = P * Vec4(0, 0, -zn, 1);
        Vec4 b = P * Vec4(0, 0, -zf, 1);
        check(approx(a.z / a.w, -1.0f), "near plane -> ndc z = -1");
        check(approx(b.z / b.w, 1.0f, 1e-3f), "far plane -> ndc z = +1");
        float d = 5.0f, h = d * std::tan(fov * 0.5f);
        Vec4 c = P * Vec4(h * asp, h, -d, 1);
        check(approx(c.x / c.w, 1.0f) && approx(c.y / c.w, 1.0f), "frustum edge -> ndc (1,1)");
        Mat4 V = lookAt(Vec3(0, 0, 4), Vec3(0, 0, 0), Vec3(0, 1, 0));
        Vec4 o = V * Vec4(0, 0, 0, 1);
        check(approx(o.x, 0) && approx(o.y, 0) && approx(o.z, -4), "lookAt puts the target 4 units in front (-Z)");
    }
    // --- winding / closed meshes
    {
        Mesh ms[5] = {makeTorus(), makeCube(), makeSphere(), makeCone(), makeKnot()};
        for (int i = 0; i < 5; ++i) {
            std::string e;
            check(ms[i].finalize(e), "builtin mesh finalizes");
            check(signedVolume(ms[i]) > 0.0f, (ms[i].name + ": triangles wound outward").c_str());
        }
        Mesh cube = makeCube();
        check(cube.t.size() == 12 && cube.v.size() == 24, "cube: 12 triangles, 24 vertices");
    }
    // --- OBJ parser
    {
        std::string obj =
            "# comment\r\n"
            "v 0 0 0\r\nv 1 0 0\r\nv 1 1 0\r\nv 0 1 0\r\n"
            "vt 0 0\nvn 0 0 1\n"
            "f 1/1/1 2/1/1 3/1/1 4/1/1\r\n"   // quad -> 2 triangles
            "f -4 -3 -2\n"                      // relative indices
            "f 1 2\n"                           // too short: skipped
            "f 1 2 99\n"                        // out of range: skipped
            "o something\ns off\n";
        Mesh m;
        std::string e;
        int skipped = 0;
        bool ok = parseObj(obj, m, e, &skipped);
        check(ok, "obj parses");
        check(m.v.size() == 4, "obj: 4 vertices");
        check(m.t.size() == 3, "obj: quad fan + relative triangle = 3 triangles");
        check(skipped == 2, "obj: 2 invalid faces skipped");
        check(m.t.size() >= 3 && m.t[2].a == 0 && m.t[2].b == 1 && m.t[2].c == 2, "obj: negative indices resolved");
        Mesh empty;
        check(!parseObj("hello\n", empty, e, NULL), "obj without geometry is rejected");
    }
    // --- input parser
    {
        InputParser p;
        std::vector<Event> ev;
        const char* s1 = "\x1b[<0;10;5M";
        p.feed(s1, (int)std::strlen(s1), 0.0, ev);
        check(ev.size() == 1 && ev[0].type == Event::MouseDown && ev[0].x == 10 && ev[0].y == 5 && ev[0].btn == 0, "sgr mouse press");
        ev.clear();
        const char* s2 = "\x1b[<32;12;7M";
        p.feed(s2, (int)std::strlen(s2), 0.0, ev);
        check(ev.size() == 1 && ev[0].type == Event::MouseMove && ev[0].x == 12 && ev[0].btn == 0, "sgr mouse drag");
        ev.clear();
        const char* s3 = "\x1b[<0;12;7m";
        p.feed(s3, (int)std::strlen(s3), 0.0, ev);
        check(ev.size() == 1 && ev[0].type == Event::MouseUp, "sgr mouse release");
        ev.clear();
        const char* s4 = "\x1b[<64;1;1M\x1b[<65;1;1M";
        p.feed(s4, (int)std::strlen(s4), 0.0, ev);
        check(ev.size() == 2 && ev[0].type == Event::Wheel && ev[0].key == 1 && ev[1].key == -1, "sgr wheel up/down");
        ev.clear();
        p.feed("\x1b[<0;1", 6, 0.0, ev);
        check(ev.empty(), "partial sequence is buffered");
        p.feed("0;5M", 4, 0.0, ev);
        check(ev.size() == 1 && ev[0].x == 10 && ev[0].y == 5, "split sequence is completed");
        ev.clear();
        const char* s5 = "\x1b[A\x1b[1;2D\x1bOC";
        p.feed(s5, (int)std::strlen(s5), 0.0, ev);
        check(ev.size() == 3 && ev[0].key == K_UP && ev[1].key == K_LEFT && ev[2].key == K_RIGHT, "arrow keys");
        ev.clear();
        p.feed("q", 1, 1.0, ev);
        check(ev.size() == 1 && ev[0].key == 'q', "plain key");
        ev.clear();
        p.feed("\x1b", 1, 2.0, ev);
        p.tick(2.01, ev);
        check(ev.empty(), "lone ESC waits");
        p.tick(2.2, ev);
        check(ev.size() == 1 && ev[0].key == 27, "lone ESC becomes Escape after timeout");
        ev.clear();
        {   // UTF-8: U+0441 (bytes D1 81) is one Key, even split between feeds
            InputParser u;
            const char part[1] = {(char)0xD1};
            u.feed(part, 1, 3.0, ev);
            check(ev.empty(), "split UTF-8 sequence is buffered");
            const char rest[1] = {(char)0x81};
            u.feed(rest, 1, 3.0, ev);
            check(ev.size() == 1 && ev[0].type == Event::Key && ev[0].key == 0x441, "utf-8 russian key decoded");
            ev.clear();
        }
        check(keyName('q') == "q" && keyName(27) == "esc" && keyName(3) == "ctrl-c" &&
                  keyName(K_UP) == "up" && keyName(0x441) == "U+0441",
              "keyName renders keys");
        {
            Event e;
            e.type = Event::Wheel; e.key = 1;
            check(eventToString(e) == "wheel up", "eventToString wheel");
            e.type = Event::MouseDown; e.btn = 0; e.x = 10; e.y = 5;
            check(eventToString(e) == "mouse-down btn=0 x=10 y=5", "eventToString mouse");
        }
        {
            std::string s;
            const char raw[3] = {(char)0x1b, '[', 'A'};
            appendEscaped(s, raw, 3);
            check(s == "<ESC>[A", "appendEscaped escapes control bytes");
        }
    }
    // --- rasterizer
    {
        Settings st;
        Renderer rdr;
        FrameStats stats;
        Mat4 view = Mat4::identity();  // camera at the origin looking down -Z
        Mat4 proj = perspective(60.0f * PI / 180.0f, 1.0f, 0.1f, 100.0f);
        {   // quad that covers the whole view: every cell covered, no gaps on the shared diagonal
            Mesh q = quadMesh(-1.0f, 10.0f);
            Framebuffer fb;
            fb.resize(40, 20);
            rdr.render(q, Mat4::identity(), view, proj, st, fb, stats);
            int covered = 0;
            for (size_t i = 0; i < fb.depth.size(); ++i) covered += fb.depth[i] > 0.0f;
            check(covered == 40 * 20, "full-screen quad covers every cell (no gaps)");
        }
        {   // Z-buffer: nearer quad wins regardless of draw order
            Mesh nearQ = quadMesh(-2.0f, 0.5f), farQ = quadMesh(-4.0f, 5.0f);
            Framebuffer fb1, fb2;
            fb1.resize(30, 30);
            fb2.resize(30, 30);
            Settings a = st;
            a.light = Vec3(0, 0, 1);  // flat face towards the camera: lum = 1
            // draw far then near, and near then far
            rdr.render(farQ, Mat4::identity(), view, proj, a, fb1, stats);
            float lumFar = fb1.lum[15 * 30 + 15];
            float dFar = fb1.depth[15 * 30 + 15];
            rdr.render(nearQ, Mat4::identity(), view, proj, a, fb1, stats);
            rdr.render(nearQ, Mat4::identity(), view, proj, a, fb2, stats);
            rdr.render(farQ, Mat4::identity(), view, proj, a, fb2, stats);
            check(approx(fb1.depth[15 * 30 + 15], 0.5f) && approx(fb2.depth[15 * 30 + 15], 0.5f),
                  "z-buffer keeps the nearer surface (1/w = 1/2) in both draw orders");
            check(approx(dFar, 0.25f), "far quad has 1/w = 1/4");
            check(approx(lumFar, 1.0f), "facing quad is fully lit");
            check(fb1.depth[0] > 0.0f && approx(fb1.depth[0], 0.25f, 0.05f), "far quad still visible around the near one");
        }
        {   // back-face culling and two-sided mode
            Mesh q = quadMesh(-2.0f, 1.0f);
            std::swap(q.t[0].b, q.t[0].c);
            std::swap(q.t[1].b, q.t[1].c);  // now facing away
            Framebuffer fb;
            fb.resize(20, 20);
            rdr.render(q, Mat4::identity(), view, proj, st, fb, stats);
            int covered = 0;
            for (size_t i = 0; i < fb.depth.size(); ++i) covered += fb.depth[i] > 0.0f;
            check(covered == 0, "back-facing quad is culled");
            Settings two = st;
            two.cull = false;
            fb.clear();
            rdr.render(q, Mat4::identity(), view, proj, two, fb, stats);
            covered = 0;
            for (size_t i = 0; i < fb.depth.size(); ++i) covered += fb.depth[i] > 0.0f;
            check(covered > 0, "back-facing quad is drawn when culling is off");
        }
        {   // near-plane clipping: a big floor tilted through the camera must not crash or invert
            Mesh q;
            q.name = "floor";
            q.v.push_back(Vec3(-5, -1, 5));
            q.v.push_back(Vec3(5, -1, 5));
            q.v.push_back(Vec3(5, -1, -20));
            q.v.push_back(Vec3(-5, -1, -20));
            q.t.push_back(Tri(0, 1, 2));
            q.t.push_back(Tri(0, 2, 3));
            std::string e;
            q.finalize(e);
            Framebuffer fb;
            fb.resize(40, 20);
            rdr.render(q, Mat4::identity(), view, proj, st, fb, stats);
            int covered = 0;
            bool sane = true;
            for (size_t i = 0; i < fb.depth.size(); ++i) {
                covered += fb.depth[i] > 0.0f;
                if (!(fb.depth[i] >= 0.0f) || !std::isfinite(fb.depth[i])) sane = false;
            }
            check(sane, "clipped geometry yields finite depth values");
            check(covered > 0 && covered < 40 * 20, "floor crossing the near plane is clipped, lower half of the screen is covered");
            bool upperEmpty = true;
            for (int y = 0; y < 8; ++y)
                for (int x = 0; x < 40; ++x)
                    if (fb.depth[(size_t)y * 40 + x] > 0.0f) upperEmpty = false;
            check(upperEmpty, "floor stays below the horizon");
        }
        {   // extreme coordinates must not crash
            Mesh q = quadMesh(-0.001f, 1e6f);
            Framebuffer fb;
            fb.resize(10, 10);
            rdr.render(q, Mat4::identity(), view, proj, st, fb, stats);
            check(true, "huge triangle does not crash");
        }
    }
    // --- shading / output
    {
        int prev = 0;
        bool mono = true, floor1 = true;
        for (int i = 0; i <= 100; ++i) {
            int idx = rampIndex(i / 100.0f);
            if (idx < prev) mono = false;
            if (idx < 1) floor1 = false;
            prev = idx;
        }
        check(mono, "ramp index is monotonic");
        check(floor1, "covered pixels never map to the blank character");
        check(rampIndex(1.0f) == RAMP_N - 1 && RAMP[RAMP_N - 1] == '@', "full light -> '@'");
        Screen s;
        s.resize(8, 3);
        for (size_t i = 0; i < s.cur.size(); ++i) s.cur[i].ch = 'x';
        std::string out;
        buildFrame(s, out);
        check(out.find("\x1b[H") != std::string::npos, "first frame is sent in full");
        s.commit();
        for (size_t i = 0; i < s.cur.size(); ++i) s.cur[i].ch = 'x';
        buildFrame(s, out);
        check(out.empty(), "identical frame produces no output (diff)");
        s.cur[1 * 8 + 3].ch = 'y';
        buildFrame(s, out);
        check(out == "\x1b[2;4Hy", "single changed cell -> one cursor move + one char");
    }
    // --- diff output replayed on a tiny terminal emulator must reproduce the frame exactly
    {
        const int W = 17, H = 6;
        Screen s;
        s.resize(W, H);
        std::vector<Cell> term((size_t)W * H);  // emulated terminal contents
        uint32_t seed = 12345u;
        bool allOk = true;
        for (int frame = 0; frame < 300 && allOk; ++frame) {
            for (size_t i = 0; i < s.cur.size(); ++i) {
                seed = seed * 1664525u + 1013904223u;
                uint32_t r = seed >> 16;
                if (frame == 0 || (r % 7) == 0) {  // change roughly 1/7 of the cells each frame
                    Cell c;
                    c.ch = (char)('a' + (r >> 3) % 26);
                    uint32_t k = (r >> 8) % 4;
                    c.color = k == 0 ? 0u : packColor((int)k * 60, (int)k * 30, 255 - (int)k * 20);
                    if ((r % 11) == 0) c = Cell();
                    s.cur[i] = c;
                } else {
                    s.cur[i] = s.prev[i];
                }
            }
            std::string out;
            buildFrame(s, out);
            // replay
            int row = 0, col = 0;
            uint32_t fg = 0;
            for (size_t i = 0; i < out.size();) {
                if (out[i] == '\x1b') {
                    if (out.compare(i, 4, "\x1b[2J") == 0) {
                        for (size_t k = 0; k < term.size(); ++k) term[k] = Cell();
                        i += 4;
                    } else if (out.compare(i, 3, "\x1b[H") == 0) {
                        row = 0; col = 0; i += 3;
                    } else if (out.compare(i, 5, "\x1b[39m") == 0) {
                        fg = 0; i += 5;
                    } else if (out.compare(i, 7, "\x1b[38;2;") == 0) {
                        int r2 = 0, g2 = 0, b2 = 0, used = 0;
                        std::sscanf(out.c_str() + i + 7, "%d;%d;%dm%n", &r2, &g2, &b2, &used);
                        fg = packColor(r2, g2, b2);
                        i += 7 + (size_t)used;
                    } else {
                        int rr = 0, cc = 0, used = 0;
                        if (std::sscanf(out.c_str() + i, "\x1b[%d;%dH%n", &rr, &cc, &used) == 2) {
                            row = rr - 1; col = cc - 1; i += (size_t)used;
                        } else { allOk = false; break; }
                    }
                } else {
                    if (row < 0 || row >= H || col < 0 || col >= W) { allOk = false; break; }
                    Cell c;
                    c.ch = out[i];
                    c.color = fg;
                    term[(size_t)row * W + col] = c;
                    if (col < W - 1) ++col;  // auto-wrap is off: the cursor stays in the last column
                    ++i;
                }
            }
            // spaces carry no visible colour; compare like a viewer would
            for (size_t k = 0; k < term.size() && allOk; ++k) {
                Cell a = term[k], b = s.cur[k];
                if (a.ch == ' ') a.color = 0;
                if (b.ch == ' ') b.color = 0;
                if (a.ch != b.ch || a.color != b.color) allOk = false;
            }
            if (fg != 0) allOk = false;  // every frame must end with the default foreground
            s.commit();
        }
        check(allOk, "300 random diff frames replayed on an emulator reproduce the screen exactly");
    }
    // --- application controls
    {
        App app;
        Mesh a = makeCube(), b = makeSphere();
        std::string e;
        a.finalize(e);
        b.finalize(e);
        app.meshes.push_back(a);
        app.meshes.push_back(b);
        app.resetView();
        Quat q0 = app.orient;
        Event down;
        down.type = Event::MouseDown; down.btn = 0; down.x = 10; down.y = 10;
        app.handle(down);
        check(app.dragging, "mouse press starts a drag");
        Event mv;
        mv.type = Event::MouseMove; mv.btn = 0; mv.x = 30; mv.y = 10;
        app.handle(mv);
        Vec4 front = quatToMat4(app.orient) * Vec4(0, 0, 1, 1);
        Vec4 front0 = quatToMat4(q0) * Vec4(0, 0, 1, 1);
        check(front.x > front0.x + 0.1f, "dragging right turns the front of the model to the right");
        Event up;
        up.type = Event::MouseUp; up.btn = 0;
        app.handle(up);
        check(!app.dragging, "mouse release ends the drag");
        Quat qn = app.orient;
        app.handle(mv);
        check(app.orient.w == qn.w && app.orient.x == qn.x, "motion without a pressed button does not rotate");
        Event wh;
        wh.type = Event::Wheel; wh.key = 1;
        float z0 = app.zoom;
        app.handle(wh);
        check(app.zoom < z0, "wheel up zooms in");
        for (int i = 0; i < 200; ++i) app.handle(wh);
        check(app.zoom >= 0.2f - 1e-6f, "zoom is clamped at the lower bound");
        wh.key = -1;
        for (int i = 0; i < 400; ++i) app.handle(wh);
        check(app.zoom <= 6.0f + 1e-6f, "zoom is clamped at the upper bound");
        app.handleKey('n');
        check(app.cur == 1 && app.zoom == 1.0f, "'n' selects the next model and resets the view");
        app.handleKey('n');
        check(app.cur == 0, "'n' wraps around");
        app.handleKey('s');
        check(app.st.smooth, "'s' toggles smooth shading");
        app.handleKey('c');
        check(app.palette == 1, "'c' cycles the palette");
        for (int i = 0; i < PALETTE_N; ++i) app.handleKey('c');
        check(app.palette == 1, "palette cycle wraps");
        app.handleKey(0x441);  // Russian U+0441 = 'c'
        check(app.palette == 2 && app.lastAction == "color=cyan", "russian layout key switches color");
        app.handleKey('z');
        check(app.lastAction == "ignored z", "unknown key is reported as ignored");
        check(!app.quit, "not quitting yet");
        app.handleKey('q');
        check(app.quit, "'q' quits");
        double norm = 0;
        for (int i = 0; i < 100000; ++i) app.update(0.016f);
        norm = app.orient.w * app.orient.w + app.orient.x * app.orient.x + app.orient.y * app.orient.y + app.orient.z * app.orient.z;
        check(approx((float)norm, 1.0f, 1e-3f), "orientation stays a unit quaternion after 100000 updates");
    }
    // --- robustness sweep: odd sizes, extreme zoom, all models, all modes; results must stay sane
    {
        Mesh ms[5] = {makeTorus(), makeCube(), makeSphere(), makeCone(), makeKnot()};
        App app;
        for (int i = 0; i < 5; ++i) {
            std::string e;
            ms[i].finalize(e);
            app.meshes.push_back(ms[i]);
        }
        const int sizes[][2] = {{1, 1}, {2, 1}, {1, 5}, {7, 3}, {80, 24}, {300, 10}, {10, 200}};
        const float zooms[] = {0.2f, 1.0f, 6.0f};
        Renderer rdr;
        bool sane = true;
        uint32_t seed = 99u;
        for (size_t si = 0; si < sizeof(sizes) / sizeof(sizes[0]); ++si)
            for (size_t zi = 0; zi < 3; ++zi)
                for (size_t mi = 0; mi < app.meshes.size(); ++mi)
                    for (int mode = 0; mode < 4; ++mode) {
                        app.cur = mi;
                        app.zoom = zooms[zi];
                        app.st.smooth = (mode & 1) != 0;
                        app.st.cull = (mode & 2) == 0;
                        seed = seed * 1664525u + 1013904223u;
                        app.orient = normalize(Quat((float)(seed >> 20) / 4096.0f - 0.5f, (float)((seed >> 10) & 1023) / 1024.0f - 0.5f,
                                                    (float)(seed & 1023) / 1024.0f - 0.5f, 0.3f));
                        Framebuffer fb;
                        fb.resize(sizes[si][0], sizes[si][1]);
                        FrameStats stats;
                        app.renderScene(rdr, fb, stats);
                        for (size_t i = 0; i < fb.depth.size(); ++i) {
                            if (!std::isfinite(fb.depth[i]) || fb.depth[i] < 0.0f) sane = false;
                            if (fb.depth[i] > 0.0f && !(fb.lum[i] >= 0.0f && fb.lum[i] <= 1.0001f)) sane = false;
                        }
                    }
        check(sane, "sweep over sizes/zoom/models/modes: finite depth, luminance within [0,1]");
    }
    if (g_fails == 0) std::printf("selftest: all %d checks passed\n", g_checks);
    else std::printf("selftest: %d of %d checks FAILED\n", g_fails, g_checks);
    return g_fails == 0 ? 0 : 1;
}

// ============================================================================
// Command line
// ============================================================================

void usage(FILE* f) {
    std::fprintf(f,
        "tty3d - software 3D renderer for the terminal (CPU only, no dependencies)\n\n"
        "usage: tty3d [options] [model.obj ...]\n\n"
        "  Built-in models are always available: donut, cube, sphere, cone, knot.\n"
        "  Files given on the command line come first; 'n' cycles through all models.\n\n"
        "options:\n"
        "  -m, --model NAME     start with this model (a built-in name or a loaded file name)\n"
        "      --color NAME     TrueColor tint: off, amber, cyan, green, magenta (default: off)\n"
        "      --smooth         start with smooth (interpolated) normals instead of flat\n"
        "      --no-cull        disable back-face culling (two-sided lighting)\n"
        "      --fps N          frame rate limit, 0..1000 (default: 60, 0 = uncapped)\n"
        "      --aspect X       terminal cell height/width, 0.5..4 (default: 2)\n"
        "      --no-hud         hide the FPS/polygon overlay at start\n"
        "      --bench N        headless: render N frames, print the last one, report timings\n"
        "      --size WxH       size for --bench (default: 100x40)\n"
        "      --log FILE       append input diagnostics (raw bytes, events, actions) to FILE\n"
        "      --debug          extra HUD line with the last input event and its action\n"
        "      --selftest       run the built-in self tests\n"
        "  -h, --help           this text\n\n"
        "keys:   mouse drag / arrows rotate   wheel / + - zoom   space spin on/off   n next model\n"
        "        c color   s flat/smooth   l orbiting light   h hud   r reset view   q / Esc / Ctrl+C quit\n");
}

bool parseInt(const char* s, long lo, long hi, long& out) {
    char* e = NULL;
    long v = std::strtol(s, &e, 10);
    if (e == s || *e != '\0' || v < lo || v > hi) return false;
    out = v;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    App app;
    std::vector<std::string> files;
    std::string modelName, logPath;
    long fps = 60, benchFrames = 0, W = 100, H = 40;
    bool bench = false, debug = false;
    int palette = 0;
    Logger log;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto need = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "tty3d: option %s needs a value\n", opt);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "-h" || a == "--help") { usage(stdout); return 0; }
        else if (a == "--selftest") return runSelfTest();
        else if (a == "-m" || a == "--model") modelName = need("--model");
        else if (a == "--color") {
            std::string v = need("--color");
            palette = -1;
            for (int k = 0; k < PALETTE_N; ++k)
                if (v == PALETTES[k].name) palette = k;
            if (palette < 0) { std::fprintf(stderr, "tty3d: unknown color '%s'\n", v.c_str()); return 2; }
        }
        else if (a == "--smooth") app.st.smooth = true;
        else if (a == "--no-cull") app.st.cull = false;
        else if (a == "--no-hud") app.hud = false;
        else if (a == "--fps") {
            if (!parseInt(need("--fps"), 0, 1000, fps)) { std::fprintf(stderr, "tty3d: --fps must be 0..1000 (0 = uncapped)\n"); return 2; }
        }
        else if (a == "--aspect") {
            char* e = NULL;
            const char* v = need("--aspect");
            float x = std::strtof(v, &e);
            if (e == v || *e != '\0' || !(x >= 0.5f && x <= 4.0f)) { std::fprintf(stderr, "tty3d: --aspect must be 0.5..4\n"); return 2; }
            app.cellAspect = x;
        }
        else if (a == "--bench") {
            if (!parseInt(need("--bench"), 1, 1000000, benchFrames)) { std::fprintf(stderr, "tty3d: --bench needs a positive frame count\n"); return 2; }
            bench = true;
        }
        else if (a == "--log") logPath = need("--log");
        else if (a == "--debug") debug = true;
        else if (a == "--size") {
            const char* v = need("--size");
            char* e = NULL;
            W = std::strtol(v, &e, 10);
            if (e == v || (*e != 'x' && *e != 'X')) { std::fprintf(stderr, "tty3d: --size expects WxH\n"); return 2; }
            const char* v2 = e + 1;
            char* e2 = NULL;
            H = std::strtol(v2, &e2, 10);
            if (e2 == v2 || *e2 != '\0' || W < 1 || W > 1000 || H < 1 || H > 500) { std::fprintf(stderr, "tty3d: --size must be WxH with W 1..1000, H 1..500\n"); return 2; }
        }
        else if (a.size() > 1 && a[0] == '-') { std::fprintf(stderr, "tty3d: unknown option '%s' (try --help)\n", a.c_str()); return 2; }
        else files.push_back(a);
    }

    for (size_t i = 0; i < files.size(); ++i) {
        Mesh m;
        std::string err;
        int skipped = 0;
        if (!loadObjFile(files[i].c_str(), m, err, &skipped)) {
            std::fprintf(stderr, "tty3d: %s: %s\n", files[i].c_str(), err.c_str());
            return 1;
        }
        if (skipped > 0) std::fprintf(stderr, "tty3d: %s: skipped %d invalid face(s)\n", files[i].c_str(), skipped);
        app.meshes.push_back(m);
    }
    Mesh builtins[5] = {makeTorus(), makeCube(), makeSphere(), makeCone(), makeKnot()};
    for (int i = 0; i < 5; ++i) {
        std::string err;
        if (!builtins[i].finalize(err)) {
            std::fprintf(stderr, "tty3d: internal error building %s: %s\n", builtins[i].name.c_str(), err.c_str());
            return 1;
        }
        app.meshes.push_back(builtins[i]);
    }
    if (!modelName.empty()) {
        bool found = false;
        for (size_t i = 0; i < app.meshes.size(); ++i)
            if (app.meshes[i].name == modelName) { app.cur = i; found = true; break; }
        if (!found) {
            std::fprintf(stderr, "tty3d: no model named '%s'\n", modelName.c_str());
            return 2;
        }
    }
    app.palette = palette;
    app.resetView();

    if (bench) return runBench(app, (int)benchFrames, (int)W, (int)H);
    if (!logPath.empty()) {
        std::string err;
        if (!log.open(logPath.c_str(), err)) {
            std::fprintf(stderr, "tty3d: %s\n", err.c_str());
            return 1;
        }
    }
    return runInteractive(app, (int)fps, log, debug);
}
