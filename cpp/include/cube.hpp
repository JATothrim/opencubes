#pragma once
#ifndef OPENCUBES_CUBE_HPP
#define OPENCUBES_CUBE_HPP

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <memory>
#include <unordered_set>
#include <vector>

#include "utils.hpp"

class XYZ {
    uint16_t data;

   public:
    explicit XYZ(int8_t a = 0, int8_t b = 0, int8_t c = 0) { xyz(a, b, c); }
    explicit XYZ(int8_t *p) : data(0) { xyz(p[0], p[1], p[2]); }
    XYZ(std::array<int8_t, 3> s) { xyz(s[0], s[1], s[2]); }
    constexpr bool operator==(const XYZ &b) const { return (uint32_t) * this == (uint32_t)b; }
    constexpr bool operator<(const XYZ &b) const { return (uint32_t) * this < (uint32_t)b; }
    constexpr operator uint16_t() const { return data; }

    constexpr std::array<int8_t, 3> g() const {
        // with bit-packing no point extracting just one field.
        return {int8_t(int8_t((data >> 10) & 0b11111) - 1), int8_t(int8_t((data >> 5) & 0b11111) - 1), int8_t(int8_t(data & 0b11111) - 1)};
    }
    void xyz(std::array<int8_t, 3> s) { xyz(s[0], s[1], s[2]); }
    void xyz(int8_t a = 0, int8_t b = 0, int8_t c = 0) {
        assert(a >= -1 && b >= -1 && c >= -1);
        data = 0;
        data |= ((a + 1) & 0b11111) << 10;
        data |= ((b + 1) & 0b11111) << 5;
        data |= ((c + 1) & 0b11111);
    }
    constexpr int8_t x() const { return g()[0]; }
    constexpr int8_t y() const { return g()[1]; }
    constexpr int8_t z() const { return g()[2]; }
    constexpr int8_t operator[](int offset) const { return g()[offset]; }
};

struct HashXYZ {
    size_t operator()(const XYZ &p) const { return (uint32_t)p; }
};

using XYZSet = std::unordered_set<XYZ, HashXYZ, std::equal_to<XYZ>>;

struct Cube {
   private:
    // proof-of-concept:
    // pointer-bit-hacking:
    struct {
        uint64_t is_shared : 1;
        uint64_t size : 7;   // MAX 127
        uint64_t addr : 56;  // low 56-bits of memory address.
    } bits;
    XYZ *get() {
        // pointer bit-hacking:
        uint64_t addr = bits.addr;
        return reinterpret_cast<XYZ *>(addr);
    }
    const XYZ *get() const {
        // pointer bit-hacking:
        uint64_t addr = bits.addr;
        return reinterpret_cast<const XYZ *>(addr);
    }

    void put(XYZ *addr) {
        uint64_t tmp = reinterpret_cast<uint64_t>((void *)addr);
        tmp &= 0xffffffffffffff;
        bits.addr = tmp;
    }

    static_assert(sizeof(bits) == sizeof(uint64_t));

   public:
    // Empty cube
    Cube() : bits{0, 0, 0} {}

    // Cube with N capacity
    explicit Cube(uint8_t N) : bits{0, N, 0} { put(new XYZ[bits.size]); }

    // Construct from pieces
    Cube(std::initializer_list<XYZ> il) : Cube(il.size()) { std::copy(il.begin(), il.end(), begin()); }

    // Construct from external source.
    // Cube shares this the memory until modified.
    // Caller guarantees the memory given will live longer than *this
    Cube(XYZ *start, uint8_t n) : bits{1, n, 0} { put(start); }

    // Copy ctor.
    Cube(const Cube &copy) : Cube(copy.size()) { std::copy(copy.begin(), copy.end(), begin()); }

    ~Cube() {
        if (!bits.is_shared) {
            delete[] get();
        }
    }
    friend void swap(Cube &a, Cube &b) {
        using std::swap;
        swap(a.bits, b.bits);
    }

    Cube(Cube &&mv) : Cube() { swap(*this, mv); }

    Cube &operator=(const Cube &copy) {
        Cube tmp(copy);
        swap(*this, tmp);
        return *this;
    }

    Cube &operator=(Cube &&mv) {
        swap(*this, mv);
        return *this;
    }

    size_t size() const { return bits.size; }

    XYZ *data() {
        if (bits.is_shared) {
            // lift to RAM: this should never happen really.
            Cube tmp(get(), bits.size);
            swap(*this, tmp);
            std::printf("Bad use of Cube\n");
        }
        return get();
    }

    const XYZ *data() const { return get(); }

    XYZ *begin() { return data(); }

    XYZ *end() { return data() + size(); }

    const XYZ *begin() const { return data(); }

    const XYZ *end() const { return data() + size(); }

    bool operator==(const Cube &rhs) const {
        if (size() != rhs.size()) return false;
        return std::mismatch(begin(), end(), rhs.begin()).first == end();
    }

    bool operator<(const Cube &b) const {
        if (size() != b.size()) return size() < b.size();
        auto [aa, bb] = std::mismatch(begin(), end(), b.begin());
        if (aa == end()) {
            return false;
        } else {
            return *aa < *bb;
        }
    }

    void print() const {
        for (auto &p : *this) std::printf("  (%2d %2d %2d)\n\r", p.x(), p.y(), p.z());
    }
};

static_assert(std::is_move_assignable_v<Cube>, "Cube must be moveable");
static_assert(std::is_swappable_v<Cube>, "Cube must swappable");

#endif
