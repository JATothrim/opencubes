#pragma once
#ifndef OPENCUBES_CUBE_HPP
#define OPENCUBES_CUBE_HPP

#include <stdint.h>

#include <unordered_set>
#include <vector>

#include "utils.hpp"

struct XYZ {
    int8_t data[3];
    explicit XYZ(int8_t a = 0, int8_t b = 0, int8_t c = 0) : data{a, b, c} {}
    constexpr bool operator==(const XYZ &b) const { return (uint32_t) * this == (uint32_t)b; }
    constexpr bool operator<(const XYZ &b) const { return (uint32_t) * this < (uint32_t)b; }
    constexpr operator uint32_t() const { return ((uint8_t)data[0] << 16) | ((uint8_t)data[1] << 8) | ((uint8_t)data[2]); }

    constexpr int8_t &x() { return data[0]; }
    constexpr int8_t &y() { return data[1]; }
    constexpr int8_t &z() { return data[2]; }
    constexpr int8_t x() const { return data[0]; }
    constexpr int8_t y() const { return data[1]; }
    constexpr int8_t z() const { return data[2]; }
    constexpr int8_t &operator[](int offset) { return data[offset]; }
    constexpr int8_t operator[](int offset) const { return data[offset]; }
    friend XYZ operator+(const XYZ &a, const XYZ &b) {
        XYZ ret = a;
        ret += b;
        return ret;
    }
    void operator+=(const XYZ &b) {
        data[0] += b.data[0];
        data[1] += b.data[1];
        data[2] += b.data[2];
    }
};

struct HashXYZ {
    size_t operator()(const XYZ &p) const { return (uint32_t)p; }
};

using XYZSet = std::unordered_set<XYZ, HashXYZ, std::equal_to<XYZ>>;

struct Cube {
    std::vector<XYZ> sparse;
    /**
     * Define subset of vector operations for Cube
     * This simplifies the code everywhere else.
     */
    std::vector<XYZ>::iterator begin() { return sparse.begin(); }

    std::vector<XYZ>::iterator end() { return sparse.end(); }

    std::vector<XYZ>::const_iterator begin() const { return sparse.begin(); }

    std::vector<XYZ>::const_iterator end() const { return sparse.end(); }

    size_t size() const { return sparse.size(); }

    void reserve(size_t N) { sparse.reserve(N); }

    template <typename T>
    T &emplace_back(T &&p) {
        return sparse.emplace_back(std::forward<T>(p));
    }

    bool operator==(const Cube &rhs) const { return this->sparse == rhs.sparse; }

    bool operator<(const Cube &b) const {
        if (size() != b.size()) return size() < b.size();
        for (size_t i = 0; i < size(); ++i) {
            if (sparse[i] < b.sparse[i])
                return true;
            else if (sparse[i] > b.sparse[i])
                return false;
        }
        return false;
    }

    void print() const {
        for (auto &p : sparse) std::printf("  (%2d %2d %2d)\n\r", p.x(), p.y(), p.z());
    }
};
#endif
