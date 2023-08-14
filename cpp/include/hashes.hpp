#pragma once
#ifndef OPENCUBES_HASHES_HPP
#define OPENCUBES_HASHES_HPP
#include <array>
#include <cassert>
#include <cstdio>
#include <deque>
#include <map>
#include <shared_mutex>
#include <unordered_set>
#include <vector>

#include "cube.hpp"
#include "utils.hpp"

struct HashCube {
    size_t operator()(const Cube &cube) const {
        // https://stackoverflow.com/questions/20511347/a-good-hash-function-for-a-vector/72073933#72073933
        std::size_t seed = cube.size();
        for (auto &p : cube) {
            auto x = HashXYZ()(p);
            seed ^= x + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
    size_t operator()(const Cube *cube) const { return (*this)(*cube); }
};

struct CubePtrEqual {
    bool operator()(const Cube *a, const Cube *b) const { return *a == *b; }
};

using CubePtrSet = std::unordered_set<const Cube *, HashCube, CubePtrEqual>;
using CubeSet = std::unordered_set<Cube, HashCube, std::equal_to<Cube>>;

// The main hashset stores *differences* to the base cube(s)
// There are usually a lot less base cubes than their expansions
// So this allows de-duplicating this data.
// At worst there is 1:1 ratio of base cubes and their expansions.
// todo: the Base cube should be pointing directly into the cache file...
struct CubeDiff {
    const Cube *base;
    XYZ expand;

    Cube flat() const {
        // the base cube is in sorted order:
        Cube tmp(base->size() + 1);
        auto put = tmp.begin();
        for (auto &c : *base) {
            *put++ = c;
        }
        *put++ = expand;
        return tmp;
    }

    // Assemble temporary canonical cube from base cube and expansion.
    Cube canonical() const {
        // the base cube is in sorted order:
        Cube tmp(flat());
        std::sort(tmp.begin(), tmp.end());
        return tmp;
    }

    bool operator==(const CubeDiff &rhs) const {
        // note: can use non-canonical compare:
        auto self = flat();
        auto other = rhs.flat();
        if (self.size() != other.size()) return false;
        return std::mismatch(self.begin(), self.end(), other.begin()).first == self.end();
    }

    size_t size() const { return base->size() + 1; };

    void copyout(int num, XYZ *dest) const {
        assert(num == (signed)size());
        auto put = dest;
        for (auto &c : *base) {
            *put++ = c;
        }
        *put++ = expand;
        std::sort(dest, put);
    }
};

struct HashCubeDiff {
    size_t operator()(const CubeDiff &cube) const { return HashCube()(cube.canonical()); }
};
using CubeDiffSet = std::unordered_set<CubeDiff, HashCubeDiff, std::equal_to<CubeDiff>>;

struct Hashy {
    struct Subsubhashy {
        CubeDiffSet diff_set;
        CubePtrSet base_set;
        std::deque<Cube> base_cubes;
        mutable std::shared_mutex set_mutex;

        void insert(const Cube *base, XYZ diff) {
            std::shared_lock lock(set_mutex);
            if (diff_set.count(CubeDiff{base, diff})) return;
            lock.unlock();

            std::lock_guard elock(set_mutex);
            diff_set.emplace(CubeDiff{base, diff});
        }

        // insert or find base cube
        template <typename CubeT>
        const Cube *insert_base(CubeT &&base) {
            // Insert into base_set
            // Problem: the base_set.emplace() invalidates references
            // but we need the reference/pointer to be stable.
            // Solution:
            // Insert pointers instead and point into Cube
            // in std::deque<Cube> that does have stable references.
            // note: the data structure doesn't care what the memory address is:
            // so we can still find "non-existing" memory addresses.
            std::shared_lock lock(set_mutex);
            auto bref = base_set.find(&base);
            if (bref == base_set.end()) {
                lock.unlock();
                std::lock_guard elock(set_mutex);
                auto base_ptr = &base_cubes.emplace_back(std::forward<CubeT>(base));
                auto [bref, isnew] = base_set.emplace(base_ptr);
                if (!isnew) {
                    // revert.. somebody succeeded before us.
                    base_cubes.pop_back();
                }
                return *bref;
            } else {
                return *bref;
            }
        }

        void clear() {
            diff_set.clear();
            base_set.clear();
            diff_set.reserve(1);
            base_set.reserve(1);
            base_cubes.clear();
            base_cubes.shrink_to_fit();
        }

        auto base_size() const {
            std::shared_lock lock(set_mutex);
            return base_set.size();
        }
        auto size() const {
            std::shared_lock lock(set_mutex);
            return diff_set.size();
        }
    };
    template <int NUM>
    struct Subhashy {
        std::array<Subsubhashy, NUM> byhash;

        template <typename CubeT>
        void insert(CubeT &&base, XYZ diff) {
            HashCube hash;
            auto idx = hash(base) % NUM;
            auto &set = byhash[idx];

            // de-duplicate base cube:
            const Cube *pbase = set.insert_base(std::forward<CubeT>(base));

            set.insert(pbase, diff);
            // printf("new size %ld\n\r", byshape[shape].size());
        }

        auto base_size() const {
            size_t sum = 0;
            for (auto &set : byhash) {
                auto part = set.base_size();
                sum += part;
            }
            return sum;
        }

        auto size() const {
            size_t sum = 0;
            for (auto &set : byhash) {
                auto part = set.size();
                sum += part;
            }
            return sum;
        }
    };

    std::map<XYZ, Subhashy<32>> byshape;

    static std::vector<XYZ> generateShapes(int n) {
        std::vector<XYZ> out;
        for (int x = 0; x < n; ++x)
            for (int y = x; y < (n - x); ++y)
                for (int z = y; z < (n - x - y); ++z) {
                    if ((x + 1) * (y + 1) * (z + 1) < n)  // not enough space for n cubes
                        continue;
                    out.emplace_back(x, y, z);
                }
        return out;
    }

    void init(int n) {
        // create all subhashy which will be needed for N
        for (auto s : generateShapes(n)) byshape[s].size();
        std::printf("%ld sets by shape for N=%d\n\r", byshape.size(), n);
    }

    template <typename CubeT>
    void insert(CubeT &&base, XYZ diff, XYZ shape) {
        auto &set = byshape[shape];
        set.insert(std::forward<CubeT>(base), diff);
    }

    auto base_size() const {
        size_t sum = 0;
        for (auto &set : byshape) {
            auto part = set.second.base_size();
            sum += part;
        }
        return sum;
    }

    auto size() const {
        size_t sum = 0;
        DEBUG1_PRINTF("%ld maps by shape\n\r", byshape.size());
        for (auto &set : byshape) {
            auto part = set.second.size();
            DEBUG1_PRINTF("bucket [%2d %2d %2d]: %ld\n", set.first.x(), set.first.y(), set.first.z(), part);
            sum += part;
        }
        return sum;
    }
};
#endif
