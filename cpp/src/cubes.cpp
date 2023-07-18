#include "cubes.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <thread>

#include "cache.hpp"
#include "cube.hpp"
#include "hashes.hpp"
#include "results.hpp"
#include "rotations.hpp"
const int PERF_STEP = 500;

void expand(const Cube &c, Hashy &hashes, XYZ shape, XYZ axisdiff, int diffsum) {
    XYZSet candidates;
    candidates.reserve(c.size() * 6);
    if (0) {
        for (const auto &p : c) {
            candidates.emplace(XYZ(p.x() + 1, p.y(), p.z()));
            candidates.emplace(XYZ(p.x() - 1, p.y(), p.z()));
            candidates.emplace(XYZ(p.x(), p.y() + 1, p.z()));
            candidates.emplace(XYZ(p.x(), p.y() - 1, p.z()));
            candidates.emplace(XYZ(p.x(), p.y(), p.z() + 1));
            candidates.emplace(XYZ(p.x(), p.y(), p.z() - 1));
        }
    } else if (diffsum == 1) {
        for (const auto &p : c) {
            if (axisdiff.x() == 1) {
                if (p.x() == shape.x()) candidates.emplace(XYZ(p.x() + 1, p.y(), p.z()));
                if (p.x() == 0) candidates.emplace(XYZ(p.x() - 1, p.y(), p.z()));
            }
            if (axisdiff.y() == 1) {
                if (p.y() == shape.y()) candidates.emplace(XYZ(p.x(), p.y() + 1, p.z()));
                if (p.y() == 0) candidates.emplace(XYZ(p.x(), p.y() - 1, p.z()));
            }
            if (axisdiff.z() == 1) {
                if (p.z() == shape.z()) candidates.emplace(XYZ(p.x(), p.y(), p.z() + 1));
                if (p.z() == 0) candidates.emplace(XYZ(p.x(), p.y(), p.z() - 1));
            }
        }
    } else {
        for (const auto &p : c) {
            if (p.x() < shape.x()) candidates.emplace(XYZ(p.x() + 1, p.y(), p.z()));
            if (p.x() > 0) candidates.emplace(XYZ(p.x() - 1, p.y(), p.z()));
            if (p.y() < shape.y()) candidates.emplace(XYZ(p.x(), p.y() + 1, p.z()));
            if (p.y() > 0) candidates.emplace(XYZ(p.x(), p.y() - 1, p.z()));
            if (p.z() < shape.z()) candidates.emplace(XYZ(p.x(), p.y(), p.z() + 1));
            if (p.z() > 0) candidates.emplace(XYZ(p.x(), p.y(), p.z() - 1));
        }
    }
    for (const auto &p : c) {
        candidates.erase(p);
    }
    DEBUG_PRINTF("candidates: %lu\n\r", candidates.size());
    for (const auto &p : candidates) {
        // std::printf("(%2d %2d %2d)\n\r", p.x(), p.y(), p.z());
        DEBUG_PRINTF("(%2d %2d %2d)\n\r", p.x(), p.y(), p.z());
        int ax = (p.x() < 0) ? 1 : 0;
        int ay = (p.y() < 0) ? 1 : 0;
        int az = (p.z() < 0) ? 1 : 0;
        Cube newCube;
        newCube.reserve(c.size() + 1);
        newCube.emplace_back(XYZ(p.x() + ax, p.y() + ay, p.z() + az));
        XYZ shape(p.x() + ax, p.y() + ay, p.z() + az);
        for (const auto &np : c) {
            auto nx = np.x() + ax;
            auto ny = np.y() + ay;
            auto nz = np.z() + az;
            if (nx > shape[0]) shape[0] = nx;
            if (ny > shape[1]) shape[1] = ny;
            if (nz > shape[2]) shape[2] = nz;
            newCube.emplace_back(XYZ(nx, ny, nz));
        }
        DEBUG_PRINTF("shape %2d %2d %2d\n\r", shape[0], shape[1], shape[2]);

        // check rotations
        Cube lowestHashCube;
        XYZ lowestShape;
        bool none_set = true;
        for (int i = 0; i < 24; ++i) {
            auto res = Rotations::rotate(i, shape, newCube);
            if (res.second.size() == 0) continue;  // rotation generated violating shape
            Cube rotatedCube{std::move(res.second)};
            std::sort(rotatedCube.begin(), rotatedCube.end());

            if (none_set || lowestHashCube < rotatedCube) {
                none_set = false;
                // std::printf("shape %2d %2d %2d\n\r", res.first.x(), res.first.y(), res.first.z());
                lowestHashCube = std::move(rotatedCube);
                lowestShape = res.first;
            }
        }
        hashes.insert(lowestHashCube, lowestShape);
        DEBUG_PRINTF("inserted! (num %2lu)\n\n\r", hashes.size());
    }
    DEBUG_PRINTF("new hashes: %lu\n\r", hashes.size());
}

struct Workset {
    std::mutex mu;
    std::vector<Cube>::iterator b;
    std::vector<Cube>::iterator e;
    Hashy &hashes;
    XYZ shape, expandDim;
    int abssum;
    Workset(std::vector<Cube> &data, Hashy &hashes, XYZ shape, XYZ expandDim, int abssum)
        : b(data.begin()), e(data.end()), hashes(hashes), shape(shape), expandDim(expandDim), abssum(abssum) {}

    struct Subset {
        std::vector<Cube>::iterator b, e;
        bool valid;
        auto begin() { return b; }
        auto end() { return e; }
    };

    Subset getPart() {
        std::lock_guard<std::mutex> g(mu);
        auto a = b;
        b += 500;
        if (b > e) b = e;
        return {a, b, a < e};
    }
};

struct Worker {
    Workset &ws;
    int id;
    Worker(Workset &ws_, int id_) : ws(ws_), id(id_) {}
    void run() {
        // std::printf("start %d\n", id);
        auto subset = ws.getPart();
        while (subset.valid) {
            // std::cout << id << " next subset " << &*subset.begin() << " to " << &*subset.end() << "\n";
            for (auto &c : subset) {
                // std::printf("%p\n", (void *)&c);
                // c.print();
                expand(c, ws.hashes, ws.shape, ws.expandDim, ws.abssum);
            }
            subset = ws.getPart();
        }
        // std::printf("finished %d\n", id);
    }
};
#if 0
void expandPart(std::vector<Cube> &base, Hashy &hashes, size_t start, size_t end) {
    auto t_start = std::chrono::steady_clock::now();
    auto t_last = t_start;
    auto total = end - start;
    for (auto i = start; i < end; ++i) {
        // expand(base[i], hashes);
        auto count = i - start;
        if (start == 0 && (count % PERF_STEP == (PERF_STEP - 1))) {
            auto t_end = std::chrono::steady_clock::now();
            auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_start).count();
            auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(t_end - t_last).count();
            t_last = t_end;
            auto perc = 100 * count / total;
            auto avg = 1000000.f * count / total_us;
            auto its = 1000000.f * PERF_STEP / dt_us;
            auto remaining = (end - i) / avg;
            std::printf(" %3ld%%, %5.0f avg baseCubes/s, %5.0f baseCubes/s, remaining: %.0fs\033[0K\r", perc, avg, its, remaining);
            std::flush(std::cout);
        }
    }
    auto t_end = std::chrono::steady_clock::now();
    auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
    std::printf("  done took %.2f s [%7lu, %7lu]\033[0K\n\r", dt_ms / 1000.f, start, end);
}
#endif
Hashy gen(int n, int threads, bool use_cache, bool write_cache) {
    Hashy hashes;
    if (n < 1)
        return {};
    else if (n == 1) {
        hashes.init(n);
        hashes.insert(Cube{{XYZ(0, 0, 0)}}, XYZ(0, 0, 0));
        std::printf("%ld elements for %d\n\r", hashes.size(), n);
        return hashes;
    }

    if (use_cache) {
        hashes = Cache::load("cubes_" + std::to_string(n) + ".bin");

        if (hashes.size() != 0) return hashes;
    }

    auto base = gen(n - 1, threads, use_cache, write_cache);
    std::printf("N = %d || generating new cubes from %lu base cubes.\n\r", n, base.size());
    hashes.init(n);
    int count = 0;
    uint64_t totalSum = 0;
    for (auto &tup : hashes.byshape) {
        XYZ targetShape = tup.first;
        std::printf("process output shape [%2d %2d %2d]\n\r", targetShape.x(), targetShape.y(), targetShape.z());
        if (threads == 1 || base.size() < 100) {
            auto start = std::chrono::steady_clock::now();
            auto last = start;
            int total = base.size();

            for (const auto &s : base.byshape) {
                auto &shape = s.first;
                int diffx = targetShape.x() - shape.x();
                int diffy = targetShape.y() - shape.y();
                int diffz = targetShape.z() - shape.z();
                int abssum = abs(diffx) + abs(diffy) + abs(diffz);
                if (abssum > 1 || diffx < 0 || diffy < 0 || diffz < 0) {
                    continue;
                }
                // handle symmetry cases
                if (diffz == 1) {
                    if (shape.z() == shape.y()) diffy = 1;
                }
                if (diffy == 1)
                    if (shape.y() == shape.x()) diffx = 1;
                std::printf("  shape %d %d %d\n\r", s.first.x(), s.first.y(), s.first.z());
                for (const auto &subset : s.second.byhash)
                    for (const auto &b : subset.set) {
                        expand(b, hashes, shape, XYZ(diffx, diffy, diffz), abssum);
                        count++;
                        if (count % PERF_STEP == (PERF_STEP - 1)) {
                            auto end = std::chrono::steady_clock::now();
                            auto total_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
                            auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(end - last).count();
                            last = end;
                            auto perc = 100 * count / total;
                            auto avg = 1000000.f * count / total_us;
                            auto its = 1000000.f * PERF_STEP / dt_us;
                            auto remaining = (total - count) / avg;
                            std::printf(" %3d%%, %5.0f avg baseCubes/s, %5.0f baseCubes/s, remaining: %.0fs\033[0K\r", perc, avg, its, remaining);
                            std::flush(std::cout);
                        }
                    }
            }
            auto end = std::chrono::steady_clock::now();
            auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
            std::printf("  took %.2f s\033[0K\n\r", dt_ms / 1000.f);
        } else {
            for (auto &s : base.byshape) {
                auto &shape = s.first;
                int diffx = targetShape.x() - shape.x();
                int diffy = targetShape.y() - shape.y();
                int diffz = targetShape.z() - shape.z();
                int abssum = abs(diffx) + abs(diffy) + abs(diffz);
                if (abssum > 1 || diffx < 0 || diffy < 0 || diffz < 0) {
                    continue;
                }
                std::vector<Cube> baseCubes;
                // handle symmetry cases
                if (diffz == 1) {
                    if (shape.z() == shape.y()) diffy = 1;
                }
                if (diffy == 1)
                    if (shape.y() == shape.x()) diffx = 1;
                // std::printf("converting to vector\n\r");
                std::printf("  shape %d %d %d\n\r", s.first.x(), s.first.y(), s.first.z());
                for (auto &subset : s.second.byhash) {
                    baseCubes.insert(baseCubes.end(), subset.set.begin(), subset.set.end());
                    // subset.set.clear();
                    // subset.set.reserve(1);
                }
                // std::cout << baseCubes.size() << " -- " << &*baseCubes.begin() << " to " << &*baseCubes.end() << "\n";
                // std::printf("starting %d threads\n\r", threads);
                std::vector<std::thread> ts;
                Workset ws(baseCubes, hashes, shape, XYZ(diffx, diffy, diffz), abssum);
                std::vector<Worker> workers;
                ts.reserve(threads);
                workers.reserve(threads);
                for (int i = 0; i < threads; ++i) {
                    workers.emplace_back(ws, i);
                    ts.emplace_back(&Worker::run, std::ref(workers[i]));
                }
                for (int i = 0; i < threads; ++i) {
                    ts[i].join();
                }
            }
        }
        std::printf("  num: %lu\n\r", hashes.byshape[targetShape].size());
        totalSum += hashes.byshape[targetShape].size();
        if (write_cache)
            Cache::save("cubes_" + std::to_string(n) + "_" + std::to_string(targetShape.x()) + "-" + std::to_string(targetShape.y()) + "-" +
                            std::to_string(targetShape.z()) + ".bin",
                        hashes, n);

        for (auto &subset : hashes.byshape[targetShape].byhash) {
            subset.set.clear();
            subset.set.reserve(1);
        }
    }
    std::printf("  num cubes: %lu\n\r", totalSum);
    if (sizeof(results) / sizeof(results[0]) > ((uint64_t)(n - 1)) && n > 1) {
        if (results[n - 1] != totalSum) {
            std::printf("ERROR: result does not equal resultstable (%lu)!\n\r", results[n - 1]);
            std::exit(-1);
        }
    }
    return hashes;
}