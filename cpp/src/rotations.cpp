#include "rotations.hpp"

#include <array>
#include <vector>

#include "cube.hpp"

std::pair<XYZ, bool> Rotations::rotate(int i, XYZ shape, const Cube &orig, Cube &dest) {
    const auto L = LUT[i];
    XYZ out_shape{shape[L[0]], shape[L[1]], shape[L[2]]};
    if (out_shape.x() > out_shape.y() || out_shape.y() > out_shape.z()) {
        return {out_shape, false};  // return here because violating shape
    }
    auto put = dest.begin();
    for (const auto &o : orig) {
        std::array<int8_t,3> n;
        if (L[3] < 0)
            n[0] = shape[L[0]] - o[L[0]];
        else
            n[0] = o[L[0]];

        if (L[4] < 0)
             n[1] = shape[L[1]] - o[L[1]];
        else
             n[1] = o[L[1]];

        if (L[5] < 0)
             n[2] = shape[L[2]] - o[L[2]];
        else
             n[2] = o[L[2]];
        *put++ = XYZ(n);
    }
    return {out_shape, true};
}
