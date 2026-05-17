// Unit test for the vector-norm step limiter (limitVectorNorm3).
//
// Regression: the 2026-05-17 field logs showed commanded_step_xyz_mm =
// ~38.1 mm with a configured max_step_xyz_mm = 22 mm. That value is
// exactly sqrt(22^2 + 22^2 + 22^2) ~ 38.1 mm, i.e. the bug was that
// each XYZ component was being clamped independently to 22 mm. The new
// limiter scales the WHOLE vector so its Euclidean norm never exceeds
// the configured cap.

#include "util/MathUtils.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace dgd;

static void expectClose(double a, double b, double tol, const char* label) {
    if (std::fabs(a - b) > tol) {
        std::fprintf(stderr, "FAIL %s: expected %.6f, got %.6f (tol=%.6f)\n",
                     label, b, a, tol);
        std::exit(1);
    }
    std::printf("  ok: %s = %.4f\n", label, a);
}

int main() {
    // 1. Vector strictly inside the cap is left untouched.
    {
        double a = 5.0, b = -3.0, c = 1.0;
        bool clipped = limitVectorNorm3(a, b, c, 22.0);
        assert(!clipped);
        expectClose(a, 5.0,  1e-9, "inside.a");
        expectClose(b, -3.0, 1e-9, "inside.b");
        expectClose(c, 1.0,  1e-9, "inside.c");
    }

    // 2. (22, 22, 22) with cap 22 must NOT pass through unchanged.
    //    Norm before = sqrt(3)*22 = 38.105... ; norm after must be 22.
    {
        double a = 22.0, b = 22.0, c = 22.0;
        bool clipped = limitVectorNorm3(a, b, c, 22.0);
        assert(clipped);
        const double norm = vec3Norm(a, b, c);
        expectClose(norm, 22.0, 1e-9, "(22,22,22).cap22.norm");
        // direction preserved -> all components equal
        expectClose(a, b, 1e-9, "(22,22,22).cap22.a==b");
        expectClose(b, c, 1e-9, "(22,22,22).cap22.b==c");
    }

    // 3. Negative components scaled the same way.
    {
        double a = -30.0, b = 0.0, c = 40.0; // norm = 50
        bool clipped = limitVectorNorm3(a, b, c, 25.0);
        assert(clipped);
        expectClose(vec3Norm(a, b, c), 25.0, 1e-9, "neg.norm");
        // direction: (-3, 0, 4) / 5 * 25 = (-15, 0, 20)
        expectClose(a, -15.0, 1e-9, "neg.a");
        expectClose(b,   0.0, 1e-9, "neg.b");
        expectClose(c,  20.0, 1e-9, "neg.c");
    }

    // 4. Zero vector is a no-op.
    {
        double a = 0.0, b = 0.0, c = 0.0;
        bool clipped = limitVectorNorm3(a, b, c, 22.0);
        assert(!clipped);
        expectClose(a, 0.0, 1e-9, "zero.a");
    }

    // 5. max_norm <= 0 disables the limiter (legitimate config sentinel).
    {
        double a = 100.0, b = 100.0, c = 100.0;
        bool clipped = limitVectorNorm3(a, b, c, 0.0);
        assert(!clipped);
        expectClose(a, 100.0, 1e-9, "disabled.a");
    }

    // 6. Direct vec3Norm sanity.
    expectClose(vec3Norm(3.0, 4.0, 0.0), 5.0, 1e-12, "norm.345");
    expectClose(vec3Norm(1.0, 2.0, 2.0), 3.0, 1e-12, "norm.122");

    std::printf("All vector-norm limiter tests passed.\n");
    return 0;
}
