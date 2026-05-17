#include "motion/IMotionBackend.hpp"

#include <algorithm>
#include <cctype>
#include <string>

namespace dgd {

const char* motionBackendName(MotionBackendKind k) {
    switch (k) {
        case MotionBackendKind::Amovel: return "amovel";
        case MotionBackendKind::Servol: return "servol";
    }
    return "?";
}

MotionBackendKind parseMotionBackendKind(const std::string& s, bool* ok) {
    std::string t = s;
    std::transform(t.begin(), t.end(), t.begin(),
                   [](unsigned char c) { return std::tolower(c); });
    if (ok) *ok = true;
    if (t == "amovel") return MotionBackendKind::Amovel;
    if (t == "servol") return MotionBackendKind::Servol;
    if (ok) *ok = false;
    return MotionBackendKind::Amovel; // safe fallback
}

} // namespace dgd
