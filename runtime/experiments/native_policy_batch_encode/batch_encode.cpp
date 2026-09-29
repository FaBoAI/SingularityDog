// Offline candidate only. No device handles, syscalls, or motor output.
// Build with -ffp-contract=off so Python's left-to-right double operations
// remain byte-identical at quantization boundaries.
#include <cmath>
#include <cstdint>
#include <cstring>

extern "C" {

struct AxisSpec {
    double offset;
    double sign;
    double lower;
    double upper;
    double initial;
    double max_displacement;
    double max_estimated_pd;
};

enum Failure : int32_t {
    OK = 0,
    INVALID_ARGUMENT = 1,
    NONFINITE_Q = 2,
    NONFINITE_KP = 3,
    NONFINITE_KD = 4,
    SOFTWARE_CAP = 5,
    QUANTIZED_PHYSICAL_RANGE = 6,
    DISPLACEMENT = 7,
    ESTIMATED_PD = 8,
};

// Output is published only after all 12 axes pass. first_id is 1..12 on
// failure, zero on success. All input arrays have exactly 12 elements.
int32_t sdbe_encode(const AxisSpec* axes, const double* model_q,
                    const double* kp, const double* kd,
                    const double* estimated_pd, uint8_t* output204,
                    int32_t* first_id) {
    if (!axes || !model_q || !kp || !kd || !estimated_pd || !output204 || !first_id)
        return INVALID_ARGUMENT;
    *first_id = 0;
    uint8_t local[12 * 17];
    uint16_t position[12];
    for (int k = 0; k < 12; ++k) {
        const AxisSpec& a = axes[k];
        const int32_t mid = k + 1;
        *first_id = mid;
        if (!(a.sign == 1.0 || a.sign == -1.0) ||
            !std::isfinite(a.offset) || !std::isfinite(a.lower) ||
            !std::isfinite(a.upper) || !std::isfinite(a.initial) ||
            !std::isfinite(a.max_displacement) || !std::isfinite(a.max_estimated_pd))
            return INVALID_ARGUMENT;
        const double raw = (model_q[k] - a.offset) / a.sign;
        if (!std::isfinite(raw)) return NONFINITE_Q;
        if (!std::isfinite(kp[k])) return NONFINITE_KP;
        if (!std::isfinite(kd[k])) return NONFINITE_KD;
        if (!(raw >= -12.57 && raw <= 12.57 &&
              kp[k] >= 0.0 && kp[k] <= 36.0 &&
              kd[k] >= 0.0 && kd[k] <= 1.0))
            return SOFTWARE_CAP;
        const uint16_t p = static_cast<uint16_t>((raw + 12.57) * 65535.0 / 25.14);
        const uint16_t pg = static_cast<uint16_t>(kp[k] * 65535.0 / 500.0);
        const uint16_t dg = static_cast<uint16_t>(kd[k] * 65535.0 / 5.0);
        position[k] = p;
        const uint32_t can_id = (1u << 24) | (32767u << 8) | static_cast<uint32_t>(mid);
        const uint32_t at_id = (can_id << 3) | 4u;
        uint8_t* wire = local + 17 * k;
        wire[0] = 'A'; wire[1] = 'T';
        wire[2] = static_cast<uint8_t>(at_id >> 24);
        wire[3] = static_cast<uint8_t>(at_id >> 16);
        wire[4] = static_cast<uint8_t>(at_id >> 8);
        wire[5] = static_cast<uint8_t>(at_id);
        wire[6] = 8;
        wire[7] = static_cast<uint8_t>(p >> 8); wire[8] = static_cast<uint8_t>(p);
        wire[9] = 0x7f; wire[10] = 0xff;
        wire[11] = static_cast<uint8_t>(pg >> 8); wire[12] = static_cast<uint8_t>(pg);
        wire[13] = static_cast<uint8_t>(dg >> 8); wire[14] = static_cast<uint8_t>(dg);
        wire[15] = '\r'; wire[16] = '\n';
    }
    // Match the Python caller: all twelve encode_motion calls precede every
    // parsed, quantized physical-envelope check.
    for (int k = 0; k < 12; ++k) {
        const AxisSpec& a = axes[k];
        *first_id = k + 1;
        const double raw = static_cast<double>(position[k]) * 25.14 / 65535.0 - 12.57;
        const double q = a.sign * raw + a.offset;
        if (!(a.lower <= q && q <= a.upper)) return QUANTIZED_PHYSICAL_RANGE;
        if (!(std::fabs(q - a.initial) <= a.max_displacement)) return DISPLACEMENT;
        const double estimated = kp[k] * (q - model_q[k]) + estimated_pd[k];
        if (!(std::fabs(estimated) <= a.max_estimated_pd)) return ESTIMATED_PD;
    }
    std::memcpy(output204, local, sizeof(local));
    *first_id = 0;
    return OK;
}

}  // extern "C"
