#pragma once

#include <sstream>
#include <string>

namespace tomobo {

struct Gaussian {
    double mean;
    double variance;

    std::string str() const {
        std::ostringstream os;
        os << "Gaussian(mean=" << std::to_string(mean)
           << ", variance=" << std::to_string(variance)
           << ")";
        return os.str();
    };

    bool operator==(const Gaussian& rhs) const {
        return mean == rhs.mean && variance == rhs.variance;
    };

    bool operator!=(const Gaussian& rhs) const {
        return !(*this == rhs);
    };

    Gaussian& operator*=(const Gaussian& rhs) {
        mean = (variance * rhs.mean + rhs.variance * mean) / (variance + rhs.variance);
        variance = (variance * rhs.variance) / (variance + rhs.variance);

        return *this;
    }

    friend Gaussian operator*(Gaussian lhs, const Gaussian& rhs) {
        lhs *= rhs;
        return lhs;
    }

    Gaussian& operator+=(const Gaussian& rhs) {
        mean += rhs.mean;
        variance += rhs.mean;

        return *this;
    }

    friend Gaussian operator+(Gaussian lhs, const Gaussian& rhs) {
        lhs += rhs;
        return lhs;
    }
};

}