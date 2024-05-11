#include <deque>
#include <numeric>

class LowPassFilter {
public:
    LowPassFilter(double cutoff_frequency, int order) : order_(order) {
        // Initialize coefficients and initial condition
        // This is a simple implementation of a Butterworth filter
        // You may need to adjust this for your specific needs
        double rc = 1.0 / (cutoff_frequency * 2 * M_PI);
        double dt = 1.0 / cutoff_frequency;
        double alpha = dt / (rc + dt);
        b_.assign(order, alpha);
        a_.assign(order, 1 - alpha);
        zi_.assign(order, 0);
    }

    double filter(double value) {
        // Apply the low-pass filter
        double result = b_[0] * value + zi_[0];
        for (int i = 1; i < order_; i++) {
            zi_[i - 1] = b_[i] * value + a_[i] * zi_[i] - b_[i] * result;
        }
        return result;
    }

private:
    int order_;
    std::deque<double> b_, a_, zi_;
};