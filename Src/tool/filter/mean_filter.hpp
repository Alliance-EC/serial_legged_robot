#pragma once

#include <deque>
#include <limits>

namespace tool::filter {

class MeanFilter {
public:
    explicit MeanFilter(size_t _filter_size) { filter_size = _filter_size; }

    double update(double input_value) {
        queue_.push_back(input_value);
        while (queue_.size() > filter_size)
            queue_.pop_front();

        double sum = 0;
        for (auto& item : queue_)
            sum += item;

        return sum / static_cast<double>(queue_.size());
    }

private:
    double input_value_ = std::numeric_limits<double>::quiet_NaN();
    size_t filter_size;
    std::deque<double> queue_;
};

} // namespace tool::filter