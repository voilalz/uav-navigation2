#pragma once
#include "nav/eskf.hpp"
namespace nav {
template <std::size_t Capacity = 256> class StateHistory {
  std::array<NominalState, Capacity> states_{};
  std::size_t start_{}, size_{};

public:
  void clear() { start_ = size_ = 0; }
  void push(const NominalState &s) {
    if (size_ && s.t_us <= states_[(start_ + size_ - 1) % Capacity].t_us)
      return;
    if (size_ == Capacity) {
      start_ = (start_ + 1) % Capacity;
      --size_;
    }
    states_[(start_ + size_++) % Capacity] = s;
  }
  bool sample(Timestamp t, NominalState &out) const {
    if (!size_ || t < states_[start_].t_us ||
        t > states_[(start_ + size_ - 1) % Capacity].t_us)
      return false;
    for (std::size_t i = 0; i < size_; ++i) {
      const auto &a = states_[(start_ + i) % Capacity];
      if (a.t_us == t) {
        out = a;
        return true;
      }
      if (i + 1 < size_) {
        const auto &b = states_[(start_ + i + 1) % Capacity];
        if (t < a.t_us || t > b.t_us)
          continue;
        const double u = double(t - a.t_us) / double(b.t_us - a.t_us);
        out = a;
        offsetPosition(out, displacementNed(a, b) * u);
        out.velocity_ned = a.velocity_ned * (1 - u) + b.velocity_ned * u;
        out.q_nb = interpolate(a.q_nb, b.q_nb, u);
        out.t_us = t;
        return true;
      }
    }
    return false;
  }
  // 用当前误差修正重基准所有历史状态；仍是短延迟近似，不是完整 OOSM。
  void rebase(const ErrorVector &d) {
    for (std::size_t i = 0; i < size_; ++i)
      injectError(states_[(start_ + i) % Capacity], d);
  }
  std::size_t size() const { return size_; }
  const NominalState *latest() const {
    return size_ ? &states_[(start_ + size_ - 1) % Capacity] : nullptr;
  }
};
} // namespace nav
