#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
namespace nav {
// 每个队列恰好一个生产者和一个消费者；跨多个任务必须分别建队列。
template <class T, std::size_t Capacity> class SpscQueue {
  static_assert(Capacity > 0, "nonzero capacity");
  std::array<T, Capacity + 1> data_{};
  std::atomic<std::size_t> head_{0}, tail_{0};
  std::atomic<std::uint32_t> dropped_{0};

public:
  bool push(const T &x) {
    const auto h = head_.load(std::memory_order_relaxed),
               next = (h + 1) % (Capacity + 1);
    if (next == tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      return false;
    }
    data_[h] = x;
    head_.store(next, std::memory_order_release);
    return true;
  }
  bool pop(T &x) {
    const auto t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire))
      return false;
    x = data_[t];
    tail_.store((t + 1) % (Capacity + 1), std::memory_order_release);
    return true;
  }
  std::uint32_t dropped() const {
    return dropped_.load(std::memory_order_relaxed);
  }
};
} // namespace nav
