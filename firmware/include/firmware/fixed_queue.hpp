#pragma once
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>
namespace firmware {
template<class T,std::size_t Capacity> class FixedQueue {
 public:
  static_assert(Capacity>1 && std::is_trivially_copyable<T>::value,"固定队列需要可平凡复制数据");
  static_assert(std::atomic<uint32_t>::is_always_lock_free,"ISR队列要求32位原子无锁");
  static_assert((Capacity&(Capacity-1))==0 && Capacity<(uint64_t{1}<<31),"容量必须为2的幂，保证计数器回绕一致");
  // 每个队列恰好一个生产者和消费者；多个ISR/任务不得共用生产入口。
  bool push(const T& value) {
    const uint32_t head=head_.load(std::memory_order_relaxed);
    if(uint32_t(head-tail_.load(std::memory_order_acquire))>=Capacity){dropped_.fetch_add(1,std::memory_order_relaxed);return false;}
    entries_[head&(Capacity-1)]=value;head_.store(head+1,std::memory_order_release);return true;
  }
  bool pop(T& value) {
    const uint32_t tail=tail_.load(std::memory_order_relaxed);
    if(tail==head_.load(std::memory_order_acquire))return false;
    value=entries_[tail&(Capacity-1)];tail_.store(tail+1,std::memory_order_release);return true;
  }
  uint32_t dropped() const {return dropped_.load();}
  std::size_t size() const {return uint32_t(head_.load(std::memory_order_acquire)-tail_.load(std::memory_order_acquire));}
  // 仅在全部生产者/消费者停止时重置，不能与ISR或任务并发。
  void resetQuiescent(){head_.store(0);tail_.store(0);dropped_.store(0);}
 private: std::array<T,Capacity> entries_{}; std::atomic<uint32_t> head_{0},tail_{0},dropped_{0};
};
}
