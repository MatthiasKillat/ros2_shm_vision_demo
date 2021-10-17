#pragma once

#include <atomic>
#include <condition_variable>
#include <mutex>

namespace demo {
// part of an ad hoc solution to simulate keep last 1
// without heavy computation in the callback
template <class T> class ExchangeBuffer {
public:
  bool try_write(T *value) {
    T *expected = nullptr;
    if (m_data.compare_exchange_strong(expected, value)) {
      return true;
    }
    return false;
  }

  T *write(T *value) {
    // T *oldValue = m_data.load();
    // while (true) {
    //   if (m_data.compare_exchange_strong(oldValue, value)) {
    //     return oldValue;
    //   }
    // }
    // return nullptr;
    return m_data.exchange(value);
  }

  T *take() { return m_data.exchange(nullptr); }

  bool empty() { return m_data.load() == nullptr; }

  bool has_data() { return m_data.load() != nullptr; }

private:
  std::atomic<T *> m_data{nullptr};
};

class ProtectedConditionVariable {
public:
  void lock() { m_mutex.lock(); }

  void unlock() { m_mutex.unlock(); }

  template <typename Predicate> void wait(Predicate predicate) {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_condvar.wait(lock, predicate);
  }

  void notify_one() { m_condvar.notify_one(); }

  void notify_all() { m_condvar.notify_all(); }

private:
  std::mutex m_mutex;
  std::condition_variable m_condvar;
};

} // namespace demo