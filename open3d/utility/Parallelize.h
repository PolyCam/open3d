#pragma once

#include <atomic>
#include <cassert>
#include <condition_variable>
#include <exception>
#include <mutex>
#include <thread>

namespace open3d {
namespace utility {

//! @param Functor a functor returning void taking in a Count parameter on which index to operate on that can be parallelized across
//! distinct indices.
//! @param count The number of indices to operate on starting with 0.
//! @brief Runs an operation across multiple indices that can be parallelized across all available CPU cores.
template <typename Functor, typename Count>
void Parallelize(Functor functor, Count count) {
  if (count == (Count)0) {
    return;
  }

  std::mutex protector;
  std::exception_ptr error;
  auto complete = std::atomic<bool>(false);
  std::condition_variable complete_signal;
  auto next = std::atomic<Count>(0);
  auto threads_completed_count = std::atomic<Count>(0);
  const auto threads_to_use = (Count)std::thread::hardware_concurrency();

  auto work = [&]() {
    try {
      for (auto current = next.fetch_add((Count)1); current < count; current = next.fetch_add((Count)1)) {
        if ((bool)complete) {  // An error occurred on another thread, abort.
          return;
        }
        functor(current);
      }
      const auto latest_threads_completed_count = threads_completed_count.fetch_add((Count)1) + (Count)1;
      if (latest_threads_completed_count == threads_to_use) {
        {
          const auto lock = std::scoped_lock<std::mutex>(protector);
          complete = true;
        }
        complete_signal.notify_one();
      }
    } catch (...) {
      {
        const auto lock = std::scoped_lock<std::mutex>(protector);
        complete = true;
        error = std::current_exception();
      }
      complete_signal.notify_one();
    }
  };
  std::vector<std::thread> threads;
  threads.reserve(threads_to_use);
  try {
    while (threads.size() < threads_to_use) {
      threads.push_back(std::thread(work));
    }
    auto lock = std::unique_lock<std::mutex>(protector);
    complete_signal.wait(lock, [&]() { return ((bool)complete); });
    if ((bool)error) {
      std::rethrow_exception(error);
    }
  } catch (...) {
    complete = true;
    for (auto &thread : threads) {
      thread.join();
    }
    throw;
  }
  for (auto &thread : threads) {
    thread.join();
  }
}

}  // namespace utility
}  // namespace open3d
