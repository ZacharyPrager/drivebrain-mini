#pragma once

#include <deque>
#include <mutex>
#include <condition_variable>

namespace core
{
    template <typename MsgType>
    struct ThreadSafeDeque
    {
        std::deque<MsgType> deque; // Data structure for easily removing / adding to both ends of array
        std::mutex mtx; // Only one thread can read / write from the deque at a time
        std::condition_variable cv; // Consumer sleeps until Producer notifies it with this
    };
}
