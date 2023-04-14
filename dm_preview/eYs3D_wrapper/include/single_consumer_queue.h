// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.
//
// Modified by eys3d alanlin on 2020/11/19.


#include <queue>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <functional>
#include <sstream>

#ifndef EYS3D_UNITY_WRAPPER_SINGLE_CONSUMER_QUEUE_H
#define EYS3D_UNITY_WRAPPER_SINGLE_CONSUMER_QUEUE_H

const int QUEUE_MAX_SIZE = 10;
// Simplest implementation of a blocking concurrent queue for thread messaging
template<class T>
class single_consumer_queue
{
    std::deque<T> _queue;
    std::mutex _mutex;
    std::condition_variable _deq_cv; // not empty signal
    std::condition_variable _enq_cv; // not empty signal

    unsigned int _cap;
    bool _accepting;

    // flush mechanism is required to abort wait on cv
    // when need to stop
    std::atomic<bool> _need_to_flush;
    std::atomic<bool> _was_flushed;
public:
    explicit single_consumer_queue<T>(unsigned int cap = QUEUE_MAX_SIZE)
            : _queue(), _mutex(), _deq_cv(), _enq_cv(), _cap(cap), _accepting(true), _need_to_flush(false), _was_flushed(false)
    {}
    T enqueue_pop_wasted(T&& item) {
        std::unique_lock<std::mutex> lock(_mutex);
        if (_accepting)
        {
            _queue.push_back(std::move(item));
            if (_queue.size() > _cap)
            {
                T wasted = _queue.front();
                _queue.pop_front();
                if (wasted) {
                    return wasted;
                }
                return nullptr;
            }
        }
        return nullptr;
    }

    void enqueue(T&& item)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        if (_accepting)
        {
            _queue.push_back(std::move(item));
            if (_queue.size() > _cap)
            {
                _queue.pop_front();
            }
        }
        lock.unlock();
        _deq_cv.notify_one();
    }

    void blocking_enqueue(T&& item)
    {
        auto pred = [this]()->bool { return _queue.size() < _cap || _need_to_flush; };

        std::unique_lock<std::mutex> lock(_mutex);
        if (_accepting)
        {
            _enq_cv.wait(lock, pred);
            _queue.push_back(std::move(item));
        }
        lock.unlock();
        _deq_cv.notify_one();
    }


    bool dequeue(T* item ,unsigned int timeout_ms)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _accepting = true;
        _was_flushed = false;
        const auto ready = [this]() { return (_queue.size() > 0) || _need_to_flush; };
        if (!ready() && !_deq_cv.wait_for(lock, std::chrono::milliseconds(timeout_ms), ready))
        {
            return false;
        }

        if (_queue.size() <= 0)
        {
            return false;
        }
        *item = std::move(_queue.front());
        _queue.pop_front();
        _enq_cv.notify_one();
        return true;
    }

    bool try_dequeue(T* item)
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _accepting = true;
        if (_queue.size() > 0)
        {
            auto val = std::move(_queue.front());
            _queue.pop_front();
            *item = std::move(val);
            _enq_cv.notify_one();
            return true;
        }
        return false;
    }

    bool peek(T** item)
    {
        std::unique_lock<std::mutex> lock(_mutex);

        if (_queue.size() <= 0)
        {
            return false;
        }
        *item = &_queue.front();
        return true;
    }

    void clear()
    {
        std::unique_lock<std::mutex> lock(_mutex);

        _accepting = false;
        _need_to_flush = true;

        _enq_cv.notify_all();
        while (_queue.size() > 0)
        {
            auto item = std::move(_queue.front());
            _queue.pop_front();
        }
        _deq_cv.notify_all();
    }

    void start()
    {
        std::unique_lock<std::mutex> lock(_mutex);
        _need_to_flush = false;
        _accepting = true;
    }

    size_t size()
    {
        std::unique_lock<std::mutex> lock(_mutex);
        return _queue.size();
    }
};

#endif //EYS3D_UNITY_WRAPPER_SINGLE_CONSUMER_QUEUE_H
