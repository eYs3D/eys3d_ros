// Copyright (C) 2014 The Android Open Source Project
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Copyright (C) 2021 eYs3D Corporation
 * All rights reserved.
 * This project is licensed under the Apache License, Version 2.0.
 */

#pragma once

#include "utils.h"
#include "debug.h"
#include "base/Compiler.h"
#include "base/synchronization/Lock.h"

#ifdef _WIN32
#include <windows.h>
#include <algorithm>
#else
#include <pthread.h>
#endif

#include <inttypes.h>
#include <assert.h>

namespace libeYs3D    {
namespace base {

// A class that implements a condition variable, which can be used in
// association with a Lock to blocking-wait for specific conditions.
// Useful to implement various synchronization data structures.
class ConditionVariable {
public:
    // A set of functions to efficiently unlock the lock used with
    // the current condition variable and signal or broadcast it.
    //
    // The functions are needed because on some platforms (Posix) it's more
    // efficient to signal the variable before unlocking mutex, while on others
    // (Windows) it's exactly the opposite. Functions implement the best way
    // for each platform and abstract it out from the user.
    void signalAndUnlock(StaticLock* lock);
    void signalAndUnlock(AutoLock* lock);
    void signalAndUnlock(AutoWriteLock* lock);
	void signalAndUnlock(AutoReadLock* lock);

    void broadcastAndUnlock(StaticLock* lock);
    void broadcastAndUnlock(AutoLock* lock);
	void broadcastAndUnlock(AutoWriteLock* lock);
	void broadcastAndUnlock(AutoReadLock* lock);

    void wait(AutoLock* userLock) {
        assert(userLock->mLocked);
        wait(&userLock->mLock);
    }

    //
    // Convenience functions to get rid of the loop in condition variable usage
    // Instead of hand-writing a loop, e.g.
    //
    //      while (mRefCount < 3) {
    //          mCv.wait(&mLock);
    //      }
    //
    // use the following two wait() overloads:
    //
    //      mCv.wait(&mLock, [this]() { return mRefCount >= 3; });
    //
    // Parameters:
    // |lock| - a Lock or AutoLock pointer used with the condition variable.
    // |pred| - a functor predicate that's compatible with "bool pred()"
    //          signature and returns a condition when one should stop waiting.
    //

    template <class Predicate>
    void wait(StaticLock* lock, Predicate pred) {
        while (!pred()) {
            this->wait(lock);
        }
    }

    template <class Predicate>
    void wait(AutoLock* lock, Predicate pred) {
        this->wait(&lock->mLock, pred);
    }

	
    template <class Predicate>
    void wait(ReadWriteLock* lock, Predicate pred) {
        this->wait(&lock->mLock, pred);
    }

#ifdef _WIN32

    ConditionVariable() {
        ::InitializeConditionVariable(&mCond);
    }

    // There's no special function to destroy CONDITION_VARIABLE in Windows.
    ~ConditionVariable() = default;

    // Wait until the condition variable is signaled. Note that spurious
    // wakeups are always a possibility, so always check the condition
    // in a loop, i.e. do:
    //
    //    while (!condition) { condVar.wait(&lock); }
    //
    // instead of:
    //
    //    if (!condition) { condVar.wait(&lock); }
    //
    void wait(StaticLock* userLock) {
        ::SleepConditionVariableSRW(&mCond, &userLock->mLock, INFINITE, 0);
    }

    bool timedWait(StaticLock *userLock, int64_t waitUntilUs) {
        const auto now = now_in_microsecond_unix_time();
        const auto timeout =
                std::max<int64_t>(0, waitUntilUs  - now) / 1000;
        return ::SleepConditionVariableSRW(
                    &mCond, &userLock->mLock, timeout, 0) != 0;
    }

    bool timedWaitDebug(StaticLock* userLock, int64_t waitUntilUs) {
        const auto now = now_in_microsecond_unix_time();
        const auto timeout =
                std::max<int64_t>(0, waitUntilUs  - now) / 1000;
        return ::SleepConditionVariableSRW(
                    &mCond, &userLock->mLock, timeout, 0) != 0;
    }

    bool timedWaitDebug(ReadWriteLock* userLock, int64_t waitUntilUs) {
        const auto now = now_in_microsecond_unix_time();
        const auto timeout =
                std::max<int64_t>(0, waitUntilUs  - now) / 1000;
        return ::SleepConditionVariableSRW(
                    &mCond, &userLock->mLock, timeout, 0) != 0;
    }

    // Signal that a condition was reached. This will wake at least (and
    // preferrably) one waiting thread that is blocked on wait().
    void signal() {
        ::WakeConditionVariable(&mCond);
    }

    // Like signal(), but wakes all of the waiting threads.
    void broadcast() {
        ::WakeAllConditionVariable(&mCond);
    }

private:
    CONDITION_VARIABLE mCond;

#else  // !_WIN32

    // Note: on Posix systems, make it a naive wrapper around pthread_cond_t.

    ConditionVariable() {
        pthread_cond_init(&mCond, NULL);
    }

#if 0 
    ConditionVariable() {
        pthread_condattr_t attr;
        
        pthread_condattr_init(&attr);
        pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
        pthread_cond_init(&mCond, &attr);
    }
#endif

    ~ConditionVariable() {
        pthread_cond_destroy(&mCond);
    }

    void wait(StaticLock* userLock) {
        pthread_cond_wait(&mCond, &userLock->mLock);
    }

    bool timedWait(StaticLock* userLock, int64_t waitUntilUs) {
        timespec abstime;
        abstime.tv_sec = waitUntilUs / 1000000LL;
        abstime.tv_nsec = (waitUntilUs % 1000000LL) * 1000;
        return pthread_cond_timedwait(&mCond, &userLock->mLock, &abstime) == 0;
    }

    bool timedWaitDebug(StaticLock* userLock, int64_t waitUntilUs) {
        timespec abstime;
        abstime.tv_sec = waitUntilUs / 1000000LL;
        abstime.tv_nsec = (waitUntilUs % 1000000LL) * 1000;
        {   
            int ret = 0;
            
            int64_t now2, now = now_in_microsecond_high_res_time_REALTIME();
            ret = pthread_cond_timedwait(&mCond, &userLock->mLock, &abstime);
            if(ret != 0)    {
                switch (ret)    {
                case ETIMEDOUT:
                    now2 = now_in_microsecond_high_res_time_REALTIME();
                    LOG_WARN("ConditionVariable",
                            "The time specified by abstime to pthread_cond_timedwait() has passed."
                            "waitUntilUs(%" PRId64 ") : in(%" PRId64 ") : now(%" PRId64 ")",
                            waitUntilUs, now, now2);
                    if(now2 > waitUntilUs)    return true;
                    break;
                case EINVAL:
                    LOG_ERR("ConditionVariable",
                            "The value specified by abstime, cond or mutex is invalid.");
                    break;
                case EPERM:
                    LOG_ERR("ConditionVariable",
                            "The mutex was not owned by the current thread at the time of the call.");
                    break;
                default:
                    LOG_ERR("ConditionVariable", "%s...", strerror(ret));
                    break;
                }
            }
            
            return ret == 0;
        }
    }

    void signal() {
        pthread_cond_signal(&mCond);
    }

    void broadcast() {
        pthread_cond_broadcast(&mCond);
    }

private:
    pthread_cond_t mCond;

#endif  // !_WIN32

    DISALLOW_COPY_ASSIGN_AND_MOVE(ConditionVariable);
};

#ifdef _WIN32
inline void ConditionVariable::signalAndUnlock(StaticLock* lock) {
    lock->unlock();
    signal();
}
inline void ConditionVariable::signalAndUnlock(AutoLock* lock) {
    lock->unlock();
    signal();
}
inline void ConditionVariable::signalAndUnlock(AutoWriteLock* lock) {
    lock->unlockWrite();
    signal();
}
inline void ConditionVariable::signalAndUnlock(AutoReadLock* lock) {
    lock->unlockRead();
    signal();
}
inline void ConditionVariable::broadcastAndUnlock(StaticLock* lock) {
    lock->unlock();
    broadcast();
}
inline void ConditionVariable::broadcastAndUnlock(AutoLock* lock) {
    lock->unlock();
    broadcast();
}
inline void ConditionVariable::broadcastAndUnlock(AutoWriteLock* lock) {
    lock->unlockWrite();
    broadcast();
}
inline void ConditionVariable::broadcastAndUnlock(AutoReadLock* lock) {
    lock->unlockRead();
    broadcast();
}
#else  // !_WIN32
inline void ConditionVariable::signalAndUnlock(StaticLock* lock) {
    signal();
    lock->unlock();
}
inline void ConditionVariable::signalAndUnlock(AutoLock* lock) {
    signal();
    lock->unlock();
}
inline void ConditionVariable::signalAndUnlock(AutoWriteLock* lock) {
    signal();
    lock->unlockWrite();
}
inline void ConditionVariable::signalAndUnlock(AutoReadLock* lock) {
    signal();
    lock->unlockRead();
}
inline void ConditionVariable::broadcastAndUnlock(StaticLock* lock) {
    broadcast();
    lock->unlock();
}
inline void ConditionVariable::broadcastAndUnlock(AutoLock* lock) {
    broadcast();
    lock->unlock();
}
inline void ConditionVariable::broadcastAndUnlock(AutoWriteLock* lock) {
    broadcast();
    lock->unlockWrite();
}
inline void ConditionVariable::broadcastAndUnlock(AutoReadLock* lock) {
    broadcast();
    lock->unlockRead();
}
#endif  // !_WIN32

}  // namespace base
}  // namespace libeYs3D
