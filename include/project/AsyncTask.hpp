#pragma once
#include <atomic>
#include <memory>

#include "main.h"

class AsyncTask {
public:
    template <class F>
    AsyncTask(F&& function) {
        static_assert(std::is_invocable_r_v<void, F>);
        task = std::make_unique<pros::Task>([&] {
            function();
            done = true;
        });
    };

    void wait() {
        while (!done.load())
            pros::delay(10);
    }

    bool isDone() {
        return done.load();
    }
    void cancel() {
        task->remove();
    }
    std::atomic_bool done{ false };

    std::unique_ptr<pros::Task> task;
};