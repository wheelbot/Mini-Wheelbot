/*
 * Copyright (c) 2020, Erik Henriksson
 * Copyright (c) 2022, Andrey Kunitsyn
 * Copyright (c) 2023, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#ifndef MODM_FIBER_SCHEDULER_HPP
#define MODM_FIBER_SCHEDULER_HPP

#include "task.hpp"
#include <modm/architecture/interface/assert.hpp>
#include <modm/platform/device.hpp>
namespace modm::fiber
{

/**
 * The scheduler executes fibers in a simple round-robin fashion. Fibers can be
 * added to a scheduler using the `modm::fiber::Task::start()` function, also
 * while the scheduler is running. Fibers returning from their function will
 * automatically unschedule themselves.
 *
 * @ingroup modm_processing_fiber
 */
class Scheduler
{
	friend class Task;
	friend void modm::this_fiber::yield();
	friend modm::fiber::id modm::this_fiber::get_id();
	Scheduler(const Scheduler&) = delete;
	Scheduler& operator=(const Scheduler&) = delete;

protected:
	Task* last{nullptr};
	Task* current{nullptr};

	uintptr_t inline
	get_id() const
	{
		// Ensure that calling this in an interrupt gives a different ID
		if (const auto irq = __get_IPSR(); irq >= 16) return irq;
		return reinterpret_cast<uintptr_t>(current);
	}

	static bool inline
	isInsideInterrupt()
	{
		return __get_IPSR();
	}

	void inline
	runNext(Task* task)
	{
		task->next = current->next;
		current->next = task;
	}

	void inline
	runLast(Task* task)
	{
		task->next = last->next;
		last->next = task;
		last = task;
	}

	inline Task*
	removeCurrent()
	{
		if (current == last) last = nullptr;
		else last->next = current->next;
		current->next = nullptr;
		current->scheduler = nullptr;
		return current;
	}

	bool inline
	empty() const
	{
		return last == nullptr;
	}

	void inline
	jump(Task* other)
	{
		auto from = current;
		current = other;
		modm_context_jump(&from->ctx, &other->ctx);
	}

	void inline
	yield()
	{
		if (current == nullptr) return;
		Task* next = current->next;
		// If there's only one fiber running, we could just return here.
		// However, we need to check the stack for overflow.
		// We do that by running the context switch!
		// if (next == current) return;
		last = current;
		jump(next);
	}

	[[noreturn]]
	void inline
	unschedule()
	{
		Task* next = current->next;
		removeCurrent();
		if (empty())
		{
			current = nullptr;
			modm_context_end(0);
		}
		jump(next);
		__builtin_unreachable();
	}

	void inline
	add(Task* task)
	{
		task->scheduler = this;
		if (last == nullptr)
		{
			task->next = task;
			last = task;
			return;
		}
		runLast(task);
	}

	bool inline
	start()
	{
		if (empty()) return false;
		current = last->next;
		const auto overflow = (Task *) modm_context_start(&current->ctx);
		modm_assert(not overflow, "fbr.stkof", "Fiber stack overflow", overflow);
		return true;
	}

protected:
	/// Returns the currently active scheduler.
	static inline Scheduler&
	instance()
	{
		static constinit Scheduler main;
		return main;
	}

public:
	constexpr Scheduler() = default;

	static constexpr unsigned int
	hardware_concurrency()
	{
		return 1;
	}

	/// Runs the currently active scheduler.
	static inline void
	run()
	{
		instance().start();
	}
};

} // namespace modm::fiber

#endif // MODM_FIBER_SCHEDULER_HPP
