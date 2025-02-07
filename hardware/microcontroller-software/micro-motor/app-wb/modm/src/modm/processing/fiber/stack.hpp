/*
 * Copyright (c) 2020, Erik Henriksson
 * Copyright (c) 2021, 2023, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once
#include "context.h"
#include <cstdint>
#include <cstddef>
#include <modm/architecture/utils.hpp>

namespace modm::fiber
{

/// @ingroup modm_processing_fiber
/// @{

/// Alignment requirements for the bottom and top of the stack.
static constexpr size_t StackAlignment = sizeof(uintptr_t) * 2;
/// Minimum stack size required to push one full fiber context.
static constexpr size_t StackSizeMinimum = 108;
/// The default stack size is estimated experimentally so that a fiber can use
/// `modm::IOStream` to log out information, which is fairly stack intensive.
/// Use `modm::fiber::Task::stack_usage()` to determine the real stack usage.
static constexpr size_t StackSizeDefault = 1024;

/**
 * Stack captures a memory area used as fiber stack with alignment and minimal
 * size requirements. The stack is stored in the `.bss` section by default, but
 * may be placed in a `.noinit` section using `modm_section(".noinit")`.
 * For available sections on your device, see `modm:platform:core`.
 *
 * @author	Erik Henriksson
 */
template< size_t Size = StackSizeDefault >
class Stack
{
	friend class Task;
	friend class Scheduler;
	Stack(const Stack&) = delete;
	Stack& operator=(const Stack&) = delete;

	static_assert(Size % StackAlignment == 0, "Stack size is not aligned!");
	static_assert(Size >= StackSizeMinimum, "Stack is too small!");

	static constexpr size_t size = Size;
	static constexpr size_t words = Size / sizeof(uintptr_t);
	alignas(StackAlignment)
	uintptr_t memory[words];

public:
	constexpr Stack() = default;
};
/// @}

} // namespace modm::fiber
