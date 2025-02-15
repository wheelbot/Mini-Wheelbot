/*
 * Copyright (c) 2016-2018, 2020, Niklas Hauser
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#pragma once

#include "assert.h"
#include <modm/architecture/interface/register.hpp>
namespace modm
{

/// @ingroup modm_architecture_assert
/// @{

/// Describes abandonment type of assertions.
enum class
Abandonment : uint8_t
{
	DontCare = Bit0,	///< Do not care about failure.
	Ignore = Bit1,		///< Ignore this failure.
	Fail = Bit2,		///< This failure is reason for abandonment.
	Debug = Bit7		///< Only set for a debug-only failure.
};
using AbandonmentBehavior = Flags8<Abandonment>;
MODM_TYPE_FLAGS(AbandonmentBehavior);

/// Contains information about the failed assertion.
struct modm_packed
AssertionInfo
{
	const char *name;				///< Can be used to recognize the assertion in code
#if MODM_ASSERTION_INFO_HAS_DESCRIPTION
	const char *description;		///< Detailed failure description
#endif
	uintptr_t context;				///< Optional context depends on assertion
	AbandonmentBehavior behavior;	///< Can this assertion be ignored?
};

/// Signature of the assertion handlers
using AssertionHandler = Abandonment (*)(const AssertionInfo &info);

/// @}

} // namespace modm


#ifdef __DOXYGEN__

/// @ingroup modm_architecture_assert
/// @{

/// Declares whether or not AssertionInfo has a `description` field.
#define MODM_ASSERTION_INFO_HAS_DESCRIPTION 0/1

/**
 * This adds a function to the list of assertion handlers to execute on
 * assertion failure. Note that this macro does not give you any influence
 * over the order of handler execution on assertion failure.
 * Do not write assertion handlers that depend on any ordered execution!
 *
 * @param handler A function of signature `AssertionHandler`.
 */
#define MODM_ASSERTION_HANDLER(handler)

/**
 * This adds an assertion handler only for debug mode.
 *
 * @param handler A function of signature `AssertionHandler`.
 */
#define MODM_ASSERTION_HANDLER_DEBUG(handler)

/**
 * Always abandons execution
 *
 * @note This assert is included in all builds!
 */
[[noreturn]] void
modm_assert(bool condition,
			const char *name, const char *description,
			uintptr_t context=uintptr_t(-1));

/**
 * Abandons execution, unless overwritten by assertion handlers to resume.
 *
 * @returns result of condition evaluation
 * @note This assert is included in all builds!
 */
bool
modm_assert_continue_fail(bool condition,
						  const char *name, const char *description,
						  uintptr_t context=uintptr_t(-1));

/**
 * Resumes execution, unless overwritten by assertion handlers to abandon.
 *
 * @returns result of condition evaluation
 * @note This assert is included in all builds!
 */
bool
modm_assert_continue_ignore(bool condition,
							const char *name, const char *description,
							uintptr_t context=uintptr_t(-1));

/**
 * Abandons execution, unless overwritten by assertion handlers to resume.
 *
 * @note This assert is only included debug builds, however the condition is
 *       still executed and the result is returned!
 * @returns result of condition evaluation
 */
bool
modm_assert_continue_fail_debug(bool condition,
								const char *name, const char *description,
								uintptr_t context=uintptr_t(-1));

/**
 * Resumes execution, unless overwritten by assertion handlers to abandon.
 *
 * @note This assert is only included debug builds, however the condition is
 *       still executed and the result is returned!
 * @returns result of condition evaluation
 */
bool
modm_assert_continue_ignore_debug(bool condition,
								  const char *name, const char *description,
								  uintptr_t context=uintptr_t(-1));

/// @}

#else

#ifdef MODM_DEBUG_BUILD
#define MODM_ASSERTION_HANDLER_DEBUG(handler) \
		MODM_ASSERTION_HANDLER(handler)
#else
#define MODM_ASSERTION_HANDLER_DEBUG(handler) \
		static const modm::AssertionHandler [[maybe_unused]] \
		handler ## _assertion_handler_ptr = handler
#endif

#endif // __DOXYGEN__

/**
 * Overwriteable abandonment handler for all targets.
 *
 * You should overwrite this handler for custom failure behaviour like blinking
 * LEDs and logging the failure via a serial connection.
 *
 * @ingroup modm_architecture_assert
 */
modm_extern_c void
modm_abandon(const modm::AssertionInfo &info) modm_weak;


// Core must implement MODM_ASSERTION_HANDLER(handler)
#include <modm/platform/core/assert_impl.hpp>
