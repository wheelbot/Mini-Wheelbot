/* polymorphic_storage.hpp
 *
 * Copyright (C) 2019 Christopher Durand
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#ifndef POLYMORPHIC_STORAGE_HPP
#define POLYMORPHIC_STORAGE_HPP

#include <variant>
#include <type_traits>

namespace librobots2::motor
{

/// std::variant wrapper for types of a common base class
/// It provides convenient access to the base interface through operator->
template<typename Base, typename... Ts>
class PolymorphicStorage
{
public:
	PolymorphicStorage();

	PolymorphicStorage(const PolymorphicStorage& other);
	PolymorphicStorage& operator=(const PolymorphicStorage& other);

	PolymorphicStorage(PolymorphicStorage&& other);
	PolymorphicStorage& operator=(PolymorphicStorage&& other);

	~PolymorphicStorage() = default;

	/// Construct from object
	// enable_if magic is necessary that this constructor
	// does not outmatch the copy constructor in overload resolution
	// when copying a non-const PolymorphicStorage.
	// (Then, 'T&&' is a better match than 'const PolymorphicStorage&')
	template<typename T, typename = std::enable_if_t<
		// if T is PolymorphicStorage, do not select this overload
		!std::is_same_v<PolymorphicStorage, std::decay_t<T>>
	>>
	PolymorphicStorage(T&& data);

	/// Assign object
	template<typename T, typename = std::enable_if_t<
		// if T is PolymorphicStorage, do not select this overload
		!std::is_same_v<PolymorphicStorage, std::decay_t<T>>
	>>
	PolymorphicStorage&
	operator=(T&& data);

	Base* operator->() { return basePtr_; }

	const Base* operator->() const { return basePtr_; }

	template<typename T>
	T* get() { return std::get_if<T>(&data_); }

	template<typename T>
	const T* get() const { return std::get_if<T>(&data_); }

private:
	Base* basePtr();

	std::variant<Ts...> data_;

	// always != nullptr after the constructor ran
	Base* basePtr_ = nullptr;
};

template<typename Base, typename... Ts>
PolymorphicStorage<Base, Ts...>::PolymorphicStorage()
	: basePtr_{basePtr()}
{
}

template<typename Base, typename... Ts>
PolymorphicStorage<Base, Ts...>::PolymorphicStorage(const PolymorphicStorage& other)
	: data_{other.data_}, basePtr_{basePtr()}
{
}

template<typename Base, typename... Ts>
PolymorphicStorage<Base, Ts...>&
PolymorphicStorage<Base, Ts...>::operator=(const PolymorphicStorage& other)
{
	data_ = other.data_;
	basePtr_ = basePtr();
	return *this;
}

template<typename Base, typename... Ts>
PolymorphicStorage<Base, Ts...>::PolymorphicStorage(PolymorphicStorage&& other)
	: data_{std::move(other.data_)}, basePtr_{basePtr()}
{
}

template<typename Base, typename... Ts>
PolymorphicStorage<Base, Ts...>&
PolymorphicStorage<Base, Ts...>::operator=(PolymorphicStorage&& other)
{
	data_ = std::move(other.data_);
	basePtr_ = basePtr();
	return *this;
}

template<typename Base, typename... Ts>
template<typename T, typename>
PolymorphicStorage<Base, Ts...>&
PolymorphicStorage<Base, Ts...>::operator=(T&& data)
{
	data_ = std::forward<T>(data);
	basePtr_ = basePtr();
	return *this;
}

template<typename Base, typename... Ts>
template<typename T, typename>
PolymorphicStorage<Base, Ts...>::PolymorphicStorage(T&& data)
	: data_{std::forward<T>(data)}, basePtr_(basePtr())
{
}

template<typename Base, typename... Ts>
Base* PolymorphicStorage<Base, Ts...>::basePtr()
{
	Base* ptr = nullptr;
	std::visit([&ptr](auto& value) {
		ptr = &value;
	}, data_);
	return ptr;
}

}

#endif
