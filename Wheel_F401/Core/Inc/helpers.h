/*
 * helpers.h
 *
 *  Created on: Dec 5, 2023
 *      Author: Vielder
 */

#ifndef INC_HELPERS_H_
#define INC_HELPERS_H_

#include <sys/_stdint.h>
#include <cstddef>
#include <iterator>

template<class T, class C>
T clip(T v, C l, C h) {
	return {v > h ? h : v < l ? l : v};
}

template<typename _Tp>
_GLIBCXX14_CONSTEXPR
inline const _Tp&
max(const _Tp &__a, const _Tp &__b) {
	// concept requirements
	__glibcxx_function_requires(_LessThanComparableConcept<_Tp>)
	//return  __a < __b ? __b : __a;
	if (__a < __b)
		return __b;
	return __a;
}

template<class T, class C>
int8_t cliptest(T v, C l, C h) {
	if (v > h) {
		return 1;
	}
	else if (v < l) {
		return -1;
	}
	else {
		return 0;
	}
}

#endif /* INC_HELPERS_H_ */
