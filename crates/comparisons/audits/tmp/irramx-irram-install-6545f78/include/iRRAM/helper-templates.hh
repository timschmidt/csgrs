
#ifndef HELPER_TEMPLATES_HH
#define HELPER_TEMPLATES_HH

#include <type_traits>

#include <iRRAM/core.h>

namespace iRRAM {

namespace internal {

/* helper for reduced overload visibility of binary operator declarations */
template <typename Base,typename EquivBase,typename Compat,typename Ret = Base>
using enable_if_compat = typename std::enable_if<
	std::is_same<typename std::remove_cv<EquivBase>::type,
	             typename std::remove_cv<Base>::type
	            >::value &&
	std::is_convertible<Compat,Base>::value
,Ret>::type;

template <typename Base,typename Ret = bool>
struct conditional_comparison_overloads {
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator<(const A &a, const B &b) { return a<Base(b); }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator<(const B &b, const A &a) { return Base(b)<a; }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator>(const A &a, const B &b) { return a>Base(b); }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator>(const B &b, const A &a) { return Base(b)>a; }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator<=(const A &a, const B &b) { return a<=Base(b); }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator<=(const B &b, const A &a) { return Base(b)<=a; }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator>=(const A &a, const B &b) { return a>=Base(b); }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator>=(const B &b, const A &a) { return Base(b)>=a; }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator==(const A &a, const B &b) { return a==Base(b); }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator==(const B &b, const A &a) { return Base(b)==a; }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator!=(const A &a, const B &b) { return a!=Base(b); }
	template <typename A,typename B> friend enable_if_compat<Base,A,B,Ret> operator!=(const B &b, const A &a) { return Base(b)!=a; }
};

template <typename...> struct disjunction : std::false_type {};
template <typename T> struct disjunction<T> : T {};

template <typename C,typename... D>
struct disjunction<C,D...> : std::conditional<C::value,C,disjunction<D...>>::type {};

template <typename T> struct negation {
	static constexpr bool value = !T::value;
};

template <typename... T>
using conjunction = negation<disjunction<negation<T>...>>;

template <typename C> struct is_continuous : public std::false_type {};
template <typename C> struct is_continuous<C &> : is_continuous<C> {};
template <typename C> struct is_continuous<const C &> : is_continuous<C> {};
template <typename C> struct is_continuous<C &&> : is_continuous<C> {};

template <> struct is_continuous<REAL> : public std::true_type {};
template <> struct is_continuous<REALMATRIX> : public std::true_type {};
template <> struct is_continuous<SPARSEREALMATRIX> : public std::true_type {};
template <> struct is_continuous<COMPLEX> : public std::true_type {};
template <> struct is_continuous<INTERVAL> : public std::true_type {};
template <typename... Args> struct is_continuous<FUNCTION<REAL,Args...>> : public std::true_type {};

template <typename... T> using any_continuous = disjunction<is_continuous<T>...>;

/* helper templates for invocation of functions:
 * return type void or not usually need different implementations */

template <typename F,typename... Args>
using result_of_t = typename std::result_of<F &&(Args &&...)>::type;

template <typename R> struct retval       { typedef R    value_type; };
template <>           struct retval<void> { typedef void void_type; };

template <typename F,typename... Args>
using ret_void_t = typename internal::retval<result_of_t<F,Args...>>::void_type;

template <typename F,typename... Args>
using ret_value_t = typename internal::retval<result_of_t<F,Args...>>::value_type;

}

}

#endif
