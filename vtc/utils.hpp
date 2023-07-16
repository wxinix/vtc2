#pragma once

#include <array>
#include <tuple>
#include <type_traits>

namespace vtc::utils {

template<typename Tuple1, typename Tuple2, typename Tuple3>
struct concatenate_tuples;

/**
 * Concatenate 3 tuples into one tuple.
 * @tparam Ts1 The first tuple.
 * @tparam Ts2 The second tuple.
 * @tparam Ts3 The third tuple.
 */
template<typename... Ts1, typename... Ts2, typename... Ts3>
struct concatenate_tuples<std::tuple<Ts1...>, std::tuple<Ts2...>, std::tuple<Ts3...>>
{
  using type = std::tuple<Ts1..., Ts2..., Ts3...>;
};

template<typename Tuple1, typename Tuple2, typename Tuple3>
using concatenate_tuples_t = typename concatenate_tuples<Tuple1, Tuple2, Tuple3>::type;

template<typename Tuple, typename T>
struct append_tuple;

/**
 * Append a new type to an existing tuple.
 * @tparam Ts The types of the elements of the existing tuple.
 * @tparam T The type of the new element to append the existing tuple.
 */
template<typename... Ts, typename T>
struct append_tuple<std::tuple<Ts...>, T>
{
  using type = std::tuple<Ts..., T>;
};

template<typename Tuple, typename T>
using append_tuple_t = typename append_tuple<Tuple, T>::type;

/**
 * Check if a type has a constexpr static member called index.
 * @tparam T
 */
template<typename T>
concept HasIndex = requires(T t) {
  requires std::is_const_v<decltype(T::index)> &&                             /**/
      std::integral<decltype(T::index)> &&                                    /**/
      !std::is_same_v<std::remove_cv_t<decltype(T::index)>, char> &&          /**/
      !std::is_same_v<std::remove_cv_t<decltype(T::index)>, signed char> &&   /**/
      !std::is_same_v<std::remove_cv_t<decltype(T::index)>, unsigned char> && /**/
      !std::is_same_v<std::remove_cv_t<decltype(T::index)>, bool>;
};

template<size_t Offset, typename SeqType>
struct offset_sequence;

/**
 * Add an offset value to each element of an integer sequence.
 * @tparam Offset The offset value, 0 means offset by 1.
 * @tparam Is The sequence to be offset.
 */
template<size_t Offset, typename T, T... Is>
struct offset_sequence<Offset, std::integer_sequence<T, Is...>>
{
  using type = std::integer_sequence<T, Is + (Offset + 1)...>;
};

/**
 * Offset value 0 means to "offset" the integer sequence by 1.
 */
template<size_t Offset, typename SeqType>
using offset_sequence_t = typename offset_sequence<Offset, SeqType>::type;

/**
  Retrieve an element from integer sequence at compile time. For example,

  auto seq = std::integer_sequence<unsigned, 9, 2, 5, 1, 9, 1, 15>{};
  auto val = get(seq, 6); // val equals to 15.

  @param i The index of the element.
  @return The element value.
 */
template<typename T, T... Is>
constexpr T get(std::integer_sequence<T, Is...>, std::size_t i)
{
  constexpr auto arr = std::array{Is...};
  return arr[i];
}

template<size_t I, typename SeqType>
struct add_sequence_front;

/**
 * Add a new integer to the front of an existing integer sequence.
 * @tparam I  - The integer value to be added to the front of the existing sequence.
 * @tparam Is - The integer sequence, for which the extra integer value will be added to its front.
 */
template<size_t I, size_t... Is>
struct add_sequence_front<I, std::integer_sequence<size_t, Is...>>
{
  using type = std::integer_sequence<size_t, I, Is...>;
};

template<size_t I, typename SeqType>
using add_sequence_front_t = typename add_sequence_front<I, SeqType>::type;

/**
 * Enumerate each element of a tuple, and apply the given functor to the element.
 * @tparam Tuple The type of the tuple.
 * @tparam F The functor type.
 * @param tuple The tuple.
 * @param func The functor.
 */
template<typename Tuple, typename F>
void for_each(Tuple &&tuple, F &&func)
{
  std::apply(
      [&func]<typename... T>(T &&...args) {
        (func(std::forward<T>(args)), ...);
      },
      std::forward<Tuple>(tuple));
}

}// namespace vtc::traits
