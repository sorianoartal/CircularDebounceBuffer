#pragma once
#include <stdint.h> // Include standard integer types for fixed-width types.
#include <Arduino.h>


/// @brief Namespace for AVR algorithms and utilities
/// @note This namespace contains various utility functions and algorithms that can be used in AVR-based applications
/// such as repeating operations, iterating over arrays, and finding elements in collections.
/// @note It provides a set of generic algorithms that can be applied to arrays, containers, and iterators.
/// @namespace avr_algorithms	
namespace avr_algorithms {

/**
 * @brief Repeats an operation a specified number of times.
 * * This function takes an integer 'n' and a callable 'operation', and executes the operation 'n' times.
 * * @note This is useful for scenarios where you need to perform an action multiple times, such as sending a signal or performing a task repeatedly.
 * * @example	
 *   repeat(5, []() {
 * 	  Serial.println("Hello, World!");
 *   });
 * 
 * @tparam Func - Type of the callable operation to be executed repeatedly
 * @note The 'Func' type can be a lambda, function pointer, or any callable
 * @param n - The number of times to repeat the operation
 * @note The 'n' parameter must be a non-negative integer. If 'n'
 * @param operation - The operation to be executed repeatedly
 * @note The 'operation' parameter is a callable that takes no arguments and returns void.
 */
template<typename Func>
inline void repeat(int n, Func&& operation) {
    while (n-- > 0) {
        operation();
    }    
}

/**
 * @brief Repeatedly executes a function while a predicate remains true.
 *
 * Calls @p func once per loop iteration, but only if @p pred evaluates to true.
 * The predicate is evaluated before each iteration (including the first).
 *
 * @tparam Predicate  Nullary callable returning a bool-like value.
 * @tparam Func       Nullary callable for the loop body.
 *
 * @param pred  Predicate checked before each iteration. May hold state (e.g., a lambda with captures).
 * @param func  Function body executed on each iteration while @p pred is true.
 * @param max_iterations Maximum number of iterations to prevent infinite loops.
 * 
 * @note  Both callables are taken by forwarding reference and are not copied. On AVR, prefer small lambdas
 *        or static functions so the compiler can inline them. Avoid heavy STL utilities in tight loops.
 *
 * @warning If @p pred never becomes false, this function never returns (infinite loop / blocking).
 *          If @p pred depends on values modified by ISRs, mark those as @c volatile and ensure atomic access.
 *
 * @par Complexity
 * One call to @p pred and (conditional) one call to @p func per iteration.
 *
 * @par Example (Arduino)
 * Blink the onboard LED while a button on D2 is held:
 * @code{.cpp}
 * int max_iterations = 100; // Safety limit to avoid infinite loops
 * auto isButtonDown = []() -> bool { return digitalRead(2) == HIGH; };
 * auto blinkOnce    = []() { digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); delay(50); };
 * repeat_while(isButtonDown, max_iterations, blinkOnce);
 * @endcode
 *
 * @par Example (bare-metal AVR)
 * Toggle PB5 while PB0 stays high:
 * @code{.cpp}
 * #include <avr/io.h>
 * #include <util/delay.h>
 *
 * auto pb0_high   = []() -> bool { return (PINB & _BV(PB0)) != 0; };
 * auto toggle_led = []() { PORTB ^= _BV(PB5); _delay_ms(25); };
 *
 * repeat_while(pb0_high, toggle_led);
 * @endcode
 */
template <typename Predicate, typename Func>
inline void repeat_while(Predicate&& pred, size_t max_iterations, Func&& func)
{
    while (pred() && max_iterations-- > 0)
    {
        func();
    }
    
}

/**
 * @brief Repeats an operation with an exit condition.
 * * This function takes an integer 'n' and a callable 'operation', and executes the operation up to 'n' times or until the operation returns false.
 * * @note This is useful for scenarios where you want to repeat an action until a certain condition is met, such as waiting for a signal or checking a status.
 * * @example
 *   repeat_withExitCondition(5, []() {
 * 		Serial.println("Checking status...");
 * 		return checkStatus(); // Returns true to continue, false to exit
 *   });
 * 
 * @tparam Func - Type of the callable operation to be executed repeatedly
 * * @note The 'Func' type can be a lambda, function pointer, or any callable that returns a boolean value
 * @param n - The maximum number of times to repeat the operation
 * * @note The 'n' parameter must be a non-negative integer. If 'n' is less than or equal to zero, the operation will not be executed.
 * @param operation - The operation to be executed repeatedly
 * * @note The 'operation' parameter is a callable that takes no arguments and returns a boolean value.
 * * @note The operation should return true to continue repeating, or false to exit the loop
 */
template<typename Func>
void repeat_withExitCondition(int n, Func&& operation) {
    while (n-- > 0) {
        if (!operation()) break;
    }    
}

/**
 * @brief Applies a function to each element of an array or buffer.
 * * This function iterates over the elements of an array or buffer and applies a specified function to each element.
 * * @note This is useful for scenarios where you want to perform an operation on each element, such as processing data or modifying values.
 * * @example
 *   int arr[] = {1, 2, 3, 4, 5};
 *   for_each(arr, [](int& value, size_t index) {
 *       value *= 2; // Double each value
 *   });
 * 
 * @tparam T - Type of the elements in the array or buffer
 * @tparam N - Size of the array (if applicable)
 * @tparam Func - Type of the callable function to apply to each element
 * @note The 'Func' type can be a lambda, function pointer, or any callable that takes an element and its index as arguments
 * @param array - The array or buffer to iterate over
 * @param func - The function to apply to each element
 */
template<typename T, size_t N, typename Func>
void for_each(T(&array)[N], Func&& func) {
    for (size_t i = 0; i < N; ++i) {
        func(array[i], i);
    }
}

/**
 * @brief Applies a function to each element of an array or buffer without the index.
 * * This function iterates over the elements of an array or buffer and applies a specified function to each element without providing the index.
 * * @note This is useful for scenarios where you want to process each element independently, such as transforming data or applying a filter.
 * * @example
 *   int arr[] = {1, 2, 3, 4, 5};
 *   for_each_element(arr, [](int& value) {
 *       value += 10; // Add 10 to each value
 *   });
 * 
 * @tparam T - Type of the elements in the array or buffer
 * @tparam N - Size of the array (if applicable)
 * @tparam Func - Type of the callable function to apply to each element
 * @note The 'Func' type can be a lambda, function pointer, or any callable that takes an element as an argument
 * @param array - The array or buffer to iterate over
 * @param func - The function to apply to each element
 */
template<typename T, size_t N, typename Func>
void for_each_element(T(&array)[N], Func&& func) {
    for (size_t i = 0; i < N; ++i) {
        func(array[i]);
    }
}


/**
 * @brief Applies a function to each element of a buffer with an index.
 * * This function iterates over the elements of a buffer and applies a specified function to each element, providing the index as an additional argument.
 * * @note This is useful for scenarios where you need to know the position of each element while processing it, such as logging or debugging.
 * * @example
 *   uint8_t buffer[] = {10, 20, 30, 40};
 *   for_each_n(buffer, sizeof(buffer), [](uint8_t& value, size_t index) {
 *       Serial.print("Index: "); Serial.print(index); Serial.print(", Value: "); Serial.println(value);
 *   });
 * 
 * @tparam T - Type of the elements in the buffer
 * @tparam F - Type of the callable function to apply to each element
 * @note The 'F' type can be a lambda, function pointer, or any callable that takes an element and its index as arguments
 * @param buf - Pointer to the buffer to iterate over
 * @param count - Number of elements in the buffer
 * @param f - The function to apply to each element
 */
template <typename T, class F>
constexpr void for_each_n(T* buf, size_t count, F&& f) {
    for (size_t i = 0; i < count; ++i) f(buf[i]);
}


/**
 * @brief Apply a callable to every element of a C-array, with the element index.
 *
 * Calls @p f(elem, index) for each element in @p a, where @p index runs from 0 to N-1.
 * The array size N is deduced at compile time (no need to pass a count).
 *
 * @tparam T  Element type of the array.
 * @tparam N  Number of elements in the array (deduced).
 * @tparam F  Callable type taking (T& element, size_t index).
 *
 * @param a   Reference to a C-array (e.g., uint8_t pins[4]).
 * @param f   Callable invoked as f(a[i], i) for i in [0, N).
 *
 * @note  constexpr, header-only, zero overhead; no dynamic allocation.
 * @note  The callable @p f may be a lambda, function pointer, or functor.
 * @note  Use when you need both the element and its position (e.g., bit routing, logging).
 *
 * @pre   @p a is a real C-array (not a pointer). For pointer + runtime count, use a separate
 *        pointer+count helper (e.g., for_each_n_indexed(ptr, count, f)).
 *
 * @warning If @p f touches hardware (GPIO/ADC), keep it short and non-blocking per call.
 *
 * @par Complexity
 * Linear in N; exactly N invocations of @p f.
 *
 * @par Example (simple logging)
 * @code
 * uint8_t buf[] = {10, 20, 30};
 * for_each_indexed(buf, [](uint8_t& v, size_t i){
 *   Serial.print("#"); Serial.print(i); Serial.print(": "); Serial.println(v);
 * });
 * @endcode
 *
 * @par Example (CD74HC4067 channel select: bit i -> S[i])
 * @code
 * // _selectPins holds S0..S3 pin numbers in order
 * for_each_indexed(_selectPins, [&](uint8_t pin, size_t i){
 *   digitalWrite(pin, (channel >> i) & 0x01);
 * });
 * @endcode
 */
template<typename T, size_t N ,typename Predicate>
constexpr void for_each_indexed( T (&array)[N] , Predicate&& func )
{
    for(size_t i = 0 ; i < N ; ++i)
        func(array[i],i);   // element , index
}

/**
 * @brief Apply a callable to each element of a buffer (pointer + count), with the element index.
 *
 * Calls func(element, index) for each element, where index runs from 0 to count-1.
 *
 * @tparam T Element type.
 * @tparam Func Callable type taking (T& element, size_t index).
 * @param buffer Pointer to the start of the buffer.
 * @param length Number of elements.
 * @param func Callable invoked as func(buffer[i], i) for i in [0, length).
 *
 * @note Similar to the array version but for pointers with explicit size.
 * @note Use when size isn't compile-time known or for decayed arrays.
 *
 * @par Example (Processing dynamic buffer)
 * @code
 * float* voltages = new float[16]; // Or from a const array via const_cast if needed
 * // Populate voltages...
 * size_t activeIdx = 0;
 * for_each_indexed(voltages, 16, [&](float& v, size_t idx) {
 *     if (v >= 0.0f) {
 *         // Draw or process valid channel at grid position based on activeIdx
 *         Serial.print("Ch"); Serial.print(idx + 1); Serial.print(": "); Serial.println(v);
 *         activeIdx++;
 *     }
 * });
 * delete[] voltages;
 * @endcode
 *
 * @par Example (With const_cast for const arrays)
 * @code
 * const float voltages[16] = {1.2f, -1.0f, 3.4f, ...};
 * size_t activeIdx = 0;
 * for_each_indexed(const_cast<float*>(voltages), 16, [&](float& v, size_t idx) {
 *     if (v >= 0.0f) {
 *         // Safe read-only processing
 *         Serial.print("Ch"); Serial.print(idx + 1); Serial.print(": "); Serial.println(v);
 *         activeIdx++;
 *     }
 * });
 * @endcode
 */
template<typename T, typename Func>
inline void for_each_indexed(T* buffer, size_t buffer_size , Func&& func)
{
    for (size_t i = 0; i < buffer_size; i++)
    {
        func(buffer[i],i);
    }    
} 

      
/**
 * @brief Applies a function to each element of a buffer with an index.
 * * This function iterates over the elements of a buffer and applies a specified function to each element, providing the index as an additional argument.
 * * @note This is useful for scenarios where you need to know the position of each element while processing it, such as logging or debugging.
 * * @example
 *   uint8_t buffer[] = {10, 20, 30, 40};
 *   for_each(buffer, sizeof(buffer), [](uint8_t& value, uint8_t index) {
 *       Serial.print("Index: "); Serial.print(index); Serial.print(", Value: "); Serial.println(value);
 *   });
 * 
 * @tparam T - Type of the elements in the buffer
 * @tparam Func - Type of the callable function to apply to each element
 * @note The 'Func' type can be a lambda, function pointer, or any callable that takes an element and its index as arguments
 * @param buffer - Pointer to the buffer to iterate over
 * @param length - Length of the buffer
 * @param func - The function to apply to each element
 */
template<typename T, typename Func>
void for_each(T* buffer, size_t length, Func&& func) {
    for (size_t i = 0; i < length; ++i) {
        func(buffer[i], static_cast<uint8_t>(i));
    }
}

/**
 * @brief Applies a function to each element of a buffer without the index.
 * * This function iterates over the elements of a buffer and applies a specified function to each element without providing the index.
 * * @note This is useful for scenarios where you want to process each element independently, such as transforming data or applying a filter.
 * * @example
 *   uint8_t buffer[] = {10, 20, 30, 40};
 *   for_each_element(buffer, sizeof(buffer), [](uint8_t& value) {
 *       value += 5; // Add 5 to each value
 *   });
 * 
 * @tparam T - Type of the elements in the buffer
 * @tparam Func - Type of the callable function to apply to each element
 * @note The 'Func' type can be a lambda, function pointer, or any callable that takes an element as an argument
 * @param buffer - Pointer to the buffer to iterate over
 * @param length - Length of the buffer
 * @param func - The function to apply to each element
 */
template<typename T, typename Func>
inline void for_each_element(T* buffer, size_t length, Func&& func) {
    for (size_t i = 0; i < length; ++i) {
        func(buffer[i]);
    }
}

/**
 * @brief Applies a function to each element of a buffer without the index.
 * * This function iterates over the elements of a buffer and applies a specified function to each element without providing the index.
 * * @note This is useful for scenarios where you want to process each element independently, such as transforming data or applying a filter.
 * * @example
 *   uint8_t buffer[] = {10, 20, 30, 40};
 *   for_each_element(buffer, sizeof(buffer), [](uint8_t& value) {
 *       value += 5; // Add 5 to each value
 *   });
 * 
 * @tparam T - Type of the elements in the buffer
 * @tparam Func - Type of the callable function to apply to each element
 * @note The 'Func' type can be a lambda, function pointer, or any callable that takes an element as an argument
 * @param buffer - Pointer to the buffer to iterate over
 * @param length - Length of the buffer
 * @param func - The function to apply to each element
 */
template<typename T, typename Func>
inline void for_each_element(const T* buffer, size_t length, Func func) {
    for (size_t i = 0; i < length; ++i) {
        func(buffer[i]);
    }
}


/**
 * @brief Applies a function to each element of a buffer until the function returns false.
 * * This function iterates over the elements of a buffer and applies a specified function to each element until the function returns false.
 * * @note This is useful for scenarios where you want to stop processing elements as soon as a certain condition is met, such as finding a specific value or checking a condition.
 * * @example
 *   uint8_t buffer[] = {1, 2, 3, 4, 5};
 *   bool allLessThanTen = for_each_until(buffer, sizeof(buffer), [](uint8_t& value) {
 *       return value < 10; // Continue until a value is not less than 10
 *   });
 * 
 * @tparam T - Type of the elements in the buffer
 * @tparam Func - Type of the callable function to apply to each element
 * @note The 'Func' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param buffer - Pointer to the buffer to iterate over
 * @param length - Length of the buffer
 * @param func - The function to apply to each element
 * @return bool - Returns true if all elements satisfied the condition, false otherwise
 */
template<typename T, typename Func>
inline bool for_each_until(const T* buffer, size_t length, Func func) {
    for (size_t i = 0; i < length; ++i) {
        if (!func(buffer[i])) return false;
    }
    return true;
}

/**
 * @brief Applies a function to each element of an array until the function returns false.
 *
 *  @details
 * This function iterates over the elements of an array and applies a specified function to each element until the function returns false.
 * * @note This is useful for scenarios where you want to stop processing elements as soon as a certain condition is met, such as finding a specific value or checking a condition.
 * * @example
 *   int arr[] = {1, 2, 3, 4, 5};
 *   bool allLessThanTen = for_each_until(arr, [](int value) {
 *       return value < 10; // Continue until a value is not less than 10
 *   });
 * 
 * @tparam T - Type of the elements in the array
 * @tparam N - Size of the array
 * @tparam Func - Type of the callable function to apply to each element
 * @note The 'Func' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param array - The array to iterate over
 * @param func - The function to apply to each element (return true to continue, false to stop)
 * @return bool - Returns true if all elements satisfied the condition (all func calls returned true), false otherwise
 */
template<typename T, typename Func, size_t N>
inline bool for_each_until(T (&arr)[N], Func&& func)
{
    for (size_t i = 0; i < N; i++)
    {
        if(!func(arr[i])) return false;
    }
    return true;    
}

/**
 * @brief Applies a function to each element of an array with index until the function returns false.
 * * This function iterates over the elements of an array, providing the index as an additional argument, and applies a specified function to each element until the function returns false.
 * * @note This is useful for scenarios where you want to stop processing elements as soon as a certain condition is met, and you need to know the position of each element.
 * * @example
 *   int arr[] = {1, 2, 3, 4, 5};
 *   bool allLessThanTen = for_each_until_indexed(arr, [](int value, size_t index) {
 *       return value < 10; // Continue until a value is not less than 10
 *   });
 * 
 * @tparam T - Type of the elements in the array
 * @tparam N - Size of the array
 * @tparam Func - Type of the callable function to apply to each element
 * @note The 'Func' type can be a lambda, function pointer, or any callable that takes an element and its index as arguments and returns a boolean value
 * @param array - The array to iterate over
 * @param func - The function to apply to each element (return true to continue, false to stop)
 * @return bool - Returns true if all elements satisfied the condition (all func calls returned true), false otherwise
 */
template<typename T, typename Func, size_t N>
inline bool for_each_until_indexed(T (&arr)[N], Func&& func)
{
    for (size_t i = 0; i < N; i++)
    {
        if(!func(arr[i], i)) return false;
    }
    return true;    
}

/**
 * @brief Iterates over indices from 0 to n-1 and applies the provided function to each index.
 *
 * This function provides a simple way to loop over a range of indices [0, n) and invoke
 * a callable object (such as a lambda, functor, or function pointer) on each index.
 * It is equivalent to a basic for loop but encapsulated for reusability and readability.
 * The function is templated to accept any callable type and is marked inline for potential
 * optimization by the compiler.
 *
 * Example usage:
 * @code
 * for_index_n(5, [](size_t i) { std::cout << i << " "; });  // Outputs: 0 1 2 3 4
 * @endcode
 *
 * @tparam Func The type of the callable object. It must be invocable with a single size_t argument.
 * @param n The number of iterations (the upper bound of the index range, exclusive).
 * @param func The callable object to invoke for each index.
 */
template<typename Func>
inline void for_index_n(size_t n, Func&& func)
{
    for (size_t i = 0; i < n; i++)
    {
        func(i);
    }
    
} 

/**
 * @brief Checks if any element in an array satisfies a given predicate.
 * 
 * @details
 * - This function iterates over the elements of an array and applies a specified predicate function to each element.
 * 
 * @tparam T     - Type of the elements in the array
 * @tparam N     - Size of the array
 * @tparam Pred  - Type of the callable predicate function
 * @param array  - The array to iterate over
 * @param func   - The predicate function to apply to each element
 * @return true  - If any element satisfies the predicate
 * @return false - If no elements satisfy the predicate
 */
template<typename T, size_t N, typename Pred>
inline bool any_of(T(&array)[N], Pred&& func)
{
    for (size_t i = 0; i < N; i++)
    {
        if(func(array[i],i)) return true;
    }
    return false;   

}

/**
 * @brief Checks if any element in a buffer satisfies a given predicate.
 * 
 * @details
 * - This function iterates over the elements of a buffer and applies a specified predicate function to each element.
 * 
 * @tparam T     - Type of the elements in the buffer
 * @tparam Pred  - Type of the callable predicate function
 * @param begin  - Pointer to the beginning of the buffer
 * @param count  - Number of elements in the buffer
 * @param func   - The predicate function to apply to each element
 * @return true  - If any element satisfies the predicate
 * @return false - If no elements satisfy the predicate
 */
template<typename T, typename Pred>
inline bool any_of(T* begin, size_t count, Pred&& func)
{
    for (size_t i = 0; i < count; i++)
    {
        if(func(begin[i],i)) return true;
    }
    return false;
    
}

/**
 * @brief Finds the first element in a range that matches a value.
 * * This function iterates over the elements of a range defined by two iterators and finds the first element that matches a specified value.
 * * @note This is useful for scenarios where you want to find a specific value in a range, such as searching for an item or checking existence.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 4, 5};
 *   auto it = find(vec.begin(), vec.end(), 3); // Find the value 3
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param value - The value to find in the range
 * @return Iterator - An iterator pointing to the first element that matches the value, or end if not found
 */
template<typename Iterator>
Iterator find_impl(Iterator begin, Iterator end, const typename Iterator::value_type& value) {
    for (auto it = begin; it != end; ++it) {
        if (*it == value) {
            return it;
        }
    }
    return end;
}

/**
 * @brief Finds the first element in a range that matches a value.
 * * This function iterates over the elements of a range defined by two iterators and finds the first element that matches a specified value.
 * * @note This is useful for scenarios where you want to find a specific value in a range, such as searching for an item or checking existence.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 4, 5};
 *   auto it = find(vec.begin(), vec.end(), 3); // Find the value 3
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param value - The value to find in the range
 * @return Iterator - An iterator pointing to the first element that matches the value, or end if not found
 */
template<typename Iterator>
Iterator find(Iterator begin, Iterator end, const typename Iterator::value_type& value) {
    return find_impl(begin, end, value);
}

/**
 * @brief Finds the first element in an array that matches a value.
 * * This function iterates over the elements of an array and finds the first element that matches a specified value.
 * * @note This is useful for scenarios where you want to find a specific value in an array, such as searching for an item or checking existence.
 * * @example
 *   int arr[] = {1, 2, 3, 4, 5};
 *   int* ptr = find(arr, 3); // Find the value 3
 * 
 * @tparam T - Type of the elements in the array
 * @tparam N - Size of the array
 * @note The 'T' type can be any type that supports equality comparison
 * @param array - The array to iterate over
 * @param value - The value to find in the array
 * @return T* - Pointer to the first element that matches the value, or nullptr if not found
 */
template<typename T, size_t N>
T* find(T(&array)[N], const T& value) {
    return find_impl(array, array + N, value);
}

/**
 * @brief Finds the first element in a range that satisfies a given predicate.
 * * This function iterates over the elements of a range defined by two iterators and finds the first element that satisfies a specified predicate.
 * * @note This is useful for scenarios where you want to find an element based on a condition, such as searching for a property or checking a state.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 4, 5};
 *   auto it = find_if(vec.begin(), vec.end(), [](int value) {
 *       return value > 2; // Find the first value greater than 2
 *   });
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @tparam Predicate - Type of the callable predicate to check each element
 * @note The 'Predicate' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param predicate - The predicate function to check each element
 * @return Iterator - An iterator pointing to the first element that satisfies the predicate, or end if not found
 */
template<typename Iterator, typename Predicate>
Iterator find_if_impl(Iterator begin, Iterator end, Predicate&& predicate) {
    for (auto it = begin; it != end; ++it) {
        if (predicate(*it)) {
            return it;
        }
    }
    return end;
}

/**
 * @brief Finds the first element in a range that satisfies a given predicate.
 * * This function iterates over the elements of a range defined by two iterators and finds the first element that satisfies a specified predicate.
 * * @note This is useful for scenarios where you want to find an element based on a condition, such as searching for a property or checking a state.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 4, 5};
 *   auto it = find_if(vec.begin(), vec.end(), [](int value) {
 *       return value > 2; // Find the first value greater than 2
 *   });
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @tparam Predicate - Type of the callable predicate to check each element
 * @note The 'Predicate' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param predicate - The predicate function to check each element
 * @return Iterator - An iterator pointing to the first element that satisfies the predicate, or end if not found
 */
template<typename Iterator, typename Predicate>
Iterator find_if(Iterator begin, Iterator end, Predicate&& predicate) {
    return find_if_impl(begin, end, predicate);
}

/**
 * @brief Finds the first element in an array that satisfies a given predicate.
 * * This function iterates over the elements of an array and finds the first element that satisfies a specified predicate.
 * * @note This is useful for scenarios where you want to find an element based on a condition, such as searching for a property or checking a state.
 * * @example
 *   int arr[] = {1, 2, 3, 4, 5};
 *   int* ptr = find_if(arr, [](int value) {
 *       return value > 2; // Find the first value greater than 2
 *   });
 * 
 * @tparam T - Type of the elements in the array
 * @tparam N - Size of the array
 * @tparam Predicate - Type of the callable predicate to check each element
 * @note The 'Predicate' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param array - The array to iterate over
 * @param predicate - The predicate function to check each element
 * @return T* - Pointer to the first element that satisfies the predicate, or nullptr if not found
 */
template<typename T, size_t N, typename Predicate>
T* find_if(T(&array)[N], Predicate&& predicate) {
    return find_if_impl(array, array + N, predicate);
}

/**
 * @brief Counts the number of occurrences of a specific value in a range.
 * * This function iterates over the elements of a range defined by two iterators and counts how many times a specific value appears.
 * * @note This is useful for scenarios where you want to count occurrences of a value, such as finding duplicates or checking frequency.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 2, 4};
 *   size_t count = count(vec.begin(), vec.end(), 2); // Count occurrences of 2
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param value - The value to count occurrences of
 * @return size_t - The number of occurrences of the specified value in the range
 */
template<typename Iterator>
size_t count_impl(Iterator begin, Iterator end, const typename Iterator::value_type& value) {
    size_t count = 0;
    for (auto it = begin; it != end; ++it) {
        if (*it == value) {
            ++count;
        }
    }
    return count;
}

/**
 * @brief Counts the number of occurrences of a specific value in a range.
 * * This function iterates over the elements of a range defined by two iterators and counts how many times a specific value appears.
 * * @note This is useful for scenarios where you want to count occurrences of a value, such as finding duplicates or checking frequency.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 2, 4};
 *   size_t count = count(vec.begin(), vec.end(), 2); // Count occurrences of 2
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param value - The value to count occurrences of
 * @return size_t - The number of occurrences of the specified value in the range
 */
template<typename Iterator>
size_t count(Iterator begin, Iterator end, const typename Iterator::value_type& value) {
    return count_impl(begin, end, value);
}


/**
 * @brief Counts the number of occurrences of a specific value in an array.
 * * This function iterates over the elements of an array and counts how many times a specific value appears.
 * * @note This is useful for scenarios where you want to count occurrences of a value, such as finding duplicates or checking frequency.
 * * @example
 *   int arr[] = {1, 2, 3, 2, 4};
 *   size_t count = count(arr, 2); // Count occurrences of 2
 * 
 * @tparam T - Type of the elements in the array
 * @tparam N - Size of the array
 * @note The 'T' type can be any type that supports equality comparison
 * @param array - The array to iterate over
 * @param value - The value to count occurrences of
 * @return size_t - The number of occurrences of the specified value in the array
 */
template<typename T, size_t N>
size_t count(const T(&array)[N], const T& value) {
    return count_impl(array, array + N, value);
}

/**
 * @brief Counts the number of elements in a range that satisfy a given predicate.
 * * This function iterates over the elements of a range defined by two iterators and counts how many elements satisfy a specified predicate.
 * * @note This is useful for scenarios where you want to count elements based on a condition, such as filtering or checking properties.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 4, 5};
 *   size_t count = count_if(vec.begin(), vec.end(), [](int value) {
 *       return value > 2; // Count elements greater than 2
 *   });
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @tparam Predicate - Type of the callable predicate to check each element
 * @note The 'Predicate' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param predicate - The predicate function to check each element
 * @return size_t - The number of elements that satisfy the predicate in the range
 */
template<typename Iterator, typename Predicate>
size_t count_if_impl(Iterator begin, Iterator end, Predicate&& predicate) {
    size_t count = 0;
    for (auto it = begin; it != end; ++it) {
        if (predicate(*it)) {
            ++count;
        }
    }
    return count;
}

/**
 * @brief Counts the number of elements in a range that satisfy a given predicate.
 * * This function iterates over the elements of a range defined by two iterators and counts how many elements satisfy a specified predicate.
 * * @note This is useful for scenarios where you want to count elements based on a condition, such as filtering or checking properties.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 4, 5};
 *   size_t count = count_if(vec.begin(), vec.end(), [](int value) {
 *       return value > 2; // Count elements greater than 2
 *   });
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @tparam Predicate - Type of the callable predicate to check each element
 * @note The 'Predicate' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param predicate - The predicate function to check each element
 * @return size_t - The number of elements that satisfy the predicate in the range
 */
template<typename Iterator, typename Predicate>
size_t count_if(Iterator begin, Iterator end, Predicate&& predicate) {
    return count_if_impl(begin, end, predicate);
}


/**
 * @brief Counts the number of elements in an array that satisfy a given predicate.
 * * This function iterates over the elements of an array and counts how many elements satisfy a specified predicate.
 * * @note This is useful for scenarios where you want to count elements based on a condition, such as filtering or checking properties.
 * * @example
 *   int arr[] = {1, 2, 3, 4, 5};
 *   size_t count = count_if(arr, [](int value) {
 *       return value > 2; // Count elements greater than 2
 *   });
 * 
 * @tparam T - Type of the elements in the array
 * @tparam N - Size of the array
 * @tparam Predicate - Type of the callable predicate to check each element
 * @note The 'Predicate' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param array - The array to iterate over
 * @param predicate - The predicate function to check each element
 * @return size_t - The number of elements that satisfy the predicate in the array
 */
template<typename T, size_t N, typename Predicate>
size_t count_if(const T(&array)[N], Predicate&& predicate) {
    return count_if_impl(array, array + N, predicate);
}

/**
 * @brief Copies elements from one range to another with a maximum count.
 * * This function copies elements from a source range defined by two iterators to a destination iterator, up to a specified maximum count.
 * * @note This is useful for scenarios where you want to copy a limited number of elements, such as transferring data between buffers or containers.
 * * @example
 *   std::vector<int> src = {1, 2, 3, 4, 5};
 *   std::vector<int> dest(3);
 *   unsigned copied = copy(src.begin(), src.end(), dest.begin(), 3); // Copy up to 3 elements
 * 
 * @tparam Iterator - Type of the iterator used to traverse the source range
 * @tparam OutputIterator - Type of the iterator used for the destination
 * @note The 'OutputIterator' type can be any output iterator that supports assignment
 * @param begin - Iterator pointing to the beginning of the source range
 * @param end - Iterator pointing to the end of the source range
 * @param dest_first - Iterator pointing to the beginning of the destination range
 * @param max_count - Maximum number of elements to copy
 * @return OutputIterator - An iterator pointing to the end of the copied elements in the destination range
 */
template<typename Iterator, typename OutputIterator>
OutputIterator copy_impl(Iterator begin, Iterator end, OutputIterator dest_first, unsigned max_count) {
    unsigned copied_count = 0;
    for (auto it = begin; it != end && copied_count < max_count; ++it, ++dest_first, ++copied_count) {
        *dest_first = *it;
    }
    return dest_first;
}

/**
 * @brief Copies elements from one array to another of the same size.
 * This function copies elements from a source array to a destination array of the same size.
 * 
 * @tparam T - Element type
 * @tparam N - Size of the arrays (deduced)
 */
template<typename T, size_t N>
inline void copy_array(const T(&scr)[N], T(&dest)[N])
{
    for (size_t i = 0; i < N; i++)
    {
        dest[i] = scr[i];
    }
    
}


/**
* @brief Copies up to 'count' elements from a raw pointer/array source to a raw destination array.
* 
* This overload is deliberately constrained to raw pointers/arrays and has higher priority
* than the generic Container version, preventing accidental wrong instantiation.
* 
* @note
* Use this when you have: const float* src, float* dest, uint8_t count
* 
* @tparam T        Element type
* @tparam N        Size of destination array (deduced)
* @param src       Source pointer (can be const T*)
* @param count     Number of elements to copy
* @param dest      Destination array
*/
template<typename T, size_t N>
inline void copy_array(const T* src, size_t count, T(&dest)[N])
{
    // Safety check to not copy pass the end of the dest size
    const size_t to_count = (count > N) 
        ? N
        : count; 

    for(size_t i=0; i < to_count ; i++)
    {
        dest[i] = src[i];
    }

    // Zero out the rest of the array for determistic state
    for (size_t i = to_count ; i < N; i++)
    {
        dest[i] = T{} ;
    }
    

}


/**
 * @brief Copies elements from one range to another with a maximum count.
 * * This function copies elements from a source range defined by two iterators to a destination iterator, up to a specified maximum count.
 * * @note This is useful for scenarios where you want to copy a limited number of elements, such as transferring data between buffers or containers.
 * * @example
 *   std::vector<int> src = {1, 2, 3, 4, 5};
 *   std::vector<int> dest(3);
 *   unsigned copied = copy(src.begin(), src.end(), dest.begin(), 3); // Copy up to 3 elements
 * 
 * @tparam Iterator - Type of the iterator used to traverse the source range
 * @tparam OutputIterator - Type of the iterator used for the destination
 * @note The 'OutputIterator' type can be any output iterator that supports assignment
 * @param begin - Iterator pointing to the beginning of the source range
 * @param end - Iterator pointing to the end of the source range
 * @param dest_first - Iterator pointing to the beginning of the destination range
 * @param dest_size - Maximum number of elements to copy
 * @return unsigned - The number of elements actually copied
 */
template<typename Iterator, typename OutputIterator>
unsigned copy(Iterator begin, Iterator end, OutputIterator dest_first, unsigned dest_size) {
    if (end - begin > dest_size) {
        end = begin + dest_size;
    }
    unsigned copied_count = (end - begin < dest_size) ? (end - begin) : dest_size;
    copy_impl(begin, end, dest_first, copied_count);
    return copied_count;
}

/**
 * @brief Copies elements from a container to a destination iterator with a maximum count.
 * * This function copies elements from a container to a destination iterator, up to a specified maximum count.
 * * @note This is useful for scenarios where you want to copy a limited number of elements from a container, such as transferring data between buffers or containers.
 * * @example
 *   std::vector<int> src = {1, 2, 3, 4, 5};
 *   std::vector<int> dest(3);
 *   unsigned copied = copy(src, dest.begin(), 3); // Copy up to 3 elements
 * 
 * @tparam Container - Type of the container to copy from
 * @tparam OutputIterator - Type of the iterator used for the destination
 * @note The 'OutputIterator' type can be any output iterator that supports assignment
 * @param container - The container to copy from
 * @param dest_first - Iterator pointing to the beginning of the destination range
 * @param dest_size - Maximum number of elements to copy
 * @return unsigned - The number of elements actually copied
 */
template<typename Container, typename OutputIterator>
unsigned copy(const Container& container, OutputIterator dest_first, unsigned dest_size) {
    if (container.empty() || dest_size == 0) {
        return 0;
    }
    unsigned elements_to_copy = (container.size() < dest_size) ? container.size() : dest_size;
    copy_impl(container.begin(), container.end(), dest_first, elements_to_copy);
    return elements_to_copy;
}


/**
 * @brief Copies elements from an array to a destination iterator with a maximum count.
 * * This function copies elements from an array to a destination iterator, up to a specified maximum count.
 * * @note This is useful for scenarios where you want to copy a limited number of elements from an array, such as transferring data between buffers or containers.
 * * @example
 *   int arr[] = {1, 2, 3, 4, 5};
 *   std::vector<int> dest(3);
 *   unsigned copied = copy(arr, dest.begin(), 3); // Copy up to 3 elements
 * 
 * @tparam T - Type of the elements in the array
 * @tparam N - Size of the array
 * @tparam OutputIterator - Type of the iterator used for the destination
 * @note The 'OutputIterator' type can be any output iterator that supports assignment
 * @param array - The array to copy from
 * @param dest_first - Iterator pointing to the beginning of the destination range
 * @param dest_size - Maximum number of elements to copy
 * @return unsigned - The number of elements actually copied
 */
template<typename T, size_t N, typename OutputIterator>
unsigned copy(const T(&array)[N], OutputIterator dest_first, unsigned dest_size) {
    unsigned elements_to_copy = (N < dest_size) ? N : dest_size;
    copy_impl(array, array + elements_to_copy, dest_first, elements_to_copy);
    return elements_to_copy;
}

/**
 * @brief Copies elements from a source array to a destination array with a maximum count.
 * * This function copies elements from a source array to a destination array, up to the size of the destination array.
 * * @note This is useful for scenarios where you want to copy elements between arrays, such as transferring data or initializing arrays.
 * * @example
 *   int src[] = {1, 2, 3, 4, 5};
 *   int dest[3];
 *   copy(src, dest, 3); // Copy up to 3 elements
 * 
 * @tparam T - Type of the elements in the arrays
 * @tparam N - Size of the source array
 * @param src - The source array to copy from
 * @param dest - The destination array to copy to
 * @param dest_size - Maximum number of elements to copy
 * @return T* - Pointer to the end of the copied elements in the destination array
 */
template<typename T, size_t N>
T* copy(const T(&src)[N], T(&dest)[N], unsigned dest_size) {
    unsigned elements_to_copy = (N < dest_size) ? N : dest_size;
    copy_impl(src, src + elements_to_copy, dest, elements_to_copy);
    return dest + elements_to_copy;
}

/**
 * @brief Removes elements from a range based on a predicate.
 * * This function iterates over the elements of a range defined by two iterators and removes elements that satisfy a specified predicate.
 * * @note This is useful for scenarios where you want to filter out elements based on a condition, such as removing unwanted values or properties.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 4, 5};
 *   auto new_end = remove_if(vec.begin(), vec.end(), [](int value) {
 *       return value % 2 == 0; // Remove even numbers
 *   });
 *   vec.erase(new_end, vec.end()); // Erase the removed elements
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @tparam Predicate - Type of the callable predicate to check each element
 * @note The 'Predicate' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param predicate - The predicate function to check each element
 * @return Iterator - An iterator pointing to the new end of the range after removal
 */
template<typename Iterator, typename Predicate>
Iterator remove_if_impl(Iterator begin, Iterator end, Predicate&& predicate) {
    auto write_it = begin;
    for (auto it = begin; it != end; ++it) {
        if (!predicate(*it)) {
            *write_it = *it;
            ++write_it;
        }
    }
    return write_it;
}


/**
 * @brief Removes elements from a range based on a predicate.
 * * This function iterates over the elements of a range defined by two iterators and removes elements that satisfy a specified predicate.
 * * @note This is useful for scenarios where you want to filter out elements based on a condition, such as removing unwanted values or properties.
 * * @example
 *   std::vector<int> vec = {1, 2, 3, 4, 5};
 *   auto new_end = remove_if(vec.begin(), vec.end(), [](int value) {
 *       return value % 2 == 0; // Remove even numbers
 *   });
 *   vec.erase(new_end, vec.end()); // Erase the removed elements
 * 
 * @tparam Iterator - Type of the iterator used to traverse the range
 * @tparam Predicate - Type of the callable predicate to check each element
 * @note The 'Predicate' type can be a lambda, function pointer, or any callable that takes an element as an argument and returns a boolean value
 * @param begin - Iterator pointing to the beginning of the range
 * @param end - Iterator pointing to the end of the range
 * @param predicate - The predicate function to check each element
 * @return Iterator - An iterator pointing to the new end of the range after removal
 */
template<typename Iterator, typename Predicate>
Iterator remove_if(Iterator begin, Iterator end, Predicate&& predicate) {
    return remove_if_impl(begin, end, predicate);
}

template<typename T, size_t N, typename Predicate>
T* remove_if(T(&array)[N], Predicate&& predicate) {
    return remove_if_impl(array, array + N, predicate);
}

/**
 * @brief Returns the size of an array at compile time.
 * 
 * @tparam T - Type of the elements in the array
 * @param array - The array for which to determine the size
 * @tparam N - 
 * @return constexpr size_t 
 */
template <typename T, size_t N>
constexpr size_t size(const T (&array)[N]) noexcept { return N; }


/**
 * @brief Removes the first occurrence of a value from a pointer+count range using swap-erase.
 *
 * @details Iterates to find the value, swaps it with the last element if found, clears the old last slot,
 * and decrements the count. Does not preserve order but is efficient (O(n) find + O(1) remove).
 * Returns true if removed. Ignores if not found.
 *
 * @tparam T Element type (e.g., const IObserver*).
 * @param begin Pointer to the start of the array.
 * @param count Reference to the current logical size (updated if removed).
 * @param value The value to remove.
 * @return bool True if the value was found and removed.
 */
template<typename T>
inline bool erase_value(T* begin , size_t& count, const T& value)
{
    for (size_t i = 0; i < count; i++)
    {
        if (begin[i] == value)
        {
            begin[i] = begin[count - 1];    // Swap the element to erase with the last element in the container
            begin[count -1 ] = T{};             // Clear last  slot(nullptr for pointer)
            --count;                                  // update size      
            return true;        
        }        
    }
    return false;   // return false to signal the the element to erase was not found
}

/**
 * @brief Applies a predicate to each element of an array.
 * 
 * 
 * @tparam T - Type of the elements in the array 
 * @tparam N - Size of the array
 * @tparam Predicate - Type function to apply to each element
 * @param predicate - The function to apply to each element
 */
template<typename T, size_t N, typename Predicate>
inline void apply(T(&arr)[N], Predicate&& predicate)
{
    for (size_t i = 0; i < N; i++)
    {
        predicate(arr[i]);
    }
}

/**
 * @brief Applies a predicate to each element of an array within a specified index range.
 * 
 * @tparam T - Type of the elements in the array 
 * @tparam N - Size of the array
 * @tparam Predicate - Type function to apply to each element
 * @param start_index - The starting index of the range (inclusive)
 * @param end_index - The ending index of the range (exclusive)
 * @param predicate - The function to apply to each element
 */
template<typename T, size_t N, typename Predicate>
inline void apply_within_range(T(&arr)[N], size_t start_index, size_t end_index, Predicate&& predicate)
{
    if (start_index >= N || end_index > N || start_index >= end_index)
    {
        return; // Invalid range
    }

    for (size_t i = start_index; i < end_index; i++)
    {
        predicate(arr[i]);
    }

}
} // namespace avr_algorithms