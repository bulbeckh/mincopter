
// Implementation of delegate calls using C++11
//
// Alternative to FastDelegate


#ifdef TARGET_ARCH_AVR
// Use minimal implementation of forward and remove_references
#include <stdint.h>

// Minimal type_traits subset for AVR
template<typename T> struct remove_reference      { typedef T type; };
template<typename T> struct remove_reference<T&>  { typedef T type; };
template<typename T> struct remove_reference<T&&> { typedef T type; };

template<typename T>
constexpr T&& forward(typename remove_reference<T>::type& t) noexcept {
    return static_cast<T&&>(t);
}
template<typename T>
constexpr T&& forward(typename remove_reference<T>::type&& t) noexcept {
    return static_cast<T&&>(t);
}

#else
// Use libstdc++
#include <utility>
#include <type_traits>

using namespace std;

#endif

template <typename>
class Delegate;

template <typename R, typename... Args>
class Delegate<R(Args...)> {
    using StubType = R(*)(void*, Args&&...);

    void* object_ptr = nullptr;
    StubType stub_ptr = nullptr;

public:
    Delegate() = default;

    // Bind a free function or static function
    template <R (*Func)(Args...)>
    static Delegate Create() {
        Delegate d;
        d.object_ptr = nullptr;
        d.stub_ptr = [](void*, Args&&... args) -> R {
            return Func(forward<Args>(args)...);
        };
        return d;
    }

    // Bind a member function
    template <typename T, R (T::*Func)(Args...)>
    static Delegate Create(T* instance) {
        Delegate d;
        d.object_ptr = instance;
        d.stub_ptr = [](void* obj, Args&&... args) -> R {
            return (static_cast<T*>(obj)->*Func)(forward<Args>(args)...);
        };
        return d;
    }

    R operator()(Args... args) const {
        return stub_ptr(object_ptr, forward<Args>(args)...);
    }

    explicit operator bool() const noexcept {
        return stub_ptr != nullptr;
    }

	bool operator==(const Delegate& other) const noexcept{
		return (object_ptr == other.object_ptr) && (stub_ptr == other.stub_ptr);
	}

	bool operator!=(const Delegate& other) const noexcept {
		return !(*this == other);
	}

	// NOTE We actually use NULL in some of the Scheduler classes to check if instances to timer processes are
	// present before we call them but C++ will implicitly cast to nullptr so we just have to override the nullptr comparators here
	
	bool operator==(decltype(nullptr)) const noexcept {
		return stub_ptr == nullptr;
	}

	bool operator!=(decltype(nullptr)) const noexcept {
		return stub_ptr != nullptr;
	}
};


/* Example usage
 *
 * auto d1 = Delegate<void(int)>::Create<&GlobalHandler>();
 * auto d2 = Delegate<void(int)>::Create<Foo, &Foo::MemberFunc>(&f);
 *
 * d1(10);
 * d2(20);
 *
 * */
