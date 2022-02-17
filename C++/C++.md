# C++

- [C++](#c)
  - [Build-in data types](#build-in-data-types)
    - [Array](#array)
    - [Auto (C++11)](#auto-c11)
  - [Statemets](#statemets)
    - [if condition](#if-condition)
    - [switch condition](#switch-condition)
    - [while loop](#while-loop)
    - [for loop](#for-loop)
    - [range for loop (C++11)](#range-for-loop-c11)
    - [for each loop (C++11)](#for-each-loop-c11)
    - [find if loop (C++11)](#find-if-loop-c11)
  - [Functions](#functions)
  - [Namespace](#namespace)
  - [Pointers](#pointers)
    - [Raw pointers](#raw-pointers)
    - [Smart pointers (C++11)](#smart-pointers-c11)
  - [Memory allocation](#memory-allocation)
    - [Static allocation (stack)](#static-allocation-stack)
    - [Dynamic allocation (heap)](#dynamic-allocation-heap)
  - [Header file](#header-file)



## Build-in data types

* Primary
  * int (integer)
    * int
    * (signed | unsigned) (short | long | long long) int
  * floating point
    * float
    * double
    * long double
  * character
    * (signed | unsigned) char
    * wchar_t
    * char16_t
    * char32_t
  * others
    * bool
    * void
* Derived
  * function
  * array
  * string
  * pointer
  * refernce
* User defined
  * class
  * struct
  * union
  * enum
  * typedef
  * template


### Array

An array (raw array) is a pointer of a contiguous list (in memory) of pointers, each of them points to a particlar object (not contiguous).

```cpp
// initialize a vector
vector<type> v;
vector<unique_ptr<ClassName>> v;

// initialize a vector with a given dimension n
vector<type> v(N);

// initialize a vector with values
vector<type> v{elem_1, ..., elem_n};
```

*Note:* From C++11 an array points to a contiguous list (in memory) of objects.
```cpp
vector<ClassName> v(elem_1, ..., elem_n);
```

*Note:* From C++11 it is added the possibility to define an array as a list of contiguous objects itself (only if the lenght of the array N is known).
```cpp
array<ClassName, N> v(elem_1, ..., elem_n);
```




### Auto (C++11)
Auto is a keyword to autodeduct the type of an already created variable.

```cpp
auto x = ...
```


## Statemets

### if condition

```cpp

```


### switch condition

```cpp

```


### while loop

```cpp
while (condition){
  expression;
}
```

*Note:* An empty condition is equal to `while(True)` which is an endless loop.



### for loop

```cpp
for (init-statement; condition; iteration-expression*){
   expression;
}
```



### range for loop (C++11)
It executes a for loop over a range.

```cpp
for (range_element : range_expression) {
  expression;
}
```


### for each loop (C++11)
It executes a for loop returning each element of an expression.

```cpp
for_each (start_iter, last_iter, [](type& expression){
  expression;
  }
);
```


### find if loop (C++11)
It executes a for loop returning True if the element match the condition, False otherwise.

```cpp
find_if(start_iter, last_iter, [](type& expression){
  expression;
  }
);
```





## Functions

**Function prototype**: contains the prototype of the function. It is common practice to store the prototypes inside a "header" file that is invoced by the "#include" directive. The input parameters are the variable to pass to the function in order to get a result. Besides it is possible to indicate "optional" parameters, that are not necessary to call. For these parameteres it is necessary to indicate a default value.

**Function declaration**: contains the implementation of the function.

**Function call**: call the function and store the returned value.


```cpp
// Function prototype
type function_name(type param_1, ..., type param_n = value);

// Function declaration
type function_name(type param_1, ..., type param_n = value){
  // implementation
  return result;
}

// Function call
value = function_name(arg, ...)
```

For void functions:
```cpp
// Function prototype
void function_name(type param_1, ..., type param_n = value);

// Function declaration
void function_name(type param_1, ..., type param_n = value){
  // implementation
  return;
}

// Function call
function_name(arg, ...)
```



## Namespace

A namespace allows to group named entities that otherwise would have global scope (namespace scope), then it is a declarative region that provides a scope to the identifiers (names of the types, function, variables etc) inside it.

```cpp
// namespace definition
namespace ns_name
{
	// elements
  type variable;
	... 
}

int main()
{
  ns_name::variable;
	return 0;
}
```



## Pointers

A pointer is a variable that stores the memory address of an object. They helps to:
* allocate new objects on the heap,
* pass functions to other functions,
* iterate over elements in some data structures.

*Notes:* Raw pointers are useful for performance benefit, but they are the source of many serious programming errors. Modern C++ provides smart poiters to solve raw pointers problems.


### Raw pointers

```cpp
// initialize a pointer and assign a variable to it
type* ptr = nullptr

// referencing: assign pointer to address of object
ptr = &x

// dereferencing: retrieve the value at its address
int adr = *ptr;

// delete pointers
delete ptr;
```


### Smart pointers (C++11)

The use of smart pointers it is raccomanded isntead of the raw "new". It's main advantage is that is automatically deleted once it is 


* Use of smart pointers: instead of "new", it is raccomanded to use "make_unique/make_shared".
  ```cpp
  container<smart_ptr<base>>

  // example 1: owning
  vector<unique_ptr<shapes>> shapes;

  // example 2: not owning
  vector<shapes*> shapes;

  ```
* No need to "delete". It is done automatically





## Memory allocation


### Static allocation (stack)

```cpp
// variable declaration and assignament
type variable = value
```


### Dynamic allocation (heap)

```cpp
// pointer creation
type* pointer_name;

// pointer allocation
pointer_name = new type;

// free allocated memory
delete pointer_name;
```



## Header file

```cpp
#ifndef NAME_H
#define NAME_H


#endif
```
