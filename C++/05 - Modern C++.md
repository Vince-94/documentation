# Modern C++


## Data type
* Auto type deduction
  ```cpp
  auto x = ...
  ```




## Class

* Use of "override" function



## Array

* Not contiguous array: the array points to a contiguous list (in memory) of pointers, each of them points to a particlar object (not contiguous):
  ```cpp
  vector<unique_ptr<ClassName>> v;
  ```
* Contiguous array: the array points to a list (in memory) of objects:
  ```cpp
  vector<ClassName> v(a, b, c);
  ```
* Contiguous fixed array: the array is a list of contiguous objects itself (only if the lenght of the array N is known)
  ```cpp
  array<ClassName, N> v(a, b, c);
  ```


## Pointers
* Use of smart pointers: instead of "new", it is raccomanded to use "make_unique/make_shared".
  ```cpp
  container<smart_ptr<base>>

  // example 1: owning
  vector<unique_ptr<shapes>> shapes;

  // example 2: not owning
  vector<shapes*> shapes;

  ```
* No need to "delete". It is done automatically



## Loops
* standard for/while/do

* for_each: to visit each element
  ```cpp
  for_each(begin(v), end(v), [](string& s) {
    ...
  });
  ```

* find_if: to find matching elements
  ```cpp
  find_if(begin(v), end(v), [](int i) {
    return expression;
  });
  ```

