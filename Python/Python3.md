# Python

- [Python](#python)
  - [Build-in data-type](#build-in-data-type)
    - [int/float/complex](#intfloatcomplex)
    - [str](#str)
    - [list](#list)
    - [tuple](#tuple)
    - [range](#range)
    - [dict](#dict)
    - [set](#set)
    - [frozenset](#frozenset)
    - [bool](#bool)
    - [bytes](#bytes)
    - [bytearray](#bytearray)
  - [Iterators](#iterators)
    - [Range](#range-1)
    - [Enumerate](#enumerate)
  - [Statement](#statement)
    - [if condition](#if-condition)
    - [if in condition](#if-in-condition)
    - [while loop](#while-loop)
    - [for loop](#for-loop)
    - [any statement](#any-statement)
  - [Function](#function)
    - [args and kwargs](#args-and-kwargs)
    - [Lambda function](#lambda-function)
    - [Function decorator](#function-decorator)
  - [Generator](#generator)
    - [Generator functions](#generator-functions)
    - [Generator expressions](#generator-expressions)
  - [Operators](#operators)
  - [I/O](#io)
    - [File handling](#file-handling)
  - [Environment](#environment)
    - [Modules manager](#modules-manager)
    - [Virtual environment](#virtual-environment)
  - [Exception](#exception)
    - [Raise exceptions](#raise-exceptions)
    - [Catch errors](#catch-errors)
  - [Context management](#context-management)



## Build-in data-type
Data-types are divided according to:
* modification: allow or not data modification (add, assignment, delete elements)
* iterativity: can be traversed through all the values (with for loop), returning its member one at a time
* ordered: items have or not a defined order
* data: admit a unique data type or not
* type: the type of the structure


| **Data-types**    | modification | iterativity | ordered   | data           | type       |
| ----------------- | ------------ | ----------- | --------- | -------------- | ---------- |
| int/float/complex | immutable    |             |           |                |            |
| str               | immutable    | iterable    | ordered   | unique type    | sequence   |
| list              | mutable      | iterable    | ordered   | arbitrary type | sequence   |
| tuple             | immutable    | iterable    | ordered   | arbitrary type | sequence   |
| range             |              |             |           |                | sequence   |
| dict              | mutable      | iterable    | unordered |                | map        |
| set               | mutable*     | iterable    | unordered | unique value   | collection |
| frozenset         | immutable    | iterable    | unordered | unique value   | collection |
| bool              |              |             |           |                |            |
| bytes             | immutable    |             |           |                |            |
| bytearray         | mutable      |             |           |                |            |

*: mutable but cannot be assigned

*Notes:* Immutable types are hashtable and work well as dict keys.



### int/float/complex
```python
int_ = 1
float_ = 0.1
complex_ = 1 + 1j
```


### str
key: immutable, ordered, sequence, allow duplicate values with character data-type

```python
str_ = 'text'

# indexing
value = str_[i]

# slicing
str_[i:s:j]     # slice form i to j index with step s

# operation
str_ = str_1 + str_2

# iteration
for x in str_:
  expression(x)

# methods
str_ = str_.strip()    # remove white spaces
str_ = str_.upper()
str_ = str_.lower()
bool_ = str_.startswith(expression)
bool_ = str_.endswith(expression)
idx = str_.fin(expression)    # return the index where the expression is found
str_ = str_.count(expression)
str_ = str_.replace(origin_exp, new_exp)

str_ = str_.split(exp)   # convert str into list of str with exp as delimiter
str_ = exp.join(list_)   # convert list of str into str with 'exp' as delimiter
str_ = str_.
str_ = str_.
str_ = str_.

# formatting
new_str = '%s/%d/%.2f' % (str_, int_, float_)
new_str = '{}, {}, {:.2f}'.format(str_, int_, float_)
new_str = f'{str_}, {int_}, {float_}'
```


### list
key: mutable, ordered, sequence, allow duplicate values with arbitrary data-type
```python
list_ = [elem_1, ..., elem_i, ..., elem_n]

# conversion
list_ = list(tuple_)

# indexing
value = list_[i]

# slicing
list_[i:s:j]     # slice form i to j index with step s

# assignation
list_[i] = x

# deleting element
del list_[i]

# iteration
for x in list_:
  expression(x)

# comprehension
new_list = [expression(x) for x in list_]
new_list_condition = [expression(x) for x in list_ if (condition)]

# methods
list_lenght = len(list_)
sorted_list = sorted(list_)

# inplace methods
list_.append(x)     # add element at the end
list_.insert(i, elem_i)    # add elem_i at i-th index
list_.pop()               # remove the last element
list_.remove(elem_i)     # remove specific element
list_.clear()             # remove all the lements
list_.reverse()           # reverse the order
list_.sort()              # sort

# operations
new_list = list_ * int_
new_list = list_1 + list_2

# create a copy
copy_list = list_.copy()
copy_list = list(list_)

```


### tuple
key: immutable, ordered, sequence, allow duplicate values with arbitrary data-type

```python
tuple_ = (elem_1, ..., elem_i, ..., elem_n)

# conversion
tuple_ = tuple(list_)

# indexing
value = tuple_[i]

# slicing
new_tuple = tuple_[i:s:j]       # slice form i to j index with step s

# iteration
for x in tuple_:
  expression(x)

# methods
tuple_length = len(tuple_)
elem_length = tuple_.count(elem_i)    # lenght of elem_i
index = tuple_.index(elem_i)    # index of elem_i
```

### range
```python
range_ = range(6, 1, 10)
```

### dict
key: mutable, unordered, map

```python
dict_ = {"key_1": value_1, "key_2": value_2, ... "key_i": value_i, ... "key_n": value_n}
dict_ = dict("key_1": value_1, "key_2": value_2, ... "key_i": value_i, ... "key_n": value_n)

# index
value = dict_[key_i]

# iteration
for key in dict_.keys():
  expression(key)

for value in dict_.values():
  expression(value)

for key, value in dict_.items():
  expression(key, value)

# comprehension
keys_list = [key_1, key_2, key_3]
values_list = [value_1, value_2, value_3]
new_dict = {key: value for key, value in zip(keys_list, values_list)}

# methods
dict_['key'] = value      # add new key-value pair or overwrites if already exists
del dict_['key']          # delete key-value pair

# inplace methods
dict_.pop()               # delete the last key-value pair
dict_.popitem()           # delete the last inserted key-value pair
dict_1.update(dict_2)     # merge dict_2 into dict_1


# create a copy
copy_dict = dict_.copy()
copy_dict = dict(dict_)

```

### set
key: mutable, unordered, sequence, does not allow duplicates

```python
set_ = {elem_1, ..., elem_i, ..., elem_n}
set_ = set(list_)

# iteration
for elem in set_:
  expression(elem)

# methods
union_set = set_1.union(set_2)            # union set
intersect_set = set_1.intersect(set_2)    # intersection set
diff_set = set_1.difference(set_2)        # set of element of set_1 not in set_2
sym_set = set_1.symmetric_difference(set_2)        # set of element of set_1 not in set_2 and vice-versa
set_1.issubset(set_2)                     # return True if set_1 is a subset of set_2
set_1.issuperset(set_2)                     # return True if set_1 is a superset of set_2
set_1.isdisjoint(set_2)                     # return True if the sets do not have the same elements


# inplace methods (create a copy)
set_.add(elem_i)
set_.remove(elem_i)     # remove existing element
set_.discard(elem_i)    # remove element if exists
set_.clear()            # empty set
set_.pop()              # remove an element
set_1.update(set_2)     # merge set_2 into set_1
set_1.intersection_update(set_2)     # merge keeping only the elements in both sets
set_1.difference_update(set_2)     # merge keeping only the elements of set_1 not in set_2
```

### frozenset
key: immutable, unordered, sequence, does not allow duplicates

```python
frozenset_ = frozenset({"apple", "banana", "cherry"})
frozenset_ = frozenset(list_)
```

### bool
```python
bool_ = True
```

### bytes
key: immutable sequence of bytes

```python
bytes_ = bytes((0, 1, 2, 3))
```

### bytearray
key: mutable sequence of bytes

```python
bytearray_ = bytearray((0, 1, 2, 3))
```


## Iterators


### Range

```python
list_ = [elem_1, ..., elem_N]

for i in range(len(list_)):
  #expression
```


### Enumerate

```python
list_ = [elem_1, ..., elem_N]

for idx, num in enumerate(list_):
  #expression
```



## Statement


### if condition

```python
if (condition):
  # expression
elif (condition):
  # expression
else:
  # expression
```


### if in condition
```python
if x in iterable:
  # expression
```



### while loop

```python

```


### for loop

```python

```


### any statement
Retrun true if any of the element match the condition.

```python
x = any(condition for i in list_)
```




## Function

```python
def function(arg_1,...,arg_n):
  expression
  return

value = function(param_1, ..., param_n)
```

### args and kwargs
It is possible passing an indefined number of parameters with:
* args: list of parameters
* kwargs: dictionary of key-paramter pairs

```python
def myfunction(*args, **kwargs):
  param_i = args[index]
  param_j = kwargs[key]

```


### Lambda function

```python
l_funtion = lambda arg_1,...,arg_n: expression

value = l_funtion(param_1, ..., param_n)
```


### Function decorator

Function decorators are function that extends capabilities of a function.

```python
def function_decorator(fun):
  
  def wrapper(*args, **kwargs):
    # expression 
    value = fun(*args, **kwargs)
    # expression
    return value

  return wrapper


@function_decorator
def function(args):
  expression
  return expression

value = function(param_1, ..., param_n)
```


## Generator

Generators are functions that returns an iterable object, through the ***yield*** keyword. A big advantages is that with large data, they save a lot of memory.

### Generator functions

```python
def fun_gen():
  yield 1
  ...
  yield n

g = fun_gen()   # iterable

next(g)   # process the next yield value
```

Example: Fibonacci sequence
```python
def fibonacci_gen(limit):
  a, b = 0, 1
  while a < limit:
    yield a
    a, b = b, a + b

fib_num = fibonacci_gen(10)   # iterable

for i in fib_num:
  print(i)
```


### Generator expressions

Generator expressions are written likely "list comprehension":

```python
gen_obj = (expression(x) for x in range())
gen_obj_condition = (expression(x) for x in range() if (condition))
```


## Operators


| **Arithmetic Operators** |      |
| ------------------------ | ---- |
| Addition                 | `+`  |
| Subtraction              | `-`  |
| Multiplication           | `*`  |
| Division                 | `/`  |
| Modulus                  | `%`  |
| Exponentiation           | `**` |
| Floor division           | `//` |


| **Comparison Operators**      |           |
| ----------------------------- | --------- |
| Equal                         | `==`      |
| Not equal                     | `!=`      |
| Greater/less than             | `>` `<`   |
| Greater/less than or equal to | `>=` `<=` |


| **Logical Operators** |       |
| --------------------- | ----- |
| And                   | `and` |
| Or                    | `or`  |
| Not                   | `not` |


| **Other Operators**                                                      |          |
| ------------------------------------------------------------------------ | -------- |
| True if both variables are the same object                               | `is`     |
| True if both variables are not the same object                           | `is not` |
| True if a sequence with the specified value is present in the object     | `in`     |
| True if a sequence with the specified value is not present in the object | `not in` |



| **Bitwise Operators** |      |
| --------------------- | ---- |
| AND                   | `&`  | Sets each bit to 1 if both bits are 1                                                                   |
| OR                    | `\|` | Sets each bit to 1 if one of two bits is 1                                                              |
| XOR                   | `^`  | Sets each bit to 1 if only one of two bits is 1                                                         |
| NOT                   | `~`  | Inverts all the bits                                                                                    |
| Zero fill left shift  | `<<` | Shift left by pushing zeros in from the right and let the leftmost bits fall off                        |
| Signed right shift    | `>>` | Shift right by pushing copies of the leftmost bit in from the left, and let the rightmost bits fall off |



| **Assignment Operators** |                                      |
| ------------------------ | ------------------------------------ |
| Equal                    | `=`                                  |
| Arithmetich              | `+=` `-=` `*=` `/=` `%=` `**=` `//=` |
| Logical                  | `&=` `\|=`                           |
| ??                       | `^=`                                 |
| ??                       | `>>=` `<<=`                          |







## I/O

### File handling

``` python
open("file_name.txt", "option")
```
option:
- r (read), a (append), w (write), x (create)



## Environment

### Modules manager
Create the list of dependancies
```sh
pip3 freeze > requirements.txt
```

Install dependancies
```sh
pip3 install -r requirements.txt
```

Unistall dependancies
```sh
pip3 uninstall -r requirements.txt
```


### Virtual environment
Set virtual environment

```sh
python3 -m penv [virtual_name]
source [virtual_name]/bin/activate
```

## Exception

### Raise exceptions

```python
if condition:
  raise Exception('Error message')
```

```python
assert(condition), 'Error message'
```


### Catch errors

```python
try:
  expression

except:
  print('Error message')
```

```python
try:
  expression

except SpecificError as ex:
  print('Error: {}'.format(ex))

except Exception as ex:
  print('Error: {}'.format(ex))

finally:
  expression
```


## Context management

It is useful for resources management

```python

```


