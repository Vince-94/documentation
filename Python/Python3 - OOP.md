# Python - OOP

- [Python - OOP](#python---oop)
  - [Principles](#principles)
  - [Classes and objects/instances](#classes-and-objectsinstances)
    - [Encapsulation](#encapsulation)
    - [Static methods](#static-methods)
    - [Overloading](#overloading)
  - [Design Patterns](#design-patterns)
    - [Singleton](#singleton)



## Principles



## Classes and objects/instances


### Encapsulation

```python
class ClassName:

  def __init__(self, value):

    # private attribute
    self.__attribute = value


  # attribute getter
  @property
  def Attribute(self):
    return self.__attribute

  @Attribute.setter
  # attribute setter
  def Attribute(self, value):
    self.__attribute = value

obj = ClassName(x)
obj.Attribute = y
```



### Static methods

```python
class ClassName:

  def __init__(self, value):

    # private attribute
    self.attribute = value

  @staticmethod
  def method_name():
    pass

```



### Overloading
In Python it is not possible overloading functions with the same name and different signatures. 



## Design Patterns

1. Creational Patterns
   - [Singleton](#singleton)
2. Structural Patterns
3. Behavioural patterns


### Singleton

```python
class SingletonClass():

  __instance = None

  def __init__(self):

    if SingletonClass.__instance != None:
      raise Exception('SingletonClass already instanciated once')
    else:
      SingletonClass.__instance = self
    
  @staticmethod
  def get_isntance():

    if SingletonClass.__instance == None:
      SingletonClass()
    return SingletonClass.__instance


s = SingletonClass()
print(s)
```
