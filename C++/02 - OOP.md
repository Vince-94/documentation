# C++ OOP


- [C++ OOP](#c-oop)
  - [Principles](#principles)
  - [Classes](#classes)
    - [class_interface.h](#class_interfaceh)
    - [class_interface.cpp](#class_interfacecpp)
  - [Abstract class](#abstract-class)


## Principles
1. Encapsulation: group together data and methods that operate on that data
2. Abstraction
3. Ineritance
4. Polymorphism

## Classes

Common practice is defining the interface in the "header" file, while the methods implementation in the code.

### class_interface.h

```cpp

#ifndef POINT_2D_H
#define POINT_2D_H
class ClassInterface
{
    public:
    // define public variables and methods

    // define private variables and methods

};
#endif

```

### class_interface.cpp

```cpp
usingsts::string

class ClassName {               // class name

    private: 
        string Name             // class attribute
        int Year;               // class attribute


    public:

        // Create a setter
        void setname(string name) {
            Name = name;
        };

        // Create a getter
        void getName() {
            return Name;
        };


        // Method
        void function_1(){
            std::cout << "Name: " << Name << std:end1;
            std::cout << "Year: " << Year << std:end1;
        }

        // Constructure
        ClassName(string name, string year) {
            Name = name;
            Year = year;
        }

};


int main()
{

    // create and object with constructor (method 2)
    ClassName object_2 = ClassName("Vince", 1994);
    object_1.function_1();


    // create and object with constructor (method 3)
    object_1.setname("Vince");


}
```



## Abstract class

```c++
class AbstractClassName {
    virtual void Function_name() = 0;       // abstract function
};
```
