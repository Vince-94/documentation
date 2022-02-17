# C++ OOP

- [C++ OOP](#c-oop)
  - [Principles](#principles)
    - [Overloading and Overriding](#overloading-and-overriding)
  - [Classes and objects/instances](#classes-and-objectsinstances)
    - [Basic class](#basic-class)
    - [Abstract class](#abstract-class)
  - [OOD](#ood)
  - [Design Patterns](#design-patterns)
    - [Builder](#builder)
    - [Singleton](#singleton)
    - [Adapter](#adapter)
    - [Strategy pattern](#strategy-pattern)



## Principles

1. Encapsulation: group together data and methods that operate on that data
2. Abstraction
3. Ineritance
4. Polymorphism


### Overloading and Overriding


* Overriding consists in defining a child function with the same name of a function of its parent class, causing the overriding of the original function. Function overriding helps us achieve runtime polymorphism. 


```cpp
class ParentClass
{
  public: void func(){// implementation}
};

class ChildClass: public ParentClass
{
  public: void func(){// implementation}
};

int main()
{
  ChildClass obj_1, obj_2;

  obj_1.func();                   // access to child method
  obj_2.ParentClass::func();      // access to parent method

  retunr 0;
}
```



## Classes and objects/instances

OOP is based on reation of objects. Classes are templates for objects.

Common practice is defining the interface in the "header" file, while the methods implementation in the code.


### Basic class

```cpp
class ClassName {         // class name

  public:
    // constructor
    ClassName(type args){
      // implementation
      attribute = value;
    }

    // 
    method(type args){
      // implementation
    }


  protected:
    // Setter method
    void setter() {
      ...
      return;
    }

    // Getter method
    void getter() {
      ...
      return value;
    }


  private:
    // attributes
    type attribute;

};

int main()
{
  // create object
  object = ClassName(args);



  // create and object with constructor (method 2)
  object_1.function_1();

  // create and object with constructor (method 3)
  object_1.setname("Vince");

}

```



### Abstract class

```c++
class AbstractClassName {
  virtual void Function_name() = 0;     // abstract function
};
```


## OOD



## Design Patterns

1. Creational Patterns
   - [Builder](#builder)
   - [Singleton](#singleton)
2. Structural Patterns
   - [Adapter](#adapter)
3. Behavioural patterns
   - [Strategy pattern](#strategy-pattern)


### Builder
The builder pattern is a design pattern that allows for the step-by-step creation of complex objects using the correct sequence of actions. The construction is controlled by a **director** object that only needs to know the type of object it is to create.

```cpp
class Director
{
  public void Construct(Builder builder)
  {
    builder.BuildPart1();
    builder.BuildPart2();
    builder.BuildPart3();
  }
}
 
 
abstract class Builder
{
  public abstract void BuildPart1();
  public abstract void BuildPart2();
  public abstract void BuildPart3();
  public abstract Product GetProduct();
}
 
 
class ConcreteBuilder : Builder
{
  private Product _product = new Product();
 
  public override void BuildPart1()
  {
    _product.Part1 = "Part 1";
  }
 
  public override void BuildPart2()
  {
    _product.Part2 = "Part 2";
  }
 
  public override void BuildPart3()
  {
    _product.Part3 = "Part 3";
  }
 
  public override Product GetProduct()
  {
    return _product;
  }
}
 
 
class Product
{
  public string Part1 { get; set; }
  public string Part2 { get; set; }
  public string Part3 { get; set; }
}
```







### Singleton
The singleton pattern is a design pattern that is used to ensure that a class can only have one concurrent instance. Whenever additional objects of a singleton class are required, the previously created, single instance is provided.

```cpp
class Singleton
{
  private:
    static Singleton _instance;   // private constructor
    Singleton() { 
      // implementation
    } 

  public:
    // the static method ensure that the object is instanciated only once and then is not modified anymore
    static Singleton GetSingleton()
    {
      _instance = new Singleton();
      return _instance;
    }
}
```



### Adapter
The adapter pattern is used to provide a link between two otherwise incompatible types by wrapping the "adaptee" with a class that supports the interface required by the client.

```cpp
// class which requires the use of an incompatible type of the Adaptee class
class Client
{
  private ITarget _target;
 
  public Client(ITarget target)
  {
    _target = target;
  }
 
  public void MakeRequest()
  {
    _target.MethodA();
  }
}
 
// expected interface for the client class
interface ITarget
{
  void MethodA();
}
 
// the functionality that is required by the client
class Adaptee
{
  public void MethodB()
  {
    Console.WriteLine("MethodB called");
  }
}
 
// provides the link between the incompatible Client and Adaptee classes
class Adapter : ITarget
{
  private Adaptee _adaptee = new Adaptee();
 
  public void MethodA()
  {
    _adaptee.MethodB();
  }
}
```



### Strategy pattern
The strategy pattern is a design pattern that allows a set of similar algorithms to be defined and encapsulated in their own classes. The algorithm to be used for a particular purpose may then be selected at run-time according to your requirements.

```cpp
// user class implementing the algorithm that sets at runtime a specific strategy
class Client
{
  public StrategyBase Strategy { get; set; }
 
  public void CallAlgorithm()
  {
    Console.WriteLine(Strategy.Algorithm());
  }
}
 
// base class for all classes that provide algorithms
abstract class StrategyBase
{
  public abstract string Algorithm();
}
 
// classes that implement the different behaviours

class ConcreteStrategyA : StrategyBase
{
  public override string Algorithm()
  {
    return "Concrete Strategy A";
  }
}
 
 
class ConcreteStrategyB : StrategyBase
{
  public override string Algorithm()
  {
    return "Concrete Strategy B";
  }
}
```






