# Comparing Methods for Accessing Private Class Members

## Introduction
In this document, two methods for testing private members of a class is introduced and compared with each other. In the following, first, gtest as

a tool for testing class members is briefly introduced. Then, the problem of testing private members is discussed.

#### GTest:
GoogleTest or gtest provides us with a framework that can be used for unit testing member functions/variables of the class.

Behavior of the class members can be evaluated using several assertion statements available in gtest library.

As an example of the assertion statements, the following can be mentioned:

|Fatal assertion | Nonfatal assertion | Verifies|
|----------------|--------------------|---------|
|ASSERT_TRUE(condition); | EXPECT_TRUE(condition); | condition is true|
|ASSERT_FALSE(condition); | EXPECT_FALSE(condition);| condition is false|
|ASSERT_EQ(val1, val2); | EXPECT_EQ(val1, val2); | val1 == val2|
|ASSERT_NE(val1, val2); | EXPECT_NE(val1, val2); | val1 != val2|

- More information about the gtest can be found in [Googletest Primer github page](https://github.com/google/googletest/blob/master/googletest/docs/primer.md/)

#### Accessing Private Members:

For testing the class member variables and functions, we need to access them. Since the private members cannot be directly accessed outside of a 
class, two methods including Friend Test and Inheritance are proposed for accessing and testing them. 

========================================================

## Comparison of Methods for Accessing Private Members
In the following, two methods for accessing private members will be reviewed.


### Method 1 - Friend Test
In this method, the gtest class needs to be made a friend of the class under test. Therefore, the gtest class as a friend
can access all the private member functions and variables of the class. The steps are as follows:
* Include <gtest/gtest_prod.h> header in the class under test
* Under the private section of the class add following for each test:

    FRIEND_TEST(TestClassName, TestCase);

    where TestClassName is name of the test class, and TestCase is name of the specific test.

* You have access to all the members of the class in the gtest!

Example:

Assume, you want to test private member **bar** in the class **Foo**
```
class Foo
{
private:
    int bar(...)
}
```
Having the following gtest case:
```
TEST(Foo, barReturnsZero)
{
    Foo foo;
    EXPECT_EQ(foo.bar(...), 0);
}
```
We need to add the friend statement to the class **Foo** as follows:
```
class Foo
{
private:
    FRIEND_TEST(Foo, barReturnsZero);
    int bar(...);
}
```


### Method 2 - Inheritance
In this method gtest can access the protected and public members of the class by inheritance. Steps are as follow:
* Change the private members into protected type in the class under test
* Make the gtest class a child of the class under test

Example:

Assume, you want to test private member **bar** in the class **Foo**
```
class Foo
{
private:
    int bar(...)
}
```
First, we need to change the private members into the protected ones.
```
class Foo
{
protected:
    int bar(...)
}
```
Then, the gtest class needs to be a child of the class under test:
```
class FooTest : public Foo, ::testing::Test
{
protected:
...
    void SetUp() override{...}
...
}
```

=========================================================

#### Advantages and Disadvantages of Method 1 and Method 2
|Friend Test  |Inheritance  |
|-------------|-------------|
|(**Disadvantage**) Needs to include gtest Libraries and link the test cases in the class under test    | (**Advantage**) Nothing from gtest is included or linked in the class under test   |
|(**Disadvantage**) Compiling and running the class depends on the gtest libraries                      | (**Advantage**) The class under test is independent of gtest libraries             |
|(**Advantage**) Keeps the private members of the class as private                                   | (**Disadvantage**) Needs to modify the class and change the private members to protected |

=========================================================

## Conclusion

* Although the introduced methods makes the private members accessible, both methods have disadvantages that make them inappropriate to use. 
The main disadvantage of the Friend Test method is dependency of the class to gtest. This can be dangerous, for example on a self-driving car, 
if the gtest library is not available, the system will crash. On the other hand, the inheretance method is not recommended since it changes the
private member into the protected ones. Therefore, child classes of the class under test will have access to the members that were supposed to be private that is dangerous.


* The private members are meant to be accessed by class members only. It is recommended that we only test the public methods.
The  public functions that have access to the private members should show how the private members behave. If there is an 
issue with the private members that cannot be detected by testing the public method, it is a sign of bad design, and we should consider
re-designing the class. As an example, the class can be divided into smaller classes where the private members become public members of the new classes.

