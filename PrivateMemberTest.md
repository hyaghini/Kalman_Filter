# Testing Private Class Members

## Introduction to gtest
GoogleTest or gtest provides us with a library that can be used for unit testing member functions/variables of the class.

In Behavior of the class members can be evaluated using several assertion statements available in gtest library.

## Accessing Private Members

Since the private members cannot be directly accessed outside of a class, several methods are proposed for testing them. 

In the following, some of these methods will be reviewed.

========================================================

### Option 1 - Friend Test
In this method, the gtest class needs to be made a friend of the class under test. Therefore, the gtest class as a friend
can access all the private member functions and variables of the class. The steps are as follows:
* Include <gtest/gtest_prod.h> header in the class under test
* Under the private section of the class add following for each test:

    FRIEND_TEST(TestClassName, TestCase);

    where TestClassName is name of the test class, and TestCase is name of the specific test.

* You have access to all the members of the class in the gtest!

=========================================================

### Option 2 - Inheritance
In this method gtest can access the protected and public members of the class by inheritance. Steps are as follow:
* Change the private members into protected type in the class under test
* Make the gtest class a child of the class under test

=========================================================

### Option 3 - Indirect Test

The private members are meant to be accessed by class members only. It is recommended that we only test the public methods.
 
The  public functions that have access to the private members should show how the private members behave. If there is an 

issue with the private members that cannot be detected by testing the public method, it is a sign of bad design, and we should consider

re-designing the class. As an example, the class can be devided into smaller classes where the private members become public members of the new classes.
