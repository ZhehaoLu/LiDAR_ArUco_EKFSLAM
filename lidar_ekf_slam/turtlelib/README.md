# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- diff_drive - Handles kinematics calculations for turtlebot
- frame_main - Perform some rigid body computations based on user input

# Conceptual Questions
1. We need to be able to  (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality
   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

* (1) Implementing as a member function that modifies the x and y component of the Vector2D that calls this function. It requires additional members, norm_x and norm_y, in the struct, which isn't what we want. But it improves readability and makes it easier to maintain.

* (2) Implement as a non-member function that takes Vector2D by value and returns a modified Vector2D. It requires more computation and memory space, but it avoids the situation that you mistakenly altering the Vector2D object which is passed to the function.

* (3) Implement as a non-member function that takes Vector2D by reference and returns the same Vector2D object. It is a more efficient way by altering the x and y component but is irrecoverable. 

   - Which of the methods would you implement and why?
* I use the second solution. Although it seems to be less efficient in this case, it improves robustness and flexibility when we want to use both the original and the normalized Vector2D object.

2. What is the difference between a class and a struct in C++?

* Use class rather than struct if any member is non-public (members of a struct are public in default).

* Use class if the class has an invariant; use struct if the data members can vary independently.

* inheritance is possible in class but not possible in struct.

3. Why is Vector2D a struct and Transform2DClass (refer to at least 2 specific C++ core guidelines in your answer)?

* C.8: Use class rather than struct if any member is non-public. Vector2D has 2 public members while Transform2D has 2 private members. It helps improve readability and makes it easier to maintain.

* C.2: Use class if the class has an invariant; use struct if the data members can vary independently. In Vector2D, the x and y component can vary seperately of each other, while a Transform2D stores the relation between two different frames, so the theta and translation component relies on each other.


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?

* C.46: By default, declare single-argument constructors explicit. All of the constructors in Transform2D are single-argument constructors. It helps avoid unintended implicit conversions.

5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

* Con.2: By default, make member functions const. A member function should be marked const unless it changes the object’s observable state. Transform2D::inv() function returns a new member of Transform2D and maintains the original one unchanged, while Transform2D::operator*=() function modifies the value of parameter that is transmitted to it.


   

# Sample Run of frame_main
Enter transform T_{a,b}:
deg: 90 x: 0 y: 1
Enter transform T_{b,c}:
deg: 90 x: 1 y: 0
T_{a,b}: deg: 90 x: 0 y: 1
T_{b,a}: deg: -90 x: -1 y: -6.12323e-17
T_{b,c}: deg: 90 x: 1 y: 0
T_{c,b}: deg: -90 x: -6.12323e-17 y: 1
T_{a,c}: deg: 180 x: 6.12323e-17 y: 2
T_{c,a}: deg: -180 x: -1.83697e-16 y: 2
Enter vector v_b:
1 1
v_bhat: [0.707107 0.707107]
v_a: [-1 2]
v_b: [1 1]
v_c: [1 1.11022e-16]
Enter twist V_b:
1 1 1
V_a [1 0 1]
V_b [1 1 1]
V_c [1 2 -1]



