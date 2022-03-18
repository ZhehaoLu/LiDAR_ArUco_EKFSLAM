#ifndef RIGID2D_INCLUDE_GUARD_HPP
#define RIGID2D_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.

#include <iosfwd> // contains forward definitions for iostream objects
#include <cmath>

namespace turtlelib
{
    /// \brief PI.  Not in C++ standard until C++20.
    constexpr double PI = 3.14159265358979323846;

    /// \brief approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 - a number to compare
    /// \param d2 - a second number to compare
    /// \param epsilon - absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    /// NOTE: implement this in the header file
    /// constexpr means that the function can be computed at compile time
    /// if given a compile-time constant as input
    constexpr bool almost_equal(double d1, double d2, double epsilon = 1.0e-12)
    {
        return std::abs(d2 - d1) < epsilon;
    }

    /// \brief convert degrees to radians
    /// \param deg - angle in degrees
    /// \returns radians
    constexpr double deg2rad(double deg)
    {
        return deg * (PI / 180.0);
    }

    /// \brief convert radians to degrees
    /// \param rad - angle in radians
    /// \returns the angle in degrees
    constexpr double rad2deg(double rad)
    {
        return (180.0 / PI) * (rad);
    }

    /// static_assertions test compile time assumptions.
    /// You should write at least one more test for each function
    /// You should also purposely (and temporarily) make one of these tests fail
    /// just to see what happens
    static_assert(almost_equal(0, 0), "is_zero failed");
    static_assert(!almost_equal(0, 1.0e-3), "is_zero failed");
    static_assert(!almost_equal(1.0e-3, 1.0e-5), "is_zero_failed");

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(45.0), 0.785375, 1.0e-2), "deg2rad failed");

    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg failed");
    static_assert(almost_equal(rad2deg(1.0), 57.29578, 1.0e-2), "rad2deg failed");

    static_assert(almost_equal(deg2rad(rad2deg(2.1)), 2.1), "deg2rad failed");
    static_assert(almost_equal(deg2rad(rad2deg(3.7)), 3.7, 1.0e-5), "deg2rad failed");

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;

        /// \brief Create a zero 2D-vector
        Vector2D();

        /// \brief Create a 2D-vector with x and y component
        /// \param x - the x component
        /// \param y - the y component
        Vector2D(double x, double y);

        /// \brief vector addition
        /// \param rhs - the vector to be added
        /// \return the output vector
        Vector2D &operator+=(const Vector2D &rhs);

        /// \brief vector substraction
        /// \param rhs - the vector to be substracted
        /// \return the output vector
        Vector2D &operator-=(const Vector2D &rhs);

        /// \brief vector multiplication by a scalar
        /// \param rhs - the scalar (left/right multiplication)
        /// \return the output vector
        Vector2D &operator*=(const double &rhs);

        /// \brief dot product of two vectors
        /// \param v1 - the vector
        /// \return the output product
        double dot(Vector2D v);
    };
    /// \brief computer the magnitude of the vector
    /// \param v - the input vector
    /// \return the output magnitude
    double magnitude(const Vector2D& v);

    /// \brief compute the angle between two vectors 
    ///  in the range of [-PI,PI] (counterclockwise - positive)
    /// \param v1 - the first vector
    /// \param v2 - the second vector
    /// \return the output angle between two vectors
    double angle(const Vector2D& v1,const Vector2D& v2);

    /// \brief vector addition
    /// \param lhs - the lefthand side vector
    /// \param rhs - the righthand side vector
    /// \return the output vector
    Vector2D operator+(Vector2D lhs, const Vector2D &rhs);

    /// \brief vector subtraction
    /// \param lhs - the lefthand side vector
    /// \param rhs - the righthand side vector
    /// \return the output vector
    Vector2D operator-(Vector2D lhs, const Vector2D &rhs);

    /// \brief scalar multiplication (scalar to the right)
    /// \param lhs - the lefthand side vector
    /// \param rhs - the righthand side scalar
    /// \return the output vector
    Vector2D operator*(Vector2D lhs, const double& rhs);

    /// \brief scalar multiplication (scalar to the left)
    /// \param lhs - the lefthand side scalar
    /// \param rhs - the righthand side vector
    /// \return the output vector
    Vector2D operator*(const double& rhs,Vector2D lhs);

    /// \brief normalize Vector2D
    /// \param vec_in the input vector
    /// \return the normalized vector
    Vector2D normalize_vec(Vector2D vec_in);

    /// \brief output a 2 dimensional vector as [xcomponent ycomponent]
    /// os - stream to output to
    /// v - the vector to print
    std::ostream &operator<<(std::ostream &os, const Vector2D &v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   [x y] or x y
    /// is - stream from which to read
    /// v [out] - output vector
    /// Hint: The following may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///
    /// The way input works is (more or less): what the user types is stored in a buffer until the user enters
    /// a newline (by pressing enter).  The iostream methods then process the data in this buffer character by character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we are expecting an int but the letter 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// We have lower level control however. For example:
    /// peek looks at the next unprocessed character in the buffer without removing it
    /// get removes the next unprocessed character from the buffer.
    std::istream &operator>>(std::istream &is, Vector2D &v);

    /// \brief a 2D-twist consists of an angular velocity and two translational velocities

    struct Twist2D
    {
        /// \param m_angular_v - angular velocity
        double m_angular_v;

        /// \param m_translation_vx - tanslational velocity x
        double m_translation_vx;

        /// \param m_translation_vy - tanslational velocity y
        double m_translation_vy;

        /// \brief Create a default constructor
        Twist2D();

        // \brief Create a twist with an angular velocity and two translational velocities
        /// \param angular_v - the angular velocity
        /// \param translation_vx - the translational velocity vx
        /// \param translation_vy - the translational velocity vy
        Twist2D(double angular_v, double translation_vx, double translation_vy);
    };

    /// \brief output a 2D-twist as [angular_v translation_vx translation_vy]
    /// \param os - stream to output to
    /// \param t - the twist to print
    std::ostream &operator<<(std::ostream &os, const Twist2D &t);

    /// \brief input a 2D-twist
    ///   You should be able to read twist entered as follows:
    ///   [angular_v translation_vx translation_vy] or angular_v translation_vx translation_vy
    /// \param is - stream from which to read
    /// \param t [out] - output twist
    std::istream &operator>>(std::istream &is, Twist2D &t);

    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    private:
        Vector2D m_translation;
        double m_radian;

    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a Vector2D
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief convert a twist to a different reference frame using the adjoint
        /// \param t - twist to be transformed by adjoint
        /// \return the transformed twist
        Twist2D operator()(Twist2D t) const;

        /// \brief invert the transformation
        /// \return the inverse transformation
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D &operator*=(const Transform2D &rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief \see operator<<(...) (declared outside this class)
        /// for a description
        friend std::ostream &operator<<(std::ostream &os, const Transform2D &tf);
        friend std::istream &operator>>(std::istream &is, Transform2D &tf);
        friend Transform2D operator*(Transform2D lhs, const Transform2D &rhs);
    };

    /// \brief should print a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream &operator<<(std::ostream &os, const Transform2D &tf);

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as output by operator<< or
    /// as 3 numbers (degrees, dx, dy) separated by spaces or newlines
    std::istream &operator>>(std::istream &is, Transform2D &tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D &rhs);

    /// \brief compute the transformation corresponding to a rigid body 
    ///  following a constant twist (in its original body frame) for one time-unit
    /// \param t - the constant twist
    /// \return the corresponding transformation
    Transform2D integrate_twist(Twist2D t);

    /// \brief make angles in the range of (-PI, PI]
    /// \param rad - the input angle in radian
    /// \return the output angle in the range of (-PI, PI]
    double normalize_angle(double rad);

}

#endif
