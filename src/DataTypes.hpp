#ifndef _DATATYPES_HPP_
#define _DATATYPES_HPP_

#include <gazebo/common/common.hh>
#include <gazebo/math/Vector3.hh>
#include <gazebo/math/Matrix3.hh>
#include <vector>
#include "uw_msgs.pb.h"

namespace gazebo_underwater
{

struct Matrix6
{
    gazebo::math::Matrix3 top_left;
    gazebo::math::Matrix3 top_right;
    gazebo::math::Matrix3 bottom_left;
    gazebo::math::Matrix3 bottom_right;

    Matrix6():
        top_left(gazebo::math::Matrix3::ZERO),
        top_right(gazebo::math::Matrix3::ZERO),
        bottom_left(gazebo::math::Matrix3::ZERO),
        bottom_right(gazebo::math::Matrix3::ZERO)
    {}
    Matrix6(const gazebo::math::Matrix3 &tl, const gazebo::math::Matrix3 &tr,
                const gazebo::math::Matrix3 &bl, const gazebo::math::Matrix3 &br):
        top_left(tl),
        top_right(tr),
        bottom_left(bl),
        bottom_right(br)
    {}
    Matrix6(const Matrix6 &matrix):
        Matrix6(matrix.top_left, matrix.top_right, matrix.bottom_left, matrix.bottom_right)
    {}
    inline Matrix6& operator+=(const Matrix6 &value)
    {
        this->top_left = this->top_left + value.top_left;
        this->top_right = this->top_right + value.top_right;
        this->bottom_left = this->bottom_left + value.bottom_left;
        this->bottom_right = this->bottom_right + value.bottom_right;
        return *this;
    }
    inline Matrix6& operator-=(const Matrix6 &value)
    {
        this->top_left = this->top_left - value.top_left;
        this->top_right = this->top_right - value.top_right;
        this->bottom_left = this->bottom_left - value.bottom_left;
        this->bottom_right = this->bottom_right - value.bottom_right;
        return *this;
    }
    inline Matrix6& operator*=(const double &value)
    {
        this->top_left = this->top_left * value;
        this->top_right = this->top_right * value;
        this->bottom_left = this->bottom_left * value;
        this->bottom_right = this->bottom_right * value;
        return *this;
    }
    inline Matrix6& operator*=(const Matrix6 &value)
    {
        this->top_left = this->top_left * value.top_left + this->top_right * value.bottom_left;
        this->top_right = this->top_left * value.top_right + this->top_right * value.bottom_right;
        this->bottom_left = this->bottom_left * value.top_left + this->bottom_right * value.bottom_left;
        this->bottom_right = this->bottom_left * value.top_right + this->bottom_right * value.bottom_right;
        return *this;
    }
    bool operator==(const Matrix6 &value)
    {
        return (value.top_left == this->top_left) &&
               (value.top_right == this->top_right) &&
               (value.bottom_left == this->bottom_left) &&
               (value.bottom_right == this->bottom_right);
    }
    bool operator!=(const Matrix6 &value)
    {
        return !(*this == value);
    }
   Matrix6 Inverse()
    {
        /** Inversion by blocks
         * [A, B;⁻¹ = [(A - BD⁻¹C)⁻¹, -A⁻¹B(D - CA⁻¹B)⁻¹;
         *  C, D]      -D⁻¹C(A - BD⁻¹C)⁻¹, (D - CA⁻¹B)⁻¹]
         */
         Matrix6 inv;
         gazebo::math::Matrix3 Sd = (this->top_left - this->top_right*this->bottom_right.Inverse()*this->bottom_left).Inverse();
         gazebo::math::Matrix3 Sa = (this->bottom_right - this->bottom_left*this->top_left.Inverse()*this->top_right).Inverse();
         inv.top_left = Sd;
         inv.top_right = this->top_left.Inverse()*this->top_right*Sa*(-1);
         inv.bottom_left = this->bottom_right.Inverse()*this->bottom_left*Sd*(-1);
         inv.bottom_right = Sa;
         return inv;
    }

    static inline Matrix6 Identity()
    {
       return  Matrix6(gazebo::math::Matrix3::IDENTITY, gazebo::math::Matrix3::ZERO,
               gazebo::math::Matrix3::ZERO, gazebo::math::Matrix3::IDENTITY);
    }

    inline gazebo_underwater::msgs::Matrix3 ConvertToMsg(const gazebo::math::Matrix3 &matrix) const
    {
        gazebo::math::Vector3 vec;
        gazebo_underwater::msgs::Matrix3 msg;
        vec.Set(matrix[0][0], matrix[1][0], matrix[2][0]);
        gazebo::msgs::Set(msg.mutable_x(), vec.Ign());
        vec.Set(matrix[0][1], matrix[1][1], matrix[2][1]);
        gazebo::msgs::Set(msg.mutable_y(), vec.Ign());
        vec.Set(matrix[0][2], matrix[1][2], matrix[2][2]);
        gazebo::msgs::Set(msg.mutable_z(), vec.Ign());
        return msg;
    }

    inline gazebo_underwater::msgs::Matrix6 ConvertToMsg() const
    {
        gazebo_underwater::msgs::Matrix6 msg;
        msg.mutable_tl()->CopyFrom(ConvertToMsg(this->top_left));
        msg.mutable_tr()->CopyFrom(ConvertToMsg(this->top_right));
        msg.mutable_bl()->CopyFrom(ConvertToMsg(this->bottom_left));
        msg.mutable_br()->CopyFrom(ConvertToMsg(this->bottom_right));
        return msg;
    }

    inline gazebo::math::Matrix3 Matrix3(const gazebo_underwater::msgs::Matrix3 &msg) const
    {
        gazebo::math::Matrix3 matrix;
        matrix.SetFromAxes(gazebo::math::Vector3(gazebo::msgs::ConvertIgn(msg.x())),
                gazebo::math::Vector3(gazebo::msgs::ConvertIgn(msg.y())),
                gazebo::math::Vector3(gazebo::msgs::ConvertIgn(msg.z())));
        return matrix;
    }

    Matrix6(const gazebo_underwater::msgs::Matrix6 &msg):
        Matrix6(Matrix3(msg.tl()), Matrix3(msg.tr()),
                Matrix3(msg.bl()), Matrix3(msg.br()))
    {}
};

struct Vector6
{
    gazebo::math::Vector3 top;
    gazebo::math::Vector3 bottom;

    Vector6()
    {
        top = gazebo::math::Vector3::Zero;
        bottom = gazebo::math::Vector3::Zero;
    }
    Vector6(const gazebo::math::Vector3 &t, const gazebo::math::Vector3 &b)
    {
        top = t;
        bottom = b;
    }
    Vector6(const Vector6 &v)
    {
        top = v.top;
        bottom = v.bottom;
    }
    inline Vector6& operator*=(double value)
    {
        this->top *= value;
        this->bottom *= value;
        return *this;
    }
    inline Vector6& operator+=(const Vector6 &value)
    {
        this->top += value.top;
        this->bottom += value.bottom;
        return *this;
    }
    inline Vector6& operator-=(const Vector6 &value)
    {
        this->top -= value.top;
        this->bottom -= value.bottom;
        return *this;
    }
    bool operator==(const Vector6 &value)
    {
        return (value.top == this->top) &&
               (value.bottom == this->bottom);
    }
    bool operator!=(const Vector6 &value)
    {
        return !(*this == value);
    }
};

Vector6 operator*(const Matrix6 &matrix, const Vector6 &vector)
{
    Vector6 result;
    result.top = matrix.top_left*vector.top + matrix.top_right*vector.bottom;
    result.bottom = matrix.bottom_left*vector.top + matrix.bottom_right*vector.bottom;
    return result;
};
inline Matrix6 operator*(Matrix6 value, const Matrix6 &matrix)
{
    return value *= matrix;
}
inline Matrix6 operator*(double scalar, Matrix6 value)
{
    return value *= scalar;
}
inline Matrix6 operator*(Matrix6 value, double scalar)
{
    return value *= scalar;
}
inline Matrix6 operator+(Matrix6 value, const Matrix6 &matrix)
{
    return value += matrix;
}
inline Matrix6 operator-(Matrix6 value, const Matrix6 &matrix)
{
    return value -= matrix;
}
inline Vector6 operator+(Vector6 value, const Vector6 &vector)
{
    return value += vector;
}
inline Vector6 operator-(Vector6 value, const Vector6 &vector)
{
    return value -= vector;
}
inline Vector6 operator*(double scalar, Vector6 vector)
{
    return vector *= scalar;
}
inline Vector6 operator*(Vector6 vector, double scalar)
{
    return vector *= scalar;
}

}
#endif
