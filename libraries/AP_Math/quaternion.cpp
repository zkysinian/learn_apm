/*
 * quaternion.cpp
 * Copyright (C) Andrew Tridgell 2012
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma GCC optimize("O3")

#include "AP_Math.h"

// return the rotation matrix equivalent for this quaternion
//传递引用:引用能修改调用函数中的数据对象
//传递的值不作修改:对于结构和类，使用const引用或const指针；数据对象为数组，const指针
//修改传递的值:对于结构和类对象，使用引用或指针；数据对象为数组，指针
//引用变量主要作为函数形参，引用对象作为参数，函数将直接使用原始数据，而非其拷贝
//利用本身的四元素求姿态矩阵，并将姿态矩阵赋值给m输出，返回值类型为空

//通过引用，将四元数计算的姿态矩阵传递出来，如此返回类型可为void
void Quaternion::rotation_matrix(Matrix3f &m) const
{//get a rotation matrix according quaternion
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q4q4 = q4 * q4;

    m.a.x = 1.0f-2.0f*(q3q3 + q4q4);
    m.a.y = 2.0f*(q2q3 - q1q4);
    m.a.z = 2.0f*(q2q4 + q1q3);
    m.b.x = 2.0f*(q2q3 + q1q4);
    m.b.y = 1.0f-2.0f*(q2q2 + q4q4);
    m.b.z = 2.0f*(q3q4 - q1q2);
    m.c.x = 2.0f*(q2q4 - q1q3);
    m.c.y = 2.0f*(q3q4 + q1q2);
    m.c.z = 1.0f-2.0f*(q2q2 + q3q3);
}

// return the rotation matrix equivalent for this quaternion after normalization
//相比上个，这里用了归一化处理
void Quaternion::rotation_matrix_norm(Matrix3f &m) const
{
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;
    float invs = 1.0f / (q1q1 + q2q2 + q3q3 + q4q4);

    m.a.x = ( q2q2 - q3q3 - q4q4 + q1q1)*invs;
    m.a.y = 2.0f*(q2q3 - q1q4)*invs;
    m.a.z = 2.0f*(q2q4 + q1q3)*invs;
    m.b.x = 2.0f*(q2q3 + q1q4)*invs;
    m.b.y = (-q2q2 + q3q3 - q4q4 + q1q1)*invs;
    m.b.z = 2.0f*(q3q4 - q1q2)*invs;
    m.c.x = 2.0f*(q2q4 - q1q3)*invs;
    m.c.y = 2.0f*(q3q4 + q1q2)*invs;
    m.c.z = (-q2q2 - q3q3 + q4q4 + q1q1)*invs;
}

// return the rotation matrix equivalent for this quaternion
// Thanks to Martin John Baker
// http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/index.htm
//利用姿态矩阵求四元素，姿态矩阵声明为const类型
void Quaternion::from_rotation_matrix(const Matrix3f &m)
{//不明白为什么这么写 if语句中判断的依据是什么
    const float &m00 = m.a.x;
    const float &m11 = m.b.y;
    const float &m22 = m.c.z;
    const float &m10 = m.b.x;
    const float &m01 = m.a.y;
    const float &m20 = m.c.x;
    const float &m02 = m.a.z;
    const float &m21 = m.c.y;
    const float &m12 = m.b.z;
    float &qw = q1;
    float &qx = q2;
    float &qy = q3;
    float &qz = q4;

    float tr = m00 + m11 + m22;

    if (tr > 0) {
        float S = sqrtf(tr+1) * 2;
        qw = 0.25f * S;
        qx = (m21 - m12) / S;
        qy = (m02 - m20) / S;
        qz = (m10 - m01) / S;
    } else if ((m00 > m11) && (m00 > m22)) {
        float S = sqrtf(1.0f + m00 - m11 - m22) * 2.0f;
        qw = (m21 - m12) / S;
        qx = 0.25f * S;
        qy = (m01 + m10) / S;
        qz = (m02 + m20) / S;
    } else if (m11 > m22) {
        float S = sqrtf(1.0f + m11 - m00 - m22) * 2.0f;
        qw = (m02 - m20) / S;
        qx = (m01 + m10) / S;
        qy = 0.25f * S;
        qz = (m12 + m21) / S;
    } else {
        float S = sqrtf(1.0f + m22 - m00 - m11) * 2.0f;
        qw = (m10 - m01) / S;
        qx = (m02 + m20) / S;
        qy = (m12 + m21) / S;
        qz = 0.25f * S;
    }
}

// convert a vector from earth to body frame
void Quaternion::earth_to_body(Vector3f &v) const
{
    Matrix3f m;
    rotation_matrix(m);
    v = m * v;
}

// create a quaternion from Euler angles
//类的成员函数中可以调用类中private的数据部分
void Quaternion::from_euler(float roll, float pitch, float yaw)
{
    float cr2 = cosf(roll*0.5f);
    float cp2 = cosf(pitch*0.5f);
    float cy2 = cosf(yaw*0.5f);
    float sr2 = sinf(roll*0.5f);
    float sp2 = sinf(pitch*0.5f);
    float sy2 = sinf(yaw*0.5f);

    q1 = cr2*cp2*cy2 + sr2*sp2*sy2;
    q2 = sr2*cp2*cy2 - cr2*sp2*sy2;
    q3 = cr2*sp2*cy2 + sr2*cp2*sy2;
    q4 = cr2*cp2*sy2 - sr2*sp2*cy2;
}

// create a quaternion from Euler angles
void Quaternion::from_vector312(float roll ,float pitch, float yaw)
{
    Matrix3f m;
    m.from_euler312(roll, pitch, yaw);

    from_rotation_matrix(m);
}
//input:三轴角度增量 output:类成员内部private数据的四元素得到更新
void Quaternion::from_axis_angle(Vector3f v)
{
    float theta = v.length();//利用三轴角度增量求出轴角
    if (is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    v /= theta;
    from_axis_angle(v,theta);//利用轴角求出四元素
}
//what 轴角 同一个函数名称具有不同的形参列表
void Quaternion::from_axis_angle(const Vector3f &axis, float theta)
{//q1/q2/q3/q4是类Quaternion中元素，通过成员函数改变了类的公有数据成员
    // axis must be a unit vector as there is no check for length
    if (is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    float st2 = sinf(theta/2.0f);

    q1 = cosf(theta/2.0f);
    q2 = axis.x * st2;
    q3 = axis.y * st2;
    q4 = axis.z * st2;
}
//input:三轴角增量 利用角增量求出轴角，再利用轴角求出四元素增量，与之前的四元素相乘，得到新的四元素
//利用三个姿态角修正当前的四元素,输出为修正后的四元素
void Quaternion::rotate(const Vector3f &v)
{
    Quaternion r;
    r.from_axis_angle(v);//r中得到的由角度增量求取的四元素增量 r中的公有数据成员中有四元数q1/q2/q3/q4
    (*this) *= r;//将之前的四元素与四元素角度增量相乘，得到当前时刻的四元素 操作符重载，实现两个类中四元数相乘
}
//输入:角增量
//输出:轴角(一个角度，一个向量)
//处理流程:见导航笔记 四元素
void Quaternion::to_axis_angle(Vector3f &v)
{
    float l = sqrtf(sq(q2)+sq(q3)+sq(q4));//L的小写l
    v = Vector3f(q2,q3,q4);
    if (!is_zero(l)) {
        v /= l;
        v *= wrap_PI(2.0f * atan2f(l,q1));//将角度限制在-pi到pi之间
    }
}

void Quaternion::from_axis_angle_fast(Vector3f v)
{
    float theta = v.length();
    if (is_zero(theta)) {
        q1 = 1.0f;
        q2=q3=q4=0.0f;
        return;
    }
    v /= theta;
    from_axis_angle_fast(v,theta);
}

void Quaternion::from_axis_angle_fast(const Vector3f &axis, float theta)
{
    float t2 = theta/2.0f;
    float sqt2 = sq(t2);
    float st2 = t2-sqt2*t2/6.0f;

    q1 = 1.0f-(sqt2/2.0f)+sq(sqt2)/24.0f;
    q2 = axis.x * st2;
    q3 = axis.y * st2;
    q4 = axis.z * st2;
}

void Quaternion::rotate_fast(const Vector3f &v)//四元数乘法
{
    float theta = v.length();
    if (is_zero(theta)) {
        return;
    }
    float t2 = theta/2.0f;
    float sqt2 = sq(t2);
    float st2 = t2-sqt2*t2/6.0f;
    st2 /= theta;

    //"rotation quaternion"
    float w2 = 1.0f-(sqt2/2.0f)+sq(sqt2)/24.0f;
    float x2 = v.x * st2;
    float y2 = v.y * st2;
    float z2 = v.z * st2;

    //copy our quaternion
    float w1 = q1;
    float x1 = q2;
    float y1 = q3;
    float z1 = q4;

    //do the multiply into our quaternion
    q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;
}

// get euler roll angle
float Quaternion::get_euler_roll() const
{
    return (atan2f(2.0f*(q1*q2 + q3*q4), 1.0f - 2.0f*(q2*q2 + q3*q3)));
}

// get euler pitch angle
float Quaternion::get_euler_pitch() const
{
    return safe_asin(2.0f*(q1*q3 - q4*q2));
}

// get euler yaw angle
float Quaternion::get_euler_yaw() const
{
    return atan2f(2.0f*(q1*q4 + q2*q3), 1.0f - 2.0f*(q3*q3 + q4*q4));
}

// create eulers from a quaternion
void Quaternion::to_euler(float &roll, float &pitch, float &yaw) const
{
    roll = get_euler_roll();
    pitch = get_euler_pitch();
    yaw = get_euler_yaw();
}

// create eulers from a quaternion
Vector3f Quaternion::to_vector312(void) const
{
    Matrix3f m;
    rotation_matrix(m);
    return m.to_euler312();
}

float Quaternion::length(void) const
{
    return sqrtf(sq(q1) + sq(q2) + sq(q3) + sq(q4));
}
//四元数求逆，返回是一个Quaternion类
//四元素的逆:沿相反的方向旋转相同的角度
//对归一化的四元素，四元素的逆就是四元素的共轭
Quaternion Quaternion::inverse(void) const
{
    return Quaternion(q1, -q2, -q3, -q4);
}
//四元数归一化
void Quaternion::normalize(void)
{
    float quatMag = length();//sqrtf(sq(q1)+sq(q2)+sq(q3)+sq(q4))
    if (!is_zero(quatMag)) {// sq powf(x,2)
        float quatMagInv = 1.0f/quatMag;
        q1 *= quatMagInv;
        q2 *= quatMagInv;
        q3 *= quatMagInv;
        q4 *= quatMagInv;
    }
}
//将两个四元数相乘的结果赋值给另一个变量，并返回 Q3 = Q1 * Q2
//每个四元素代表1次选择，相乘即两次旋转的合成，即先旋转Q1,在旋转Q2
Quaternion Quaternion::operator*(const Quaternion &v) const
{
    Quaternion ret;
    const float &w1 = q1;
    const float &x1 = q2;
    const float &y1 = q3;
    const float &z1 = q4;

    float w2 = v.q1;
    float x2 = v.q2;
    float y2 = v.q3;
    float z2 = v.q4;

    ret.q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    ret.q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    ret.q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    ret.q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    return ret;
}
//四元数相乘的物理意义
//将两个四元数的乘积结果赋给给调用该函数的对象，因为返回的是*this  Q1 = Q1 * Q2
Quaternion &Quaternion::operator*=(const Quaternion &v)
{
    float w1 = q1;
    float x1 = q2;
    float y1 = q3;
    float z1 = q4;

    float w2 = v.q1;
    float x2 = v.q2;
    float y2 = v.q3;
    float z2 = v.q4;

    q1 = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q2 = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q3 = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q4 = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    return *this;
}
//四元素相除的物理意义 相等于乘以另一个四元素的逆，
//物理意义:在当前四元素的基础上再反转回去
Quaternion Quaternion::operator/(const Quaternion &v) const
{
    Quaternion ret;
    const float &quat0 = q1;
    const float &quat1 = q2;
    const float &quat2 = q3;
    const float &quat3 = q4;

    float rquat0 = v.q1;
    float rquat1 = v.q2;
    float rquat2 = v.q3;
    float rquat3 = v.q4;

    ret.q1 = (rquat0*quat0 + rquat1*quat1 + rquat2*quat2 + rquat3*quat3);
    ret.q2 = (rquat0*quat1 - rquat1*quat0 - rquat2*quat3 + rquat3*quat2);
    ret.q3 = (rquat0*quat2 + rquat1*quat3 - rquat2*quat0 - rquat3*quat1);
    ret.q4 = (rquat0*quat3 - rquat1*quat2 + rquat2*quat1 - rquat3*quat0);
    return ret;
}
