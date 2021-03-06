//
// Created by jonathan on 3/6/21.
//

#include "Quat.h"

class Quat{
    float x,y,z,w;

public:
    Quat(float x,float y,float z,float w){
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }
    Quat(){
    }

    Quat diff(const Quat &a, const Quat &b)
    {
        Quat inv = a;
        inv.inverse();
        return inv * b;
    }


    void conjugate()
    {
        Quat q;
        q.x = -this->x;
        q.y = -this->y;
        q.z = -this->z;
        q.w = this->w;

        (*this) = q;
    }


    float dot(const Quat &q1, const Quat &q2)
    {
        return q1.x*q2.x + q1.y*q2.y + q1.z*q2.z + q1.w*q2.w;
    }


    const Quat operator* ( const Quat &q) const
    {
        Quat qu;
        qu.x = this->w*q.x + this->x*q.w + this->y*q.z - this->z*q.y;
        qu.y = this->w*q.y + this->y*q.w + this->z*q.x - this->x*q.z;
        qu.z = this->w*q.z + this->z*q.w + this->x*q.y - this->y*q.x;
        qu.w = this->w*q.w - this->x*q.x - this->y*q.y - this->z*q.z;
        return qu;
    }


    const Quat operator/ (float s) const
    {
        Quat q = (*this);
        return Quat(q.x / s, q.y / s, q.z / s, q.w / s);
    }
    void inverse()
    {
        Quat q = (*this);
        q.conjugate();
        (*this) = q / dot((*this), (*this));
    }

};