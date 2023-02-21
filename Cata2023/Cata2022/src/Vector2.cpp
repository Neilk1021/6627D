#include <cmath>
#include <iostream>
#include <iterator>
#include <ostream>
#include "Vector2.h"

bool Vector2::operator<(Vector2 const &obj){
    return ( ( this->X < obj.X ) && (this->Y < obj.Y));
}

bool Vector2::operator>(Vector2 const &obj){
    return ( ( this->X > obj.X ) || (this->Y > obj.Y));
}

Vector2 Vector2::operator-(Vector2 const &obj){
    return Vector2((this->X - obj.X), (this->Y - obj.Y));
}

Vector2 Vector2::operator+(Vector2 const &obj){
    return Vector2((this->X + obj.X), (this->Y + obj.Y));
}

double Vector2::DOT(Vector2 a, Vector2 b){
    return a.X * b.X + a.Y * b.Y;
}

int Vector2::GetDirec(Vector2 a, Vector2 b){
    if(a>b) return 1;
    return -1;
}

double Vector2::MAG(Vector2 a){
    return std::sqrt(a.X * a.X + a.Y * a.Y);
}

double Vector2::DOTDEG(Vector2 a , Vector2 b){
    int direc;
    if(a>b) direc = 1;
    else direc = -1;
    return std::acos(Vector2::DOT(a,b)/(Vector2::MAG(a)*Vector2::MAG(b))) * 180/3.1415926;
}

double Vector2::DOTRAD(Vector2 a, Vector2 b){
    return std::acos(Vector2::DOT(a,b)/(Vector2::MAG(a)*Vector2::MAG(b)));
}

void Vector2::Update(double X, double Y){
    this->X = X;
    this->Y = Y;
}

Vector2::Vector2(double X, double Y){
    this->X = X;
    this->Y = Y;
}
