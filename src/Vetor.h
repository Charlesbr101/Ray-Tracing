#ifndef VETORHEADER
#define VETORHEADER
#include <iostream>
#include <cmath>

class Vetor{
public:
    Vetor(double x=0, double y=0, double z=0): x(x), y(y), z(z) {}

    //Vetor + Vetor
    Vetor  operator+ (const Vetor&v) const{ 
        return Vetor(x+v.x, y+v.y, z+v.z); 
    }

    Vetor  operator- (const Vetor&v) const{ 
        return Vetor(x-v.x, y-v.y, z-v.z); 
    }

    Vetor operator* (double scalar) const {
        return Vetor(x * scalar, y * scalar, z * scalar);
    }

    //Norm of the vector
    double norm() const {
        return sqrt(x*x + y*y + z*z);
    }
    Vetor normalized() const {
        double n = norm();
        if (n == 0) return Vetor(0, 0, 0); // Avoid division by zero
        return Vetor(x/n, y/n, z/n);
    }

    Vetor cross(const Vetor& v) const {
        return Vetor(
            y * v.z - z * v.y,
            z * v.x - x * v.z,
            x * v.y - y * v.x
        );
    }

    double dot(const Vetor& v) const {
        return x * v.x + y * v.y + z * v.z;
    }

    // cout << Vetor
    friend std::ostream& operator<<(std::ostream& os, const Vetor &v){ 
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")T"; 
    }
    
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

private:
    double x, y, z;
};

#endif