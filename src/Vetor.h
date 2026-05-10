#ifndef VETORHEADER
#define VETORHEADER
#include <iostream>
#include <cmath>
#include <vector>

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

    Vetor operator/ (double scalar) const {
        if (scalar == 0) throw std::runtime_error("Division by zero");
        return Vetor(x / scalar, y / scalar, z / scalar);
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
    Vetor normalize() {
        double n = norm();
        if (n == 0) return *this; // Avoid division by zero
        x /= n;
        y /= n;
        z /= n;
        return *this;
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

    // Transformations
    Vetor applied(const std::vector<std::vector<double>>& matrix) const {
        double newX = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3];
        double newY = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3];
        double newZ = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3];

        return Vetor(newX, newY, newZ);
    }
    Vetor apply(const std::vector<std::vector<double>>& matrix) {
        Vetor result = applied(matrix);
        x = result.x;
        y = result.y;
        z = result.z;
    
        return *this;
    }

    Vetor scaled(Vetor scale) const {
        return applied({  
            {scale.getX(),            0,            0, 0}, 
            {           0, scale.getY(),            0, 0}, 
            {           0,            0, scale.getZ(), 0}, 
            {           0,            0,            0, 1}
        });
    }
    Vetor rotated(Vetor rotation) const {
        // To Radians
        rotation = rotation * M_PI / 180.0;

        return applied({  
            {1,                     0,                      0, 0}, 
            {0,  cos(rotation.getX()),  -sin(rotation.getX()), 0}, 
            {0,  sin(rotation.getX()),   cos(rotation.getX()), 0}, 
            {0,                     0,                      0, 1}
        }).apply({  
            { cos(rotation.getY()), 0, sin(rotation.getY()), 0}, 
            {                    0, 1,                    0, 0}, 
            {-sin(rotation.getY()), 0, cos(rotation.getY()), 0}, 
            {                    0, 0,                    0, 1}
        }).apply({  
            {cos(rotation.getZ()),  -sin(rotation.getZ()), 0, 0}, 
            {sin(rotation.getZ()),   cos(rotation.getZ()), 0, 0}, 
            {                   0,                      0, 1, 0}, 
            {                   0,                      0, 0, 1}
        });
    }
    Vetor translated(Vetor translation) const {
        return applied({  
            {1, 0, 0, translation.getX()}, 
            {0, 1, 0, translation.getY()}, 
            {0, 0, 1, translation.getZ()}, 
            {0, 0, 0, 1}
        });
    }

    Vetor scale(Vetor scale) {
        Vetor result = scaled(scale);
        x = result.x;
        y = result.y;
        z = result.z;
    
        return *this;
    }
    Vetor rotate(Vetor rotation) {
        Vetor result = rotated(rotation);
        x = result.x;
        y = result.y;
        z = result.z;
    
        return *this;
    }
    Vetor translate(Vetor translation) {
        Vetor result = translated(translation);
        x = result.x;
        y = result.y;
        z = result.z;
    
        return *this;
    }
    
    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

private:
    double x, y, z;
};

#endif