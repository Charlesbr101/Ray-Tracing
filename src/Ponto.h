#ifndef PONTOHEADER
#define PONTOHEADER
#include <iostream>
#include "Vetor.h"

class Ponto{
public:
    Ponto(double x=0, double y=0, double z=0) : x(x), y(y), z(z) {}

    // Ponto + vetor → Ponto
    Ponto  operator+ (const Vetor&v) const{ 
        return Ponto(x+v.getX(), y+v.getY(), z+v.getZ()); 
    }
    // Ponto + Ponto (Vetor a partir da origem) → Ponto
    Ponto  operator+ (const Ponto&a) const{ 
        return Ponto(x+a.getX(), y+a.getY(), z+a.getZ()); 
    }
    // Ponto - vetor → Ponto
    Ponto  operator- (const Vetor&v) const{ 
        return Ponto(x-v.getX(), y-v.getY(), z-v.getZ()); 
    }
    // Ponto - Ponto → Vetor
    Vetor  operator- (const Ponto&a) const { 
        return Vetor(x-a.x, y-a.y, z-a.z); 
    }
    // cout << Ponto
    friend std::ostream& operator<<(std::ostream& os, const Ponto& p){ 
        return os << "(" << p.x << ", " << p.y << ", " << p.z << ")"; 
    }

    // Transformations
    Ponto applied(const std::vector<std::vector<double>>& matrix) const {
        double newX = matrix[0][0] * x + matrix[0][1] * y + matrix[0][2] * z + matrix[0][3];
        double newY = matrix[1][0] * x + matrix[1][1] * y + matrix[1][2] * z + matrix[1][3];
        double newZ = matrix[2][0] * x + matrix[2][1] * y + matrix[2][2] * z + matrix[2][3];

        return Ponto(newX, newY, newZ);
    }
    Ponto apply(const std::vector<std::vector<double>>& matrix) {
        Ponto result = applied(matrix);
        x = result.x;
        y = result.y;
        z = result.z;
    
        return *this;
    }

    Ponto scaled(Vetor scale) const {
        return applied({  
            {scale.getX(),            0,            0, 0}, 
            {           0, scale.getY(),            0, 0}, 
            {           0,            0, scale.getZ(), 0}, 
            {           0,            0,            0, 1}
        });
    }
    Ponto rotated(Vetor rotation) const {
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
    Ponto translated(Vetor translation) const {
        return applied({  
            {1, 0, 0, translation.getX()}, 
            {0, 1, 0, translation.getY()}, 
            {0, 0, 1, translation.getZ()}, 
            {0, 0, 0, 1}
        });
    }

    Ponto scale(Vetor scale) {
        Ponto result = scaled(scale);
        x = result.x;
        y = result.y;
        z = result.z;
    
        return *this;
    }
    Ponto rotate(Vetor rotation) {
        Ponto result = rotated(rotation);
        x = result.x;
        y = result.y;
        z = result.z;
    
        return *this;
    }
    Ponto translate(Vetor translation) {
        Ponto result = translated(translation);
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