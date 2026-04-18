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

    double getX() const { return x; }
    double getY() const { return y; }
    double getZ() const { return z; }

private:
    double x, y, z;
};

#endif