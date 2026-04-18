#ifndef CAMERAHEADER
#define CAMERAHEADER
#include <iostream>
#include <fstream>
#include "../utils/Scene/sceneSchema.hpp"

class Camera{
public:
    Camera(CameraData data): data(data) {
        // Initialize the pixel grid based on the image dimensions
        pixels = vector<vector<Pixel>>(data.image_height, vector<Pixel>(data.image_width, Pixel(Ponto(0, 0, 0))));
        
        Vetor forward = (data.lookat - data.lookfrom).normalized();
        Vetor right = forward.cross(data.upVector).normalized();
        Vetor up = right.cross(forward).normalized();

        // Set the position of each pixel in the grid based on the camera's lookfrom and lookat points
        Ponto centroid = data.lookfrom + forward * data.screen_distance;
        
        Ponto topLeftPoint = centroid + up * ((data.image_height-1) / 2) - right * ((data.image_width-1) / 2);

        for(int i = 0; i < data.image_height; i++){
            for(int j = 0; j < data.image_width; j++){

                pixels[i][j] = Pixel(topLeftPoint - up * i  + right * j); // Assuming the pixel positions are in the XY plane at Z=0
            }
        }
    }
    
    // cout << Vetor
    friend std::ostream& operator<<(std::ostream& os, const Camera &c){ 
        return os << "LookFrom: " << c.data.lookfrom << ", LookAt: " << c.data.lookat << ", UpVector: " << c.data.upVector 
                  << ", ImageWidth: " << c.data.image_width << ", ImageHeight: " << c.data.image_height 
                  << ", ScreenDistance: " << c.data.screen_distance;
    }

    // Getters
    Ponto getLookFrom() const { return data.lookfrom; }
    Ponto getLookAt() const { return data.lookat; }
    Vetor getUpVector() const { return data.upVector; }
    int getImageWidth() const { return data.image_width; }
    int getImageHeight() const { return data.image_height; }
    double getScreenDistance() const { return data.screen_distance; }

    void plotPixels() const {

        // Create a PPM file with C++ io file creation and write the pixel data in the PPM format

        ofstream outputFile("imagem.ppm", ios::trunc); 
            
        outputFile << "P3" << endl;
        outputFile << data.image_width << " " << data.image_height << endl;
        outputFile << "255" << endl;

        for(const auto& row : pixels){
            for(const auto& pixel : row){
                outputFile << pixel.getR() << " " << pixel.getG() << " " << pixel.getB() << std::endl;
            }
        }

        outputFile.close();
    }

    //Raycasting

    void rayCast(vector<ObjectData> objects) {
        // For each pixel, calculate the ray direction and check for intersections with objects in the scene

        for(int i = 0; i < data.image_height; i++){
            for(int j = 0; j < data.image_width; j++){
                Pixel& pixel = pixels[i][j];
                Vetor rayDirection = (pixel.getPos() - data.lookfrom).normalized();

                // Check for intersections with objects in the scene
                // If an intersection is found, calculate the color based on the material properties and lighting
                // Set the pixel color accordingly

                double closestT = std::numeric_limits<double>::max(); // Placeholder for the closest intersection

                for (auto& obj : objects) {
                    // Implement intersection logic based on the object type (e.g., sphere, plane, mesh)
                    // If an intersection is detected, calculate the color contribution from the object's material and lighting
                    // Update the pixel color using pixel.setColor(r, g, b);
                    if (obj.objType == "sphere"){

                        double radius = obj.getNum("radius");
                        Ponto center = obj.getPonto("center");

                        //  (data.lookfrom + rayDirection * t - center).dot(data.lookfrom + rayDirection * t - center) = radius^2
                        // Implement the quadratic formula to find the intersection points (t values) and determine if the ray intersects the sphere through delta
                        // (fromX-centerX + rayDirection.x * t)^2 + (fromY-centerY + rayDirection.y * t)^2 + (fromZ-centerZ + rayDirection.z * t)^2 = radius^2
                        // rayDirection.dot(rayDirection) * t^2 + 2 * ((data.lookfrom - center).dot(rayDirection)) * t + (data.lookfrom - center).dot(data.lookfrom - center) - radius^2 = 0
                        
                        Vetor oc = pixel.getPos() - center;
                        double A = rayDirection.dot(rayDirection);
                        double B = 2.0 * oc.dot(rayDirection);
                        double C = oc.dot(oc) - radius * radius;
                        double delta = B * B - 4.0 * A * C;
                        
                        // If delta < 0, no intersection; if delta = 0, one intersection
                        if (delta < 0) continue; // No intersection
                        else {
                            double t, t1, t2;
                            t1 = (-B - sqrt(delta)) / (2.0 * A);
                            t2 = (-B + sqrt(delta)) / (2.0 * A);
                            
                            if (t1 < 0) {
                                t = t2;
                            } else if (t2 < 0) {
                                continue; // Both intersections are behind the camera
                            } else {
                                t = min(t1, t2); // Choose the closest intersection in front of the camera
                            }
                            
                            if (t < closestT && t > 0) { // Check if it's the closest intersection and in front of the camera
                                closestT = t;
                                pixel.setColor((int)(obj.material.color.r*255), (int)(obj.material.color.g*255), (int)(obj.material.color.b*255));

                                // if (i == 300 && j == 300){
                                //     cout << pixel.getPos() << "(" << i << ", " << j << "): " <<
                                //     "rayDirection: " << rayDirection << ", center: " << center << ", radius: " << radius <<
                                //     ", oc: " << oc << ", A: " << A << ", B: " << B << ", C: " << C << ", delta: " << delta << 
                                //     ", t: " << t << ", intercept point: " << pixel.getPos() + rayDirection * t << endl;
                                // }
                            }
                        }
                    } else if (obj.objType == "plane") {
                        
                        Ponto point_on_plane = obj.getPonto("point_on_plane");
                        Vetor normal = obj.getVetor("normal").normalized();

                        if (abs(rayDirection.dot(normal)) < 0.1) continue; // Ray is parallel to the plane, no intersection

                        double t = (point_on_plane - pixel.getPos()).dot(normal) / rayDirection.dot(normal);

                        if (t < closestT && t > 0) { // Check if it's the closest intersection and in front of the camera
                            closestT = t;
                            pixel.setColor((int)(obj.material.color.r*255), (int)(obj.material.color.g*255), (int)(obj.material.color.b*255));
                        }

                        // if (i == 0 && j == 0){
                        //     cout << pixel.getPos() << "(" << i << ", " << j << "): " <<
                        //     "rayDirection: " << rayDirection << ", point_on_plane: " << point_on_plane << ", normal: " << normal <<
                        //     ", t: " << t << ", intercept point: " << pixel.getPos() + rayDirection * t << ", dot product: " << rayDirection.dot(normal) <<
                        //     ", plane_name: " << obj.getProperty("name") << endl;
                        // }

                    } else if (obj.objType == "mesh") {
                        continue;
                        // Implement mesh intersection logic
                    }
                }
            }
        }
    }

private:
    class Pixel {
    public:
        Pixel(Ponto position, int r=0, int g=0, int b=0) : r(r), g(g), b(b), position(position) {}

        void setColor(int r, int g, int b) {
            this->r = r;
            this->g = g;
            this->b = b;
        }

        int getR() const { return r; }
        int getG() const { return g; }
        int getB() const { return b; }
        Ponto getPos() const { return position; }

    private:        
        int r, g, b;
        Ponto position; // Position of the pixel in the image
    };

    CameraData data;
    vector<vector<Pixel>> pixels; // 3D vector to store RGB values and position of each pixel

    
};

#endif