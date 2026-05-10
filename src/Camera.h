#ifndef CAMERAHEADER
#define CAMERAHEADER
#include <fstream>
#include <future>
#include <iostream>

#include "../utils/MeshReader/ObjReader.cpp"
#include "../utils/Scene/sceneSchema.hpp"

class Camera {
   public:
	Camera(CameraData data) : data(data) {
		// Initialize the pixel grid based on the image dimensions
		pixels = vector<vector<Pixel>>(data.image_height, vector<Pixel>(data.image_width, Pixel(Ponto(0, 0, 0))));

		Vetor forward = (data.lookat - data.lookfrom).normalized();
		Vetor right = forward.cross(data.upVector).normalized() / data.image_width;	 // Scale right vector by the image width to get the correct pixel spacing
		Vetor up = right.cross(forward).normalized() / data.image_height;			 // Scale up vector by the image height to get the correct pixel spacing

		// Set the position of each pixel in the grid based on the camera's lookfrom and lookat points
		Ponto centroid = data.lookfrom + forward * data.screen_distance;

		Ponto topLeftPoint = centroid + up * ((data.image_height - 1) / 2) - right * ((data.image_width - 1) / 2);

		for (int i = 0; i < data.image_height; i++) {
			for (int j = 0; j < data.image_width; j++) {
				pixels[i][j] = Pixel(topLeftPoint - up * i + right * j);  // Assuming the pixel positions are in the XY plane at Z=0
			}
		}
	}

	// cout << Vetor
	friend std::ostream& operator<<(std::ostream& os, const Camera& c) {
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

	void plotPixels(string filename) const {
		// Create a PPM file with C++ io file creation and write the pixel data in the PPM format

		ofstream outputFile(filename + ".ppm", ios::trunc);

		outputFile << "P3" << endl;
		outputFile << data.image_width << " " << data.image_height << endl;
		outputFile << "255" << endl;

		for (const auto& row : pixels) {
			for (const auto& pixel : row) {
				outputFile << pixel.getR() << " " << pixel.getG() << " " << pixel.getB() << std::endl;
			}
		}

		outputFile.close();
	}

	// Raycasting

	vector<int> rayCast(Ponto rayOrigin, Vetor rayDirection, vector<ObjectData>& objects, int depth = 0) {
		if (depth > 5) {	   // Limit the recursion depth to prevent infinite loops in case of reflective/refractive materials
			return {0, 0, 0};  // Return black color for rays that exceed the recursion depth
		}

		// Check for intersections with objects in the scene
		// If an intersection is found, calculate the color based on the material properties and lighting
		// Set the pixel color accordingly

		double closestT = std::numeric_limits<double>::max();  // Placeholder for the closest intersection
		vector<int> pixelColor = {0, 0, 0};					   // Default to black

		for (auto& obj : objects) {
			// Implement intersection logic based on the object type (e.g., sphere, plane, mesh)
			// If an intersection is detected, calculate the color contribution from the object's material and lighting

			if (obj.objType == "sphere") {
				double radius = obj.numericData["radius"];
				Ponto center = obj.relativePos;

				//  (rayOrigin + rayDirection * t - center).dot(rayOrigin + rayDirection * t - center) = radius^2
				// Implement the quadratic formula to find the intersection points (t values) and determine if the ray intersects the sphere through delta
				// (fromX-centerX + rayDirection.x * t)^2 + (fromY-centerY + rayDirection.y * t)^2 + (fromZ-centerZ + rayDirection.z * t)^2 = radius^2
				// rayDirection.dot(rayDirection) * t^2 + 2 * ((rayOrigin - center).dot(rayDirection)) * t + (rayOrigin - center).dot(rayOrigin - center) - radius^2 = 0

				Vetor oc = rayOrigin - center;
				double A = rayDirection.dot(rayDirection);
				double B = 2.0 * oc.dot(rayDirection);
				double C = oc.dot(oc) - radius * radius;
				double delta = B * B - 4.0 * A * C;

				// If delta < 0, no intersection; if delta = 0, one intersection
				if (delta < 0)
					continue;  // No intersection
				else {
					double t, t1, t2;
					t1 = (-B - sqrt(delta)) / (2.0 * A);
					t2 = (-B + sqrt(delta)) / (2.0 * A);

					if (t1 < 0) {
						t = t2;
					} else if (t2 < 0) {
						continue;  // Both intersections are behind the camera
					} else {
						t = min(t1, t2);  // Choose the closest intersection in front of the camera
					}

					if (t < closestT && t > 0) {  // Check if it's the closest intersection and in front of the camera
						closestT = t;
						pixelColor = {(int)(obj.material.color.r * 255), (int)(obj.material.color.g * 255), (int)(obj.material.color.b * 255)};

						// if (i == 300 && j == 300){
						//     cout << pixel.getPos() << "(" << i << ", " << j << "): " <<
						//     "rayDirection: " << rayDirection << ", center: " << center << ", radius: " << radius <<
						//     ", oc: " << oc << ", A: " << A << ", B: " << B << ", C: " << C << ", delta: " << delta <<
						//     ", t: " << t << ", intercept point: " << pixel.getPos() + rayDirection * t << endl;
						// }
					}
				}
			} else if (obj.objType == "plane") {
				Ponto point_on_plane = obj.relativePos;
				Vetor normal = obj.vetorPointData["normal"].normalized();

				double dotNorm = rayDirection.dot(normal);

				if (fabs(dotNorm) == 0) continue;	 // Ray is parallel to the plane, no intersection

				double t = (point_on_plane - rayOrigin).dot(normal) / dotNorm;

				if (t < closestT && t > 0) {  // Check if it's the closest intersection and in front of the camera
					closestT = t;
					pixelColor = {(int)(obj.material.color.r * 255), (int)(obj.material.color.g * 255), (int)(obj.material.color.b * 255)};
				}

				// if (i == 0 && j == 0){
				//     cout << pixel.getPos() << "(" << i << ", " << j << "): " <<
				//     "rayDirection: " << rayDirection << ", point_on_plane: " << point_on_plane << ", normal: " << normal <<
				//     ", t: " << t << ", intercept point: " << pixel.getPos() + rayDirection * t << ", dot product: " << rayDirection.dot(normal) <<
				//     ", plane_name: " << obj.getProperty("name") << endl;
				// }

			} else if (obj.objType == "mesh") {
				// ObjReader mesh(obj.getProperty("path"));

				vector<vector<Ponto>> faces = obj.facePoints;
				vector<Vetor> normals = obj.faceNormals;

				// if obj.relativePos is projected in this pixel ray, print it as {0,0,255} and continue
				// if(fabs((obj.relativePos - rayOrigin).normalized().dot(rayDirection)) >= .9999) {
				// 	pixelColor = {0, 0, 255};
				// 	continue;
				// }


				// Iterate through each face and perform ray-triangle intersection
				for (size_t i = 0; i < faces.size(); ++i) {
					vector<Ponto>& face = faces[i];
					for (auto& point : face) {
						point = point + obj.relativePos;  // Apply the mesh's relative position to each vertex of the face
					}

					// Vetor normal = (face[1] - face[0]).normalized().cross((face[2] - face[0]).normalized()).normalized(); // Calculate the normal of the triangle face using the cross product of two edges
					Vetor normal = normals[i];

					// Check for parallelism
					double dotNorm = rayDirection.dot(normal);
					if (fabs(dotNorm) == 0) continue;	 // Ray is parallel to triangle plane

					// Compute intersection t with the triangle plane
					double t = (face[0] - rayOrigin).dot(normal) / dotNorm;
					// Must be in front of camera and closer than previous hit
					if (t <= 0 || t >= closestT) continue;

					// Intersection point
					Ponto P = rayOrigin + rayDirection * t;

					// Inside-triangle test using edge-cross tests
					Vetor C;
					C = (face[1] - face[0]).cross(P - face[0]);
					if (normal.dot(C) < 0) continue;
					C = (face[2] - face[1]).cross(P - face[1]);
					if (normal.dot(C) < 0) continue;
					C = (face[0] - face[2]).cross(P - face[2]);
					if (normal.dot(C) < 0) continue;
					// Passed all checks: it's inside the triangle
					closestT = t;
					pixelColor = {(int)(obj.material.color.r * 255), (int)(obj.material.color.g * 255), (int)(obj.material.color.b * 255)};
				}
			}
		}

		return pixelColor;	// Return the final pixel color
	}

	void render(vector<ObjectData>& objects) {
		const int tile_size = 32;
		vector<future<void>> futures;

		processObjects(objects);

		for (int i = 0; i < data.image_height; i += tile_size) {
			for (int j = 0; j < data.image_width; j += tile_size) {
				int end_x = std::min(i + tile_size, data.image_height);
				int end_y = std::min(j + tile_size, data.image_width);

				futures.push_back(async(launch::async, &Camera::rayTrace, this, std::ref(objects), i, j, end_x, end_y));
			}
		}

		for (auto& f : futures) {
			f.get();
		}
	}

	void rayTrace(vector<ObjectData>& objects, int start_x, int start_y, int end_x, int end_y) {
		// For each pixel, calculate the ray direction and check for intersections with objects in the scene
		for (int i = start_x; i < end_x; i++) {
			for (int j = start_y; j < end_y; j++) {
				Pixel& pixel = pixels[i][j];
				Vetor rayDirection = (pixel.getPos() - data.lookfrom).normalized();

				pixel.setColor(rayCast(data.lookfrom, rayDirection, objects));
			}
		}
	}

	void rayTracer(vector<ObjectData> objects) {
		processObjects(objects);

		// For each pixel, calculate the ray direction and check for intersections with objects in the scene
		for (int i = 0; i < data.image_height; i++) {
			for (int j = 0; j < data.image_width; j++) {
				Pixel& pixel = pixels[i][j];
				Vetor rayDirection = (pixel.getPos() - data.lookfrom).normalized();

				pixel.setColor(rayCast(data.lookfrom, rayDirection, objects));
			}
		}
	}

   private:
	class Pixel {
	   public:
		Pixel(Ponto position, int r = 0, int g = 0, int b = 0) : r(r), g(g), b(b), position(position) {}

		void setColor(vector<int> colorRGB) {
			if (colorRGB.size() != 3) throw std::runtime_error("Color RGB vector must have 3 components");

			this->r = colorRGB[0];
			this->g = colorRGB[1];
			this->b = colorRGB[2];
		}

		int getR() const { return r; }
		int getG() const { return g; }
		int getB() const { return b; }
		vector<int> getColor() const { return {r, g, b}; }
		Ponto getPos() const { return position; }

	   private:
		int r, g, b;
		Ponto position;	 // Position of the pixel in the image
	};

	void processObjects(vector<ObjectData>& objects) {
		for (auto& obj : objects) {
			if (obj.objType == "mesh") {
				ObjReader mesh(obj.getProperty("path"));
				obj.facePoints = mesh.getFacePoints();
			}

			if (obj.transforms.empty()) continue;  // No transformations to apply

			vector<pair<Ponto*, bool>> pointsToTransform;  // Pair of pointer to Ponto and a boolean indicating if it's a normal vector
			vector<Vetor*> vectorsToTransform;
			vector<double*> scalarsToTransform;

			if (obj.objType == "sphere") {
				pointsToTransform.push_back({&obj.relativePos, false});
				scalarsToTransform.push_back(&obj.numericData["radius"]);
			} else if (obj.objType == "plane") {
				pointsToTransform.push_back({&obj.relativePos, false});
				vectorsToTransform.push_back(&obj.vetorPointData["normal"]);
			} else if (obj.objType == "mesh") {
				for (auto& face : obj.facePoints) {
					for (auto& point : face) {
						pointsToTransform.push_back({&point, true});
					}
				}
			}

			for (const auto& t : obj.transforms) {
				cout << *pointsToTransform[0].first << "transformed with " << t.tType << " " << t.data << " -> ";
				for (auto& [point, isVertex] : pointsToTransform) {
					if (t.tType == "translation") {
						point->translate(t.data);
					} else if (t.tType == "scaling" && isVertex) {
						point->scale(t.data);
					} else if (t.tType == "rotation" && isVertex) {
						point->rotate(t.data);
					}
				}
				cout << *pointsToTransform[0].first << endl;

				for (auto& vector : vectorsToTransform) {
					if (t.tType == "rotation") {
						vector->rotate(t.data);
					}
				}

				for (auto& scalar : scalarsToTransform) {
					if (t.tType == "scaling") {
						*scalar *= t.data.getX();  // Scale the radius by the maximum scaling factor
					}
				}
			}

			if (obj.objType == "mesh") {
				for (const auto& face : obj.facePoints) {
					obj.faceNormals.push_back((face[1] - face[0]).normalized().cross((face[2] - face[0]).normalized()).normalized());
				}
			}

		}
	}

	CameraData data;
	vector<vector<Pixel>> pixels;  // 3D vector to store RGB values and position of each pixel
};

#endif