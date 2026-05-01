#ifndef OBJREADERHEADER
#define OBJREADERHEADER

/*
Classe leitora de arquivos .obj. Onde o arquivo contém os vários pontos, normais e faces do objeto. No projeto 
trabalhamos com faces triangulares, ou seja, uma face consiste em 3 pontos. 

No arquivo .obj, temos:
    - v = pontos
    - vn = normais
    - vt = texturas
    - f = faces

Nessa classe podem ser obtidas as seguintes informações (por meio dos Getters):
    - Pontos
    - Normais
    - Lista de faces com seus respectivos pontos
    - Informações de cor, brilho, opacidade, etc.

Obs: -  Para fins de abstração, as normais de cada ponto são ignoradas e assumimos apenas uma normal para cada face. 
     -  As texturas também são ignoradas.

Caso sintam necessidade, podem editar a classe para obter mais informações.
*/

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sstream>
#include "../../src/Vetor.h"
#include "../../src/Ponto.h"
#include "Colormap.cpp"

struct FaceData {
    int verticeIndice[3];
    int normalIndice[3];
    MaterialProperties material;
};


class ObjReader {

private:
    std::ifstream file;                         // Arquivo .obj
    std::vector<Ponto> vertices;                // Lista de pontos
    std::vector<Vetor> normals;                 // Lista de normais
    std::vector<FaceData> faces;                    // Lista de indices de faces
    std::vector<std::vector<Ponto>> facePoints; // Lista de pontos das faces
    MaterialProperties curMaterial;             // Material atual
    colormap cmap;                              // Objeto de leitura de arquivos .mtl
    string Filename;

public:
    ObjReader(std::string filename) {
        Filename = filename;
        // Abre o arquivo
        file.open(filename);
        if (!file.is_open()) {
            std::cerr << "Erro ao abrir o arquivo: " << filename << std::endl;
            return;
        }

        // Leitura do arquivo
        std::string line, mtlfile, colorname, filename_mtl;
        while (std::getline(file, line)) {
            std::istringstream iss(line);
            std::string prefix;
            iss >> prefix;

            if (prefix == "mtllib") {
                iss >> filename_mtl;
                std::string filename_mtl_path = filename.replace(filename.length() - 3, 3, "mtl");
                cmap = colormap(filename_mtl_path);
            } else if (prefix == "usemtl") {
                iss >> colorname;
                curMaterial = cmap.getMaterialProperties(colorname);
            } else if (prefix == "v") {
                double x, y, z;
                iss >> x >> y >> z;
                vertices.emplace_back(x, y, z);
            } else if (prefix == "vn") {
                double x, y, z;
                iss >> x >> y >> z;
                normals.emplace_back(x, y, z);
            } else if (prefix == "f") {
                // Suporta formatos v/vt/vn e v//vn
                FaceData face;
                face.material = curMaterial;
                for (int i = 0; i < 3; ++i) {
                    std::string token;
                    iss >> token;
                    size_t first  = token.find('/');
                    size_t second = token.find('/', first + 1);
                    face.verticeIndice[i] = std::stoi(token.substr(0, first)) - 1;
                    face.normalIndice[i]  = std::stoi(token.substr(second + 1)) - 1;
                }
                faces.push_back(face);
            }
        }
        for (const auto& face : faces) {
            std::vector<Ponto> points = {
                vertices[face.verticeIndice[0]],
                vertices[face.verticeIndice[1]],
                vertices[face.verticeIndice[2]]
            };
            facePoints.push_back(points);
        }

        file.close();
    }

    // Getters

    // Método para retornar as coordenadas dos pontos das faces
    std::vector<std::vector<Ponto>> getFacePoints() {
        return facePoints;
    }

    /*
    Retorna uma lista com um struct faces do objeto. Cada face contém:
        - Índices dos pontos
        - Índices das normais
        - Cores (ka, kd, ks, ke)
        - Brilho (ns)
        - Índice de refração (ni)
        - Opacidade (d)
    */
    // Método para retornar a cor do material (Coeficiente de difusão)
    Vetor getKd() {
        return curMaterial.kd;
    }

    // Método para retornar a cor do ambiente
    Vetor getKa() {
        return curMaterial.ka;
    }

    // Método para retornar o coeficiente especular (Refletência)
    Vetor getKe() {
        return curMaterial.ke;
    }

    // Método para retornar o coeficiente de brilho
    double getNs() {
        return curMaterial.ns;
    }

    // Método para retornar o índice de refração
    double getNi() {
        return curMaterial.ni;
    }

    // Método para retornar o coeficiente de especularidade
    Vetor getKs() {
        return curMaterial.ks;
    }

    // Método para retornar o indice de opacidade
    double getD() {
        return curMaterial.d;
    }

    // Método para retornar as coordenadas dos pontos
    std::vector<Ponto> getVertices() {
        return vertices;
    }

    std::vector<Vetor> getNormals() {
        return normals;
    }

    // Emite um output no terminal para cada face, com seus respectivos pontos (x, y, z)
    void print_faces() {
        int i = 0;
        for (const auto& face : facePoints) {
            i++;
            std::clog << "Face " << i << ": ";
            for (const auto& point : face) {
                std::cout << "(" << point.getX() << ", " << point.getY() << ", " << point.getZ() << ")";
            }
            std::clog << std::endl;
        }
    }

    string getFilename() { return Filename; }
};

#endif