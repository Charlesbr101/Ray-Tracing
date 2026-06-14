#pragma once

#include "../../src/Vetor.h"
#include "../../src/Ponto.h"
#include <string>
#include <vector>
#include <map>
#include <algorithm>
using namespace std;


struct CameraData {
    Ponto lookfrom;
    Ponto lookat;
    Vetor upVector = Vetor(0, 1, 0);
    
    int image_width;
    int image_height;
    double screen_distance = 1;
};


struct ColorData {
    double r, g, b;
    ColorData(){}
    ColorData(double r, double g, double b) : r(r), g(g), b(b) {}

    ColorData operator+( const ColorData& other ) const {
        return ColorData(r + other.r, g + other.g, b + other.b);
    }
    ColorData operator-( const ColorData& other ) const {
        return ColorData(r - other.r, g - other.g, b - other.b);
    }
    ColorData operator*( double scalar ) const {
        return ColorData(r * scalar, g * scalar, b * scalar);
    }
    ColorData operator*( ColorData c) const {
        return ColorData(r * c.r, g * c.g, b * c.b);
    }

	bool positive() {
		return r > 0 || g > 0 || b > 0;
	}
};


struct LightData {
    Ponto       pos;
    ColorData   color;
    map<string, string> extraData;
};


struct TransformData {
    string tType;  // translation  | scaling   | rotation
    Vetor data;    // (dx, dy, dz) | (x, y, z) | (alpha, theta, gama)
};


struct MaterialData {
    string    name;
    ColorData color;  // Difuso      (kd)
    ColorData ks;     // Especular   (ks)
    ColorData ka;     // Ambiente    (ka)
    ColorData kr;     // Reflexivo   (kr)  [Emissivo (ke)]
    ColorData kt;     // Transmicivo (kt)
    double    ns;     // Rugosidade  (eta) [Brilho]
    double    ni;     // Refração 
    double    d;      // Opacidade
};


struct ObjectData {
    string objType;     // sphere, plane, mesh, (custom?) ...
    
    Ponto relativePos = Ponto(0, 0, 0);
    MaterialData material;
    
    map<string, double> numericData;
    map<string, Vetor>  vetorPointData;
    map<string, string> otherProperties;
    
    vector<TransformData> transforms; // a ordem importa!

    vector<vector<Ponto>> facePoints; // Only for meshes: list of faces, each face is a list of vertices (Pontos)
    vector<Vetor> faceNormals;        // Only for meshes: list of normals for each

    double  getNum     (string key){ return numericData[key]; }
    int64_t getInt     (string key){ return (int64_t)numericData[key]; }
    Vetor   getVetor   (string key){ return vetorPointData[key]; }
    Ponto   getPonto   (string key){ auto p = vetorPointData[key]; return Ponto(p.getX(), p.getY(), p.getZ()); }
    string  getProperty(string key){ return otherProperties[key]; }
};


struct SceneData {
    CameraData camera;
    
    LightData globalLight;
    vector<LightData> lightList;

    vector<ObjectData> objects;
};





/* ---------------------------------------------------
   Output format  - Ignore this
--------------------------------------------------- */

inline int IdentSpacing = 0;
static void indent(ostream& os){ for (int i = 0; i < IdentSpacing; i++) os << "    "; }
static void printAsArray(ostream& os, Vetor v) { os << "[" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]"; }
static void printAsArray(ostream& os, Ponto v) { os << "[" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]"; }

ostream& operator<<(ostream& os, const ColorData& c) { return os << "[" << c.r << ", " << c.g << ", " << c.b << "]"; }

ostream& operator<<(ostream& os, const TransformData& t) {
    indent(os); os << "{\n"; IdentSpacing++;

    indent(os); os << "\"type\": \"" << t.tType << "\",\n";
    indent(os); os << "\"data\": "; printAsArray(os, t.data);
    
    os << "\n"; IdentSpacing--; indent(os);
    return os << "}";
}

ostream& operator<<(ostream& os, const MaterialData& m) {
    indent(os); os << "{\n"; IdentSpacing++;
    indent(os); os << "\"name\": \"" << m.name << "\",\n";
    indent(os); os << "\"color\": " << m.color << ",\n";
    indent(os); os << "\"ks\": " << m.ks << ",\n";
    indent(os); os << "\"ka\": " << m.ka << ",\n";
    indent(os); os << "\"kr\": " << m.kr << ",\n";
    indent(os); os << "\"kt\": " << m.kt << ",\n";
    indent(os); os << "\"ns\": " << m.ns << ",\n";
    indent(os); os << "\"ni\": " << m.ni << ",\n";
    indent(os); os << "\"d\": "  << m.d  << "\n";
    IdentSpacing--; indent(os);
    return os << "}"; 
}


ostream& operator<<(ostream& os, const LightData& l) {
    indent(os); os << "{\n"; IdentSpacing++;
    auto extraData = l.extraData;

    indent(os); os << "\"position\": "; printAsArray(os, l.pos); os << ",\n";
    indent(os); os << "\"color\": "     << l.color << ",\n";
    indent(os); os << "\"name\": \""    << extraData["name"] << "\"";

    for (auto& [k, v] : extraData) if(k != "name") {
        os << ",\n";
        indent(os); os << "\"" << k << "\": \"" << v << "\"";
    }

    os << "\n"; 
    IdentSpacing--; indent(os); os << "}";
    return os;
}


ostream& operator<<(ostream& os, const ObjectData& obj) {
    os << "{\n"; IdentSpacing++;

    indent(os); os << "\"type\": \""     << obj.objType << "\",\n";
    indent(os); os << "\"relativePos\": "; printAsArray(os, obj.relativePos); os << ",\n";
    indent(os); os << "\"material\": \"" << obj.material.name << "\"";

    for (auto& [k, v] : obj.numericData) {
        os << ",\n";
        indent(os); os << "\"" << k << "\": " << v;
    }

    for (auto& [k, v] : obj.vetorPointData) {
        os << ",\n";
        indent(os); os << "\"" << k << "\": "; printAsArray(os, v);
    }

    for (auto& [k, v] : obj.otherProperties) {
        os << ",\n";
        indent(os); os << "\"" << k << "\": \"" << v << "\"";
    }

    // transforms
    if (!obj.transforms.empty()) {
        os << ",\n";

        indent(os); os << "\"transform\": [\n";
        IdentSpacing++;

        for(int i = 0; i < obj.transforms.size(); ++i) {
            os << obj.transforms[i];
            
            if(i < obj.transforms.size() - 1) os << ",";
            os << "\n";
        }

        IdentSpacing--; indent(os); os << "]";
    }

    os << "\n";
    IdentSpacing--; indent(os); 
    return os << "}";
}


ostream& operator<<(ostream& os, const CameraData& c) {
    indent(os); os << "{\n"; IdentSpacing++;
    
    indent(os); os << "\"lookfrom\": "; printAsArray(os, c.lookfrom); os << ",\n";
    indent(os); os << "\"lookat\": ";   printAsArray(os, c.lookat);   os << ",\n";
    indent(os); os << "\"upVector\": "; printAsArray(os, c.upVector); os << ",\n";

    indent(os); os << "\"image_width\": "     << c.image_width  << ",\n";
    indent(os); os << "\"image_height\": "    << c.image_height << ",\n";
    indent(os); os << "\"screen_distance\": " << c.screen_distance << "\n";

    IdentSpacing--; indent(os); os << "}";
    return os;
}


ostream& operator<<(ostream& os, const SceneData& s) {
    indent(os); os << "{\n"; IdentSpacing++;

    indent(os); os << "\"camera\": " << s.camera << ",\n";
    indent(os); os << "\"globalLight\": " << s.globalLight.color << ",\n";
    
    // Luzes
    indent(os); os << "\"lights\": [\n"; IdentSpacing++;
    
    for (size_t i = 0; i < s.lightList.size(); ++i) {
        os << s.lightList[i];
        if (i < s.lightList.size() - 1) os << ",";
        os << "\n";
    }

    IdentSpacing--; indent(os); os << "],\n";

    // materials
    vector<MaterialData> materials;
    for (const auto& obj : s.objects) {
        materials.push_back(obj.material);
    }

    sort(materials.begin(), materials.end(), [](const MaterialData& a, const MaterialData& b){ return a.name < b.name; });
    materials.erase(unique(materials.begin(), materials.end(), [](const MaterialData& a, const MaterialData& b){ return a.name == b.name; }), materials.end());

    indent(os); os << "\"materials\": {\n"; IdentSpacing++;
    for (size_t i = 0; i < materials.size(); ++i) {
        indent(os); os << "\"" << materials[i].name << "\": " << materials[i];
        if (i < materials.size() - 1) os << ",";
        os << "\n";
    }
    IdentSpacing--; indent(os); os << "},\n";

    // Objetos
    indent(os); os << "\"objects\": [\n"; IdentSpacing++;

    for (size_t i = 0; i < s.objects.size(); ++i) {
        indent(os); os << s.objects[i];
        if (i < s.objects.size() - 1) os << ",";
        os << "\n";
    }

    IdentSpacing--; indent(os); os << "]\n";


    IdentSpacing--; indent(os); os << "}"; 
    return os;
}
