    #pragma once

#include <cctype>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "../../src/Vetor.h"
#include "../../src/Ponto.h"
#include "sceneSchema.hpp"
#include "jsonParser.cpp"


class SceneJsonLoader {
public:
    static  SceneData loadFile(const std::string& filename) {
        Json root = parser.parseFile(filename);
        return build(root);
    }

    static SceneData loadString(const std::string& text) {
        Json root = parser.parseString(text);
        return build(root);
    }

private:
    static JsonParser parser;

    static SceneData makeEmptySceneData() {
        SceneData scene;

        scene.camera.lookfrom = Ponto(0, 0, 0);
        scene.camera.lookat   = Ponto(0, 0, -1);
        scene.camera.upVector = Vetor(0, 1, 0);
        scene.camera.image_width  = 0;
        scene.camera.image_height = 0;
        scene.camera.screen_distance = 1.0;

        scene.globalLight.pos = Ponto(0, 0, 0);
        scene.globalLight.color = ColorData(0, 0, 0);
        scene.globalLight.extraData.clear();

        scene.lightList.clear();
        scene.objects.clear();

        return scene;
    }

    static MaterialData makeDefaultMaterial(const std::string& name = "") {
        MaterialData m;
        m.name = name;
        m.color = ColorData(0.0, 0.0, 0.0);
        m.ks    = ColorData(0.0, 0.0, 0.0);
        m.ka    = ColorData(0.0, 0.0, 0.0);
        m.kr    = ColorData(0.0, 0.0, 0.0);
        m.kt    = ColorData(0.0, 0.0, 0.0);
        m.ns    = 0.0;
        m.ni    = 1.0;
        m.d     = 1.0;
        return m;
    }

    static bool isTripletArray(const Json& j) {
        return j.isArray() && j.size() == 3 &&
               j[0].isNumber() && j[1].isNumber() && j[2].isNumber();
    }

    static std::array<double, 3> readTriple(const Json& j, const std::string& what) {
        if (j.isArray()) {
            if (j.size() != 3) throw std::runtime_error("Expected 3 values for " + what);
            if (!j[0].isNumber() || !j[1].isNumber() || !j[2].isNumber()) throw std::runtime_error("Expected numeric 3-vector for " + what);
            return { j[0].asNumber(), j[1].asNumber(), j[2].asNumber() };
        }

        if (j.isObject()) {
            const auto& o = j.asObject();

            auto has = [&](const std::string& a, const std::string& b, const std::string& c) {
                return o.find(a) != o.end() && o.find(b) != o.end() && o.find(c) != o.end();
            };

            if (has("x", "y", "z")) return { o.at("x").asNumber(), o.at("y").asNumber(), o.at("z").asNumber() };
            if (has("r", "g", "b")) return { o.at("r").asNumber(), o.at("g").asNumber(), o.at("b").asNumber() };
        }

        throw std::runtime_error("Expected a 3-component vector for " + what);
    }

    static ColorData parseColor(const Json& j, const std::string& what = "color") {
        auto v = readTriple(j, what);
        return ColorData(v[0], v[1], v[2]);
    }

    static Vetor parseVector(const Json& j, const std::string& what = "vector") {
        auto v = readTriple(j, what);
        return Vetor(v[0], v[1], v[2]);
    }

    static Ponto parsePoint(const Json& j, const std::string& what = "point") {
        auto v = readTriple(j, what);
        return Ponto(v[0], v[1], v[2]);
    }

    static std::string requireString(const Json& obj, const std::string& key) {
        if (!obj.isObject() || !obj.contains(key) || !obj.at(key).isString()) throw std::runtime_error("Expected string field: " + key);
        return obj.at(key).asString();
    }

    static double requireNumber(const Json& obj, const std::string& key) {
        if (!obj.isObject() || !obj.contains(key) || !obj.at(key).isNumber()) throw std::runtime_error("Expected numeric field: " + key);
        return obj.at(key).asNumber();
    }

    static MaterialData parseMaterialObject(const Json& node, const std::string& fallbackName) {
        if (!node.isObject()) throw std::runtime_error("Material entry must be an object");

        MaterialData m = makeDefaultMaterial(fallbackName);

        if (node.contains("name") && node["name"].isString()) m.name = node["name"].asString();

        if (node.contains("color")) m.color = parseColor(node["color"], m.name + ".color");
        if (node.contains("ka"))    m.ka    = parseColor(node["ka"],    m.name + ".ka");
        if (node.contains("ks"))    m.ks    = parseColor(node["ks"],    m.name + ".ks");
        if (node.contains("kr"))    m.kr    = parseColor(node["kr"],    m.name + ".kr");
        if (node.contains("kt"))    m.kt    = parseColor(node["kt"],    m.name + ".kt");

        if (node.contains("ns") && node["ns"].isNumber()) m.ns = node["ns"].asNumber();
        if (node.contains("ni") && node["ni"].isNumber()) m.ni = node["ni"].asNumber();
        if (node.contains("d")  && node["d"].isNumber())  m.d  = node["d"].asNumber();

        return m;
    }

    static std::map<std::string, MaterialData> parseMaterials(const Json& materialsNode) {
        if (!materialsNode.isObject()) throw std::runtime_error("'definitions.materials' must be an object");

        std::map<std::string, MaterialData> table;

        for (const auto& [name, node] : materialsNode.asObject()) table[name] = parseMaterialObject(node, name);

        return table;
    }

    static MaterialData resolveMaterial(const Json& node, const std::map<std::string, MaterialData>& table) {
        if (node.isString()) {
            const std::string& ref = node.asString();
            auto it = table.find(ref);
            if (it == table.end()) throw std::runtime_error("Unknown material reference: " + ref);
            return it->second;
        }

        if (node.isObject()) return parseMaterialObject(node, node.contains("name") && node["name"].isString() ? node["name"].asString() : "");

        throw std::runtime_error("Invalid material format");
    }

    static CameraData parseCamera(const Json& node) {
        if (!node.isObject()) throw std::runtime_error("'camera' must be an object");

        CameraData cam;
        cam.lookfrom = Ponto(0, 0, 0);
        cam.lookat   = Ponto(0, 0, -1);
        cam.upVector = Vetor(0, 1, 0);
        cam.image_width = 800;
        cam.image_height = 800;
        cam.screen_distance = 1.0;

        if (node.contains("image_width"))  cam.image_width  = static_cast<int>(requireNumber(node, "image_width"));
        if (node.contains("image_height")) cam.image_height = static_cast<int>(requireNumber(node, "image_height"));

        if (node.contains("screen_distance")) cam.screen_distance = requireNumber(node, "screen_distance");

        if (node.contains("lookfrom")) cam.lookfrom = parsePoint(node["lookfrom"], "camera.lookfrom");
        if (node.contains("lookat"))   cam.lookat   = parsePoint(node["lookat"],   "camera.lookat");
        if (node.contains("vup"))      cam.upVector  = parseVector(node["vup"],     "camera.vup");

        return cam;
    }

    static LightData parseLight(const Json& node) {
        if (!node.isObject()) throw std::runtime_error("Each light must be an object");

        LightData light;
        light.pos = Ponto(0, 0, 0);
        light.color = ColorData(0, 0, 0);
        light.extraData.clear();

        if (node.contains("name") && node["name"].isString()) light.extraData["name"] = node["name"].asString();
        if (node.contains("position")) light.pos = parsePoint(node["position"], "light.position");
        if (node.contains("color")) light.color = parseColor(node["color"], "light.color");

        for (const auto& [key, val] : node.asObject()) {
            if (key == "name" || key == "position" || key == "color") continue;
            if (val.isString()) light.extraData[key] = val.asString();
            else if (val.isNumber()) light.extraData[key] = std::to_string(val.asNumber());
            else if (val.isBool()) light.extraData[key] = val.asBool() ? "true" : "false";
        }

        return light;
    }

    static TransformData parseTransform(const Json& node) {
        if (!node.isObject()) throw std::runtime_error("Each transform must be an object");

        TransformData t;
        t.tType = requireString(node, "type");

        for (const auto& [key, val] : node.asObject()) {
            if(key == "type") continue;
            t.data = parseVector(val, "transform." + key);
        }

        return t;
    }

    static std::vector<TransformData> parseTransforms(const Json& node) {
        if (!node.isArray()) throw std::runtime_error("'transform' must be an array");

        std::vector<TransformData> list;
        for (const auto& item : node.asArray())
            list.push_back(parseTransform(item));
        
        return list;
    }

    static ObjectData parseObject(const Json& node, const std::map<std::string, MaterialData>& materials) {
        if (!node.isObject()) throw std::runtime_error("Each object entry must be an object");

        ObjectData obj;
        obj.objType = "";
        obj.relativePos = Ponto(0, 0, 0);
        obj.material = makeDefaultMaterial();
        obj.numericData.clear();
        obj.vetorPointData.clear();
        obj.otherProperties.clear();
        obj.transforms.clear();


        auto isPositionKey = [](const std::string& key){
            static const std::vector<std::string> hints = {
                "center",
                "position",
                "point",
                "origin",
                "point_on_plane",
                "relativePos"
            };

            for (const auto& h : hints) {
                if (key == h)
                    return true;
            }
            return false;
        };
        
        if (node.contains("name") && node["name"].isString()) obj.otherProperties["name"] = node["name"].asString();
        if (node.contains("material")) obj.material = resolveMaterial(node["material"], materials);
        if (node.contains("transform")) obj.transforms = parseTransforms(node["transform"]);
        if (node.contains("relativePos")) obj.relativePos = parsePoint(node["relativePos"], "object.relativePos");
        if (node.contains("type") && node["type"].isString()) obj.objType = node["type"].asString();
        else throw std::runtime_error("Object missing required field: type");

        for (const auto& [key, val] : node.asObject()) {
            if (key == "type" || key == "material" || key == "transform" || key == "name" || key == "relativePos") continue;


            if (val.isNumber())      obj.numericData[key] = val.asNumber();
            else if (val.isString()) obj.otherProperties[key] = val.asString();
            else if (val.isBool())   obj.otherProperties[key] = val.asBool() ? "true" : "false";
            else if (isTripletArray(val)) {
                Vetor v = parseVector(val, "object." + key);
                obj.vetorPointData[key] = v;

                if(isPositionKey(key)) obj.relativePos = Ponto(v.getX(), v.getY(), v.getZ());
            }
            else  obj.otherProperties[key] = val.asString();
        }

        return obj;
    }

    static SceneData build(const Json& root) {
        if (!root.isObject()) {
            throw std::runtime_error("Root JSON value must be an object");
        }

        SceneData scene = makeEmptySceneData();
        std::map<std::string, MaterialData> materials;

        // globalLight
        if (root.contains("globalLight")) {
            scene.globalLight.color = parseColor(root["globalLight"], "globalLight");
        }

        // materials
        if (root.contains("materials")) {
            materials = parseMaterials(root["materials"]);
        }

        // camera
        if (root.contains("camera")) {
            scene.camera = parseCamera(root["camera"]);
        }

        // lights
        if (root.contains("lights")) {
            const Json& lights = root["lights"];
            if (!lights.isArray()) throw std::runtime_error("'lights' must be an array");
            
            for (const auto& item : lights.asArray()) {
                scene.lightList.push_back(parseLight(item));
            }
        }

        // objects
        if (root.contains("objects")) {
            const Json& objects = root["objects"];
            if (!objects.isArray()) throw std::runtime_error("'objects' must be an array");
            for (const auto& item : objects.asArray()) scene.objects.push_back(parseObject(item, materials));
        }

        return scene;
    }
};

// static member definition
inline JsonParser SceneJsonLoader::parser;