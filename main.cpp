#include <iostream>
#include "src/Ponto.h"
#include "src/Vetor.h"
#include "src/Camera.h"
#include "utils/Scene/sceneParser.cpp"

using namespace std;

int main(){

    SceneData scene = SceneJsonLoader::loadFile("utils/input/minhaScene.json");
    
    Camera camera(scene.camera);

    camera.rayTrace(scene.objects);

    camera.plotPixels();
}