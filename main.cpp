#include <iostream>

#include "src/Camera.h"
#include "src/Ponto.h"
#include "src/Vetor.h"
#include "utils/Scene/sceneParser.cpp"

using namespace std;

struct arguments {
    string inputName;
    string sceneFile;
    string outputFile;

    Ponto bspOrigin;
    bool useBSP = false;
    Ponto bspLookat;
    Vetor bspUp;

    arguments()
        : inputName("utils/input/caso4.json"),
          sceneFile(""),
          outputFile("imagem"),
          bspOrigin(0, 0, 0),
          bspLookat(0, 0, 0),
          bspUp(0, 1, 0) {}
};

arguments parseArguments(int argc, char* argv[]) {
    arguments args;
    for (int i = 0; i < argc; i++) {
        string arg = argv[i];
        if (arg == "-i" && i + 1 < argc) {
            args.inputName = string("utils/input/") + argv[++i] + ".json";
        } else if (arg == "-f" && i + 1 < argc) {
            args.sceneFile = argv[++i];
            cout << "Scene file: " << args.sceneFile << endl;
        } else if (arg == "-o" && i + 1 < argc) {
            args.outputFile = argv[++i];
        } else if (arg == "--bsp-cam" && i + 3 < argc) {
            args.useBSP = true;
            args.bspOrigin =
                Ponto(std::stod(argv[i + 1]), std::stod(argv[i + 2]),
                      std::stod(argv[i + 3]));
            i += 3;
            cout << "BSP debug camera: " << args.bspOrigin << endl;
        } else if (arg == "--bsp-lookat" && i + 3 < argc) {
            args.bspLookat =
                Ponto(std::stod(argv[i + 1]), std::stod(argv[i + 2]),
                      std::stod(argv[i + 3]));
            i += 3;
        } else if (arg == "--bsp-up" && i + 3 < argc) {
            args.bspUp = Vetor(std::stod(argv[i + 1]), std::stod(argv[i + 2]),
                               std::stod(argv[i + 3]));
            i += 3;
        }
    }
    return args;
}

struct Timer {
    std::chrono::time_point<std::chrono::high_resolution_clock> start;

    Timer() { start = std::chrono::high_resolution_clock::now(); }

    ~Timer() {
        double time = elapsed();
        std::cout << "Execution time: " << time << " seconds" << std::endl;
    }

    double elapsed() const {
        auto now = std::chrono::high_resolution_clock::now();
        return std::chrono::duration<double>(now - start).count();
    }
};

int main(int argc, char* argv[]) {
    Timer timer;  // Start the timer

    auto args = parseArguments(argc, argv);

    string sceneFile = args.sceneFile.empty() ? args.inputName : args.sceneFile;
    SceneData scene = SceneJsonLoader::loadFile(sceneFile);

    Camera camera(scene);

    if (args.useBSP) {
        camera.render(scene.objects, &args.bspOrigin, &args.bspLookat,
                      &args.bspUp);
    } else {
        camera.render(scene.objects);
    }

    camera.plotPixels(args.outputFile);
}