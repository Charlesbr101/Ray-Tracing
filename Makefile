flags = -O3 -std=c++23

render: main.cpp src/Camera.h src/Ponto.h src/Vetor.h
	g++ -o render main.cpp -std=c++17

render_nobsp: main.cpp src/Camera.h src/Ponto.h src/Vetor.h
	g++ -o render_nobsp main.cpp -std=c++17 -DNO_BSP

run: render
	./render
	python3 utils/convert_ppm.py imagem.ppm imagem.png

clean:
	rm -rf ./render ./render_nobsp
	rm -f ./*.ppm
	rm -f ./*.png

debugger: main.cpp src/Camera.h src/Ponto.h src/Vetor.h
	@g++ $(flags) main.cpp -o debugger
	@echo "Build: Debug"
	@echo