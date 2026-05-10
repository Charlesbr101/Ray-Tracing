flags = -O3 -std=c++23

render: main.cpp src/Camera.h src/Ponto.h src/Vetor.h
	g++ -o render main.cpp -std=c++17

run: render
	./render
	python3 utils/convert_ppm.py imagem.ppm imagem.png

clean:
	rm -rf ./render
	rm -f ./*.ppm
	rm -f ./*.png

debugger: main.cpp src/Camera.h src/Ponto.h src/Vetor.h
	@g++ $(flags) main.cpp -o debugger
	@echo "Build: Debug"
	@echo