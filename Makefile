VENV := .venv
PYTHON := $(VENV)/bin/python
PIP := $(VENV)/bin/pip

.PHONY: setup run clean

setup:
	python3 -m venv $(VENV)
	$(PYTHON) -m pip install --upgrade pip
	$(PIP) install Pillow

run:
	g++ -std=c++17 main.cpp -o raytracer
	./raytracer
	python3 utils/convert_ppm.py

clean:
	rm -f raytracer
	rm -rf $(VENV)
