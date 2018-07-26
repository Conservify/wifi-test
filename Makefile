default:
	@echo "Usage make (fkcore|fknat|feather)"

fkcore: gitdeps
	mkdir -p build
	cd build && cmake ../ -DFK_CORE=ON -DFK_NATURALIST=OFF -DADAFRUIT_FEATHER=OFF
	cmake --build build

fknat: gitdeps
	mkdir -p build
	cd build && cmake ../ -DFK_CORE=OFF -DFK_NATURALIST=ON -DADAFRUIT_FEATHER=OFF
	cmake --build build

feather: gitdeps
	mkdir -p build
	cd build && cmake ../ -DFK_CORE=OFF -DFK_NATURALIST=OFF -DADAFRUIT_FEATHER=ON
	cmake --build build

gitdeps:
	simple-deps --config arduino-libraries

clean:
	rm -rf build
