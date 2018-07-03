default:
	@echo "Usage make (fkcore|fknat|feather)"

fkcore: gitdeps
	cd build && cmake ../ -DFK_CORE=ON -DFK_NATURALIST=OFF -DADAFRUIT_FEATHER=OFF
	cmake --build build

fknat: gitdeps
	cd build && cmake ../ -DFK_CORE=OFF -DFK_NATURALIST=ON -DADAFRUIT_FEATHER=OFF
	cmake --build build

feather: gitdeps
	cd build && cmake ../ -DFK_CORE=OFF -DFK_NATURALIST=OFF -DADAFRUIT_FEATHER=ON
	cmake --build build

gitdeps:
	simple-deps --config arduino-libraries

clean:
	rm -rf build
