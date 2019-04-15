default:
	@echo "Usage make (fkcore|fknat|feather)"

fkcore: gitdeps
	cd build && cmake ../ -DFK_CORE=ON -DFK_NATURALIST=OFF -DADAFRUIT_FEATHER=OFF
	$(MAKE) -C build

fknat: gitdeps
	cd build && cmake ../ -DFK_CORE=OFF -DFK_NATURALIST=ON -DADAFRUIT_FEATHER=OFF
	$(MAKE) -C build

feather: gitdeps
	cd build && cmake ../ -DFK_CORE=OFF -DFK_NATURALIST=OFF -DADAFRUIT_FEATHER=ON
	$(MAKE) -C build

gitdeps: build
	simple-deps --config dependencies.sd

build:
	mkdir -p build

clean:
	rm -rf build
