all:
	rm -rf build
	mkdir build
	cd build && cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DMMU=on -DCMAKE_CROSS_COMPILER_PATH=/u/cs452/public/xdev/bin ..
	cd build && ninja
	cp build/kernel.img build/kernel.img.bak

local:
	rm -rf build
	mkdir build
	cd build && cmake -G Ninja -DCMAKE_BUILD_TYPE=Release -DMMU=on ..
	cd build && ninja

clean:
	rm -rf build
