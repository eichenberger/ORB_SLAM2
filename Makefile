all:
	cd build; make -j4

python_mod:
	cd python; make -j4

test_python_mod:
	cd python/build/lib.linux-x86_64-3.6/; LD_LIBRARY_PATH=../../../lib python3 -c "import orbslam"
