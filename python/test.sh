base_dir=/home/eichest/projects/ORB_SLAM2
export LD_LIBRARY_PATH=$base_dir/lib
export PYTHONPATH=$base_dir/python/build/lib.linux-x86_64-3.6

#pdb3 test.py ../Vocabulary/ORBvoc.txt test.yaml
python3 $base_dir/python/test.py $base_dir/python/test.mp4 $base_dir/Vocabulary/ORBvoc.txt $base_dir/python/test.yaml $base_dir/python/test2.map $base_dir/python/keyframes

