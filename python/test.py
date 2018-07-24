import sys
import io
import json
import cv2
import orbslam
import numpy as np
#from multiprocessing import Process, Pipe
import time

def run_animation(conn):
    print("Create mlab stuff now")
    from mayavi import mlab
    x = [0]
    y = [0]
    z = [0]
    point_plot = mlab.points3d(x, y, z, scale_factor=0.01, mode='axes')
    ms = point_plot.mlab_source
    @mlab.show
    @mlab.animate(delay=200)
    def show_animation():
        while True:
            print("Animate")
            if conn.poll():
                (x, y, z) = conn.recv()
                ms.reset(x=x, y=y, z=z, s=[0.001]*len(x))
            yield
    show_animation()

def run_visualization(conn):
    print("Hallo thread")
    run_animation(conn)

#parent_conn, child_conn = Pipe()
#visualization = Process(target=run_visualization, args=(child_conn, ))
#visualization.start()

class KeyPoints:
    def __init__(self):
        self.known_points = 0

    def show_points(self, points):
        x = []
        y = []
        z = []

        for point in points:
            x.append(np.asarray(point.position)[0])
            y.append(np.asarray(point.position)[1])
            z.append(np.asarray(point.position)[2])
        self.known_points = self.known_points + len(x)
        parent_conn.send((x,y,z))

cam = sys.argv[1]
vocabulatory = sys.argv[2]
configuration = sys.argv[3]
outputmap = sys.argv[4]
if len(sys.argv) == 6:
    keyframedir = sys.argv[5]
else:
    keyframedir = None

cam = cv2.VideoCapture(cam)
#cam = cv2.VideoCapture(0)
if not cam.isOpened():
    print("Could not open cam")
    sys.exit(1)

cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

print(vocabulatory)
print(configuration)
print(outputmap)
slam = orbslam.OrbSlam(vocabulatory, configuration, orbslam.OrbSlam.MONOCULAR, True)

points = KeyPoints()
while True:
    ret, im = cam.read()
    if not ret:
        break
    timestamp = time.time()
    slam.TrackMonocular(orbslam.Mat.from_array(im), timestamp)
    if slam.NewKeyFrameSinceLastCall() and keyframedir:
        #TODO Export image and key frame
        print("New key frame inserted")
        keyframes = slam.GetKeyFrames()
        keyframe_name = "keyframe_{}".format(len(keyframes))
        cv2.imwrite(keyframedir + "/" + keyframe_name + ".png", im)
        current_keyframe = keyframes[-1] # take last one
        key_frame_data = {
            'position': np.asarray(current_keyframe.keyPoints).tolist(),
            'descriptors': np.asarray(current_keyframe.descriptors).tolist()
        }
        outfile = keyframedir + "/" + keyframe_name + ".json"
        with io.open(outfile, 'w') as f:
            json_kf= json.JSONEncoder().encode(key_frame_data)
            f.write(json_kf)



key_points = slam.GetKeyPoints()
if len(key_points) > 0:
    map = [None]*len(key_points)
    for i, key_point in enumerate(key_points):
        map[i] = {
            'position': np.transpose(np.asarray(key_point.position))[0].tolist(),
            'descriptors': np.asarray(key_point.descriptors).tolist()[0],
        }
    json_map = json.JSONEncoder().encode(map)
    with io.open(outputmap, 'w') as f:
        f.write(json_map)

slam.Shutdown()

sys.exit(0)
