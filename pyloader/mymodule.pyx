from libc.stdlib cimport malloc, free
import numpy as np
import torch
import threading
import time
import sys

import matplotlib.pyplot as plt
import cv2

cdef extern from "unistd.h":
    long syscall(long number, ...)

a = torch.zeros(10, dtype=torch.float32)
a = a.cuda()
print(a)
print("Threadid1:", syscall(186))

cdef public void process_image(float* buf, int length):
    cdef int i
    print("Threadid:", syscall(186))

    a = torch.zeros(length, dtype=torch.float32)
    a = a.cuda()
    
    # Load data into the tensor
    for i in range(length):
        a[i] = buf[i]

    # Rewrite the buffer with the processed data
    for i in range(length):
        buf[i] = a[i].item()


cpdef public void test_show_image():
    print("test_show_image")
    print("Threadid2:", syscall(186))

    a = torch.ones(10, dtype=torch.float32)
    a = a.cuda()
    print(torch.sin(a))
    
    # Create a dummy image
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    cv2.putText(img, "Hello, World!", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.imshow("Image", img)
    cv2.waitKey(1)

# int track_klt(const unsigned char *img, int width, int height, float *x_out,
#               float *y_out, int *ids_out, int *cnt_out, int min_dist,
#               int max_cnt, bool flow_back)

cdef public int track_klt_cy(const unsigned char *img, int width, int height,
                    float *x_out, float *y_out, int *ids_out,
                    int *cnt_out, int min_dist, int max_cnt,
                    char flow_back):
    print("track_klt_cy called", tracker)

    global tracker
    cdef int i, j

    # load image into a numpy array
    cdef object cur_img_ = np.empty((height, width), dtype=np.uint8)
    cdef unsigned char[:, :] cur_img = cur_img_ 
    for i in range(height):
        for j in range(width):
            cur_img[i, j] = img[i * width + j]
    
    # track the image
    try:
        x, y, ids, cnt = tracker.track_image(cur_img_, flow_back, max_cnt, min_dist)
    except Exception as e:
        import traceback
        traceback.print_exc()
        print(f"Error during tracking: {e}")

    print(f"Tracked {len(x)} points.")   
    # copy results to output arrays
    for i in range(len(x)):
        x_out[i] = x[i]
        y_out[i] = y[i]
        ids_out[i] = ids[i]
        cnt_out[i] = cnt[i]
    return len(x)

import sys
sys.path.append("/root/catkin_ws/src/VINS-Fusion/pyloader/")
from kltpy import KLTTracker as Tracker

tracker = Tracker()