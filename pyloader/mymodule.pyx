from libc.stdlib cimport malloc, free
import numpy as np
import torch
import threading
import time

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