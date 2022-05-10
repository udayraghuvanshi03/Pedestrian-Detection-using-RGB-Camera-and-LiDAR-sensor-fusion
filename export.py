#!usr/bin/python3

# from bagpy import bagreader
# from matplotlib import scale
# import pandas as pd
# import numpy as np
# import matplotlib.pyplot as plt
# from scipy.spatial.transform import Rotation as R

# b=bagreader('/home/uday/catkin_ws/src/Final_project/nighttest.bag')
# #print(b.topic_table)
# image_data = []
# for t in b.topics:
#     data = b.message_by_topic(t)
#     image_data.append(data)

# irread=pd.read_csv(image_data[1])

import torch
x = torch.rand(5, 3)
print(x)
