#!/usr/bin/env python
# AKAMAV
# project: imav2022
# author: Junbo Li
# Version: 2022.09.05

import cv2
import torch
import numpy as np
import tensorrt as trt
from geometry_msgs.msg import Pose
from collections import OrderedDict, namedtuple


#---------- function: Infer_init ----------
#   purpose: Initialize the inference engine
#   parameters:
#   ~engine: model file (TensorRT engine)
#   ~device: GPU device number
def Infer_init(engine, device):
    Binding = namedtuple('Binding', ('name', 'dtype', 'shape', 'data', 'ptr'))
    logger = trt.Logger(trt.Logger.INFO)
    trt.init_libnvinfer_plugins(logger, namespace="")
    with open(engine, 'rb') as f, trt.Runtime(logger) as runtime:
        model = runtime.deserialize_cuda_engine(f.read())
    bindings = OrderedDict()
    for index in range(model.num_bindings):
        name = model.get_binding_name(index)
        dtype = trt.nptype(model.get_binding_dtype(index))
        shape = tuple(model.get_binding_shape(index))
        data = torch.from_numpy(np.empty(shape, dtype=np.dtype(dtype))).to(device)
        bindings[name] = Binding(name, dtype, shape, data, int(data.data_ptr()))
    binding_addrs = OrderedDict((n, d.ptr) for n, d in bindings.items())
    context = model.create_execution_context()
    return bindings, binding_addrs, context


#---------- function: letterbox ----------
#   purpose: image resize
#   parameters:
#   ~im: input image
#   ~new_shape: new shape
#   ~color: border color
#   ~auto: minimum rectangle (w, h < stride)
#   ~scaleup: whether to scale up
#   ~stride: stride
def letterbox(im, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleup=True, stride=32):
    # Resize and pad image while meeting stride-multiple constraints
    shape = im.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding

    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    return im, r, (dw, dh)


#---------- function: img_preprocess ----------
#   purpose: preprocess the input image
#   parameters:
#   ~image: Input opencv format image
#   ~device: GPU device number
def img_preprocess(image, device):
    img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    img, ratio, dwdh = letterbox(img, auto=False)
    img = img.transpose((2, 0, 1))
    img = np.expand_dims(img, 0)
    img = np.ascontiguousarray(img)
    img = img.astype(np.float32)
    #print(im.shape)
    img = torch.from_numpy(img).to(device)
    img /= 255
    return img, ratio, dwdh


#---------- function: postprocess ----------
#   purpose: Post-processing the anchor box of the located object to get its coordinates in the original image
#   parameters:
#   ~boxes: anchor boxes of all located object
#   ~r: scale ratio
#   ~dwdh: padding height & width
def postprocess(boxes, r, dwdh):
    dwdh = torch.tensor(dwdh*2).to(boxes.device)
    boxes -= dwdh
    boxes /= r
    return boxes


#---------- function: truck_pos_estimator ----------
#   purpose: estimate the coordinates of the truck center point based on the truck's anchor box
#   parameters:
#   ~ratio: scale ratio
#   ~dwdh: padding height & width
#   ~boxes: anchor boxes of all located object
#   ~scores: the probability of the class inferred by the model
#   ~classes: different classes (in number 0,1,2...)
def truck_pos_estimator(ratio, dwdh, boxes, scores, classes):
    count = 0
    index_list = []
    score_list = []
    for index in range(len(classes)):
        if int(classes[index]) == 5:
            index_list.append(index)
            score_list.append(scores[index])
            count += 1
    #print(score_list)
    if count == 0:
        truck_pos = False
    else:
        best_match = score_list.index(max(score_list))
        box = postprocess(boxes[index_list[best_match]], ratio, dwdh).tolist()
        truck_pos = (int((box[0]+box[2])/2), int((box[1]+box[3])/2))
    return truck_pos


#---------- function: pose2msg_converter ----------
#   purpose: estimate truck's pos from its center point coordinates in the image and convert it to rosmsg
#   parameters:
#   ~truck_pos: the coordinates of the truck center point
#   ~cam_pos: drone pos at the same time
#   ~img_height: original image height
#   ~img_width: original image width
#   ~scale_factor: scaling factor for estimated distance based on the height of the drone 
#                  (!!! It needs to be adjusted manually according to the actual situation !!!)
def pose2msg_converter(truck_pos, cam_pos, img_height, img_width, scale_factor):
    output_pose = Pose()
    bias_x = (truck_pos[0] - img_width/2) * scale_factor
    bias_y = -(truck_pos[1] - img_height/2) * scale_factor
    output_pose = cam_pos.pose
    output_pose.position.x += bias_x
    output_pose.position.y += bias_y
    return output_pose


#---------- function: visualizer ----------
#   purpose: visualizer for debugging
#   parameters:
#   ~img: original input image
#   ~truck_pos: truck's center point coordinates in the image
def visualizer(img, truck_pos):
    cv2.circle(img, truck_pos, 5, (0, 0, 255), -1)
    result = cv2.resize(img, [int(img.shape[1]/2), int(img.shape[0]/2)], interpolation=cv2.INTER_LINEAR)
    cv2.imshow("result", result)
    cv2.waitKey(1)