#!/usr/bin/env python3
import cv2
import torch
from numpy import random
import sys
from time import sleep

from ament_index_python.packages import get_package_share_directory
path = get_package_share_directory('sanet_onionsorting')
sys.path.append(path + '/thirdparty/yolov7_deepsort/')
from models.experimental import attempt_load
from utils.datasets import letterbox, np
from utils.general import check_img_size, non_max_suppression, apply_classifier,scale_coords, xyxy2xywh
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, TracedModel


class Detector:
    def __init__(self, weightsfile:str = 'best_realkinect.pt',conf_thres:float = 0.25, iou_thresh:float = 0.45, agnostic_nms:bool = False, save_conf:bool = False, classes:list = None):
        '''
        args:
        conf_thres: Thresholf for Classification
        iou_thres: Thresholf for IOU box to consider
        agnostic_nms: whether to use Class-Agnostic NMS
        save_conf: whether to save confidences in 'save_txt' labels afters inference
        classes: Filter by class from COCO. can be in the format [0] or [0,1,2] etc
        '''
        self.device = select_device("0" if torch.cuda.is_available() else 'cpu')
        # self.device = os.environ["CUDA_VISIBLE_DEVICES"] = "0"        
        self.conf_thres = conf_thres
        self.iou_thres = iou_thresh
        self.classes = classes
        self.agnostic_nms = agnostic_nms
        self.save_conf = save_conf
        self.weights = path + '/thirdparty/yolov7_deepsort/weights/'+ weightsfile

    def load_model(self, img_size:int = 640, trace:bool = True, classify:bool = False):
        '''
        weights: Path to the model
        img_size: Input image size of the model
        trace: Whether to trace the model or not
        classify: whether to load the second stage classifier model or not
        '''

        # Load model
        self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
        self.stride, self.imgsz = None, img_size

        if trace:
            print("Traced model not found. Creating new traced model.")
            self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
            self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
            self.stride = int(self.model.stride.max())  # model stride
            self.imgsz = check_img_size(img_size, s=self.stride)  # check img_size

            self.model = TracedModel(self.model, self.device, self.imgsz)
        else:
            self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
            self.half = self.device.type != 'cpu'  # half precision only supported on CUDA
            self.stride = int(self.model.stride.max())  # model stride
            self.imgsz = check_img_size(img_size, s=self.stride)  # check img_size

        if self.half:
            self.model.half()  # to FP1
        
        # Run inference for CUDA just once
        if self.device.type != 'cpu':
            self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once

        # Second-stage classifier
        self.classify = classify
        if classify:
            self.modelc = load_classifier(name='resnet101', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=self.device)['model']).to(self.device).eval()

        # Get names and colors of Colors for BB creation
        if hasattr(self.model, 'module'):
            self.names = self.model.module.names
        else:
            self.names = self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]
        
    @torch.no_grad()
    def detect(self, source, plot_bb:bool =True):
        '''
        source: Path to image file, video file, link or text etc
        plot_bb: whether to plot the bounding box around image or return the prediction
        '''
        img, im0 = self.load_image(source)
        img = torch.from_numpy(img).to(self.device)
        img = img.half() if self.half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3: # Single batch -> single image
            img = img.unsqueeze(0)

        # Inference
        pred = self.model(img, augment=False)[0] # We don not need any augment during inference time

        # print("Yolo classes: ", self.classes)

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)

        # Apply Classifier
        if self.classify:
            pred = apply_classifier(pred, self.modelc, img, im0) # I thnk we need to add a new axis to im0


        # Post - Process detections
        if pred[0] != None:
            pred[0] = pred[0].cpu().numpy()
            # print("Pred: ", pred[0])
            # print("Pred[0][:,0] ", pred[0][:,0])
            pred[0] = pred[0][(-pred[0][:,0]).argsort()]   # Sorting them by descending x values. This makes it same order as deepsort o/p.
            # print("Sorted: ", pred[0])
            # sleep(1000)
            pred[0] = torch.from_numpy(pred[0])

        det = pred[0] # detections per image but as we have just 1 image, it is the 0th index
        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # Write results
            for *xyxy, conf, cls in reversed(det):

                if plot_bb:  # Add bbox to image   # save_img
                    label = f'{self.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=1)
                    
        
            return im0, det.detach().cpu().numpy()

        return im0, None # just in case there's no detection, return the original image. For tracking purpose plot_bb has to be False always
    
    def load_image(self, img0):
        '''
        Load and pre process the image
        args: img0: Path of image or numpy image in 'BGR" format
        '''
        if isinstance(img0, str): img0 = cv2.imread(img0)  # BGR
        assert img0 is not None, 'Image Not Found '

        # Padded resize
        img = letterbox(img0, self.imgsz, stride=self.stride)[0]

        # Convert
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, to 3x416x416
        img = np.ascontiguousarray(img)

        return img, img0
    

    def save_txt(self, det, im0_shape, txt_path):
        '''
        Save the results of an image in a .txt file
        args:
            det: detecttions from the model
            im0_shape: Shape of Original image
            txt_path: File of the text path
        '''
        gn = torch.tensor(im0_shape)[[1, 0, 1, 0]]  # normalization gain whwh
        for *xyxy, conf, cls in reversed(det):
            xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
            line = (cls, *xywh, conf) if self.save_conf else (cls, *xywh)  # label format
            with open(txt_path + '.txt', 'a') as f:
                f.write(('%g ' * len(line)).rstrip() % line + '\n')
