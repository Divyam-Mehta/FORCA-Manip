#!/usr/bin/env python3
# coding: utf8
'''
This file was modified from the original in order to get it working
with ROS by Prasanth Suresh(ps32611@uga.edu).
Please make sure you provide credit if you are using this code.
'''
from ultralytics import YOLO as Y
import torch.backends.cudnn as cudnn
import time
import sys
import cv2
import os
from ament_index_python import get_package_share_directory
path = get_package_share_directory('sanet_onionsorting')
sys.path.append(path + '/thirdparty/yolov5')
from thirdparty.yolov5.utils.datasets import *      
from thirdparty.yolov5.utils import torch_utils
from thirdparty.yolov5.utils.utils import *
from thirdparty.yolov5.models import *

class YOLO():
    def __init__(self, weightsfile = 'best_realkinect.pt', conf_thres = 0.8):
        
        path = get_package_share_directory('sanet_onionsorting')
        self.weights = path + '/thirdparty/yolov5/weights/'+ weightsfile
        self.source = path + '/thirdparty/yolov5/inference/images/onion_test.JPG'
        self.output = path + '/thirdparty/yolov5/inference/output'
        self.img_size = 640
        self.conf_thres = conf_thres
        self.iou_thres = 0.3
        self.fourcc = 'mp4v'
        self.device = ''
        self.view_img = False
        self.save_txt = False
        self.classes = None
        self.agnostic_nms = False
        self.augment = False
        self.bounding_boxes = []
        self.model = torch.load(self.weights)['model'].float()  # load to FP32
        print("Yolo init complete")

    def detect(self, Image = None):
        out, source, weights, view_img, save_txt, imgsz = \
            self.output, self.source, self.weights, self.view_img, self.save_txt, self.img_size
        webcam = source == '0' or source.startswith(
            'rtsp') or source.startswith('http') or source.endswith('.txt')

        # Load source image if none is passed to detect function (JK)
        if Image is None:
            Image = cv2.imread(source)
            # Double check that an Image is produced (JK)
            if Image is None:
                print("Path to image is incorrect")
                return
            
        # Initialize
        self.bounding_boxes = []
        device = torch_utils.select_device(self.device)
        half = device.type != 'cpu'  # half precision only supported on CUDA
        self.model.to(device).eval()
        if half:
            self.model.half()  # to FP16

        # Second-stage classifier
        classify = False
        if classify:
            modelc = torch_utils.load_classifier(
                name='resnet101', n=2)  # initialize
            modelc.load_state_dict(torch.load(
                'weights/resnet101.pt', map_location=device)['model'])  # load weights
            modelc.to(device).eval()

        # Set Dataloader
        vid_path, vid_writer = None, None
        if webcam:
            view_img = True
            cudnn.benchmark = True  # set True to speed up constant image size inference
            dataset = LoadStreams(source, img_size=imgsz)
        else:
            save_img = True
            cudnn.benchmark = True  # set True to speed up constant image size inference
            dataset = LoadImages(source, img_size=imgsz)

        # Get names and colors
        names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        colors = [[random.randint(0, 255) for _ in range(3)]
                for _ in range(len(names))]

        # Run inference
        t0 = time.time()

        # Added this to handle static images, gets numpy array error for static images (no height attribute) (JK)
        if isinstance(Image, np.ndarray):
            im0s = Image.astype('float32')
        else:
            im0s = np.frombuffer(Image.data, dtype=np.uint8).reshape(Image.height, Image.width, -1).astype('float32')     # Added by Prasanth Suresh

        img = letterbox(im0s, new_shape=imgsz)[0]
        img = img[:, :, ::-1].transpose(2, 0, 1)
        img = np.ascontiguousarray(img)

        vid_cap = None
        path = 'img.jpg'
        img = torch.from_numpy(img).to(device)
        img = img.half() if half else img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)


        # Inference
        t1 = torch_utils.time_synchronized()
        pred = self.model(img, augment=self.augment)[0]

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, classes=self.classes, agnostic=self.agnostic_nms)
        t2 = torch_utils.time_synchronized()

        # Apply Classifier
        if classify:
            pred = apply_classifier(pred, modelc, img, im0s)
            
        # print('**** pred: ',pred)
        ''' Sorting the bounding boxes according to ascending x values '''
        # JK: Restructured commented out lines below due to errors
        if pred[0] != None:
            pred_numpy = pred[0].cpu().numpy()
            pred_numpy = pred_numpy[pred_numpy[:,0].argsort()]
            pred_tensor_sorted = torch.from_numpy(pred_numpy).to(pred[0].device)
            pred[0] = pred_tensor_sorted
            '''pred[0] = pred[0].cpu().numpy()
            pred[0] = pred[0][pred[0][:,0].argsort()]
            pred[0] = torch.from_numpy(pred[0])'''
            # print('**** Sorted pred: \n',pred)
        # Process detections
        for i, det in enumerate(pred):  # detections per image
            if webcam:  # batch_size >= 1
                p, s, im0 = path[i], '%g: ' % i, im0s[i].copy()
            else:
                p, s, im0 = path, '', im0s

            if det != None:
                self.bounding_boxes.append(det)
            
            save_path = str(Path(out) / Path(p).name)
            s += '%gx%g ' % img.shape[2:]  # print string
            #  normalization gain whwh
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]
            if det is not None and len(det):
                # Rescale boxes from img_size to im0s size
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                # # Print results
                # for c in det[:, -1].unique():
                #     n = (det[:, -1] == c).sum()  # detections per class
                #     s += '%g %ss, ' % (n, names[int(c)])  # add to string

                # Write results
                minx = 0
                miny = 0
                maxx = 0
                maxy = 0

                for *xyxy, conf, cls in det:

                    tlx, tly, brx, bry = int(xyxy[0]), int(
                        xyxy[1]), int(xyxy[2]), int(xyxy[3])
                    if tlx < minx:
                        minx = tlx
                    if tly < miny:
                        miny = tly
                    if bry > maxy:
                        maxy = bry
                    if brx > maxx:
                        maxx = brx  # crop_img = img[y:y+h, x:x+w]

                    if save_txt:  # Write to file
                        xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)
                                        ) / gn).view(-1).tolist()  # normalized xywh
                        with open(save_path[:save_path.rfind('.')] + '.txt', 'a') as file:
                            file.write(('%g ' * 5 + '\n') %
                                    (cls, *xywh))  # label format

                    if save_img or view_img:  # Add bbox to image
                        label = '%s %.2f' % (names[int(cls)], conf)
                        plot_one_box(xyxy, im0, label=label,
                                    color=colors[int(cls)], line_thickness=3)

            # Stream results
            if view_img:
                cv2.imshow(p, im0)
                if cv2.waitKey(1) == ord('q'):  # q to quit
                    raise StopIteration

            # Save results (image with detections)
            if save_img:
                if dataset.mode == 'images':
                    cv2.imwrite(save_path, im0)
                else:
                    if vid_path != save_path:  # new video
                        vid_path = save_path
                        if isinstance(vid_writer, cv2.VideoWriter):
                            vid_writer.release()  # release previous video writer

                        fps = vid_cap.get(cv2.CAP_PROP_FPS)
                        w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                        h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                        vid_writer = cv2.VideoWriter(
                            save_path, cv2.VideoWriter_fourcc(*self.fourcc), fps, (w, h))
                    vid_writer.write(im0)

        if save_txt or save_img:
            # print('Results saved to %s' % os.getcwd() + os.sep + out)
            if platform == 'darwin':  # MacOS
                os.system('open ' + save_path)
################################ END OF COMMENTED OUT FOR LOOP TO RESTOR INDENT LATER ################################

        #print('Done. (%.3fs)' % (time.time() - t0))
        return self.bounding_boxes


# Used this for tsting (JK)
'''path = get_package_share_directory('onion')
image_path = path + "/thirdparty/yolov5/inference/images/onion_test.JPG"
image = cv2.imread(image_path)
detect = YOLO(weightsfile = 'best_realkinect.pt', conf_thres = 0.6)
detect.detect()'''

# if __name__ == '__main__':
#     parser = argparse.ArgumentParser()
#     parser.add_argument('--weights', type=str,
#                         default='weights/yolov5s.pt', help='model.pt path')
#     # file/folder, 0 for webcam
#     parser.add_argument('--source', type=str,
#                         default='inference/images', help='source')
#     parser.add_argument('--output', type=str, default='inference/output',
#                         help='output folder')  # output folder
#     parser.add_argument('--img-size', type=int, default=640,
#                         help='inference size (pixels)')
#     parser.add_argument('--conf-thres', type=float,
#                         default=0.4, help='object confidence threshold')
#     parser.add_argument('--iou-thres', type=float,
#                         default=0.5, help='IOU threshold for NMS')
#     parser.add_argument('--fourcc', type=str, default='mp4v',
#                         help='output video codec (verify ffmpeg support)')
#     parser.add_argument('--device', default='',
#                         help='cuda device, i.e. 0 or 0,1,2,3 or cpu')
#     parser.add_argument('--view-img', action='store_true',
#                         help='display results')
#     parser.add_argument('--save-txt', action='store_true',
#                         help='save results to *.txt')
#     parser.add_argument('--classes', nargs='+',
#                         type=int, help='filter by class')
#     parser.add_argument('--agnostic-nms', action='store_true',
#                         help='class-agnostic NMS')
#     parser.add_argument('--augment', action='store_true',
#                         help='augmented inference')
#     opt = parser.parse_args()
#     opt.img_size = check_img_size(opt.img_size)
#     print(opt)

#     with torch.no_grad():
#         detect()

        # Update all models
        # for opt.weights in ['yolov5s.pt', 'yolov5m.pt', 'yolov5l.pt', 'yolov5x.pt', 'yolov3-spp.pt']:
        #    detect()
        #    create_pretrained(opt.weights, opt.weights)
