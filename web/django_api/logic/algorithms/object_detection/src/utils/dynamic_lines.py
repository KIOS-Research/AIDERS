import cv2
from collections import deque
import numpy as np
import math
import numba as nb
from .rect import to_tlwh, get_center
from .file_handler import File_handler
from .numba import  transform,perspective_transform
import time


refPt= []
refRightPT = []
left_click = 0
right_click = 0
timer = 0


LINE_DTYPE = np.dtype(
    [('index', int),
     ('xy1', int, 2),
     ('xy2', int, 2)],
    align=True
)

QUEUE_DTYPE = np.dtype(
    [('index', int),
     ('xy1', int, 2),
     ('xy2', int, 2),
     ('xy3', int, 2),
     ('xy4', int, 2),
     ('area', float, 1)],
    align=True
)

class Line:
    def __init__(self,index,xy1,xy2):
        self.line = []
        new_line = []
        new_line.append((index,xy1,xy2))
        self.line= np.asarray(new_line, dtype=LINE_DTYPE).view(np.recarray)
        self.track_ids = []

    def add_track(self,trackid):
        self.track_ids.append(trackid)

class Rectangle_Queue:
    def __init__(self, index, xy1, xy2, xy3, xy4, area):
        self.rect = []
        new_rect = []
        new_rect.append((index, xy1, xy2, xy3, xy4, area))
        self.rect= np.asarray(new_rect, dtype=QUEUE_DTYPE).view(np.recarray)
        self.track_ids = []

    def add_track(self, trackid):
        self.track_ids.append(trackid)


class DynaLines:
    def __init__(self,buffer_size=30,export=None):
        self.lines = deque([], maxlen=buffer_size)
        self.queues = deque([], maxlen=buffer_size)
        self.export = export
        self.frame = 0

    def get_mouse_clicks(self,event, x, y, flags, param):
        # grab references to the global variables
        global refPt, refRightPT, left_click, right_click, timer

        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
        if event == cv2.EVENT_LBUTTONDOWN:
            refPt = [(x, y)]
            left_click = 1
            # print("button click event down")

        # check to see if the left mouse button was released
        elif event == cv2.EVENT_LBUTTONUP:
            # record the ending (x, y) coordinates and indicate that
            # the cropping operation is finished
            if left_click == 1:
                refPt.append((x, y))
                self.frame = param
                self.draw_lines(refPt[0],refPt[1])
                refPt = []
                left_click = 0

        elif event == cv2.EVENT_RBUTTONDOWN:
            if right_click == 0:
                right_click = 1
                refRightPT = [(x, y)]
            elif right_click == 2:
                right_click = 3
                refRightPT.append((x, y))
                timer = time.perf_counter()
            # for line in self.lines:
            #     if self.intersect((x,y),(x,y),line.line.xy1[0],line.line.xy2[0]) is True:
            #         print('Lines cannot overlap, line discarded!!!')

        elif event == cv2.EVENT_RBUTTONUP:
            if right_click == 1:
                refRightPT.append((x, y))
                right_click = 2
            elif right_click == 3 :#and (time.perf_counter()-timer < 3.0):
                refRightPT.append((x, y))
                self.frame = param
                self.draw_queue(refRightPT)
                # self.draw_queue(refRightPT[0], refRightPT[1])
                refRightPT = []
                right_click = 0
                timer = 0
            else:
                refRightPT = []
                right_click = 0
                timer = 0

    def draw_lines(self,xy,xy2):
        for line in self.lines:
            if self.intersect(xy,xy2,line.line.xy1[0],line.line.xy2[0]):
                print('Lines cannot overlap, line discarded!!!')
                return

        #checking if the line is too small
        if self.eucl_dist(xy[0], xy2[0], xy[1], xy2[1]) < 15:
            print('Line cannot be that small, line discarded!!!')
            return

        self.lines.append(Line(len(self.lines),xy,xy2))

        if self.export is not None:
            self.export.add_line(len(self.lines)-1,xy,xy2,self.frame)

    def draw_queue(self,pts):
        #checking if the line is too small
        xy, xy2, xy3, xy4 = pts
        if (xy[0] == xy2[0]) or (xy3[0] == xy4[0]) or (xy[1] == xy2[1]) or (xy3[1] == xy4[1]):
            print('This is not a rectangle.')
        # if xy[1] == xy2[1]:
        #     print('This is not a rectangle.')
        if (self.eucl_dist(xy[0], xy2[0], xy[1], xy2[1]) < 15) or (self.eucl_dist(xy3[0], xy4[0], xy3[1], xy4[1]) < 15):
            print('Line cannot be that small, line discarded!!!')
            return
        area = 0
        area = calc_area(xy,xy2)
        self.queues.append(Rectangle_Queue(len(self.queues),xy,xy2,xy3,xy4,area))

        if self.export is not None:
            self.export.add_queue(len(self.queues)-1, xy, xy2, xy3, xy4, self.frame)

    # def draw_queue(self,xy,xy2):
    #     #checking if the line is too small
    #     if xy[0] == xy2[0]:
    #         print('This is not a rectangle.')
    #     if xy[1] == xy2[1]:
    #         print('This is not a rectangle.')
    #     if self.eucl_dist(xy[0], xy2[0], xy[1], xy2[1]) < 15:
    #         print('Line cannot be that small, line discarded!!!')
    #         return
    #     area = 0
    #     area = calc_area(xy,xy2)
    #     self.queues.append(Rectangle_Queue(len(self.queues),xy,xy2,area))

    def check_queue_intersect(self, track, id, label, frame, lost=None):
        tl,br = (track[0],track[1]),(track[2],track[3])
        x,y,w,h = to_tlwh(track)
        tr, bl = (int(x+w), int(y)),(x, int(y + h))

        indx=0
        for queue in self.queues:
            indx = queue.rect.index[0]
            if lost:
                if id in queue.track_ids:
                    self.queues[indx].track_ids.remove(id)
                    if self.export is not None:
                        self.export.add_queue_count(indx, id, label, frame, len(self.queues[indx].track_ids), lost)
                continue
            pts = np.vstack((queue.rect.xy1[0], queue.rect.xy2[0], queue.rect.xy3[0], queue.rect.xy4[0]))
            pts = pts.reshape((-1, 1, 2))
            if cv2.pointPolygonTest(pts, get_center(track), False) == 1.0:
            # if self.check_overlap((queue.rect.xy1[0], queue.rect.xy2[0], queue.rect.xy3[0], queue.rect.xy4[0]), tl):
                if id not in queue.track_ids:
                        self.queues[indx].track_ids.append(id)
                        if self.export is not None:
                            # veh_id,type,frame,total
                            self.export.add_queue_count(indx, id, label, frame, len(self.queues[indx].track_ids), lost)
            else:
                if id in queue.track_ids:
                    self.queues[indx].track_ids.remove(id)
                    if self.export is not None:
                        self.export.add_queue_count(indx, id, label, frame, len(self.queues[indx].track_ids), True)
                    return
            # indx+=1

    def check_line_intersect(self,track, id, label, frame):
        tl,br = (track[0],track[1]),(track[2],track[3])
        x,y,w,h = to_tlwh(track)
        tr, bl = (int(x+w), int(y)),(x, int(y + h))

        indx=0
        for line in self.lines:
            if id not in line.track_ids:
                if self.intersect(tl, br, line.line.xy1[0], line.line.xy2[0]) \
                    or self.intersect(tr, bl, line.line.xy1[0], line.line.xy2[0]) :
                    self.lines[indx].track_ids.append(id)
                    if self.export is not None:
                        # veh_id,type,frame,total
                        self.export.add_line_count(indx, id, label, frame, len(self.lines[indx].track_ids))
                    return
            indx+=1


    # Return true if line segments AB and CD intersect
    def intersect(self, A, B, C, D):
        return self.ccw(A, C, D) != self.ccw(B, C, D) and self.ccw(A, B, C) != self.ccw(A, B, D)

    # for line check
    @staticmethod
    @nb.njit(cache=True)
    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])

    # #for queue check
    # @staticmethod
    # @nb.njit(cache=True)
    # def check_overlap(queue, tl):
    #     xy1, xy2, xy3, xy4 = queue
    #
    #     X, Y = tl
    #     # print(pts)
    #     dist = cv2.pointPolygonTest(pts, (52, 288), False)
    #     if dist == 1.0:
    #         return True
    #     else:
    #         return False
    #     # xmax, ymax, xmin, xmin = [], [], [], []
    #     # xmax , ymax = max(xy1[0], xy2[0]), max(xy1[1], xy2[1])
    #     # xmin , ymin = min(xy1[0], xy2[0]), min(xy1[1], xy2[1])
    #     #
    #     # if xmin < X < xmax and ymin < Y < ymax :
    #     #     return True
    #     # else:
    #     #     return False

    """    
    Euclidean distance calculation. 
    """
    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def eucl_dist( x1, x2, y1, y2):
        result = math.sqrt(pow(abs(x2 - x1), 2.0) + pow(abs(y2 - y1), 2.0))
        return result

    def draw_count_lines(self,frame):
        if len(self.lines)>0:
            for line in self.lines:
                cv2.line(frame, tuple(line.line.xy1[0]),tuple(line.line.xy2[0]), (0,255,0), 1)
                if len(line.track_ids)>0:
                    text = str(len(line.track_ids))
                    tl = line.line.xy1[0]
                    (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)
                    cv2.rectangle(frame, tl, (tl[0] + text_width - 1, tl[1] + text_height - 1),
                                  (0.0,255.0,0.0), cv2.FILLED)
                    cv2.putText(frame, text, (tl[0], tl[1] + text_height - 1), cv2.FONT_HERSHEY_DUPLEX,
                                0.5, 0, 1, cv2.LINE_AA)


    def draw_queues(self,frame):
        if len(self.queues)>0:
            for queue in self.queues:
                # cv2.rectangle(frame, queue.rect.xy1[0], queue.rect.xy2[0], (0,255,0), 1)
                # print([ queue.rect.xy1, queue.rect.xy2[0], queue.rect.xy3[0], queue.rect.xy4[0]])
                pts = np.vstack((queue.rect.xy1, queue.rect.xy2[0], queue.rect.xy3[0], queue.rect.xy4[0]))
                # pts = pts.reshape((-1, 1, 2))

                cv2.polylines(frame, [pts] \
                              , True,(0,255,0), 1)
                if len(queue.track_ids)>0:
                    text = str(len(queue.track_ids))
                    tl = queue.rect.xy1[0]
                    (text_width, text_height), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, 0.5, 1)
                    cv2.rectangle(frame, tl, (tl[0] + text_width - 1, tl[1] + text_height - 1),
                                  (0.0, 255.0, 0.0), cv2.FILLED)
                    cv2.putText(frame, text, (tl[0], tl[1] + text_height - 1), cv2.FONT_HERSHEY_DUPLEX,
                                0.5, 0, 1, cv2.LINE_AA)

    def correct_lines(self,homography):
        for line in self.lines:
            line = line.line
            # print('bfr',line.xy1[0], line.xy2[0])
            self.lines[line.index[0]].line.xy1[0] =  self._estimate_line(line.xy1[0],homography)
            self.lines[line.index[0]].line.xy2[0] =  self._estimate_line(line.xy2[0],homography)
            # print('after',self.lines[line.index[0]].line.xy1[0] ,self.lines[line.index[0]].line.xy2[0])

    def correct_queues(self,homography):
        for line in self.queues:
            line = line.rect
            # print('bfr',line.xy1[0], line.xy2[0])
            self.queues[line.index[0]].rect.xy1[0] =  self._estimate_line(line.xy1[0],homography)
            self.queues[line.index[0]].rect.xy2[0] =  self._estimate_line(line.xy2[0],homography)
            self.queues[line.index[0]].rect.xy3[0] =  self._estimate_line(line.xy3[0],homography)
            self.queues[line.index[0]].rect.xy4[0] =  self._estimate_line(line.xy4[0],homography)
            # print('after',self.lines[line.index[0]].line.xy1[0] ,self.lines[line.index[0]].line.xy2[0])

    @staticmethod
    @nb.njit(fastmath=True, cache=True)
    def _estimate_line(xy, affine_mat):
        tl = perspective_transform(xy, affine_mat).ravel()
        # print(xy)
        # print(tl)
        # print("===")
        # tl[0] = int(tl[0]) if np.abs(tl[0] - xy[0]) >= 0.1 else xy[0]
        # tl[1] = int(tl[1]) if np.abs(tl[1] - xy[1]) >= 0.1 else xy[1]
        return round(tl[0]),round(tl[1])

@nb.njit(cache=True, inline='always')
def calc_area(xy,xy2):
    w, h = np.abs(xy2[0]-xy[0]), np.abs(xy2[1]-xy[1])
    if w <= 0 or h <= 0:
        return 0.
    return w * h
