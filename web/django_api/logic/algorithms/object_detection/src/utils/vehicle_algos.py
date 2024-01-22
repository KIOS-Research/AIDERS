import math
import numpy as np
from .rect import get_center, to_tlwh
import numba as nb


def get_directions(frame_id, bboxes):
    direction = ""
    # for pts in box:
    # loop over the set of tracked points
    nsize = min(25,len(bboxes))
    bbn  = to_tlwh(bboxes[-nsize])
    bblb = to_tlwh(bboxes[-1])

    if frame_id >= nsize :#and bboxes[-nsize] is not None:  # and i == 1
        # compute the difference between the x and y
        # coordinates and re-initialize the direction
        # text variables
        dX = bbn[0] - bblb[0] #bboxes[-nsize][0] - bboxes[lb][0]
        dY = bbn[1] - bblb[1] # bboxes[-nsize][1] - bboxes[lb][1]
        (dirX, dirY) = ("", "")
        # ensure there is significant movement in the
        # x-direction
        if np.abs(dX) > int(bblb[2]/ 2):
            dirX = "Left" if np.sign(dX) == 1 else "Right"
        # ensure there is significant movement in the
        # y-direction
        if np.abs(dY) > int(bblb[3] / 2):
            dirY = "Up" if np.sign(dY) == 1 else "Down"
        # handle when both directions are non-empty
        if dirX != "" and dirY != "":
            direction = "{}-{}".format(dirY, dirX)
        # otherwise, only one direction is non-empty
        else:
            direction = dirX if dirX != "" else dirY
    return direction

@nb.njit(cache=True)
def calc_ppkm(im_size,live_altitude,altitude, horizontal_fov, vertical_fov):
    imWidth, imHeight = im_size
    flight_alt = altitude if live_altitude == 0 else live_altitude
    #fov = 2(atan(imdistm/(2*alt))
    #imdistm = tan(fov/2)*2alt
    ppkmX = (imWidth / (
            2 * math.tan(math.radians(horizontal_fov / 2)) * flight_alt)) * 1000  # tan(W/2)=X/height
    ppkmY = (imHeight / (
            2 * math.tan(math.radians(vertical_fov / 2)) * flight_alt)) * 1000  # pixels per kilometer
    return ppkmX,ppkmY

# @nb.njit(fastmath=True)
def calc_velocity(bboxes,stream_fps,frame_ids,im_size, ppkm):
    # if len(bboxes)<stream_fps:
    #     return 0,0,0
    limit = min(int(stream_fps),len(frame_ids)) # 30 usually is the frame rate so it corresponds to 1 second
    # print('lengths',len(frame_ids),len(bboxes))
    frame_rate = stream_fps*3600 # convert frame rate to hours
    frame_diff = frame_ids[-1] - frame_ids[-limit]
    # print('frame',frame_diff,frame_ids[-1],frame_ids[-limit],limit)

    imWidth,imHeight = im_size

    ppkmX,ppkmY = ppkm
    # km_pixels = 1 / (math.sqrt(pow(ppkmX, 2.0) + pow(ppkmY, 2.0)) / 1000)  # meters per pixel
    np_boxes = np.reshape(list(bboxes),(len(bboxes),4))
    Vs, Vxs, Vys = calc_gsd(ppkmX, ppkmY, np_boxes,limit)
    sumV, sumVx, sumVy = np.sum(Vs), np.sum(Vxs), np.sum(Vys)
    velo  = (sumV ) / (frame_diff/frame_rate) # total distance divided by the time needed to reach it
    veloX = (sumVx) / (frame_diff/frame_rate)
    veloY = (sumVy) / (frame_diff/frame_rate)

    #Set velocity as 0 if it is below 6  and TODO below 7px distance
    if velo <= 6 :#and euc_dist<15 :#or self.cars[index].parked[-1].parked:
        velo = 0.0
    if veloX <= 6:  # and euc_dist<15 :#or self.cars[index].parked[-1].parked:
        veloX = 0.0
    if veloY <= 6:  # and euc_dist<15 :#or self.cars[index].parked[-1].parked:
        veloY = 0.0
    return velo, veloX, veloY

@nb.njit(fastmath=True)
def calc_acceleration(velo1,velo2,acceleration,stream_fps):
    frame_rate = stream_fps # frames per second

    v1,  v1X,  v1Y, fid1 = velo1 # -1 index velocity
    v2,  v2X,  v2Y, fid2 = velo2 # -2 index velocity
    acc, accX, accY = acceleration # latest acceleration

    dV  = (v1  - v2 ) * 0.277777778 # km/h to m/s
    dVX = (v1X - v2X) * 0.277777778 # km/h to m/s
    dVY = (v1Y - v2Y) * 0.277777778 # km/h to m/s
    dT = (fid1 - fid2) / frame_rate

    da  = dV  / dT
    daX = dVX / dT
    daY = dVY / dT

    a  =  (acc  + da ) / 2
    aX =  (accX + daX) / 2
    aY =  (accY + daY) / 2

    return a, aX, aY


@nb.njit(fastmath=True, cache=True)
def calc_gsd( ppmX, ppmY, bboxes, limit):
    Vz, Vx, Vy = [], [] ,[]
    # out = np.empty((len(bboxes),1),dtype=np.float32)
    lenB = len(bboxes)
    # print('length',lenB-limit,lenB-1)
    count=0
    for n in range(lenB-limit,lenB-1):
        #tl br
        x1,y1 = (bboxes[n][0] + bboxes[n][2])/2 , (bboxes[n][1] + bboxes[n][3])/2  #get_center(bboxes[n])
        x2,y2 = (bboxes[n+1][0] + bboxes[n+1][2])/2 , (bboxes[n+1][1] + bboxes[n+1][3])/2 # get_center(bboxes[n+1])
        Vz.append(math.sqrt(pow(abs(x2 - x1) / ppmX, 2.0) + pow(abs(y2 - y1) / ppmY, 2.0)))
        Vx.append((x2-x1)/ppmX)
        Vy.append((y2-y1)/ppmY)
        count+=1
    # print('count',count)
    return  Vz,Vx,Vy #np.sum(result,axis=0)

def eucl_dist(self,carbox1, carbox2):
    result = []
    for n in range(0,len(carbox1)):
        x1,y1 = get_center(carbox1[n])
        x2,y2 = get_center(carbox2[n])
        result = math.sqrt(pow(abs(x2 - x1), 2.0) + pow(abs(y2 - y1), 2.0))
    return np.mean(result)
