import numpy as np
import numba as nb


# @nb.njit(fastmath=True, cache=True)
def tilesToimg(tile, detections):
    # normW, normH = tile[0]/img_sz[0], tile[1]/img_sz[1]
    detections['xmin'] = detections['xmin'] + tile[0]
    detections['ymin'] = detections['ymin'] + tile[1]
    detections['xmax'] = detections['xmax'] + tile[0]
    detections['ymax'] = detections['ymax'] + tile[1]
    return detections

@nb.njit(fastmath=True, cache=True)
def start_points(size, split_size, overlap=0):
    points = [0]
    stride = int(split_size * (1-overlap))
    counter = 1
    while True:
        pt = stride * counter
        if pt + split_size >= size:
            points.append(size - split_size)
            break
        else:
            points.append(pt)
        counter += 1
    return points

@nb.njit(fastmath=True, cache=True)
def split_image(img, split_size, overlap=0):
    img_size = img.shape[:2]

    X_points = start_points(img_size[1], split_size[0], overlap)
    Y_points = start_points(img_size[0], split_size[1], overlap)
    tiles = []
    tile_pts = []
    for n in range(0, len(Y_points)):
        for z in range(0,len(X_points)):
            i = Y_points[n]
            j = X_points[z]
            split = img[i:i+split_size[1], j:j+split_size[0]]
            x, y, w, h = j, i, split_size[0], split_size[1]
            tiles.append(split)
            tile_pts.append((x,y,w,h))
    return tiles,tile_pts
    # return np.array(tiles), np.array(list(zip(X_points,Y_points)))
