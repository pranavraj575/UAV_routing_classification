import os, sys, numpy as np
import matplotlib.image
import itertools


def blob_detection(ground_arr, mask_arr, seed, building_arr=None):
    if building_arr is None:
        building_arr = np.zeros_like(ground_arr, dtype=bool)
    n, m = ground_arr.shape
    fronteir = [seed]
    while fronteir:
        i, j = fronteir.pop()
        building_arr[i, j] = True
        # look at neighbors as the 3x3 grid around a pixel (the pixel iteself will be ignored)
        for neighbor in itertools.product(range(i - 1, i + 2), range(j - 1, j + 2)):
            ip, jp = neighbor
            if ip >= 0 and jp >= 0 and ip < n and jp < m:
                # if in bounds and there is an unvisited building in the index
                if ground_arr[ip, jp] and not mask_arr[ip, jp] and not building_arr[ip, jp]:
                    fronteir.append(neighbor)
    return building_arr


def building_detection(ground_arr, mask_arr=None):
    """
    detects buildings in ground arr array
    Args:
        ground_arr: 2D boolean array, 1 if building, 0 if no building
    Returns:
        list of 2D bool arrays that are the buildings
    """
    if mask_arr is None:
        mask_arr = np.zeros_like(ground_arr, dtype=bool)
    all_buildings = []

    while np.sum(np.logical_and(ground_arr, np.logical_not(mask_arr))):
        for i in range(len(ground_arr)):
            for j in range(len(ground_arr[i])):
                if ground_arr[i, j] and not mask_arr[i, j]:
                    building_arr = blob_detection(ground_arr=ground_arr, mask_arr=mask_arr, seed=(i, j))
                    all_buildings.append(building_arr)
                    mask_arr = np.logical_or(mask_arr, building_arr)
    return all_buildings


def save_img(arr, filename):
    if len(arr.shape) == 2:
        arr = arr.astype(np.uint8)*255
        arr = np.stack((arr, arr, arr), axis=2)

    matplotlib.image.imsave(
        filename,
        arr,
    )


PRINTIMG = True
PROPORTION_CUTOFF = 5e-5  # proportion of pixels needed to register a building
DIR = os.path.dirname(sys.argv[0])
temp_folder = os.path.join(DIR, 'temp')
if not os.path.exists(temp_folder):
    os.makedirs(temp_folder)

# (height, width of image in meters), name of file
# obtained from https://movingai.com/benchmarks/street/index.html
to_run = (((511.8, 511.8), os.path.join(DIR, 'maps', 'NewYork_0_1024.map')),
          ((511.6, 511.6), os.path.join(DIR, 'maps', 'Berlin_0_1024.map'))
          )

to_run = (((511.6, 511.6), os.path.join(DIR, 'maps', 'street-map', filename)) for filename in
          os.listdir(os.path.join(DIR, 'maps', 'street-map')))

for dims, filepath in to_run:
    filename = os.path.basename(filepath)
    print('running', filename)
    trash_lines = 4
    f = open(filepath)
    line = f.readline()
    arr = None
    while line:
        if trash_lines > 0:
            trash_lines -= 1
        else:
            yarr = np.array([c == '@' for c in line.strip()], dtype=bool).reshape((1, -1))
            if arr is None:
                arr = yarr
            else:
                arr = np.concatenate((arr, yarr), axis=0)
        line = f.readline()
    f.close()
    N, M = arr.shape
    SIZE_CUTOFF = PROPORTION_CUTOFF*M*N
    # building detection
    things = building_detection(ground_arr=arr)
    if PRINTIMG:
        colorful_buildings = arr.copy()
        colorful_buildings = np.stack((colorful_buildings, colorful_buildings, colorful_buildings), axis=2, dtype=float)
        excess = np.zeros_like(colorful_buildings)

    building_centers = dict()
    for k, thing in enumerate(things):
        if np.sum(thing) > SIZE_CUTOFF:
            stuff = [[[i, j] for j, item in enumerate(row) if item] for i, row in enumerate(thing)]
            all_stuff = []
            for r in stuff:
                all_stuff += r
            center = np.mean(np.array(all_stuff), axis=0)
            building_centers[tuple(center)] = thing
            if PRINTIMG:
                if False:
                    save_img(thing,
                             filename=os.path.join(temp_folder,
                                                   filepath[:filepath.index('.')] + 'building_' + str(
                                                       k) + '_' + '.png'))
                mask = np.stack((thing, thing, thing), axis=2, dtype=float)
                pix = np.random.rand(3).reshape((1, 1, 3))
                pix = pix/np.linalg.norm(pix)
                colorful_buildings = colorful_buildings*(1 - mask)

                colorful_buildings += mask*pix
        elif PRINTIMG:
            excess += np.expand_dims(thing, axis=2)
    if PRINTIMG:
        colorful_buildings = (colorful_buildings*128 + excess*127).astype(np.uint8)
        rad = .003*(N + M)/2
        for center in building_centers:
            for i in range(int(center[0] - rad*2), int(center[0] + rad*2)):
                for j in range(int(center[1] - rad*2), int(center[1] + rad*2)):
                    if i >= 0 and j >= 0 and i < len(colorful_buildings) and j < len(colorful_buildings[i]):
                        if np.linalg.norm(np.array(center) - (i, j)) < rad:
                            colorful_buildings[i, j, :] = (255, 0, 0)
        save_name = os.path.join(temp_folder, filename[:filename.index('.')] + '.png')
        print('saving img to', save_name)
        save_img(arr=colorful_buildings, filename=save_name)

    euclidean_centers = []
    for i, j in building_centers:
        coords = np.array(dims)*(i/N, j/M) - np.array(dims)/2
        euclidean_centers.append(coords)

    euclidean_centers = np.array(euclidean_centers)
    save_name = os.path.join(temp_folder, 'center_data_' + filename[:filename.index('.')] + '.txt')

    print(euclidean_centers)
    print('saving points to', save_name)
    np.savetxt(save_name, euclidean_centers)
