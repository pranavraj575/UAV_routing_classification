import os, sys, numpy as np
import matplotlib.image
import itertools

# random seed
# so for each file, we generate the same map every time
seed=69


PRINTIMG = False  # whether to save image
PROPORTION_CUTOFF = 5e-5  # proportion of pixels needed to register a building
map_gen_dir = os.path.dirname(os.path.join(os.getcwd(), sys.argv[0]))
DIR = os.path.dirname(map_gen_dir)

# proportion of buildings to make depots (randomly chooses a proportion in the range)
depot_proportion_range=(.05,.2)


# (height, width of image in meters), name of file
# obtained from https://movingai.com/benchmarks/street/index.html
to_run = (
            ((511.8, 511.8), os.path.join(map_gen_dir, 'maps', 'NewYork_0_1024.map')),
            ((511.6, 511.6), os.path.join(map_gen_dir, 'maps', 'Berlin_0_1024.map')),
          )

# to_run = (((511.6, 511.6), os.path.join(map_gen_dir, 'maps', 'street-map', filename)) for filename in
#          os.listdir(os.path.join(map_gen_dir, 'maps', 'street-map')))

image_folder = os.path.join(DIR, 'output', 'plots', 'generated_maps')
output_folder = os.path.join(DIR, "output", "data_files", "generated_maps")
for d in (image_folder, output_folder):
    if not os.path.exists(d):
        os.makedirs(d)

def random_split(points,random_range=depot_proportion_range):
    # splits into (targets, depots) and returns those
    # random range is the proportion of depots
    points=points.copy()
    np.random.shuffle(points)
    p=random_range[0]+np.random.random()*(random_range[1]-random_range[0])
    idx=int(len(points)*p)+1
    print(p)
    return points[idx:], points[:idx]

def create_tsp_files(targets,depots,folder,basename,decimals=4):
    for points,depot in ((targets,False),(depots,True)):
        instance_name=basename+('_depots' if depot else '_targets')
        tring=("NAME : "+instance_name+
        "\nCOMMENT : Instance 1 in MD algorithm. Coordinates of depots and vhcl speeds given."+
        "\nTYPE : TSP"+
        "\nDIMENSION : "+str(len(points))+
        "\nEDGE_WEIGHT_TYPE : ATT"+
        "\nNODE_COORD_SECTION"
        )
        for (i,point) in enumerate(points):
            target_no=i+(len(targets) if depot else 0)+1
            tring+='\n'+' '.join([
                                    str(target_no),
                                    str(round(point[0],decimals)), 
                                    str(round(point[1],decimals)),
                                    '1' if depot else '_'
                                ])
        tring+='\nEOF'

        filename=os.path.join(folder,instance_name+'.tsp')
        f=open(filename,'w')
        f.write(tring)
        f.close()

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


for dims, filepath in to_run:
    np.random.seed(seed)
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
                             filename=os.path.join(image_folder,
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
        save_name = os.path.join(image_folder, filename[:filename.index('.')] + '.png')
        print('saving img to', save_name)
        save_img(arr=colorful_buildings, filename=save_name)

    euclidean_centers = []
    for i, j in building_centers:
        coords = np.array(dims)*(i/N, j/M) - np.array(dims)/2
        euclidean_centers.append(coords)

    # make sure the split/targets chosen is the same every time
    np.random.seed(seed)
    euclidean_centers = np.array(euclidean_centers)
    targets,depots=random_split(euclidean_centers)
    create_tsp_files(targets,depots,output_folder,filename[:filename.index('.')])

    print('number of buildings:', len(euclidean_centers))
    print('saving points at', os.path.join(output_folder,filename[:filename.index('.')]))
