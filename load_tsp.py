import os,sys,gzip,tarfile

#FILE DIRECTORIES HERE
tsp_dir='temp' # place to put .tsp.gz files
output_folder="tsp_files" # place to put the _depots.tsp and the _targets.tsp

# number of depots to use
num_depots=1

# max dimension to use, ignore any larger
max_dim=250

# unzips tar.gz as well, and goes through all subdirs
recursive=True 


def targz(file,out_dir):
    tar = tarfile.open(file, "r:gz")
    tar.extractall(path=out_dir)
    tar.close()

def decompress(infile, tofile):
    with open(infile, 'rb') as inf, open(tofile, 'w', encoding='utf8') as tof:
        decom_str = gzip.decompress(inf.read()).decode('utf-8')
        tof.write(decom_str)

def unzip_and_find_tsp(directory,recursive=True):
    # returns the .tsp filenames generated, as well as any .tsp files found
    for fn in os.listdir(directory):
        full=os.path.join(directory,fn)
        if full.endswith('.tsp'):
            yield full
        elif full.endswith('.tsp.gz'):
            decompress(full,full[:-3])
            yield full[:-3]
        elif fn.endswith('.opt.tour.gz'):
            full=os.path.join(directory,fn)
            decompress(full,full[:-3])
        elif fn.endswith('.tar.gz'):
            result_dir=os.path.join(directory,fn[:-7])
            targz(os.path.join(directory,fn),out_dir=result_dir)
            if recursive:
                for thing in unzip_and_find_tsp(result_dir,recursive=recursive):
                    yield thing
        elif os.path.isdir(full):
            if recursive:
                for thing in unzip_and_find_tsp(full,recursive=recursive):
                    yield thing


DIR = os.path.dirname(os.path.join(os.getcwd(), os.path.dirname(sys.argv[0])))



SOURCE_DIR=os.path.join(DIR,tsp_dir)
OUT_DIR=os.path.join(DIR,output_folder)

#making ouput folder if not existing
if not os.path.exists(OUT_DIR):
    os.makedirs(OUT_DIR)


def generate_target_depots_from_file(tsp_file,out_dir=OUT_DIR,num_depots=1,max_dim=float('inf')):
    with open(tsp_file) as f:
        name=os.path.basename(tsp_file)[:-4]
        target_fn=os.path.join(out_dir,name+"_targets.tsp")
        depots_fn=os.path.join(out_dir,name+"_depots.tsp")
        target_file=open(target_fn,'w')
        depots_file=open(depots_fn,'w')

        lines=f.read().split('\n')
        line_num=0
        dimension=-1
        NODE_COORD=False
        for line_num,line in enumerate(lines):
            # header part
            if line.startswith("DIMENSION"):
                # need to decrease target file dimension by num_depots, 
                # and set depot dimension to num depots
                dimension=int(line[line.index(':')+1:].strip())
                target_file.write("DIMENSION: "+str(dimension-num_depots))
                target_file.write('\n')
                depots_file.write("DIMENSION: "+str(num_depots))
                depots_file.write('\n')

            elif line.startswith("NODE_COORD_SECTION"):
                # out of header part
                target_file.write(line+'\n')
                depots_file.write(line+'\n')
                NODE_COORD=True
                break
            else:
                target_file.write(line+'\n')
                depots_file.write(line+'\n')
        if not NODE_COORD or dimension<0 or dimension>max_dim:
            print("file not useful: "+tsp_file)
            target_file.close()
            depots_file.close()
            os.remove(target_fn)
            os.remove(depots_fn)
            return
        line_num+=1
        # line_num now has line of first node

        for i in range(line_num,line_num+dimension-num_depots):
            target_file.write(lines[i]+'\n')
        target_file.write("EOF\n")
        target_file.close()

        for i in range(line_num+dimension-num_depots,line_num+dimension):
            depot_line=lines[i]
            depots_file.write(depot_line+'\n')
        depots_file.write("EOF\n")
        depots_file.close()

for tsp_file in unzip_and_find_tsp(SOURCE_DIR,recursive=recursive):
    generate_target_depots_from_file(tsp_file=tsp_file,
                                     out_dir=OUT_DIR,
                                     num_depots=num_depots,
                                     max_dim=max_dim)


