import os,sys,ast
import numpy as np
import matplotlib.pyplot as plt
plt.rc('font', size=14) 
#plt.rc('xtick', labelsize=10.5)
input_file="temp//targets_alpha_1.0_tau_20.0.txt"#sys.argv[1]
output_file="plots//targets_alpha_1.0_tau_20.0.png"#sys.argv[2]
cleanup=False#sys.argv[3]=='true'


f=open(input_file)
listt=ast.literal_eval(f.read())
f.close()

plt.plot(listt)

plt.xlabel("Number of Targets")
plt.ylabel("Vehicle Objective Value")

plt.savefig(output_file)
if cleanup:
    os.remove(input_file)
