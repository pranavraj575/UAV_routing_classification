# called by target_number_experiment.jl
# plots result of how vehicle objective value varies based on targets
import os,sys,ast
import numpy as np
import matplotlib.pyplot as plt
plt.rc('font', size=14) 
#plt.rc('xtick', labelsize=10.5)
input_file=sys.argv[1]
output_file=sys.argv[2]
cleanup=sys.argv[3]=='true'


f=open(input_file)
listt=ast.literal_eval(f.read())
f.close()

plt.plot(listt)

plt.xlabel("Number of Targets")
plt.ylabel("Vehicle Objective Value")

plt.savefig(output_file)
if cleanup:
    os.remove(input_file)
