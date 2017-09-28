import pcd_utils as pu
import numpy as np
import matplotlib.pyplot as pl
import sys
from os import listdir
from os.path import isfile, join


data="all"
filt="filtered"

#pcd_folder="/home/kodda/Dropbox/p2pflab/LettuceReconstruct/data/%s"%data
pcd_folder="/home/kodda/Dropbox/p2pflab/LettuceReconstruct/data/%s"%data

#ICP=np.loadtxt(join(pcd_folder,"ICPs_steps_0.txt") )

i=0
j=(i+1)%120

source=pu.getPC(pcd_folder+"/"+filt+"/%03d.pcd"%i)[:,:3]
target=pu.getPC(pcd_folder+"/"+filt+"/%03d.pcd"%j)[:,:3]
s2t_pcl=pu.getPC(pcd_folder+"/transform/%03d_%03d.pcd"%(i,j))[:,:3]

tr=np.fromfile(pcd_folder+'/transform/%03d_%03d.transform'%(i,j),dtype=np.float32).reshape([4,4])
iTr=pu.invTransf(tr)
s2t_py= pu.transf(tr, source)
t2s_py= pu.transf(iTr, target)

#pu.plot3d([source, target, s2t_py], ["g","r","k"])
pu.plot3d([source, target, t2s_py], ["g","r","k"])
#pu.plot3d([source, target, s2t_pcl], ["g","r","k"])
