import pcd_utils as pu
import numpy as np
import matplotlib.pyplot as pl
import sys

data="planecut"
filt="raw"

pcd_folder="/home/kodda/Dropbox/p2pflab/LettuceScan/PCD_proc/data/csl_feuillue/%s"%data
icp_folder="/home/kodda/Dropbox/p2pflab/LettuceScan/PCD_proc/data/csl_feuillue/%s/ICP_%sDS"%(data,filt)

i=0
j=3

pcd1=pu.getPC(pcd_folder+"/"+filt+"/%03d.pcd"%i)
pcd2=pu.getPC(pcd_folder+"/"+filt+"/%03d.pcd"%j)
pcd3=pu.getPC(icp_folder+"/transform/%03d_%03d.pcd"%(i,j))
pu.plot3d([pcd1,pcd2,pcd3], ["g","r","k"])

