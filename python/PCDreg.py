import pcd_utils as pu
import numpy as np
import matplotlib.pyplot as pl
import sys

def figReg(pcd_folder,icp_folder):
   i=0

   for j in range(1,11):
      pcd1=pu.getPC(pcd_folder+"/%03d.pcd"%i)
      pcd2=pu.getPC(pcd_folder+"/%03d.pcd"%j)
      pcd3=pu.getPC(icp_folder+"/transform/%03d_%03d.pcd"%(i,j))
      pu.plot3d([pcd1,pcd2,pcd3], ["g","r","k"], icp_folder+"/%03d.png"%j)
   pl.clf()
   ICPs=np.array([np.loadtxt(icp_folder+"/%s_%s.txt"%(i,j)) for j in range(1,11)])
   
   pl.plot(ICPs[:,:,1])
   pl.savefig(icp_folder+"/ICPS.png",bbox_inches="tight")
   np.savetxt(icp_folder+"/ICPs.txt", ICPs[:,:,1])

pcd_folder=sys.argv[1]
icp_folder=sys.argv[2]

figReg(pcd_folder,icp_folder)

