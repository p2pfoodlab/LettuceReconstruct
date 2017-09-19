import pcd_utils as pu
import numpy as np
import sys

folder=sys.argv[1]
i=int(sys.argv[2])
print "plotting ", folder+"/pics/%03d_raw.png"%i

pcd=pu.getPC(folder+"/raw/%03d.pcd"%i)
pu.plot3d([pcd], ["k"], folder+"/pics/%03d_raw.png"%i)
pcd=pu.getPC(folder+"/filtered/%03d.pcd"%i)
pu.plot3d([pcd], ["k"], folder+"/pics/%03d_filtered.png"%i)

