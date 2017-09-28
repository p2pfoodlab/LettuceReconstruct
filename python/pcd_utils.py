import pypcd as pcd
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as pl

"""
def getPC(filename):
   ps=pcd.point_cloud_from_path(filename).pc_data
   pc=np.array([[p[0],p[1],p[2],p[3]] for p in ps if not(np.isnan(p[0]))])
   col=pc[:,3]#.astype(np.int)

   b=col & 255
   g=(col>>8) & 255
   r=(col>>16) & 255
   col=np.array([r,g,b]).T
   return np.hstack([pc[:,:3],col])
"""

def getPC(filename):
   ps=pcd.point_cloud_from_path(filename).pc_data
   idx=np.where(((np.isnan(ps["x"])==False)*ps["rgb"]))
   pc=np.hstack([np.array([ps[idx]["x"],ps[idx]["y"],ps[idx]["z"]]).T, ps[idx]["rgb"].view((np.uint8,4))[:,:3]])
   return pc

def plot3d(pcds, colors=["k"], svg=None):
   fig = pl.figure()
   ax = fig.add_subplot(111, projection='3d')

   for i,p in enumerate(pcds):
      if colors:
         ax.scatter(p[:,0], p[:,1], p[:,2], c=colors[i],s=.1,alpha=.2,depthshade=False)
      else: ax.scatter(p[:,0], p[:,1], p[:,2], c=p[:,3:][:,::-1]/255.,s=.1,alpha=.2,depthshade=False)

   ax.set_xlim([-250,250])
   ax.set_ylim([-250,250])
   ax.set_zlim([0,400])
   ax.view_init(-80,50)
   if svg: pl.savefig(svg, bbox_inches="tight")
   else: pl.show()

def rotation_from_matrix(matrix):
    R = np.array(matrix, dtype=np.float64, copy=False)
    R33 = R[:3, :3]
    # direction: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, W = np.linalg.eig(R33.T)
    i = np.where(abs(np.real(l) - 1.0) < 1e-5)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    direction = np.real(W[:, i[-1]]).squeeze()
    # point: unit eigenvector of R33 corresponding to eigenvalue of 1
    l, Q = np.linalg.eig(R)
    i = np.where(abs(np.real(l) - 1.0) < 1e-5)[0]
    if not len(i):
        raise ValueError("no unit eigenvector corresponding to eigenvalue 1")
    point = np.real(Q[:, i[-1]]).squeeze()
    point /= point[3]
    # rotation angle depending on direction
    cosa = (np.trace(R33) - 1.0) / 2.0
    if abs(direction[2]) > 1e-5:
        sina = (R[1, 0] + (cosa-1.0)*direction[0]*direction[1]) / direction[2]
    elif abs(direction[1]) > 1e-5:
        sina = (R[0, 2] + (cosa-1.0)*direction[0]*direction[2]) / direction[1]
    else:
        sina = (R[2, 1] + (cosa-1.0)*direction[1]*direction[2]) / direction[0]
    angle = math.atan2(sina, cosa)
    return angle, direction, point

def get_hs(fname,N=100,s=1,saveit=False):
   dirs=[]
   angles=[]
   ps=[]
   Hs=[]
   for k in range(s,N): 
      H=np.fromfile(fname+'%s_%s.transform'%(k-s,k),dtype=np.float32).reshape([4,4])
      angle, direction, point=rotation_from_matrix(H)
      angles.append(angle)
      dirs.append(direction)
      ps.append(point)
      Hs.append(H)
   if saveit: np.save(saveit,H)
   return np.array(angles), np.array(dirs), np.array(ps), np.array(Hs)

def transf(tr,x):
   return np.dot(tr[:3,:3],x.T).T+tr[:3,3]

def invTransf(tr):
   iTr=np.zeros([4,4])
   iTr[:3,:3]=tr[:3,:3].T   
   iTr[:3,3]=-np.dot(iTr[:3,:3],tr[:3,3])   
   return iTr
