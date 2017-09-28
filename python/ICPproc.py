import numpy as np
import matplotlib.pyplot as pl
from os import listdir
from os.path import isfile, join

def fnametopars(f):
   fsplit=f.strip(".txt").split("_")
   return float(fsplit[2]), float(fsplit[4]) 
   

respath="../res/SOR_N_th/"
files=[f for f in listdir(respath) if isfile(join(respath,f))]

pars=np.array([fnametopars(f) for f in files])
pars=np.array(sorted(pars,key=lambda x: (x[0],x[1])))

ICPs=[]
N=len(files)
for i in range(N):
   p1,p2=pars[i]

   ICP=np.loadtxt(join(respath,"ICPs_N_%s_th_%s.txt"%(int(p1),p2)) )
   ICPs.append(ICP)
   Nstep=ICP.shape[1]
   
   for k in range(Nstep): pl.plot(ICP[:,k],color=pl.cm.spring(k*1./Nstep))
   pl.xlabel("Image id")
   pl.ylabel("ICP error")
   pl.title("N %s th %s"%(int(p1),p2))
   pl.savefig("../res/SOR_N_th/figs/N_%s_th_%s.png"%(p1,p2))
   pl.clf()
   
m0s=[np.zeros(N) for i in range(10)]
mfs=[np.zeros(N) for i in range(10)]
dfs=[np.zeros(N) for i in range(10)]

for i in range(N):
   for j in range(10):
      m0s[j][i]=ICPs[i][j,0]      
      mfs[j][i]=ICPs[i][j,-1]      
      dfs[j][i]=np.diff(ICPs[i][j,:])[1:].mean()      


for i in range(10):
   pl.subplot(131)
   pl.imshow(m0s[i].reshape([N/10,10]),origin="lowest")
   pl.subplot(132)
   pl.imshow(mfs[i].reshape([N/10,10]),origin="lowest")
   pl.subplot(133)
   #pl.imshow(m0s[i].reshape([6,10])-mfs[0].reshape([6,10]),origin="lowest")
   pl.imshow(dfs[i].reshape([N/10,10]),origin="lowest")
   pl.savefig("../res/SOR_N_th/figs/ICP_%s.png"%i,bbox_inches="tight")
   pl.clf()


for i in range(10):
   pl.subplot(1,10,i+1)
   pl.imshow(m0s[i].reshape([N/10,10]),origin="lowest")
   #pl.imshow(mfs[i].reshape([6,10]),origin="lowest")
pl.savefig("../res/SOR_N_th/figs/M0.png",bbox_inches="tight")
pl.clf()

for i in range(10):
   pl.subplot(1,10,i+1)
   pl.imshow(mfs[i].reshape([N/10,10]),origin="lowest")
pl.savefig("../res/SOR_N_th/figs/MF.png",bbox_inches="tight")
pl.clf()

for i in range(10):
   pl.subplot(1,10,i+1)
   pl.imshow(dfs[i].reshape([N/10,10]),origin="lowest")
pl.savefig("../res/SOR_N_th/figs/DF.png",bbox_inches="tight")
pl.clf()


"""
minICP=np.array([ICP.min() for ICP in ICPs])
maxICP=np.array([ICP.max() for ICP in ICPs])

minICP_last=np.array([ICP[-1].min() for ICP in ICPs]).reshape([6,10])
maxICP_last=np.array([ICP[-1].max() for ICP in ICPs]).reshape([6,10])


p1={"name": ["prefilter", "SOR", "N"], "value": 0}
p2={"name": ["prefilter", "SOR", "th"], "value": 0}

ICPs=[]
name="../data/csl_feuillue/planecut"

for i in range(3):
   for j in range(3):
      ICPs.append(np.loadtxt(name+"/ICPs_%s_%s_%s_%s.txt"%(p1["name"][2], 10*(i+1), p2["name"][2], .5*(j+1))))
"""      
