import subprocess
import os
import json
import time
import numpy as np

def mkdirs(name):
   DATA="/home/kodda/Dropbox/p2pflab/data/2017/17.09.11/feuillue_z1b"
   curdir=os.getcwd()
   SVG=curdir+name
   for f in ["/raw/", "/filtered/", "/transform/"]:
      if not os.path.exists(SVG+f): os.makedirs(SVG+f)
   return DATA, SVG

def gencloud(DATA, SVG, planecut=0,N=120):
   pars=json.load(open("params.json"))
   pars["prefilter"]["ON"]=1
   pars["selector"]["planecut"]=planecut
   
   json.dump(pars, open("%s/params.json"%SVG,'w'))
   for i in range(N):
      print i 
      subprocess.call(["./cpp/build/cloudgen", DATA, SVG,str(i)])


t0=time.time()
name="/data/csl_feuillue/planecut"
planecut=1
N=12
DATA,SVG=mkdirs(name)
#gencloud(DATA, SVG, planecut, N)

for k in range(10):
   pars=json.load(open("params.json"))
   pars["prefilter"]["ON"]=1
   pars["prefilter"]["bifil_sigR"]=5+2*k
   pars["selector"]["planecut"]=planecut
   json.dump(pars, open("%s/params.json"%SVG,'w'))
   for i in range(N):
      print i 
      subprocess.call(["./cpp/build/cloudgen", DATA, SVG,str(i)])
   res=[]
   i0=0
   K=12   
   for i in range(i0+1,i0+K):
      s=subprocess.check_output(["./cpp/build/cloudreg", SVG, SVG, str(i0), str(i)])
      res.append(np.array(s.split("\n")[:-1], dtype=np.float))
   np.savetxt("ICPs_%s.txt"%pars["prefilter"]["SOR"]["N"], np.array(res))


"""
for i in range(2): 
   subprocess.call(["./cpp/build/cloudgen", DATA, SVG,str(i)])
   if (i>0):    
      FNAME=SVG+"/0_%s.txt"%i
      with  open(FNAME,"w") as myfile: 
         subprocess.call(["./cpp/build/cloudreg", SVG, SVG, str(0), str(i)], stdout=myfile)
"""
print time.time()-t0
