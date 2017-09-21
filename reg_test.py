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


def ICP_2params(par1, par2, i0, N, name):
   DATA,SVG=mkdirs(name)
   pars=json.load(open("params.json"))
   pars["prefilter"]["ON"]=1
   pars[par1["name"][0]][par1["name"][1]][par1["name"][2]]=par1["value"]
   pars[par2["name"][0]][par2["name"][1]][par2["name"][2]]=par2["value"]
   pars["selector"]["planecut"]=1
   json.dump(pars, open("%s/params.json"%SVG,'w'))

   for i in range(i0,i0+N):
      print i 
      subprocess.call(["./cpp/build/cloudgen", DATA, SVG,str(i)])
   res=[]
   for i in range(i0+1,i0+N):
      s=subprocess.check_output(["./cpp/build/cloudreg", SVG, SVG, str(i0), str(i)])
      res.append(np.array(s.split("\n")[:-1], dtype=np.float))
   res=np.array(res)   
   np.savetxt("."+name+"/ICPs_%s_%s_%s_%s.txt"%(par1["name"][2],par1["value"],par2["name"][2],par2["value"]), res)

      
t0=time.time()
name="/data/csl_feuillue/planecut"
planecut=1
N=6

#p1={"name": ["prefilter", "bifil", "sigR"], "value": 2}
#p2={"name": ["prefilter", "bifil", "sigR"], "value": 2}

for i in range(3):
   for j in range(3):
      p1={"name": ["prefilter", "SOR", "N"], "value": 10*(i+1)}
      p2={"name": ["prefilter", "SOR", "th"], "value": .5*(j+1)}
      ICP_2params(p1, p2, 0, N, name)

"""
for i in range(2): 
   subprocess.call(["./cpp/build/cloudgen", DATA, SVG,str(i)])
   if (i>0):    
      FNAME=SVG+"/0_%s.txt"%i
      with  open(FNAME,"w") as myfile: 
         subprocess.call(["./cpp/build/cloudreg", SVG, SVG, str(0), str(i)], stdout=myfile)
"""
print time.time()-t0
