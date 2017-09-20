import subprocess
import os
import json
import time

t0=time.time()

DATA="/home/kodda/Dropbox/p2pflab/data/2017/17.09.11/feuillue_z1b"

curdir=os.getcwd()
SVG=curdir+"/data/csl_feuillue/test"
if not os.path.exists(SVG+"/raw/"): os.makedirs(SVG+"/raw/")
if not os.path.exists(SVG+"/filtered/"): os.makedirs(SVG+ "/filtered/")
if not os.path.exists(SVG+"/pics/"): os.makedirs(SVG+ "/pics/")
if not os.path.exists(SVG+"/transform/"): os.makedirs(SVG+"/transform/")

pars=json.load(open("params.json"))
#pars[""]
json.dump(pars, open("%s/params.json"%SVG,'w'))

for i in range(2): 
   subprocess.call(["./cpp/build/cloudgen", DATA, SVG,str(i)])
   if (i>0):    
      FNAME=SVG+"/0_%s.txt"%i
      with  open(FNAME,"w") as myfile: 
         subprocess.call(["./cpp/build/cloudreg", SVG, SVG, str(0), str(i)], stdout=myfile)

print time.time()-t0
