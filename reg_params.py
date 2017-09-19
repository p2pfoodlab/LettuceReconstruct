import subprocess
import os
import json
import time

t0=time.time()

DAY="17.09.11"
DATA="feuillue_z1b"

SVG="data/csl_feuillue/test"
if not os.path.exists(SVG+"/raw/"): os.makedirs(SVG+"/raw/")
if not os.path.exists(SVG+"/filtered/"): os.makedirs(SVG+ "/filtered/")
if not os.path.exists(SVG+"/pics/"): os.makedirs(SVG+ "/pics/")
if not os.path.exists(SVG+"/transform/"): os.makedirs(SVG+"/transform/")

pars=json.load(open("params.json"))
json.dump(pars, open("%s/params.json"%SVG,'w'))

for i in range(11): 
   subprocess.call(["/home/kodda/Dropbox/p2pflab/LettuceScan/PCD_proc/cpp/build/cloudgen", DAY,DATA,SVG,str(i)])
   if (i>0):    
      FNAME=SVG+"/0_%s.txt"%i
      with  open(FNAME,"w") as myfile: 
         subprocess.call(["/home/kodda/Dropbox/p2pflab/LettuceScan/PCD_proc/cpp/build/cloudreg", SVG, SVG, str(0), str(i)], stdout=myfile)

print time.time()-t0
