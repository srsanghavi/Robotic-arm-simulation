import sys
import subprocess
import time
import csv

runTime31 = []
for i in range(5):
     start = time.time()
     returncode = subprocess.call("python cs5335_puma.py BWG cs5335_33.env.xml ", shell=True)
     end = time.time()
     runTime31.append(end-start)

runTime41 = []
for i in range(5):
     start = time.time()
     returncode = subprocess.call("python cs5335_puma.py BRWG cs5335_43.env.xml", shell=True)
     end = time.time()
     runTime41.append(end-start)

runTime51 = []
for i in range(5):
     start = time.time()
     returncode = subprocess.call("python cs5335_puma.py BRWAG cs5335_53.env.xml", shell=True)
     end = time.time()
     runTime51.append(end-start)

res = [runTime31,runTime41,runTime51]

csvfile = "time3.csv"
#Assuming res is a list of lists
with open(csvfile, "w") as output:
    writer = csv.writer(output, lineterminator='\n')
    writer.writerows(res)