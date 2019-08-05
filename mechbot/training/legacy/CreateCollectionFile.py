import os
import sys


posDirectory = sys.argv[1] #folder with pos-samples
negDirectory = sys.argv[2] #folder with neg-samples

#pos-samples
with open('info.txt','a') as f: #open textfile
	for img in os.listdir(posDirectory): #iterate through files in directory
		filename = os.fsdecode(img)
		if not filename.endswith(".png"): #make sure file is .png 
			continue

		#number of objects, startX, startY, width, height
		line = posDirectory+'/'+filename+'  1  5 5 45 45\n'
		f.write(line)

#same with neg-samples
with open('bg.txt','a') as f:
	for img in os.listdir(negDirectory):
		filename = os.fsdecode(img)
		if not filename.endswith(".png"): 
			continue

		line = negDirectory+'/'+filename+'\n' #no specific coordinates
		f.write(line)
