import os,sys


posDirectory = sys.argv[1] #folder with pos-samples
negDirectory = sys.argv[2] #folder with neg-samples

#pos-samples

f = open('info.txt','a') #open textfile
for img in os.listdir(posDirectory): #iterate through files in directory
	filename = os.fsdecode(img)
	if filename.endswith(".png"): #check if file is .png 

		line = 'pos'+'/'+filename+' 1 0 0 50 50\n' #number of objects,startX, startY, width, height
		f.write(line) #write in file
f.close()

#same with neg-samples

f = open('bg.txt','a')
for img in os.listdir(negDirectory):
	filename = os.fsdecode(img)
	if filename.endswith(".png"): 

		line = 'neg'+'/'+filename+'\n' #no specific coordinates
		f.write(line)
f.close()