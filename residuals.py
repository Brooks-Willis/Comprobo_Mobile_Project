import numpy as np

def makeFunc(r1,r2):
	return lambda x: r1 + (r2-r1)*x

def wrapInterp(val,interpArray):
	floor = int(np.floor(val))
	delta = val-floor
	return interpArray[floor](delta)

def interpolateLazers(lazerData):
	#assume array goes 1-360
	dataPairs = zip(lazerData,lazerData[1:]+lazerData[:1])
	interp = [makeFunc(r1,r2) for r1,r2 in dataPairs]
	return lambda x: wrapInterp(x,interp)

if __name__ == "__main__":
	pass