import numpy as np

def makeFunc(r1,r2):
	return lambda x: r1 + (r2-r1)*x

def wrapInterp(val,interpArray):
	floor = int(np.floor(val))
	delta = val-floor
	return interpArray[floor](delta)

def interpaLazers(lazerData):
	#assume array goes 1-360
	dataPairs = zip(lazerData,lazerData[1:]+lazerData[:1])
	interp = [makeFunc(r1,r2) for r1,r2 in dataPairs]
	return lambda x: wrapInterp(x,interp)

def computeResidual(old, new, new_thetas):
	oldInterpFunc = interpaLazers(old)
	oldInterped = [oldInterpFunc(theta) for theta in new_thetas]	
	return abs(sum([o-n for o,n in zip(oldInterped,new)]))

if __name__ == "__main__":
	pass