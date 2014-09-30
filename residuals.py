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

def translate(new, dx, dy, dtheta):
	cartesian = [(r * np.cos(np.radians(theta)), r * np.sin(np.radians(theta))) for theta, r in enumerate(new)]
	translation = [(x - dx, y - dy) for x, y in cartesian]
	st = np.sin(np.radians(-1*dtheta))
	ct = np.cos(np.radians(-1*dtheta))
	rotation = [(ct*x+st*y, -1*st*x+ct*y) for x,y in translation]
	temp = [(np.sqrt(x**2+y**2), np.degrees(np.arctan(x/y))) for x,y in rotation]
	rs, thetas = zip(*temp)
	return rs, thetas

if __name__ == "__main__":
	pass