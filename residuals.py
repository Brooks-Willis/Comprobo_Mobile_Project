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

def cartesian(r,theta):
	return (r * np.cos(np.radians(theta)), r * np.sin(np.radians(theta)))

def rotate_cart(x,y,cost,sint):
	return (cost*x+sint*y, -1*sint*x+cost*y)

def translate(new, dx, dy, dtheta):
	'''takes a rotation/translation to perform, not the rotation we think the robot made'''
	cart = [cartesian(r,theta) for theta, r in enumerate(new)]
	translation = [(x + dx, y + dy) for x, y in cart]
	st = np.sin(np.radians(dtheta))
	ct = np.cos(np.radians(dtheta))
	rotation = [rotate_cart(x,y,ct,st) for x,y in translation]
	rs = [np.sqrt(x**2+y**2) for x,y in rotation]
	thetas = [np.degrees(np.arctan2(y,x)) for x,y in rotation]
	thetas = [360+t if t<0 else t for t in thetas]
	return rs, thetas

if __name__ == "__main__":
	pass