import numpy as np

def makeFunc(r1,r2):
	return lambda x: r1 + (r2-r1)*x

def wrapInterp(val,interpArray):
	floor = int(np.floor(val))
	delta = val-floor
	return interpArray[np.mod(floor,len(interpArray))](delta)

def interpaLazers(lazerData):
	#assume array goes 1-360
	dataPairs = zip(lazerData,lazerData[1:]+lazerData[:1])
	interp = [makeFunc(r1,r2) for r1,r2 in dataPairs]
	return lambda x: wrapInterp(x,interp)

def computeResidual(old, new, new_thetas):
	oldInterpFunc = interpaLazers(old)
	oldInterped = [oldInterpFunc(theta) for theta in new_thetas]	
	return sum([o-n for o,n in zip(oldInterped,new)])**2

def cartesian(r,theta):
	return (r * np.cos(np.radians(theta)), r * np.sin(np.radians(theta)))

def rotate_cart(x,y,cost,sint):
	return (cost*x-sint*y, sint*x+cost*y)

def translate(rs_in, dx, dy, dtheta):
	'''rotates and translates the WORLD, not the robot'''
	rs_in = [r+1e-6 if r==0 else r for r in rs_in]
	cart = [cartesian(r,theta) for theta, r in enumerate(rs_in)]
	translation = [(x + dx, y + dy) for x, y in cart]
	st = np.sin(np.radians(dtheta))
	ct = np.cos(np.radians(dtheta))
	rotation = [rotate_cart(x,y,ct,st) for x,y in translation]
	rs = [np.sqrt(x**2+y**2) for x,y in rotation]
	thetas = [np.degrees(np.arctan2(y,x)) for x,y in rotation]
	thetas = [360+t if t<0 else t for t in thetas]
	return rs, thetas

def residual(old,new,dx,dy,dtheta):
	new_rs, new_thetas = translate(new,dx,dy,dtheta)
	print "\n\n"
	print new_rs
	print new_thetas
	return computeResidual(old, new_rs, new_thetas)

if __name__ == "__main__":
	pass