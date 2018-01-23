''' Rotation demo with matrices and quaternions'''
'''  by David Conner CPSC 495 Spring 2016 '''

import numpy as np 


# Rotation about x axis
def rotX(angle):
	m = np.array([ [	1.0, 		0.0	  ,       0.0     ],
				   [	0.0, np.cos(angle), -np.sin(angle)],
				   [	0.0, np.sin(angle),  np.cos(angle)]]);
	return m;


# Rotation about y axis
def rotY(angle):
	m = np.array([ [ np.cos(angle), 0.0, np.sin(angle)],
				   [   0.0		  , 1.0, 	0.0		  ],
				   [-np.sin(angle), 0.0, np.cos(angle)]]);
	return m;

# Rotation about z axis
# (also functions as homogeneous transform for pure 2D rotation)
def rotZ(angle):
	m = np.array([ [np.cos(angle), -np.sin(angle), 0.0],
				   [np.sin(angle),  np.cos(angle), 0.0],
				   [    0.0		 , 		0.0		 , 1.0]]);
	return m;

# create homogeneous transform with x,y,z displacements
def translate3D(dx,dy,dz):
	T = np.array([ [    1.0      ,   	0.0	 , 0.0, dx ],
				   [	0.0		 ,  	1.0  , 0.0, dy ],
				   [    0.0		 , 		0.0	 , 1.0, dz ],
				   [	0.0		 ,		0.0	 , 0.0, 1.0]]);
	return T;

# convert x,y displacements and rotation matrix into homogeneous transform
def transform2D(dx,dy,R):
	T = np.array([ [    R[0,0] ,  R[0,1] ,  dx ],
				   [	R[1,0] ,  R[1,1] ,  dy ],
				   [	0.0	   ,	0.0	 , 1.0 ]]);
	return T;

# convert x,y,z displacements and rotation matrix into homogeneous transform
def transform3D(dx,dy,dz,R):
	T = np.array([ [    R[0,0] ,  R[0,1] , R[0,2],  dx ],
				   [	R[1,0] ,  R[1,1] , R[1,2],  dy ],
				   [    R[2,0] ,  R[2,1] , R[2,2],  dz ],
				   [	0.0	   ,	0.0	 ,   0.0 , 1.0 ]]);
	return T;

# convert translation and quaternion into homogeneous transform
def Q2trans(t,q):
	R = Q2M(q)
	T = np.array([ [    R[0,0] ,  R[0,1] , R[0,2],  t[0] ],
				   [	R[1,0] ,  R[1,1] , R[1,2],  t[1] ],
				   [    R[2,0] ,  R[2,1] , R[2,2],  t[2] ],
				   [	0.0	   ,	0.0	 ,   0.0 , 1.0 ]]);
	return T;

# convert homogeneous transform to translation and quaternion pair
def trans2Q(T):
	R = T[0:3,0:3]
	t = T[0:3,3];
	q = M2Q(R)

	return (t,q);




# convert rotation matrix to normalized quaternion
def M2Q(rot):
	m00 = rot[0,0];
	m01 = rot[0,1]
	m02 = rot[0,2]
	m10 = rot[1,0];
	m11 = rot[1,1]
	m12 = rot[1,2]
	m20 = rot[2,0];
	m21 = rot[2,1]
	m22 = rot[2,2]

	qw = np.sqrt(1 + m00 + m11 + m22)/2.0;
	qx = (m21 - m12)/(4*qw)
	qy = (m02 - m20)/(4*qw)
	qz = (m10 - m01)/(4*qw)

	return np.array([qw, qx, qy, qz])

# Convert normalized quaternion to rotation matrix
def Q2M(quat):
	qw = quat[0]
	qx = quat[1]
	qy = quat[2]
	qz = quat[3]

	m00 = 1-2*qy*qy - 2*qz*qz
	m01 = 2.0*(qx*qy - qz*qw)
	m02 = 2.0*(qx*qz + qy*qw)

	m10 = 2.0*(qx*qy + qz*qw)
	m11 = 1-2*qx*qx - 2*qz*qz  
	m12 = 2.0*(qy*qz - qx*qw)

	m20 = 2.0*(qx*qz - qy*qw)
	m21 = 2.0*(qy*qz + qx*qw)
	m22 =  1-2*qx*qx - 2*qy*qy  

	return np.array([[ m00, m01, m02], [ m10, m11, m12], [ m20, m21, m22] ])

# conjugate of quaternion
def ConjQ(q):
	return np.array([q[0], -q[1], -q[2], -q[3]])

# Multipy 2 quaternions
def Q1xQ2(p,q):
	pw = p[0]
	px = p[1]
	py = p[2]
	pz = p[3]

	qw = q[0]
	qx = q[1]
	qy = q[2]
	qz = q[3]

	pqw = pw*qw - px*qx - py*qy - pz*qz;
	pqx = pw*qx + px*qw + py*qz - pz*qy;
	pqy = pw*qy + py*qw - px*qz + pz*qx;
	pqz = pw*qz + pz*qw + px*qy - py*qx;

	return np.array([pqw, pqx, pqy, pqz])

# Get rid of very small numerical dust before output 
def NoDust(m):
	if (len(m.shape) > 1):
	  # Handle 2D matrices
	  for i in range(0,m.shape[0]):
		for j in range(0,m.shape[1]):
			if (np.fabs(m[i,j]) < 1e-14):
				m[i,j] = 0.0
	else:
		# Handle row or column vectors
		for j in range(0,m.shape[0]):
			if (np.fabs(m[j]) < 1e-14):
				m[j] = 0.0

	return m

