''' Rotation demo with matrices and quaternions'''
'''  by David Conner CPSC 495 Spring 2016 '''

import numpy as np

from rotations import *

# Demo rotation conversions with matrices and quaternions
alpha = np.pi/4.0
beta  = np.pi/6.0;
theta = np.pi/3.0;
Rx = rotX(alpha);
Ry = rotY(beta);
Rz = rotZ(theta);

R= np.dot(Rx,np.dot(Ry,Rz)) # Rx*Ry*Rz = Rx*(Ry*Rz)

print "Rx(",alpha,") =",Rx
print "Ry(",beta, ") =",Ry
print "Rz(",theta,") =",Rz
print "R =",R

q = M2Q(R)
rot = Q2M(q)

qx = M2Q(Rx)
qy = M2Q(Ry)
qz = M2Q(Rz)
qq = Q1xQ2(qx,Q1xQ2(qy,qz))

print "Conversion to quat:"
print "q=",q
print "q=qx.qy.qz=",qq
print "qx =",qx
print "qy =",qy
print "qz =",qz


print "quat to matrix "
print "rot=",rot
print " inverse test:"

Ri = np.linalg.inv(rot)
eye=NoDust(np.dot(R,Ri))
print eye

print "Vector to rotate:"
v = np.array([1.,1.,1.])
vn = v/np.sqrt(np.dot(v,v))

print " vn=",vn

vr = np.dot(R,vn)
print " R*vn=",vr

print "Pure vector quaternion form"
qv = np.array([0.0, vn[0], vn[1], vn[2]])
print "qv=",qv

print "Rotation by quaternions"
print "   quaternion conjugate:"
print "   qc=",ConjQ(q)

qr = Q1xQ2(q,Q1xQ2(qv,ConjQ(q)))
print " qr = ",NoDust(qr)
qqr = Q1xQ2(qq,Q1xQ2(qv,ConjQ(qq)))
print " qr = ",NoDust(qqr)
