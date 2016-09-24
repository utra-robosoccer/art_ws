import rospy
import numpy as np
from numpy import linalg as linalg
from numpy import random as random
import math as math
from std_msgs.msg import String

class ExtendedKalmanFilter:
# http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it
# http://en.wikipedia.org/wiki/Kalman_filter

	def __init__(self):
		# declare instance variables
		self.lastState = np.zeros(3)
		self.newState = np.zeros(3) # systm output is x,y, sigma (bearing)
		self.predictedNewState = np.zeros(3)
		self.b = 1. # Wheel base in metres
		self.kr = 1000#error in the optical encoders in m
		self.kl = 1000
		self.FpJacobian = np.identity(3)
		self.FrlJacobian = np.array([[0.5, 0.5], [0, 0], [1/self.b, -1/self.b]])
		self.lastPredictedUncertainty = np.identity(3) * 1000 # start with high uncertainty
		self.newPredictedUncertainty = np.identity(3)
		self.newGain = np.zeros([3,2])
		self.H_k =  np.array([[1, 0, 0], [0, 1, 0]])# equation 6.18


	def initializeState(self):
		print "init state"



	'''
	dSl: "distance travelled by left wheel"
	dSr: "distance travelled by right wheel"
	theta: "bearing"
	system model tells us the true location x_k of the robot at time k
	PREDICT
	'''
	def systemModel(self, dSl, dSr, theta):
		# equation 3.1, system model
		dS = (dSr + dSl) / 2
		dTheta = (dSr - dSl) /self.b		
		dX = dS * (math.cos(theta + dTheta/2))
		dY = dS * (math.sin(theta + dTheta/2))
		deltaState = np.array([dX, dY, dTheta])

		# equation 3.3
		encoderErrorCovariance = np.array([[self.kr * abs(dSr), 0], [0, self.kl * abs(dSl)]])

		# equation 3.5
		dD = (dSr + dSl) /2

		self.FpJacobian = np.array([[1, 0, - dD * math.sin(theta + dTheta/2)], 
			[0, 1, dD * math.cos(theta + dTheta/2)], 
			[0, 0, 1]])

		self.FrlJacobian = np.array([
			[math.cos(theta + dTheta/2)/2 - dD * math.sin(theta + dTheta/2)/ (2 * self.b), 
			 math.cos(theta + dTheta/2)/2 + dD * math.sin(theta + dTheta/2)/ (2 * self.b)],
			[math.sin(theta + dTheta/2)/2 + dD * math.cos(theta + dTheta/2)/ (2 * self.b), 
			 math.sin(theta + dTheta/2)/2 - dD * math.cos(theta + dTheta/2)/ (2 * self.b)],
			[1/self.b,
			-1/self.b]])


		# equation 6.12, calculate new state based on old state 
		self.predictedNewState = self.lastState + deltaState
		#print "deltaState: " + str(deltaState)

		# 6.14
		self.propagationUncertainty = self.varianceCovariance(self.FrlJacobian, encoderErrorCovariance)

		# equation 6.13
		self.newPredictedUncertainty = self.varianceCovariance(self.FpJacobian, self.lastPredictedUncertainty) + self.propagationUncertainty
		#print "newPredictedUncertainty: " + str(self.newPredictedUncertainty)

	'''
	measurement model tells us the relation between the location of the robot x_k and measurement z_k
	UPDATE
	'''
	def measurementModel(self, gpsX, gpsY):

		# measurementModelCovariance
		z_k = np.array([gpsX, gpsY])
		R_k = np.array([[0, 0], [0,0]]) # need sandro for this
		print "inputGPS: " + str(z_k)

		# equation 6.15, calculate kalman gain
		self.gain_k = np.dot(self.newPredictedUncertainty, np.transpose(self.H_k))
		tmp = self.varianceCovariance(self.H_k, self.newPredictedUncertainty)
		print "modelError: " + str(tmp) 
		print "gpsError: " + str(R_k)
		self.gain_k = np.dot(self.gain_k, linalg.inv(tmp + R_k))
		print "gain: " + str(self.gain_k)

		# equation 6.16, calculate the new estimated state
		self.newState = self.predictedNewState + np.dot(self.gain_k, (z_k - self.predictedNewState[0:2])) # python indexes using [0:2] takes elements 0, 1 
		
		# equation 6.17, update the propagation uncertainty 		
		self.newPredictedUncertainty = np.dot(np.identity(3) - np.dot(self.gain_k, self.H_k), self.newPredictedUncertainty)

		# shift new values to old values
		self.lastPredictedUncertainty = np.copy(self.newPredictedUncertainty)
		self.lastState = np.copy(self.newState)

		
		return self.newState


	'''
	Used for propagation of uncertainty principle returns A * B * BTranspose
	'''
	def varianceCovariance(self, a, b):
		return np.dot(np.dot(a, b), np.transpose(a))


def talker():
	pub = rospy.Publisher('chatter', String, queue_size=10)
	rospy.init_node('talker', anonymous=True)
	rate = rospy.Rate(10) # 10hz

	kf = ExtendedKalmanFilter()
	count = 0


	while not rospy.is_shutdown():
		hello_str = "hello world %s" % rospy.get_time()
		#rospy.loginfo(hello_str)
		#pub.publish(hello_str)
		rate.sleep()
		kf.systemModel(1, 1, 0)

		randX = random.normal(loc=0, scale= 3)
		randY = random.normal(loc=0, scale= 3)		
		print "Output state: " + str(kf.measurementModel(6+ randX,6 + randY))
 
if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass


