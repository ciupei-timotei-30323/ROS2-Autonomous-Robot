#v1 12Jun2025 - all commands ok, robot moves well; 
#v2 26June2025 - added navigation command, robot moves to desired angle and distance


import sys
import time
import threading
import numpy as np
import math

# Constants (tune these for your robot)
KP_ANGLE_ROTATION = 0.1  # Proportional gain for rotation angle correction
KP_ANGLE = 0.1     # Proportional gain for angle correction
KD_ANGLE = 0.1    # Derivative gain for angle correction
KP_DISTANCE = 0.01  # Proportional gain for distance correction
BASE_SPEED = 1    # Base speed for motors

ACC = 2
MAX_SPEED = 80 # Maximum set speed for motors

#TODO aici add PI to rotation speed calculation
def calculate_motor_speeds_rotation_only(currentAngle, gMaxSpeed):
	global gDesiredAngle

	angle_error = gDesiredAngle - currentAngle
	#manage angle error to be within -180 to 180 degrees
	if angle_error > 1800:
		angle_error -= 3600
	elif angle_error < -1800:
		angle_error += 3600
			
	turn = KP_ANGLE_ROTATION * angle_error
	left_speed = 0 - turn
	right_speed = 0 + turn

	# limit speeds
	left_speed = max(0-gMaxSpeed, min(gMaxSpeed, left_speed))
	right_speed = max(0-gMaxSpeed, min(gMaxSpeed, right_speed))

	if abs(angle_error) < 5:
		return 0, 0
	return int(left_speed), int(right_speed)

def calculate_motor_speeds_straight_only(left_enc, right_enc, currentAngle, maxSpeed):
	global gDesiredAngle, gDesiredDistance, oldangle_error
	avg_encoder = (left_enc + right_enc) / 2
	distance_error = gDesiredDistance - avg_encoder
	angle_error = gDesiredAngle - currentAngle
	#manage angle error to be within -180 to 180 degrees
	if angle_error > 1800:
		angle_error -= 3600
	elif angle_error < -1800:
		angle_error += 3600
    # Only correct small angle errors while moving straight
	turn = (KP_ANGLE * angle_error) + (KD_ANGLE * (angle_error - oldangle_error))
	oldangle_error = angle_error  # Store the current angle error for the next iteration
	forward = KP_DISTANCE * distance_error
	forward = max(0-maxSpeed, min(maxSpeed, forward))
	left_speed = BASE_SPEED + forward - turn
	right_speed = BASE_SPEED + forward + turn

    # Allow negative speeds for reverse
	left_speed = max(-100, min(100, left_speed))
	right_speed = max(-100, min(100, right_speed))

	if abs(distance_error) < 100:
		return 0, 0
	return int(left_speed), int(right_speed)

def sendCmdToSerialInterface():
	global root
	global mSpdRef
	global relLedCtlBin
	global ackVal
	global rcVal
	global mSpi
	global mRsp
	global gTxRxOn
	global flagTxOnce
	global wait4TxRx
	global gInhibitPrint
	global fExitCondition
	global rxElemInt
	global flagNavigatingOn
	global flagNavigationOnGetStartValues
	global enc1ValAtStartNavigation
	global enc2ValAtStartNavigation
	global relEnc1
	global relEnc2
	global navigationPhase  # <-- add this
	global cMaxSpeed
	
	wait4Answer = False

	while False == fExitCondition:
		if True == gTxRxOn:
			if True == wait4Answer:
				wait4Answer = False
				getAnswerFromSerialInterface()
				wait4TxRx = False
				if True == flagTxOnce:
					flagTxOnce = False
					#only one tx + rx requested
					gTxRxOn = False
				if True == flagNavigatingOn:
					# Sequential navigation logic
					if navigationPhase == 1:
						# Rotation phase
						if cMaxSpeed <= ( MAX_SPEED - ACC ):
							cMaxSpeed +=ACC # ramp up speed gradually
						mSpdRef[0], mSpdRef[3] = calculate_motor_speeds_rotation_only(rxElemInt["gyrZ"], cMaxSpeed)
						if (mSpdRef[0] == 0 and mSpdRef[3] == 0):
							#reinitialize encoders
							flagNavigationOnGetStartValues = True
							cMaxSpeed = 10
							navigationPhase = 2  # Switch to straight phase
					elif navigationPhase == 2:
						# Straight movement phase
						if cMaxSpeed <= (MAX_SPEED - ACC):
							cMaxSpeed +=ACC # ramp up speed gradually
						mSpdRef[0], mSpdRef[3] = calculate_motor_speeds_straight_only(relEnc1, relEnc2, rxElemInt["gyrZ"], cMaxSpeed)
						if (mSpdRef[0] == 0 and mSpdRef[3] == 0):
							flagNavigatingOn = False
							flagTxOnce = True  # send stop command
							navigationPhase = 0  # Reset phase
				else:
					cMaxSpeed = 10
				logDict.append({"timestamp": time.time(), "angle": float(rxElemInt["gyrZ"])/10.0, "desiredAngle": float(gDesiredAngle)/10, "distance": (relEnc1+relEnc2)/2, "desiredDistance": gDesiredDistance, "speedLeft": mSpdRef[0], "speedRight": mSpdRef[3]})
				time.sleep(0.02)
			else:
				relLedCtlBin = 0

				if rcVal < 255:
					rcVal += 1
				else:
					rcVal = 0
				tDat = []
				tDat.extend(mSpdRef)		#bytes 0..3
				tDat.append(relLedCtlBin) 	#byte 4
				tDat.append(ackVal)			#byte 5
				tDat.append(0)				#byte 6
				tDat.append(rcVal)			#byte 7

				if 'rpi' == platform:
					# if ch[0] != 0:
						#use SPI active
						tDatSpi = tDat
						tDatSpi.extend([8,9,10]) #bytes 8 to 10 for IMU data
						if False == gInhibitPrint:
							print("txDataSPI:",tDatSpi)
						mRsp = mSpi.xfer2(tDatSpi)
					# if ch[1] != 0:
					# 	#UART active
					# 	ser.write(tDat)
					# 	print("txDataUART:",tDat)
				wait4Answer = True
				time.sleep(0.01)
	print ("thread end")

def getAnswerFromSerialInterface():
	global mSpi
	global ser
	global mRsp
	global rxData
	global rxElemInt
	global rxElemsString
	global vEncBuf
	global vToggle
	global rxElemInt
	global gInhibitPrint
	global enc1Val 
	global enc1ValOld
	global enc1AccumulatedVal
	global enc2Val
	global enc2ValOld
	global enc2AccumulatedVal
	global flagNavigationOnGetStartValues
	global enc1ValAtStartNavigation
	global enc2ValAtStartNavigation
	global relEnc1
	global relEnc2

	cColor = ["lightgray","lightgreen"]

	rxData = []

	if 'rpi' == platform:
		# if ch[0] != 0:
			#SPI active
			rxData = mRsp
			
		# if ch[1] != 0:
			#UART active
			# serData = ser.read(19)
			# if rxData.count == 0:
				# rxData = serData #use UART data only if SPI is inactive
	else:
		# print("pc mode active - simulated rx data")
		if (0 == vToggle ):
			rxData.extend([0,100,0,100,0,16,0,20,0,30,0])
		else:
			rxData.extend([0,110,0,110,0,17,0,25,0,35,0])
		vToggle = 1 - vToggle
	
	if ( rxData.count != 0):
		if False == gInhibitPrint:
			print("rxData:", rxData)
		#rxData format: 
		# 0 	  1 	  2 	  3 	  4 				  			  5 		6 	  7 	8 	  9 	10 	  11 	12 	  13 	14 	  15 	16 	  17    18
		# ENC1Hi, ENC1Lo, ENC2Hi, ENC2Lo, Rel0..3Sta[0..3]+LEDsSta[4..7], Stat[0,1] + BV[2..7], gZHi, gZLo, PwmM1 PwmM2 RC
		tVal1 = np.uint16((0xFFFF & (rxData[0] << 8)) + rxData[1])
		enc1Val = np.uint32(tVal1)
		#manage overflow
		if (enc1Val < 1000 and enc1ValOld > 64000): #encoder overflow - positive direction
			enc1AccumulatedVal += 65536 
		elif (enc1Val > 64000 and enc1ValOld < 1000): #encoder underflow - negative direction
			enc1AccumulatedVal -= 65536
		enc1ValOld = enc1Val
		rxElemInt["enc1"] = enc1AccumulatedVal + enc1Val
		# vEncBuf["1"].append(tVal2)
		
		tVal1 = np.uint16((0xFFFF & (rxData[2] << 8)) + rxData[3])
		enc2Val = np.uint32(tVal1)
		#manage overflow
		if (enc2Val < 1000 and enc2ValOld > 64000): #encoder overflow - positive direction
			enc2AccumulatedVal += 65536 
		elif (enc2Val > 64000 and enc2ValOld < 1000): #encoder underflow - negative direction
			enc2AccumulatedVal -= 65536
		enc2ValOld = enc2Val
		rxElemInt["enc2"] = enc2AccumulatedVal + enc2Val
		# vEncBuf["2"].append(tVal2)

		rxElemInt["stat"] = (rxData[5] >> 6)
		rxElemInt["batVoltage"] = rxData[5] & 0x3F

		# tVal1 = np.uint16((0xFFFF & (rxData[6] << 8)) + rxData[7])
		# tVal2 = np.int16(tVal1)
		# rxElemInt["accX"] = tVal2
		
		# tVal1 = np.uint16((0xFFFF & (rxData[8] << 8)) + rxData[9])
		# tVal2 = np.int16(tVal1)
		# rxElemInt["accY"] = tVal2	
		
		# tVal1 = np.uint16((0xFFFF & (rxData[10] << 8)) + rxData[11])
		# tVal2 = np.int16(tVal1)		
		# rxElemInt["accZ"] = tVal2

		# tVal1 = np.uint16((0xFFFF & (rxData[12] << 8)) + rxData[13])
		# tVal2 = np.int16(tVal1)
		# rxElemInt["gyrX"] = tVal2		

		# tVal1 = np.uint16((0xFFFF & (rxData[14] << 8)) + rxData[15])
		# tVal2 = np.int16(tVal1)
		# rxElemInt["gyrY"] = tVal2	
		
		tVal1 = np.uint16((0xFFFF & (rxData[6] << 8)) + rxData[7])
		tVal2 = np.int16(tVal1)
		rxElemInt["gyrZ"] = tVal2

		rxElemInt["pwmM1"] = rxData[8]
		rxElemInt["pwmM4"] = rxData[9]

		rxElemInt["RC"] = rxData[10]

		if	flagNavigationOnGetStartValues == True:
			flagNavigationOnGetStartValues = False
			enc1ValAtStartNavigation = rxElemInt["enc1"]
			enc2ValAtStartNavigation = rxElemInt["enc2"]	
		relEnc1 = rxElemInt["enc1"] - enc1ValAtStartNavigation
		relEnc2 = rxElemInt["enc2"] - enc2ValAtStartNavigation
		

		print ("speeds:", mSpdRef[0], mSpdRef[3], "relEnc1", relEnc1, " relEnc2", relEnc2, "angle", float(rxElemInt["gyrZ"])/10.0, "pwmM1", rxElemInt["pwmM1"], "pwmM4", rxElemInt["pwmM4"] ) #, end='\r', flush=True)
	else:
		print("no communication channel active")

def cmdInterpreter():
	global gTxRxOn
	global mSpdRef
	global flagTxOnce
	global wait4TxRx
	global gInhibitPrint
	global fExitCondition
	global flagNavigatingOn
	global gDesiredAngle
	global gDesiredDistance
	global flagNavigationOnGetStartValues
	global navigationPhase  # <-- add this

	cmdText = 'n'
	while 'exit' != cmdText:
			if False == wait4TxRx:
				cmdText = input("cmd [tx1, txon, txoff, m1s[0..100], m4s[0..100], rel, print, nav[ang,dist], exit]:")
				# print("cmd is ", cmdText, len(cmdText))
				if 'tx1' == cmdText:
					flagTxOnce = True
					gTxRxOn = True
					wait4TxRx = True
				if 'txon' == cmdText:
					gTxRxOn = True
					gInhibitPrint = True
				if 'txoff' == cmdText:
					gInhibitPrint = False
					gTxRxOn = False
				if 'm1s' == cmdText[0:3]:
					mSpdRef[0] = int(cmdText[3:])
					print("speed ref: ", mSpdRef, flush=True)
				if 'm4s' == cmdText[0:3]:
					mSpdRef[3] = int(cmdText[3:])
					print("speed ref: ", mSpdRef, flush=True)
				if 'rel' == cmdText[0:3]:
					relLedCtlBin = int(cmdText[3:])
				if 'nav' == cmdText[0:3]:
					gInhibitPrint = True
					flagNavigatingOn = True
					#clear log
					logDict.clear()
					navigationPhase = 1  # Start with rotation phase
					if len(cmdText) > 3:
						#parse angle and distance
						try:
							gDesiredAngle = int(cmdText[3:cmdText.index(',')])*10 #for angles 10 == 1 degree
							gDesiredDistance = int(cmdText[cmdText.index(',')+1:])
						except ValueError:
							print("Invalid navigation command format. Use 'nav <angle>,<distance>'")
							flagNavigatingOn = False
							navigationPhase = 0
						if flagNavigatingOn:
							if gDesiredDistance != 0:
								gTxRxOn = True
								flagNavigationOnGetStartValues = True
							else:
								flagNavigatingOn = False
								gTxRxOn = False
								mSpdRef[0] = 0
								mSpdRef[3] = 0
								navigationPhase = 0
					else:
						print("Navigation command requires angle and distance parameters.")
				if 'print' == cmdText:
					print('acc (x,y,z): ',rxElemInt["accX"],',',rxElemInt["accY"],',',rxElemInt["accZ"])
					print('gyr (roll,pitch,yaw): ',rxElemInt["gyrX"],',',rxElemInt["gyrY"],',',float(rxElemInt["gyrZ"])/10.0)
					print('spd (left motor, notUsed, notUsed, right motor):',mSpdRef, flush=True)
				
	gTxRxOn = False #this will also make the thread to end
	fExitCondition = True

#program start

platform = 'rpi' #change to 'rpi' when running this code on RaspberryPi
fExitCondition = False
flagTxOnce = False
gTxRxOn = False

left_encoder = 0
right_encoder = 0

gDesiredAngle = 0
gDesiredDistance = 0
flagNavigatingOn = False

oldangle_error = 0  # Initialize old angle error for derivative calculation

enc1Val = int(0)
enc1ValOld = int(0)
enc1AccumulatedVal = int(0)
enc2Val = int(0)
enc2ValOld = int(0)
enc2AccumulatedVal = int(0)
flagNavigationOnGetStartValues = False
enc1ValAtStartNavigation = 0
enc2ValAtStartNavigation = 0
relEnc1 = 0
relEnc2 = 0
navigationPhase = 0  # 0: idle, 1: rotate, 2: move straight

mRsp = []
rxElemInt = {
"rel1": 0,
"rel2": 0,
"rel3": 0,
"rel4": 0,
"ledW1": 0,
"ledW2": 0,
"ledR1": 0,
"ledR2": 0,
"enc1": 0,
"enc2": 0,
"pwmM1": 0,
"pwmM4": 0,
"gyrZ": 0,
"stat": 0,
"batVoltage": 0,
"RC": 0}

rxData = []

mSpdRef=[0,0,0,0]
mEncVal=[0,0]
ch = [1,0] #SPI,UART

cMaxSpeed = 10 #initial value of max speed for straight movement, used in navigation

logFileName = "aMPwWCtl.log"
logDict = [{"timestamp": time.time(), "angle": rxElemInt["gyrZ"], "desiredAngle": gDesiredAngle, "distance": (relEnc1+relEnc2)/2, "desiredDistance": gDesiredDistance, "speedLeft": mSpdRef[0], "speedRight": mSpdRef[3]}]


if 'pc' == sys.argv[1]:
	platform = 'pc'

print("running on: ", platform)

if 'rpi' == platform:
	import spidev
	global mSpi

	mSpi = spidev.SpiDev()
	mSpi.open(0,0)
	mSpi.mode=0b00
	mSpi.max_speed_hz=1000000



ackVal = 0
rcVal = 0
vToggle = 0

gEnTxRx = False
wait4TxRx = False
gInhibitPrint = False

tSer = threading.Thread(target=sendCmdToSerialInterface)
tSer.start()

cmdInterpreter()

if ( True == tSer.is_alive() ):
	tSer.join()

if 'rpi' == platform:
	mSpi.close()
	# ser.close()

#write log file
with open(logFileName, 'w') as logFile:
	for entry in logDict:
		logFile.write(f"{entry['timestamp']}, {entry['angle']}, {entry['desiredAngle']}, {entry['distance']}, {entry['desiredDistance']}, {entry['speedLeft']}, {entry['speedRight']}\n")
