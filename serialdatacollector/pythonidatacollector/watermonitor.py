import RPi.GPIO as GPIO
import time
import serial
import csv
from collections import deque
import requests
import json
import os
import hashlib
import datetime

usart = serial.Serial("/dev/ttyAMA0", 57600)
usart.flushInput()


from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
import SocketServer
import simplejson
import random

class S(BaseHTTPRequestHandler):
    def _set_headers(self):
        self.send_response(200)
        self.send_header('Content-type', 'text/html')
        self.end_headers()

    def do_GET(self):
        self._set_headers()
        f = open("index.html", "r")
        self.wfile.write(f.read())

    def do_HEAD(self):
        self._set_headers()

    def do_POST(self):
        self._set_headers()
        print "in post method"
        self.data_string = self.rfile.read(int(self.headers['Content-Length']))

        self.send_response(200)
        self.end_headers()

        data = simplejson.loads(self.data_string)
        with open("test123456.json", "w") as outfile:
            simplejson.dump(data, outfile)
        print "{}".format(data)
        f = open("for_presen.py")
        self.wfile.write(f.read())
        return


def run(server_class=HTTPServer, handler_class=S, port=8082):
    server_address = ('', port)
    httpd = server_class(server_address, handler_class)
    print 'Starting httpd...'
    httpd.serve_forever()

run();

def roundTimeDown(dt=None, roundTo=60):

   if dt == None : dt = datetime.datetime.now()
   seconds = (dt.replace(tzinfo=None) - dt.min).seconds
   rounding = (seconds) // roundTo * roundTo
   return dt + datetime.timedelta(0,rounding-seconds,-dt.microsecond)

class Rollup :
	def addMeasure(self, value) :
		if (value > self.max) :
			self.max = value
		if (value < self.min) :
			self.min = value
		self.total += value
		self.sampleCount += 1
		
	def __init__(self, interval, max, min, startTime) :
		self.interval = interval
		self.max = max
		self.min = min
		self.total = 0
		self.sampleCount = 0
		self.startTime = startTime
		self.localTime = roundTimeDown(datetime.datetime.now(), interval)
	
class Rollups :
	
	
	def addMeasure(self, value) :
		if (datetime.datetime.now() > (self.currentRollup.localTime + datetime.timedelta(seconds=self.interval))):
			print "adding new rollup for interval ", self.interval, self.currentRollup.startTime + self.interval, time.time()
			self.rollupList.append(self.currentRollup)
			self.currentRollup = Rollup(self.interval, 0, 0, time.time())
		self.currentRollup.addMeasure(value)
		while(len(self.rollupList) > self.maxRollups) :
			self.rollupList.popleft()

	def __init__(self, interval, maxRollups) :
		self.interval = interval
		self.maxRollups = maxRollups
		self.currentRollup = Rollup(interval, 0, 0, time.time())
		self.rollupList = deque()
		self.baseLineAverage = 0
		
	def calcBaselineAverage(self): 
		minAverage = 999999999
		for rollup in self.rollupList :
			if (rollup.sampleCount > 0):
				average = rollup.total/rollup.sampleCount
				if (average < minAverage):
					minAverage = average
		self.baseLineAverage = minAverage
		
class RollupIntervals :
	
	def __init__(self) :
		self.rollups = { '60' : Rollups(60, 60) }
		self.rollups['900']= Rollups(900, 240)
		self.rollups['3600'] = Rollups(3600, 96)
		self.rollups['84600'] = Rollups(86400, 60)
		
	def addMeasure(self, value) :
		for interval, rollup in self.rollups.items() :
			rollup.addMeasure(value);
			

class RollupManager :
	def __init__(self) :
		self.rollups = {}
	
	def addRollup(self, name) :
		self.rollups[name]=RollupIntervals()
		return self.rollups[name]
		
	def addMeasure(self, name, value) :
		self.rollups[name].addMeasure(value)
		
	def publishStats(self, filePart) :
		publishTime = time.localtime()
		baseFileName = '/home/Stryder2/WaterWeb/data/' + str(publishTime.tm_year) + '-' + str(publishTime.tm_mon).zfill(2) + '-' + str(publishTime.tm_mday).zfill(2) + '-' + filePart
		publishFile = open(baseFileName+'.jsonpending', 'w+')
		#publishFile.write("<WaterStats>\r\n")
		publishFile.write('{\r\n')
		publishFile.write('  "WaterStats": {\r\n')
		publishFile.write('     "WaterStat": [')
		
		comma = "\r\n"
		print 'rollup count', len(self.rollups.items())
		for name, rollupManager in self.rollups.items() :
			print name, 'rollup interval count', len(rollupManager.rollups.items())
			for interval, rollupList in rollupManager.rollups.items() :
				print 'publishing interval', interval, 'with count', len(rollupList.rollupList)
				rollupList.calcBaselineAverage()
				for rollup in rollupList.rollupList :
					average = 0
					if (rollup.sampleCount > 0) :
						average = rollup.total/rollup.sampleCount
						
					#publishFile.write('    <WaterStat name = "%s" interval="%d" max="%d" min="%d"  average = "%d" sampleCount="%d" startTime="%d" localTime="%04d-%02d-%02d %02d:%02d:%02d" />\r\n' % (name, rollup.interval, rollup.max, rollup.min, average, rollup.sampleCount, rollup.startTime, rollup.localTime.tm_year, rollup.localTime.tm_mon, rollup.localTime.tm_mday, rollup.localTime.tm_hour, rollup.localTime.tm_min, rollup.localTime.tm_sec))
					publishFile.write(comma + '      {\r\n          "name": "%s",\r\n          "interval": "%d",\r\n          "max": "%d",\r\n          "min": "%d",\r\n          "average": "%d",\r\n          "baseLineAverage": "%d",\r\n          "adjustedAverage": "%d",\r\n          "sampleCount": "%d",\r\n          "startTime": "%d",\r\n          "localTime": "%04d-%02d-%02d %02d:%02d:%02d"\r\n          }' % (name, rollup.interval, rollup.max, rollup.min, average, rollupList.baseLineAverage, average-rollupList.baseLineAverage, rollup.sampleCount, rollup.startTime, rollup.localTime.year, rollup.localTime.month, rollup.localTime.day, rollup.localTime.hour, rollup.localTime.minute, rollup.localTime.second))
					comma = ",\r\n"
					
		publishFile.write("\r\n    ]\r\n")
		publishFile.write("  }\r\n")
		publishFile.write("}\r\n")
		#publishFile.write("</WaterStats>\r\n")
		publishFile.close();

		if (os.path.isfile(baseFileName+'.json')):
			os.remove(baseFileName+'.json')
			
		os.rename(baseFileName+ '.jsonpending', baseFileName + '.json')
		
class Vector3 :
	def __init__(self, x, y, z, magName) :
		self.x = x
		self.y = y
		self.z = z
		self.magName = magName
		self.apiKey = "opensecret"
		
rawLinesFile = open('rawdata.xml', 'w')
rawLinesFile.write("<RawWaterDatas>\r\n")

badHash = 0
badLine = 0
badExcept = 0
goodLines = 0
startStatTime = time.time()
lastStatPrint = time.time()
totalBytes=0

def parseLine(line, magneticVector):
	try:
		global badHash
		global badLine
		global badExcept
		global lastStatPrint
		global startStatTime
		global goodLines
		global totalBytes
		if (lastStatPrint + 20 < time.time()):
			lastStatPrint = time.time();
			print 'goodLines: ', goodLines, 'goodLines/sec: ', goodLines/(time.time()-startStatTime), 'bytes/sec: ', totalBytes/(time.time()-startStatTime),'badHash: ',badHash, 'badLine: ', badLine, 'badExcept: ', badExcept
			
		totalBytes += len(line)
		#print line
		parsedData = line.split(',')
		if len(parsedData) >= 5 :
			messageBody = parsedData[0] + ',' + parsedData[1] + ',' + parsedData[2]+ ',' + parsedData[3];
			hash = hashlib.md5();
			hash.update(messageBody);
			if (hash.hexdigest() == parsedData[4]):
				
				magneticVector[0] = int(parsedData[1])
				magneticVector[1] = int(parsedData[2])
				magneticVector[2] = int(parsedData[3])
				magneticVector[3] = parsedData[0]
				goodLines += 1
				#rawLinesFile.write('     <RawWaterData seconds="%f" x="%d" y="%d" z="%d" \>\r\n' %(time.time(), magneticVector[0], magneticVector[1], magneticVector[2]))  
				return True
			else:
				print 'bad line, expected hash', parsedData[3], 'but got hash', hash.hexdigest()
				badHash += 1
				badLine += 1
		else:
			#print 'bad line', line		
			badLine += 1
	except Exception as e: 
		print 'bad line', line, e
		badLine += 1
		badExcept += 1
	return False
			

def processVectors(lastMagneticVector, currentMagneticVector, vectorDeltaList, rollupX, rollupY, rollupZ) :
	#outLine = ""	
#	dX=currentMagneticVector[0]-lastMagneticVector[0]
#	dY=currentMagneticVector[1]-lastMagneticVector[1]
#	dZ=currentMagneticVector[2]-lastMagneticVector[2]
	timeStamp=time.time()
	
	#vectorDeltaList.append(VectorTimeRecords(timeStamp, list([dX, dY, dZ])))

	#firstVectorTimeStamp = 	vectorDeltaList[0].timeStamp
	#print 'reading', currentMagneticVector
	vectorCalculation=[0.0, 0.0, 0.0]
 #       if ((timeStamp - firstVectorTimeStamp) >= 3) :
			#for record in vectorDeltaList :
				#vectorCalculation[0] += abs(record.vector[0])
				#vectorCalculation[1] += abs(record.vector[1])
				#vectorCalculation[2] += abs(record.vector[2])
			
			#vectorCalculation[0] /= ((timeStamp - firstVectorTimeStamp))
			#vectorCalculation[1] /= ((timeStamp - firstVectorTimeStamp))
			#vectorCalculation[2] /= ((timeStamp - firstVectorTimeStamp))

			#vectorDeltaList.clear()

			#totalVectorDelta = (vectorCalculation[0]*vectorCalculation[0]+vectorCalculation[1]*vectorCalculation[1] + vectorCalculation[2]*vectorCalculation[2])**(1/2.0)
			#print 'totalVectorDelta=', totalVectorDelta
	vectorCalculation[0] = currentMagneticVector[0];
	vectorCalculation[1] = currentMagneticVector[1];
	vectorCalculation[2] = currentMagneticVector[2];
	magnometers[currentMagneticVector[3]].rollups["X"].addMeasure(vectorCalculation[0]);
	magnometers[currentMagneticVector[3]].rollups["Y"].addMeasure(vectorCalculation[1]);
	magnometers[currentMagneticVector[3]].rollups["Z"].addMeasure(vectorCalculation[2]);
	#print vectorCalculation
	
	vec3 = Vector3(vectorCalculation[0], vectorCalculation[1], vectorCalculation[2], currentMagneticVector[3])
	dataString = json.dumps(vec3.__dict__)
	print 'posting', dataString
	try:
		requests.post('http://localhost:8081/realTimeMagneticReading', data=dataString, headers = {'content-type': 'application/json'})
	except requests.ConnectionError, e:
		print e 
	try:
		requests.post('http://www.pristineplanet.org/realTimeMagneticReading', data=dataString, headers = {'content-type': 'application/json'})
	except requests.ConnectionError, e:
		print e 


	#if (dX >= 0) :
	#	outLine += '+'
	#	outLine += str(dX).zfill(8) + ','
	#else :
	#	outLine += str(dX).zfill(9) + ','

	#if (dY >= 0) :
	#	outLine += '+'
	#	outLine += str(dY).zfill(8) + ','
	#else :
	#	outLine += str(dY).zfill(9) + ','
	#if (dZ >= 0) :
	#	outLine += '+'
	#	outLine += str(dZ).zfill(8)
	#else :
	#	outLine += str(dZ).zfill(9) + ','


class VectorTimeRecords :
	def __init__(self, timeStamp, vector):
		self.timeStamp = timeStamp
		self.vector = vector

var=1

print "Start loop"
currentLine=""
first = True
lastMagneticVector=[0,0,0]

vectorDeltaList=deque()

magnometers = {"Mag3110" : RollupManager(), "Mag9Dof" : RollupManager() }
for name, rollupManager in magnometers.items():
	rollupX = rollupManager.addRollup("X");
	rollupY = rollupManager.addRollup("Y");
	rollupZ = rollupManager.addRollup("Z");

lastPublish = time.time()



while var==1 :
	gotByte=False
	 
	if (usart.inWaiting()>0) :
		receive=usart.read(1)
		gotByte = True
		if (receive=='\n') :
			magneticVector =[0,0,0,'']
			print currentLine
			
			if (parseLine(currentLine, magneticVector)):
				#if (first == False) :
				processVectors(lastMagneticVector, magneticVector, vectorDeltaList, rollupX, rollupY, rollupZ)
				#else : 
				#	firstVectorTimeStamp=time.time()

				#first=False
				#lastMagneticVector = magneticVector
			currentLine = ""
		else :
			currentLine += receive

	if (time.time() - lastPublish > 90) :
		print 'publishing'
		lastPublish = time.time()
		for name, rollupManager in magnometers.items():
			rollupManager.publishStats(name + "-XYZ")
		
	if (gotByte == False) :
		time.sleep(10.0/1000.0)
		
rawLinesFile.write("<\RawWaterDatas>\r\n")
rawLinesFile.close()
