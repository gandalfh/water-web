var ws = require("nodejs-websocket")
var express = require('express');
var app = express();  //use express js module
const util = require('util')
var bodyParser = require('body-parser')
const fs = require('fs')
var glob = require('glob')
var path = require('path')

//add handlebars view engine
var handlebars = require('express-handlebars')
	.create({defaultLayout: 'main'});  //default handlebars layout page

app.engine('handlebars', handlebars.engine);
app.set('view engine', 'handlebars'); //sets express view engine to handlebars
app.use(bodyParser.json());
app.use('/data', express.static(__dirname + '/data'));
app.set('port', process.env.PORT || 8081);  //sets port 8081

var lastMagneticReadings = {};

app.get('/realTimeMagneticReading', function(request, response){
	response.header("Content-Type",'application/json');
	response.send(JSON.stringify(lastMagneticReadings));
});

function broadcastClients(magneticReading) {
		lastMagneticReading = magneticReading;
		magneticReading.lastUpdateTime = new Date();

		lastMagneticReadings[magneticReading.magName] = magneticReading;
		
		webSocketServer.connections.forEach(function (conn) {
				//console.log('sending to client: ' + JSON.stringify(magneticReading));
				try {
					conn.sendText(JSON.stringify(magneticReading));
				} catch (e)
				{
					console.log("Erorr while trying to send data to web socket client" + e);
				}
			})	
}

app.post('/realTimeMagneticReading', function(request, response){
	if (request.body.apiKey === 'opensecret') {
		delete request.body.apiKey;
		console.log('post received' + request.body);
		broadcastClients(request.body);
		response.send(request.body);    // echo the result back
	}
});

app.post('/magneticReading', function(request, response){
	console.log('putMagneticReading received' + JSON.stringify(request.body));
	if (request.body.apiKey === 'opensecret') {
		delete request.body.apiKey;
		broadcastClients(request.body);
	}
	var currentTime = {
		epoch: Math.floor(new Date().getTime()/1000)	
	};

	response.send(currentTime);
});

function pad(n, width, z) {
  z = z || '0';
  n = n + '';
  return n.length >= width ? n : new Array(width - n.length + 1).join(z) + n;
}
function MakeFileNameFromDate(today, magName) {
	return "./data/" + today.getFullYear() + "-" + pad(today.getMonth()+1, 2) + "-" + pad(today.getDate()+1, 2) + "-"+ magName + "-XYZ.json";
}

function MakeKeyFromWaterStat(waterStat) {
	return waterStat.interval + '-' + waterStat.startTimeEpoch + '-' + waterStat.name	
}

function RecalculateBaselineAverages(waterData) {
	var baselineAverages = {};
	for (var i = 0; i < waterData.WaterStats.WaterStat.length; i++) {
		var waterStat = waterData.WaterStats.WaterStat[i];
		var baselineAverageKey = waterStat.name.toUpperCase() + waterStat.interval; 
		if (!baselineAverages[baselineAverageKey] || waterStat.average < baselineAverages[baselineAverageKey]) {
			baselineAverages[baselineAverageKey] = waterStat.average;
		}
	}

	for (var i = 0; i < waterData.WaterStats.WaterStat.length; i++) {
		var waterStat = waterData.WaterStats.WaterStat[i];
		var baselineAverageKey = waterStat.name + waterStat.interval;
		waterStat.baseLineAverage = baselineAverages[baselineAverageKey];
		waterStat.adjustedAverage = waterStat.average - baselineAverages[baselineAverageKey];
	}
}

function WriteAndRenameFileData(fileName, waterData) {
	fs.writeFile(fileName + "new" , JSON.stringify(waterData),
			function(error){
				console.log("file saved:" +fileName + "new");
				if (fs.existsSync(fileName)) {
					fs.unlink(fileName);
				}
				fs.rename(fileName + 'new', fileName);
				console.log("file renamed to " + fileName)
			}
		);	
}

function TrimLowSampleCounts(waterData) {
	var newArray = [];
	for (var i = 0; i < waterData.WaterStats.WaterStat.length; i++) {
		if (waterData.WaterStats.WaterStat[i].sampleCount >= waterData.WaterStats.WaterStat[i].interval/10) {
			newArray.push(waterData.WaterStats.WaterStat[i]);
		}
	}
	waterData.WaterStats.WaterStat = newArray;
}

function ProcessMetrics(metricData) {
	var waterData = {WaterStats: {WaterStat: []}}
	//console.log("Got " + metricData.magData.length + " records of magData");
	for(var i = 0; i < metricData.magData.length; i++) {
		console.log("sample count " + metricData.magData[i].sampleCount);
		var average = Math.round(metricData.magData[i].sampleCount > 0 ? metricData.magData[i].total/metricData.magData[i].sampleCount : 0);
		if (metricData.magData[i].sampleCount > metricData.magData[i].intervalSeconds/10) {
			waterData.WaterStats.WaterStat.push({
				name: metricData.magData[i].axisName.toUpperCase(),
				interval: metricData.magData[i].intervalSeconds,
				average: average,
				baseLineAverage: 0,
				adjustedAverage: 0,
				sampleCount: metricData.magData[i].sampleCount,
				startTicksMs: metricData.magData[i].startTicks,
				startTimeEpoch: metricData.magData[i].startTime,
				first: metricData.magData[i].first ? true:false,
				whole: metricData.magData[i].whole ? true:false
			});
		}

	}



	var today = new Date();
	var fileName = MakeFileNameFromDate(today, metricData.magName);
	var loadFileName = fileName;

	if (!fs.existsSync(fileName)) {
		console.log(fileName + " does not exist");
		var yesterday = new Date();
		yesterday.setDate(yesterday.getDate()-1);

		loadFileName = MakeFileNameFromDate(yesterday);
	}

	if (fs.existsSync(fileName)) {
		console.log(fileName + " does exist");
		fs.readFile(fileName, 'utf8', function(error, data){
			console.log('loading existing data from ' + fileName);
			var existingData = JSON.parse(data);
			var intervalEpochKeys = {};
			for (var i = 0; i < existingData.WaterStats.WaterStat.length; i++) {			
				var waterStat = existingData.WaterStats.WaterStat[i];
				intervalEpochKeys[MakeKeyFromWaterStat(waterStat)] = waterStat;
			}
			
			for (var i = 0; i < waterData.WaterStats.WaterStat.length; i++) {
				var newWaterStat = waterData.WaterStats.WaterStat[i];

				if (intervalEpochKeys[MakeKeyFromWaterStat(newWaterStat)]) {
					/*var old = intervalEpochKeys[MakeKeyFromWaterStat(newWaterStat)];
					if (newWaterStat.first && newWaterStat.sampleCount > 0 && newWaterStat.sampleCount != old.sampleCount) {
						newWaterStat.average = (newWaterStat.sampleCount*newWaterStat.average + old.sampleCount*old.average)/(newWaterStat.sampleCount + old.sampleCount);
						newWaterStat.sampleCount = newWaterStat.sampleCount + old.sampleCount;
					}
					else
					{*/
						delete intervalEpochKeys[MakeKeyFromWaterStat(newWaterStat)];
					//}
				}
			}

			var count = 0;
			for(var key in intervalEpochKeys) {
				waterData.WaterStats.WaterStat.push(intervalEpochKeys[key]);
				count++;
			}

			console.log('added ' + count + 'records');


			RecalculateBaselineAverages(waterData);
			TrimLowSampleCounts(waterData);

			fileName = MakeFileNameFromDate(today, metricData.magName);

			console.log("After adjustment" + JSON.stringify(waterData));
			WriteAndRenameFileData(fileName, waterData);
			

		});


	} else {
		console.log(fileName + " does not exist, starting new file");
		fileName = MakeFileNameFromDate(today, metricData.magName);
		RecalculateBaselineAverages(waterData);
		WriteAndRenameFileData(fileName, waterData);
	}
	



}

app.put('/magneticMetrics', function(request, response){
	console.log('Metrics Received' + JSON.stringify(request.body));

    var size = 0;
	var completedData =[];

    request.on('data', function (data) {
        size += data.length;
		completedData.push(data);
        //console.log('Got chunk: ' + data.length + ' total: ' + size);
    });

    request.on('end', function () {
		var metricData = JSON.parse(completedData.join(''));
		ProcessMetrics(metricData);
		console.log('Data Received: ' + JSON.stringify(metricData));
        console.log("total size = " + size);
    }); 

    request.on('error', function(e) {
        console.log("ERROR ERROR: " + e.message);
    });

});


app.get('/jsonToXml', function(req,res) { 
	console.log('converting json file ' + req.query.jsonFile);
    fs.readFile(__dirname + '/data/' + req.query.jsonFile, 'utf8', function (err, data) {
		if (!err) 
		{
			console.log('Loaded json file ' + req.query.jsonFile);
			var obj = JSON.parse(data);
			console.log('parsed json file ' + req.query.jsonFile);
			res.set('Content-Type', 'text/xml');
			res.send(js2xmlparser.parse("WaterStatsXml", obj));
		}
		else
			console.log(err);
	});
});

app.get('/', function(req,res){ 
    getFileList(function (er, fileList) {
		fileListArray = [];
		for(var i = 0; i  < fileList.length; i++) {
			splits = fileList[i].split('-');
			fileListArray.push({fileName: fileList[i], fileTitle: splits[3] + ' ' + fileList[i].substring(0, 10) }); 
		}
		fileListArray.reverse();
		
		var data = {
				files: fileListArray,
				jsonFile: req.query.jsonFile
			}
			
		console.log('data: ' + util.inspect(data));
			
		res.render('home',
			data);  
	
	});
});

app.use(function(req,res){  //express catch middleware if page doesn't exist
	res.status(404);  //respond with status code
	res.render('404'); //respond with 404 page
});

app.listen(app.get('port'), function(){ //start express server
	console.log( 'Express Server Started on http://localhost:8081');
});

var webSocketServer = ws.createServer(function (conn) {
    console.log("New web socket connection")
    conn.on("text", function (str) {
        console.log("Received "+str)
    })
    conn.on("error", function (str) {
        console.log("websocket error: "+str)
    })
    conn.on("close", function (code, reason) {
        console.log("Connection closed")
    })
}).listen(8080)

function  getFileList(returnFn) {
	glob(__dirname + "/data/*-XYZ.json", function (er, fileList) {
		if (fileList) {
			for(var i = 0; i < fileList.length; i++) {
				fileList[i] = path.basename(fileList[i]);
			}
			console.log('got ' + fileList.length + 'files :' + fileList);
			returnFn(er, fileList);
		}
		
	});
}