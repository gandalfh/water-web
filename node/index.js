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
		lastMagneticReadings[magneticReading.magName] = magneticReading;
		
		webSocketServer.connections.forEach(function (conn) {
				console.log('sending to client: ' + JSON.stringify(magneticReading));
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