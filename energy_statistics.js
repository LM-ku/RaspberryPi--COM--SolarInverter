// Скрипт IOBroker для учета выработанной и потребленной электроэнергии
// по обновлению значений солнечной энергии и потребленной электроэнергии,
// полученных от инвертора, формируются запросы в InfluxDB 

createState("energy_statistics.pv_hour", function () {
});
createState("energy_statistics.pv_day", function () {
});
createState("energy_statistics.pv_week", function () {
});
createState("energy_statistics.pv_month", function () {
});
createState("energy_statistics.pv_year", function () {
});
createState("energy_statistics.pv_total", function () {
});
createState("energy_statistics.ac_hour", function () {
});
createState("energy_statistics.ac_day", function () {
});
createState("energy_statistics.ac_week", function () {
});
createState("energy_statistics.ac_month", function () {
});
createState("energy_statistics.ac_year", function () {
});
createState("energy_statistics.ac_total", function () {
});


var date, now, year, month, week, day, hour; 

// CТАТИСТИКА ГЕНЕРАЦИИ СОЛНЕЧНОЙ ЭНЕРГИИ
// при обновлении значения солнечной энергии обновить значения счетчиков из InfluxDB

on({id: "mqtt.0.my_solar.status.pv_energy"/*my_solar/status/pv_energy*/, change: "any"}, function (obj) {
	var value = obj.state.val;
	var oldValue = obj.oldState.val;

// инициализация переменных
	date = new Date();
	now = date.getTime();
	year = new Date(Number(date.getFullYear()),0,1,0,0,0).getTime()
	month = new Date(Number(date.getFullYear()),Number(date.getMonth()),1,0,0,0).getTime()
	week = new Date(Number(date.getFullYear()),Number(date.getMonth()),Number(date.getDate()-date.getDay()+1),0,0,0).getTime()
	day = new Date().setHours(0,0,0,0)
	hour = new Date().setHours(Number(date.getHours()),0,0,0)

// от начала часа   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.pv_energy" WHERE time >=' + String(hour) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('PV_Hour: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.pv_hour", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*10000)/10000);
		}
	});
// от начала cуток   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.pv_energy" WHERE time >=' + String(day) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('PV_Day: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.pv_day", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*1000)/1000);   
		}
	});
// от начала недели   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.pv_energy" WHERE time >=' + String(week) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('PV_Week: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.pv_week", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*1000)/1000);
		}
	});
// от начала месяца   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.pv_energy" WHERE time >=' + String(month) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('PV_Month: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.pv_month", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*100)/100);
		}
	});
// от начала года   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.pv_energy" WHERE time >=' + String(year) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('PV_Year: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.pv_year", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*10)/10);
		}
	});
// всего до настоящего времени   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.pv_energy" WHERE time <=' + String(now) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('PV_Total: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.pv_total", Math.round(parseFloat((JSON.stringify(result.result[0])).slice(8, 27))));
		}
	});

});

// CТАТИСТИКА ПОТРЕБЛЕНИЯ ЭЛЕКТРОЭНЕРГИИ
// при обновлении значения потребления электроэнергии обновить значения счетчиков из InfluxDB

on({id: "mqtt.0.my_solar.status.ac_energy"/*my_solar/status/ac_energy*/, change: "any"}, function (obj) {
	var value = obj.state.val;
	var oldValue = obj.oldState.val;

// инициализация переменных
	date = new Date();
	now = date.getTime();
	year = new Date(Number(date.getFullYear()),0,1,0,0,0).getTime()
	month = new Date(Number(date.getFullYear()),Number(date.getMonth()),1,0,0,0).getTime()
	week = new Date(Number(date.getFullYear()),Number(date.getMonth()),Number(date.getDate()-date.getDay()+1),0,0,0).getTime()
	day = new Date().setHours(0,0,0,0)
	hour = new Date().setHours(Number(date.getHours()),0,0,0)

// от начала часа   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.ac_energy" WHERE time >=' + String(hour) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('AC_Hour: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.ac_hour", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*10000)/10000);
		}
	});
// от начала cуток   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.ac_energy" WHERE time >=' + String(day) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('AC_Day: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.ac_day", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*1000)/1000);   
		}
	});
// от начала недели   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.ac_energy" WHERE time >=' + String(week) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('AC_Week: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.ac_week", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*1000)/1000);
		}
	});
// от начала месяца   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.ac_energy" WHERE time >=' + String(month) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('AC_Month: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.ac_month", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*100)/100);
		}
	});
// от начала года   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.ac_energy" WHERE time >=' + String(year) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('AC_Year: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.ac_year", Math.round(parseFloat(((JSON.stringify(result.result[0])).slice(8, 27)))*10)/10);
		}
	});
// всего до настоящего времени   
	sendTo("influxdb.0", "query", 'SELECT sum("value") FROM "iobroker".."mqtt.0.my_solar.status.ac_energy" WHERE time <=' + String(now) + 'ms', function (result) {
		if (result.error) {
			console.error(result.error);
		} 
		else {
			//console.log('AC_Total: ' + JSON.stringify(result.result[0]));
			setState("javascript.0.energy_statistics.ac_total", Math.round(parseFloat((JSON.stringify(result.result[0])).slice(8, 27))));
		}
	});

});
