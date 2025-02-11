// Get current sensor readings when the page loads  
window.addEventListener('load', getReadings);

var animationDurationms = 50;



// Create LinearSum Gauge
var gaugeLinearSumxyz = new LinearGauge({
  renderTo: 'gauge-LinearSum',
  width: 120,
  height: 400,
  units: "Sum g",
  minValue: 0,
  startAngle: 90,
  ticksAngle: 180,
  maxValue: 50,
  barBeginCircle: 0,
  colorValueBoxRect: "#049faa",
  colorValueBoxRectEnd: "#049faa",
  colorValueBoxBackground: "#f1fbfc",
  valueDec: 2,
  valueInt: 2,
  majorTicks: [
      "0",
      "5",
      "10",
      "15",
      "20",
      "25",
      "30",
      "35",
      "40",
      "45",
      "50"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": 30,
          "to": 40,
          "color": "rgba(200, 50, 50, .75)",
          "from": 40,
          "to": 50,
          "color": "rgba(250, 20, 10, .75)"
      }
  ],
  colorPlate: "#fff",
  colorBarProgress: "#CC2936",
  colorBarProgressEnd: "#049faa",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  needleWidth: 2,
  needleCircleSize: 7,
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: animationDurationms,  //1500
  animationRule: "linear",
  barWidth: 10,
}).draw();

// Create LinearSum Gauge
var gaugeLinearSumxyz10 = new LinearGauge({
  renderTo: 'gauge-LinearSum10',
  width: 120,
  height: 400,
  units: "Sum g",
  minValue: 0,
  startAngle: 90,
  ticksAngle: 180,
  maxValue: 50,
  barBeginCircle: 0,
  colorValueBoxRect: "#049faa",
  colorValueBoxRectEnd: "#049faa",
  colorValueBoxBackground: "#f1fbfc",
  valueDec: 2,
  valueInt: 2,
  majorTicks: [
      "0",
      "5",
      "10",
      "15",
      "20",
      "25",
      "30",
      "35",
      "40",
      "45",
      "50"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": 30,
          "to": 40,
          "color": "rgba(200, 50, 50, .75)",
          "from": 40,
          "to": 50,
          "color": "rgba(250, 20, 10, .75)"
      }
  ],
  colorPlate: "#fff",
  colorBarProgress: "#CC2936",
  colorBarProgressEnd: "#049faa",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  needleWidth: 2,
  needleCircleSize: 7,
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: animationDurationms,  //1500
  animationRule: "linear",
  barWidth: 10,
}).draw();

// Create LinearSum Gauge
var gaugeLinearSumxyz20 = new LinearGauge({
  renderTo: 'gauge-LinearSum20',
  width: 120,
  height: 400,
  units: "Sum g",
  minValue: 0,
  startAngle: 90,
  ticksAngle: 180,
  maxValue: 50,
  barBeginCircle: 0,
  colorValueBoxRect: "#049faa",
  colorValueBoxRectEnd: "#049faa",
  colorValueBoxBackground: "#f1fbfc",
  valueDec: 2,
  valueInt: 2,
  majorTicks: [
      "0",
      "5",
      "10",
      "15",
      "20",
      "25",
      "30",
      "35",
      "40",
      "45",
      "50"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": 30,
          "to": 40,
          "color": "rgba(200, 50, 50, .75)",
          "from": 40,
          "to": 50,
          "color": "rgba(250, 20, 10, .75)"
      }
  ],
  colorPlate: "#fff",
  colorBarProgress: "#CC2936",
  colorBarProgressEnd: "#049faa",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  needleWidth: 2,
  needleCircleSize: 7,
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: animationDurationms,  //1500
  animationRule: "linear",
  barWidth: 10,
}).draw();

// Create LinearSum Gauge
var gaugeLinearSumxyz30 = new LinearGauge({
  renderTo: 'gauge-LinearSum30',
  width: 120,
  height: 400,
  units: "Sum g",
  minValue: 0,
  startAngle: 90,
  ticksAngle: 180,
  maxValue: 50,
  barBeginCircle: 0,
  colorValueBoxRect: "#049faa",
  colorValueBoxRectEnd: "#049faa",
  colorValueBoxBackground: "#f1fbfc",
  valueDec: 2,
  valueInt: 2,
  majorTicks: [
      "0",
      "5",
      "10",
      "15",
      "20",
      "25",
      "30",
      "35",
      "40",
      "45",
      "50"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": 30,
          "to": 40,
          "color": "rgba(200, 50, 50, .75)",
          "from": 40,
          "to": 50,
          "color": "rgba(250, 20, 10, .75)"
      }
  ],
  colorPlate: "#fff",
  colorBarProgress: "#CC2936",
  colorBarProgressEnd: "#049faa",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  needleWidth: 2,
  needleCircleSize: 7,
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: animationDurationms,  //1500
  animationRule: "linear",
  barWidth: 10,
}).draw();

// Create LinearSum Gauge
var gaugeLinearSumxyz40 = new LinearGauge({
  renderTo: 'gauge-LinearSum40',
  width: 120,
  height: 400,
  units: "Sum g",
  minValue: 0,
  startAngle: 90,
  ticksAngle: 180,
  maxValue: 50,
  barBeginCircle: 0,
  colorValueBoxRect: "#049faa",
  colorValueBoxRectEnd: "#049faa",
  colorValueBoxBackground: "#f1fbfc",
  valueDec: 2,
  valueInt: 2,
  majorTicks: [
      "0",
      "5",
      "10",
      "15",
      "20",
      "25",
      "30",
      "35",
      "40",
      "45",
      "50"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": 30,
          "to": 40,
          "color": "rgba(200, 50, 50, .75)",
          "from": 40,
          "to": 50,
          "color": "rgba(250, 20, 10, .75)"
      }
  ],
  colorPlate: "#fff",
  colorBarProgress: "#CC2936",
  colorBarProgressEnd: "#049faa",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  needleWidth: 2,
  needleCircleSize: 7,
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: animationDurationms,  //1500
  animationRule: "linear",
  barWidth: 10,
}).draw();

// Create LinearSum Gauge
var gaugeLinearSumxyz50 = new LinearGauge({
  renderTo: 'gauge-LinearSum50',
  width: 120,
  height: 400,
  units: "Sum g",
  minValue: 0,
  startAngle: 90,
  ticksAngle: 180,
  maxValue: 50,
  barBeginCircle: 0,
  colorValueBoxRect: "#049faa",
  colorValueBoxRectEnd: "#049faa",
  colorValueBoxBackground: "#f1fbfc",
  valueDec: 2,
  valueInt: 2,
  majorTicks: [
      "0",
      "5",
      "10",
      "15",
      "20",
      "25",
      "30",
      "35",
      "40",
      "45",
      "50"
  ],
  minorTicks: 4,
  strokeTicks: true,
  highlights: [
      {
          "from": 30,
          "to": 40,
          "color": "rgba(200, 50, 50, .75)",
          "from": 40,
          "to": 50,
          "color": "rgba(250, 20, 10, .75)"
      }
  ],
  colorPlate: "#fff",
  colorBarProgress: "#CC2936",
  colorBarProgressEnd: "#049faa",
  borderShadowWidth: 0,
  borders: false,
  needleType: "arrow",
  needleWidth: 2,
  needleCircleSize: 7,
  needleCircleOuter: true,
  needleCircleInner: false,
  animationDuration: animationDurationms,  //1500
  animationRule: "linear",
  barWidth: 10,
}).draw();

// Function to get current readings on the webpage when it loads for the first time
function getReadings(){
  var xhr = new XMLHttpRequest();
  xhr.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      var myObj = JSON.parse(this.responseText);
      console.log(myObj); 
     var sum = myObj.Sumaccel;
     gaugeLinearSumxyz.value = tempsum;
     var sum10 = myObj.Sumaccel10;
     gaugeLinearSumxyz10.value = tempsum10;
     var sum20 = myObj.Sumaccel20;
     gaugeLinearSumxyz20.value = tempsum20;
     var sum30 = myObj.Sumaccel30;
     gaugeLinearSumxyz30.value = tempsum30;
     var sum40 = myObj.Sumaccel40;
     gaugeLinearSumxyz40.value = tempsum40;
     var sum50 = myObj.Sumaccel50;
     gaugeLinearSumxyz50.value = tempsum50;
      
    }
  }; 
  xhr.open("GET", "/readings", true);
  xhr.send();
}

if (!!window.EventSource) {
  var source = new EventSource('/events');
  
  source.addEventListener('open', function(e) {
    console.log("Events Connected");
  }, false);

  source.addEventListener('error', function(e) {
    if (e.target.readyState != EventSource.OPEN) {
      console.log("Events Disconnected");
    }
  }, false);
  
  source.addEventListener('message', function(e) {
    console.log("message", e.data);
  }, false);
  
  

  source.addEventListener('accelerometer_readings', function(e) {
    console.log("accelerometer_readings", e.data);
    var obj = JSON.parse(e.data);
   
    gaugeLinearSumxyz.value = obj.Sumaccel;
    gaugeLinearSumxyz10.value = obj.Sumaccel10;
    gaugeLinearSumxyz20.value = obj.Sumaccel20;
    gaugeLinearSumxyz30.value = obj.Sumaccel30;
    gaugeLinearSumxyz40.value = obj.Sumaccel40;
    gaugeLinearSumxyz50.value = obj.Sumaccel50;
  }, false);

 

}