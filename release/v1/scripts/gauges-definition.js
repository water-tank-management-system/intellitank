// Create Temperature Gauge
function createTemperatureGauge() {
  var gauge = new LinearGauge({
    renderTo: 'gauge-temperature',
    width: 120,
    height: 400,
    units: 'Temperature C',
    minValue: 0,
    startAngle: 90,
    ticksAngle: 180,
    maxValue: 40,
    colorValueBoxRect: '#049faa',
    colorValueBoxRectEnd: '#049faa',
    colorValueBoxBackground: '#f1fbfc',
    valueDec: 2,
    valueInt: 2,
    majorTicks: ['0', '5', '10', '15', '20', '25', '30', '35', '40'],
    minorTicks: 4,
    strokeTicks: true,
    highlights: [
      {
        from: 30,
        to: 40,
        color: 'red',
      },
    ],
    colorPlate: '#fff',
    colorBarProgress: '#CC2936',
    colorBarProgressEnd: '#049faa',
    borderShadowWidth: 0,
    borders: false,
    needleType: 'arrow',
    needleWidth: 2,
    needleCircleSize: 7,
    needleCircleOuter: true,
    needleCircleInner: false,
    animationDuration: 1500,
    animationRule: 'linear',
    barWidth: 10,
  });
  return gauge;
}

// Create WaterLevel Gauge
function createWaterLevelGauge() {
  var gauge = new RadialGauge({
    renderTo: 'gauge-water-level',
    width: 300,
    height: 300,
    units: 'Water Level (%)',
    minValue: 0,
    maxValue: 100,
    colorValueBoxRect: '#049faa',
    colorValueBoxRectEnd: '#049faa',
    colorValueBoxBackground: '#f1fbfc',
    valueInt: 2,
    majorTicks: ['0', '20', '40', '60', '80', '100'],
    minorTicks: 4,
    strokeTicks: true,
    highlights: [
      {
        from: 0,
        to: 25,
        color: 'red',
      },
    ],
    colorPlate: '#fff',
    borderShadowWidth: 0,
    borders: false,
    needleType: 'line',
    colorNeedle: '#007F80',
    colorNeedleEnd: '#007F80',
    needleWidth: 2,
    needleCircleSize: 3,
    colorNeedleCircleOuter: '#007F80',
    needleCircleOuter: true,
    needleCircleInner: false,
    animationDuration: 1500,
    animationRule: 'linear',
  });
  return gauge;
}

// Create Turbidity Gauge
function createTurbidityGauge() {
  var gauge = new LinearGauge({
    renderTo: 'gauge-turbidity',
    width: 120,
    height: 400,
    units: 'Turbidity NTU',
    minValue: 0,
    startAngle: 90,
    ticksAngle: 180,
    maxValue: 1000,
    colorValueBoxRect: '#049faa',
    colorValueBoxRectEnd: '#049faa',
    colorValueBoxBackground: '#f1fbfc',
    valueDec: 2,
    valueInt: 2,
    majorTicks: ['0', '100', '200', '300', '400', '500', '600', '700', '800', '900', '1000'],
    minorTicks: 4,
    strokeTicks: true,
    highlights: [
      {
        from: 30,
        to: 1000,
        color: 'red',
      },
    ],
    colorPlate: '#fff',
    colorBarProgress: '#CC2936',
    colorBarProgressEnd: '#049faa',
    borderShadowWidth: 0,
    borders: false,
    needleType: 'arrow',
    needleWidth: 2,
    needleCircleSize: 7,
    needleCircleOuter: true,
    needleCircleInner: false,
    animationDuration: 1500,
    animationRule: 'linear',
    barWidth: 10,
  });
  return gauge;
}