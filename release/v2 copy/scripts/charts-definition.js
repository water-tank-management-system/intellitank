// Create the charts when the web page loads
window.addEventListener('load', onload);

function onload(event) {
  chartT = createTemperatureChart();
  chartL = createWaterLevelChart();
  chartF = createWaterFlowChart();
  chartN = createTurbidityChart();
}

// Create Temperature Chart
function createTemperatureChart() {
  var chart = new Highcharts.Chart({
    chart: {
      renderTo: 'chart-temperature',
      type: 'spline',
    },
    series: [
      {
        name: 'Temperature - NTC Sensor',
      },
    ],
    title: {
      text: undefined,
    },
    plotOptions: {
      line: {
        animation: false,
        dataLabels: {
          enabled: true,
        },
      },
    },
    xAxis: {
      type: 'datetime',
      dateTimeLabelFormats: { second: '%H:%M:%S' },
    },
    yAxis: {
      title: {
        text: 'Temperature Celsius Degrees',
      },
    },
    credits: {
      enabled: false,
    },
  });
  return chart;
}

// Create WaterLevel Chart
function createWaterLevelChart() {
  var chart = new Highcharts.Chart({
    chart: {
      renderTo: 'chart-water-level',
      type: 'spline',
    },
    series: [
      {
        name: 'Water Level',
      },
    ],
    title: {
      text: undefined,
    },
    plotOptions: {
      line: {
        animation: false,
        dataLabels: {
          enabled: true,
        },
      },
    },
    xAxis: {
      type: 'datetime',
      dateTimeLabelFormats: { second: '%H:%M:%S' },
    },
    yAxis: {
      title: {
        text: 'Height (%)',
      },
    },
    credits: {
      enabled: false,
    },
  });
  return chart;
}

// Create WaterFlow Chart
function createWaterFlowChart() {
  var chart = new Highcharts.Chart({
    chart: {
      renderTo: 'chart-water-flow',
      type: 'spline',
    },
    series: [
      {
        name: 'Water Flow',
      },
    ],
    title: {
      text: undefined,
    },
    plotOptions: {
      line: {
        animation: false,
        dataLabels: {
          enabled: true,
        },
      },
    },
    xAxis: {
      type: 'datetime',
      dateTimeLabelFormats: { second: '%H:%M:%S' },
    },
    yAxis: {
      title: {
        text: 'Flow L/min',
      },
    },
    credits: {
      enabled: false,
    },
  });
  return chart;
}

// Create Turbidity Chart
function createTurbidityChart() {
  var chart = new Highcharts.Chart({
    chart: {
      renderTo: 'chart-turbidity',
      type: 'spline',
    },
    series: [
      {
        name: 'Turbidity',
      },
    ],
    title: {
      text: undefined,
    },
    plotOptions: {
      line: {
        animation: false,
        dataLabels: {
          enabled: true,
        },
      },
    },
    xAxis: {
      type: 'datetime',
      dateTimeLabelFormats: { second: '%H:%M:%S' },
    },
    yAxis: {
      title: {
        text: 'Turbidity NTU',
      },
    },
    credits: {
      enabled: false,
    },
  });
  return chart;
}