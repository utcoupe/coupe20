class AsservController {
  constructor (Ros, $interval) {
    this.ros = Ros.ros;
    this.$interval = $interval;


    Ros.listen('/drivers/ard_asserv/pose2d', function (e) {
      this.pushDataToChart(2, e.x);
      this.pushDataToChart(3, e.theta);
      this.pushDataToChart(4, e.y);
    }.bind(this))

    Ros.listen('/drivers/ard_asserv/speed', function (e) {
      this.pushDataToChart(0, e.pwm_speed_left);
      this.pushDataToChart(1, e.pwm_speed_right);
      this.pushDataToChart(5, e.wheel_speed_left);
      this.pushDataToChart(7, e.wheel_speed_right);
      this.pushDataToChart(6, e.linear_speed);

    }.bind(this))

    this.charts = []

    for(let i = 0; i < 8; i++) {
      let c = new SmoothieChart({grid:{verticalSections:4}, labels:{fontSize:20}, responsive: true,  tooltip:true});
      let ts = new TimeSeries();

      c.addTimeSeries(ts, { maxValueScale:1.32, minValueScale:1.32, strokeStyle: 'rgba(255, 0, 0, 1)', fillStyle: 'rgba(255, 0, 0, 0.2)', lineWidth: 4 });
      c.streamTo(document.getElementById("line"+(i+1)), 10);

      this.charts.push({chart: c, data: ts});



    }


  }

  pushDataToChart(i, e) {
    this.charts[i].data.append(new Date().getTime(), e)
  }


  $onInit ($interval) {
    var canvas = document.getElementsByTagName('canvas')
    for (let c of canvas) { fitToContainer(c) }

    function fitToContainer (canvas) {
      // Make it visually fill the positioned parent
      canvas.style.width = '100%'
      canvas.style.height = '100%'
      // ...then set the internal size to match
      canvas.width = canvas.offsetWidth
      canvas.height = canvas.offsetHeight
    }

  }
}

angular.module('roscc').component('ccAsserv', {
  templateUrl: 'app/asserv/asserv.html',
  controller: AsservController
})
