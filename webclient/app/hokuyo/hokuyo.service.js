angular.module('roscc').service('Hokuyo', ['$rootScope', '$sce', 'Ros',
function() {
  this.displays = {
    main: null,
    one: null,
    two: null
  };

  this.lastData = {
    one: null,
    two: null,
    main: null
  };

  // format laserscan ros msg to a list of lists [theta, r], theta in degrees
  // r in cm
  this.formatPolarPoints = function(msg) {
    let amin = msg.angle_min;
    let ainc = msg.angle_increment;


    let acurr = amin;
    let out = [];
    for(let r of msg.ranges) {
      out.push([(acurr * 180 / Math.PI )%360, r * 100]);
      acurr += ainc;
    }

    return out;
  }

  this.init = function () {
    //Client.order(this.onOrder); //FIXME
  };

  this.onOrder = function (from, name, data) {
    // if($rootScope.act_page == 'hokuyo') {
    if (name == 'hokuyo.polar_raw_data') {
      if (this.displays[data.hokuyo]) {
        // Save for later
        this.lastData[data.hokuyo] = data.polarSpots;

        // console.log("Received data from " + data.hokuyo);

        // Show
        if (!this.displays[data.hokuyo].isBusy) {
          this.displays[data.hokuyo].updatePolarSpots(data.polarSpots);
        }
      }
    } else if (name == 'lidar.all'
    && !!this.displays.main) {
      // Save for later
      this.lastData.main = {};
      this.lastData.main.hokuyos = data.hokuyos;
      this.lastData.main.robotsSpots = data.robotsSpots;
      this.lastData.main.cartesianSpots = data.cartesianSpots;

      // console.log("Received all");

      // Show
      if (!this.displays.main.isBusy) {
        this.displays.main.updateAll(data.hokuyos, data.robotsSpots, data.cartesianSpots);
      }
    } else if (name == 'lidar.light'
    && !!this.displays.main) {
      // Save for later
      this.lastData.main = {};
      this.lastData.main.hokuyos = data.hokuyos;
      this.lastData.main.robotsSpots = data.robotsSpots;

      // console.log("Received all");

      // Show
      if (!this.displays.main.isBusy) {
        this.displays.main.updateAll(data.hokuyos, data.robotsSpots, null);
      }
    } else if (name == 'lidar.robots'
    && !!this.displays.main) {
      // this.displays.main.updateRobots(data.robots);
    }
    // }
  }.bind(this);
}]);
