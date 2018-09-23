/* eslint-disable no-undef */

class HokuyoController {
  constructor($rootScope, $scope, Hokuyo, Settings, Ros) {
    $rootScope.act_page = 'hokuyo';
    this.$scope = $scope;
    this.ros = Ros;
    this.setting = Settings.get();

    let changedTab = false;
    if (Hokuyo.displays.one != null) {
      changedTab = true;
    }

    Hokuyo.displays.one = new HokuyoDisplay("hok1", "one", true);
    Hokuyo.displays.two = new HokuyoDisplay("hok2", "two");
    Hokuyo.displays.main = new HokuyoDisplay("mainHokDisp", "main", false, Hokuyo.displays.one.dotColor, Hokuyo.displays.two.dotColor);

    if (changedTab) {
      // Restore data
      if (Hokuyo.lastData.one != null) {
        Hokuyo.displays.one.updatePolarSpots(Hokuyo.lastData.one);
      }
      if (Hokuyo.lastData.two != null) {
        Hokuyo.displays.two.updatePolarSpots(Hokuyo.lastData.two);
      }
      if (Hokuyo.lastData.main != null) {
        Hokuyo.displays.main.updateAll(Hokuyo.lastData.main.hokuyos, Hokuyo.lastData.main.robotsSpots, Hokuyo.lastData.main.cartesianSpots);
      }
    }

    this.ros.listen(this.setting.hokuyo_1, function(msg) {

      this.$scope.name = "hokuyo.polar_raw_data";
      this.$scope.from = "hokuyo";
      this.$scope.data = {
        hokuyo: "one",
        polarSpots: Hokuyo.formatPolarPoints(msg)
      };
      this.$scope.update();
    }.bind(this));

    this.ros.listen(this.setting.hokuyo_2, function(msg) {
      this.$scope.name = "hokuyo.polar_raw_data";
      this.$scope.from = "hokuyo";
      this.$scope.data = {
        hokuyo: "two",
        polarSpots: Hokuyo.formatPolarPoints(msg)
      };

      this.$scope.update();
    }.bind(this));



//     $scope.name = "hokuyo.polar_raw_data";
//     $scope.from = "hokuyo";
//     $scope.data = '{\n\
//     "hokuyo": "one",\n\
//     "polarSpots": [\n\
//     [ -90, 350 ],\n\
//     [ -30, 235 ],\n\
//     [ -35, 230 ],\n\
//     [ -25, 230 ],\n\
//     [ -20, 100 ],\n\
//     [ -15, 105 ],\n\
//     [ -5, 120 ],\n\
//     [ 0, 100 ],\n\
//     [ 5, 90 ],\n\
//     [ 10, 95 ],\n\
//     [ 15, 100 ],\n\
//     [ 20, 100 ],\n\
//     [ 25, 230 ],\n\
//     [ 30, 235 ]\n\
//   ]\n\
// }';
//
//
//     $scope.name = "lidar.all";
//     $scope.from = "lidar";
//     $scope.data = '{\n\
//       "hokuyos": [\n\
//         {\n\
//           "name": "one",\n\
//           "position": {\n\
//             "x": -6.2,\n\
//             "y": -6.2,\n\
//             "w": 0\n\
//           }\n\
//         },\n\
//         {\n\
//           "name": "two",\n\
//           "position": {\n\
//             "x": 306.2,\n\
//             "y": 100,\n\
//             "w": 180\n\
//           }\n\
//         }\n\
//       ],\n\
//       "cartesianSpots": [\n\
//         [ 20, 200 ],\n\
//         [ 30, 202 ],\n\
//         [ 40, 199 ],\n\
//         [ 148, 104 ],\n\
//         [ 145, 95 ],\n\
//         [ 153, 105 ],\n\
//         [ 155, 98 ],\n\
//         [ 95, 135 ],\n\
//         [ 100, 129 ],\n\
//         [ 101, 140 ],\n\
//         [ 105, 132 ],\n\
//         [ 104, 137 ],\n\
//         [ 230, 2 ],\n\
//         [ 234, 4 ]\n\
//       ],\n\
//       "robotsSpots": [\n\
//         [ 150, 100 ],\n\
//         [ 100, 135 ]\n\
//       ]\n\
//     }';

    $scope.update = function() {
      Hokuyo.onOrder($scope.from, $scope.name, $scope.data);
    }

    $scope.update();
  }
}


angular.module('roscc').component('ccHokuyo', {
  templateUrl: 'app/hokuyo/hokuyo.html',
  controller: HokuyoController,
});

/* eslint-enable no-undef */
