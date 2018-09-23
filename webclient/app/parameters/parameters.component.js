class ParameterController {
  constructor($timeout, Ros) {
    this.ros = Ros;
    this.$timeout = $timeout;
  }

  $onInit() {
    this.$timeout(function() {
      this.param = new ROSLIB.Param({ ros: this.ros.ros, name: this.parameter.name });
    }.bind(this), 500);
  }

  setValue(value) {
    this.param.set(value);
  }
}

angular.module('roscc').component('ccParameter', {
  bindings: { parameter: '=' },
  templateUrl: 'app/parameters/parameters.html',
  controller: ParameterController,
});
