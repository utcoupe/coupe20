class ServiceController {
  constructor($scope, $http, $timeout, Ros) {
    this.$scope = $scope;
    this.$http = $http;
    this.ros = Ros;
    this.$timeout = $timeout;
    this.flashState = -1;
  }

  $onInit() {

    this.$scope.$watchGroup(['$ctrl.service.type', '$ctrl.service.active'], () => {
      this.setFileName();
    }, () => {});
  }

  setFileName() {
    const path = 'app/services/';


    this.fileName = `${path}default.html`;


    if(!this.service.active) {
      this.fileName = '';
      this.service.isOpen = false;
      return;
    }
    else if (!this.service.type) {
      this.fileName = `${path}default.html`;
      return;
    }

    const fileName = `${path}${this.service.type}.html`;
    this.$http.get(fileName).then((result) => {
      if (result.data) {
        this.fileName = fileName;
      }
    }, () => {});
  }

  callService(input, isJSON) {
    //(!this.service.active)
    //  return;

    const data = isJSON ? angular.fromJson(input) : input;
    const ROSservice = new ROSLIB.Service({
      ros: this.ros.ros,
      name: this.service.name,
      serviceType: this.service.type,
    });
    const request = new ROSLIB.ServiceRequest(data);

    this.flashState = -1;

    ROSservice.callService(request, (result) => {
      this.result = result;
      this.flashState = -1;
      // -1 : no flash
      // 0 : returned some object
      // 1 : return some object with a success = true
      // 2 : return some object with a success = false

      //assume this is the success state
      if(_.keys(result).length == 1 && typeof result[_.keys(result)[0]] === 'boolean') {
        let success = result[_.keys(result)[0]];
        if(success)
          this.flashState = 1
        else
          this.flashState = 2
      } else {
        this.flashState = 0
      }
      if(this.flashBackPromise)
        this.$timeout.cancel(this.flashBackPromise);

      this.flashBackPromise = this.$timeout(() => { this.flashState = -1 }, 2000);
    });
  }
}

angular.module('roscc').component('ccService', {
  bindings: { service: '=' },
  template: `<ng-include src="'./app/services/meta.html'"></ng-include>`,
  controller: ServiceController,
});
