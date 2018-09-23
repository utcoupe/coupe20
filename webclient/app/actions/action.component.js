class ServiceController {
  constructor($scope, $http, $timeout, Ros) {
    this.$scope = $scope;
    this.$http = $http;
    this.ros = Ros;
    this.$timeout = $timeout;
    this.status = -1;
  }

  $onInit() {

    this.$scope.$watchGroup(['$ctrl.action.type', '$ctrl.action.active'], () => {
      this.setFileName();
      this.client = new ROSLIB.ActionClient({
        ros : this.ros.ros,
        serverName : this.action.name,
        actionName : this.action.type
      });
    }, () => {});
  }

  sendGoal(goalmsg) {



    var goal = new ROSLIB.Goal({
      actionClient : this.client,
      goalMessage : goalmsg
    });
    this.goal = goal;
    goal.send();
  }

  setFileName() {
    const path = 'app/actions/';


    this.fileName = `${path}default.html`;


    if(!this.action.active) {
      this.fileName = '';
      this.action.isOpen = false;
      return;
    }
    else if (!this.action.type) {
      this.fileName = `${path}default.html`;
      return;
    }

    const fileName = `${path}${this.action.type}.html`;
    this.$http.get(fileName).then((result) => {
      if (result.data) {
        this.fileName = fileName;
      }
    }, () => {});
  }
}

angular.module('roscc').component('ccAction', {
  bindings: { action: '=' },
  template: `<ng-include src="'./app/actions/meta.html'"></ng-include>`,
  controller: ServiceController,
});
