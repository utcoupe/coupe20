class TopicController {
  constructor($scope, $http, Settings, Quaternions, Ros) {
    this.$scope = $scope;
    this.$http = $http;
    this.setting = Settings.get();
    this.Quaternions = Quaternions;
    this.ros = Ros;
    this.isSubscribing = false;
    this.toggle = true;
    this.isOpen = true;
  }

  $onInit() {
    this.roslibTopic = new ROSLIB.Topic({
      ros : this.ros.ros,
      name: this.topic.name,
      messageType: this.topic.type,
    });

    const path = 'app/topics/';
    this.fileName = `${path}default.html`;

    this.$scope.$watchGroup(['$ctrl.topic.type', '$ctrl.topic.active'], () => {
      this.isSubscribing = false;
      this.toggle = false;

      if(!this.topic.active) {
        this.fileName = '';
        this.topic.isOpen = false;
        return;
      }

      else if (!this.topic.type) {
        this.fileName = `${path}default.html`;
        //this.toggleSubscription(false);
        return;
      }

      const fileName = `${path}${this.topic.type}.html`;
      this.$http.get(fileName).then((result) => {
        if (result.data) {
          this.fileName = fileName;
          //this.toggleSubscription(false);
        }
      }, () => {});
    });
  }

  toggleSubscription(data) {
    if(!this.topic.active)
      return;
    if (!data) {
      this.roslibTopic.subscribe((message) => {
        if(!this.fileName.includes('default.html'))
          this.message = message;
        else
          this.message = JSON.stringify(message);
      });
    } else {
      this.roslibTopic.unsubscribe();
    }
    this.isSubscribing = !data;

  }

  publishMessage(input) {
    const data = !this.fileName.includes('default.html') ? angular.fromJson(input) : input;
    const message = new ROSLIB.Message(data);
    this.roslibTopic.publish(message);
  }

}

angular.module('roscc').component('ccTopic', {
  bindings: { topic: '=' },
  template: `<ng-include src="'./app/topics/meta.html'"></ng-include>`,
  controller: TopicController
});
