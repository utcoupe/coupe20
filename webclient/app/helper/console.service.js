class ConsoleService {

  constructor(Ros, Settings, $rootScope) {
    this.ros = Ros;
    this.setting = Settings.get();

    this.logs = [];
    $rootScope.$watch('isConnected', () => {
      if($rootScope.isConnected)
        this.setConsole();
      else if(this.consoleTopic)
        this.consoleTopic.unsubscribe();
      });
  }

  // Setup of console (in the right sidebar)
  setConsole() {
    this.consoleTopic = new ROSLIB.Topic({
      ros: this.ros.ros,
      name: this.setting.log,
      messageType: 'rosgraph_msgs/Log',
    });
    this.consoleTopic.subscribe((message) => {
      const nameArray = message.name.split('/');
      const d = new Date((message.header.stamp.secs * 1E3) + (message.header.stamp.nsecs * 1E-6));

      message.abbr = (nameArray.length > 1) ? nameArray[1] : message.name;

      // String formatting of message time and date
      function addZero(i) { return i < 10 ? `0${i}` : `${i}`; }
      message.dateString = `${addZero(d.getHours())}:
      ${addZero(d.getMinutes())}:
      ${addZero(d.getSeconds())}`;

      this.logs.unshift(message);

      if (this.logs.length > this.setting.maxConsoleEntries) {
        this.logs.pop();
      }

    });
  }


}

angular.module('roscc').service('Console', ConsoleService);
