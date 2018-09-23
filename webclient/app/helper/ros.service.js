class RosService {
  constructor($rootScope, $log, $interval, $timeout, Settings, Domains) {
    $rootScope.isConnected = false;
    this.isConnected = $rootScope.isConnected;

    this.setting = Settings.get();
    this.$interval = $interval;
    this.$timeout = $timeout;
    this.$log = $log;
    this.$rootScope = $rootScope;
    this.ros = false;
    this.Domains = Domains;

    this.resetData();
    this.newRosConnection();
    $interval(() => {
      this.newRosConnection();
    }, 1000 / this.setting.refresh_rate);

    $interval(() => {
      this.loadData();
    }, 1000 / this.setting.refresh_rate);
  }

  newRosConnection(callback) {
    if (this.$rootScope.isConnected || !this.setting) {
      return;
    }

    if (this.ros) {
      this.ros.close(); // Close old connection
      this.ros = false;
      return;
    }

    this.ros = new ROSLIB.Ros({ url: `ws://${this.setting.address}:${this.setting.port}` });

    this.ros.on('connection', () => {
      this.$rootScope.isConnected = true;
      this.isConnected = this.$rootScope.isConnected;
      this.resetData();
      this.loadData();
      this.$log.log('Successfully connected to server !');
    });

    this.ros.on('error', () => {
      this.$rootScope.isConnected = false;
      this.isConnected = this.$rootScope.isConnected;
      this.ros = false;
      this.$log.log('Error trying to connect to server !');

    });

    this.ros.on('close', () => {
      this.$rootScope.isConnected = false;
      this.isConnected = this.$rootScope.isConnected;
      this.$log.log('Connection to server closed !');
    });


    if(callback) {
      this.$timeout(function() {
        callback();
      }.bind(this), 1000); // [ms]
    }
  }


  sendOrder(to, data, callback) {
    if(!this.ros) return;
    let service = new ROSLIB.Service({ ros: this.ros, name: to });
    let request = new ROSLIB.ServiceRequest(data);
    service.callService(request, callback)
  }

  publish(to, data) {
    if(!this.ros) return;
    let topic = new ROSLIB.Topic({ ros: this.ros, name: to })
    let msg = new ROSLIB.Message(data);
    topic.publish(msg);
  }

  listen(from, callback) {
    if(!this.ros) return;
    let topic = new ROSLIB.Topic({ ros: this.ros, name: from })
    topic.subscribe(callback);
    return topic;
  }

  getParam(name, callback) {
    if(!this.ros) return;
    let param = new ROSLIB.Param({ ros: this.ros, name: name })
    param.get(callback);
  }

  setParam(name, value) {
    if(!this.ros) return;
    let param = new ROSLIB.Param({ ros: this.ros, name: name })
    param.set(value);
  }



  resetData() {
    this.data = {
      rosout: [],
      topics: [],
      nodes: [],
      parameters: [],
      services: [],
      actions: []
    };

    //populate expected topics/services
    angular.forEach(this.$rootScope.domains, (d) => {

      angular.forEach(d.topics, (t) => {
        let name = '/'+d.name+'/'+t;
        let newT = {
          name: name,
          abbr: t,
          active: false,
          expected: true,
          isOpen: true
        };
        this.data.topics.push(newT);
      });

      angular.forEach(d.services, (s) => {
        let name = '/'+d.name+'/'+s;
        let newS = {
          name: name,
          abbr: s,
          active: false,
          expected: true,
          isOpen: true
        };
        this.data.services.push(newS);
      });

      angular.forEach(d.actions, (a) => {
        let name = '/'+d.name+'/'+a;
        let newA = {
          name: name,
          abbr: a,
          active: false,
          expected: true,
          isOpen: true
        };
        this.data.actions.push(newA);
      });
    });
  }

  // updates structure, all data, parameters, topics, services, nodes...
  loadData() {
    this.ros.getTopics((topics) => { // TODO: check if type is already returned here
      angular.forEach(topics.topics, (name) => {
        let topic = _.findWhere(this.data.topics, { name });

        if(!topic && (name.match("(/feedback|/goal|/status|/cancel|/result)$"))) {
          return;
        }
        if(topic) { //to update
          topic.active = true;
        } else { //to add
          topic = {
            name: name,
            active: true,
            isOpen: true
          };
          this.data.topics.push(topic);
        }

        if(!topic.fetched) {
          this.ros.getTopicType(name, (type) => {
            topic.type = type;
            topic.fetched = true;
          });
        }
      });

      for(let i = this.data.topics.length - 1; i >= 0; i--) {
        let t = this.data.topics[i];
        let found = false;
        for(let y = 0; y < topics.topics.length; y++) {
          if(topics.topics[y] == t.name)
            found = true;
        }

        if(!found) {
          if(!t.expected) {
            this.data.topics.splice(i, 1);
          } else {
            t.active = false;
          }
        }
      }
    }); // end getTopics

    this.ros.getServices((services) => {
      angular.forEach(services, (name) => {
        let service = _.findWhere(this.data.services, { name });

        if(service) { //to update
          service.active = true;
        } else { //to add
          service = {
            name: name,
            active: true,
            isOpen: true
          };
          this.data.services.push(service);
        }
        if(!service.fetched) {
          this.ros.getServiceType(name, (type) => {
            service.type = type;
            service.fetched = true;
          });
        }
      });

      for(let i = this.data.services.length - 1; i >= 0; i--) { //angular foreach not working for this
        let s = this.data.services[i];
        let found = false;
        for(let y = 0; y < services.length; y++) {
          if(services[y] == s.name)
            found = true;
        }

        if(!found) {
          if(!s.expected) {
            this.data.services.splice(i, 1);
          } else {
            s.active = false;
          }
        }
      }
    }); // end getServices


    this.ros.getActionServers((actions) => {
      angular.forEach(actions, (name) => {
        let action = _.findWhere(this.data.actions, { name });

        if(action) { //to update
          action.active = true;
        } else { //to add
          action = {
            name: name,
            active: true,
            isOpen: true
          };
          this.data.actions.push(action);
        }
        if(!action.fetched) {
          this.ros.getTopicType(name+'/goal', (type) => {
            action.type = type.slice(0, -4);
            action.fetched = true;
          });
        }
      });

      for(let i = this.data.actions.length - 1; i >= 0; i--) { //angular foreach not working for this
        let a = this.data.actions[i];
        let found = false;
        for(let y = 0; y < actions.length; y++) {
          if(actions[y] == a.name)
            found = true;
        }

        if(!found) {
          if(!a.expected) {
            this.data.actions.splice(i, 1);
          } else {
            a.active = false;
          }
        }
      }
    }); // end getActionServerss

    this.ros.getParams((params) => { //TODO : update like topics
      angular.forEach(params, (name) => {

        let param = _.findWhere(this.data.parameters, { name });
        if(!param) {
          param = { name }
          this.data.parameters.push(param);
        }

        if(!param.fetched) {
          const rosparam = new ROSLIB.Param({ ros: this.ros, name });
          rosparam.get((value) => {
            param.value = value;
            param.fetched = true;
          });
        }
      });

      for(let i = this.data.parameters.length - 1; i >= 0; i--) { //angular foreach not working for this
        let p = this.data.parameters[i];

        if(!_.contains(params, p.name)) {
            this.data.parameters.splice(i, 1);
        }
      }
    });

    this.ros.getNodes((nodes) => { //TODO : update like topics
      this.data.nodes = [];
      angular.forEach(nodes, (name) => {
        this.data.nodes.push({ name });
      });
    });
  }


  getDomains() {
    if(!this.data)
      return;
    const allData = this.data.topics.concat(this.data.services, this.data.nodes, this.data.parameters);
    const domains = this.Domains.getDomains(allData);

    let expectedD = _.pluck(this.$rootScope.domains, 'name');

    //set expected domains first in the list
    return _.uniq(expectedD.concat(domains));
  }

  getExpectedDomains() {
    return _.pluck(this.$rootScope.domains, 'name');
  }

  getUnexpectedDomains() {
    return _.difference(this.getDomains(), this.getExpectedDomains());
  }

  getTopicsForDomain(domain) {
    return this.Domains.getDataForDomain(this.data.topics, domain, false);
  }

  getServicesForDomain(domain) {
    return this.Domains.getDataForDomain(this.data.services, domain, false);
  }

  getActionsForDomain(domain) {
    return this.Domains.getDataForDomain(this.data.actions, domain, false);
  }

  getGlobalParameters() {
    return this.Domains.getGlobalParameters(this.data.parameters);
  }



}

angular.module('roscc').service('Ros', RosService);
