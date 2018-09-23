
class DiagnosticController {
  constructor($rootScope, $timeout, $interval, Settings, Domains, Ros, Console) {
    this.$timeout = $timeout;
    this.Domains = Domains;
    this.domains = $rootScope.domains;
    this.logs = Console.logs;

    this.ros = Ros;
    this.setting = Settings.get();


    if($rootScope.isConnected) {
      this.$timeout(() => { this.onConnected(); }, 1000);
    } else {
      $rootScope.$watch('isConnected', function(newValue) {
        if(newValue)
          this.$timeout(() => { this.onConnected(); }, 1000);
      }.bind(this));
    }

  }

  // The active domain shows further information in the center view
  setActiveDomain(domain) {
    this.activeDomain = domain;
  }


  onConnected() {
    if (!this.activeDomain) {
      for(let d of this.domains) {
        if(this.ros.getDomains().includes(d.name)) {
          this.setActiveDomain(d.name);
        }
      }
    }
  }



  refresh() {
    this.ros.loadData();
  }

  isDomainActive(domain) {
    return _.some(this.ros.getTopicsForDomain(domain), (t) => t.active == true);
  }

  collapseAll(domain) {
    this.ros.getTopicsForDomain(domain).map(function(item){
      item.isOpen = false;
    });
  }

  expandAll(domain) {
    this.ros.getTopicsForDomain(domain).map(function(item){
      if(item.type)
        item.isOpen = true;
    });
  }
}

angular.module('roscc').component('ccDiagnostic', {
  templateUrl: 'app/diagnostic/diagnostic.html',
  controller: DiagnosticController,
});
