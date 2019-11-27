function ROSCCConfig($routeProvider, localStorageServiceProvider) {
  $routeProvider
    .when('/diagnostic',   { template: '<cc-diagnostic></cc-diagnostic>' })
    .when('/asserv',       { template: '<cc-asserv></cc-asserv>' })
    .when('/hokuyo',       { template: '<cc-hokuyo></cc-hokuyo>' })
    .when('/telecommande', { template: '<cc-control></cc-control>' })
    .when('/settings',     { template: '<cc-settings></cc-settings>' })
    .otherwise({ redirectTo: '/diagnostic' });

  localStorageServiceProvider
    .setPrefix('roscc');
}

function run($rootScope) {

  $rootScope.domains = [
    {
      name: 'drivers',
      topics: ['ard_asserv/pose2d',
               'ard_asserv/speed'],
      services: ['ard_asserv/emergency_stop',
                 'ard_asserv/goto',
                 'ard_asserv/set_pos',
                 'ard_asserv/speed',
                 'ard_asserv/pwm',
                 'ard_asserv/management',
                 'ard_asserv/parameters'],
      actions: ['ard_asserv/goto_action']
    },
    {
      name: 'ai',
      topics: [],
      services: [],
      actions: []
    },
    {
      name: "recognition",
      topics: [],
      services: [],
      actions: []
    },
    {
      name: "processing",
      topics: [],
      services: [],
      actions: []
    },
    {
      name: "sensors",
      topics: [],
      services: [],
      actions: []
    },
    {
      name: "movement",
      topics: [],
      services: [],
      actions: []
    },
    {
      name: "memory",
      topics: [],
      services: [],
      actions: []
    },
    {
      name: "feedback",
      topics: [],
      services: [],
      actions: []
    },
    {
      name: "navigation",
      topics: [],
      services: [],
      actions: []
    }
  ];
}


angular.module('roscc', ['ngRoute', 'ui.bootstrap', 'LocalStorageModule', 'ngAnimate']).config(ROSCCConfig).run(run);
