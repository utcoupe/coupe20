class NavbarController {
  constructor($location, Ros) {
    this.$location = $location;
    this.ros = Ros;
  }

  isPath(path) {
    return this.$location.path() === path;
  }
}

angular.module('roscc').component('ccNavbar', {
  templateUrl: 'app/navbar/navbar.html',
  controller: NavbarController,
});
