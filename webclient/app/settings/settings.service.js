class SettingsService {
  constructor($location, localStorageService) {
    this.$location = $location;
    this.localStorageService = localStorageService;
  }

  load() {
    this.index = this.localStorageService.get('selectedSettingIndex');
    this.settings = this.localStorageService.get('settings');
    if (this.settings && this.index) {
      this.setting = this.settings[this.index];
    }

    // If there are no saved settings, redirect to /settings for first setting input
    if (!this.setting) {
      this.$location.path('/settings').replace();
    }
  }

  save(newSettings, newIndex) {
    this.settings = newSettings;
    this.index = newIndex;
    this.localStorageService.set('selectedSettingIndex', newIndex);
    this.localStorageService.set('settings', newSettings);
  }

  get() {
    if (!this.setting) {
      this.load();
    }

    return this.setting;
  }

  getIndex() {
    if (!this.setting) {
      this.load();
    }

    return this.index;
  }

  getSettings() {
    if (!this.setting) {
      this.load();
    }

    return this.settings;
  }

  getDefaultSetting() {
    return {
      name: 'Robot Name',
      address: window.location.hostname,
      port: 9090, // default port of rosbridge_server
      log: '/rosout',
      advanced: false,
      hokuyo_1: '/sensors/hokuyo_1_raw',
      hokuyo_2: '/sensors/hokuyo_2_raw',
      maxConsoleEntries: 1000,
      refresh_rate: 1,
      log_level: 2
    };
  }
}


angular.module('roscc').service('Settings', SettingsService);
