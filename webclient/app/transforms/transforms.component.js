class TransformController {
  constructor(Ros) {
    this.ros = Ros;
  }

  $onInit() {
    this.refresh();
  }

  refresh() { // relative to fixed
    if(!this.transform || !this.transform.fixed || !this.transform.frame)
      return;

    this.tfClient = new ROSLIB.TFClient({
      ros : this.ros.ros,
      fixedFrame : this.transform.fixed,
      angularThres : 0.001,
      transThres : 0.001
    });

    this.tfClient.subscribe(this.transform.frame, function(tf) {
      let eulAngles = new THREE.Euler();

      eulAngles.setFromQuaternion(new THREE.Quaternion( tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w ))

      this.transform.response = tf
      this.transform.response.rotation = eulAngles;

    }.bind(this));
  }


}

angular.module('roscc').component('ccTransform', {
  templateUrl: 'app/transforms/transforms.html',
  controller: TransformController,
});
