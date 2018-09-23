"use strict";

/* eslint-disable */

class HokuyoDisplay {
  constructor(parentId, mode, reinitColor = false, oneColor = null, twoColor = null) {
    this.MAIN = "main";
    this.ONE = "one";
    this.TWO = "two";

    this.dotRadius = 1; 			// cm

    this.parentId = parentId;
    this.mode = mode;
    this.div = $('#' + this.parentId);

    this.isBusy = false; 			// lock

    this.clearMainTimeout = null; 	// timeout in case we don't receive data for a while

    if (reinitColor) {
      Raphael.getColor.reset();
    }

    this.onResize();// Math.max($('body').height() - $('#div_menu').outerHeight() - 2*$('#simu_before').outerHeight(), 200);

    // On window resize (doesn't seem to work)
    // window.addEventListener('resize', this.onResize());
    // this.div.resize(this.onResize());
    // setInterval(this.onResize(), 1000);

    this.initChart(oneColor, twoColor);
  }

  onResize() {
    this.realW = 0;
    this.realH = 0;
    this.W = 0;
    this.H = 0;
    this.center = {};
    this.center.x = 0;
    this.center.y = 0;
    this.center.str = this.center.x + "," + this.center.y; // in the viewport frame
    this.viewportScale = 1;

    if (this.mode == this.MAIN) {
      var maxWidth = 1000; // px

      this.realW = 300;
      this.realH = 200;

      // if (this.W != this.div.width()) {
      this.W = 1000; //this.div.width();
      if (this.W > maxWidth) {
        this.W = maxWidth;
      }
      this.H = ( this.realH / this.realW ) * this.W;

      // this.center.x = 0; // see initChart()
      // this.center.y = 0; // see initChart()

      // }

      let scaleTable = 0.9;

      // Center
      this.center.x = Math.round(((1 - scaleTable) * this.W) / 2);
      this.center.y = Math.round(((1 - scaleTable) * this.H) / 2);

      // viewport scale : position (cm) * scale = position (pixel)
      // we keep space for the outer towers
      this.viewportScale = (scaleTable * this.W) / this.realW;
    } else {
      this.realW = 400;
      this.realH = this.realW;

      this.W = 490;
      this.H = this.W;

      this.center.x = this.W/2;
      this.center.y = this.H/2;

      this.viewportScale = (this.W/2) / this.realW;
    }

    this.center.str = this.center.x + "," + this.center.y;

    this.div.width(this.W);
    this.div.height(this.H);

    // console.log(this.viewportScale);
    // console.log(this.center.str);
    // console.log(this.W);
    // console.log(this.H);
  }

  initChart(oneColor = null, twoColor = null) {
    this.isBusy = true;

    this.r = Raphael(this.parentId, this.div.width(), this.div.height());

    var grey = Raphael.color("#333");

    if (this.mode != this.MAIN) {

      this.dotColor = Raphael.getColor(1);

      // Outmost circle
      // Unit: cm
      this.r.circle(0, 0, this.realW).attr({stroke: this.dotColor, fill: grey, transform: "t " + this.center.str + "s" + this.viewportScale, "fill-opacity": .1});

      // Inner circles
      // Unit: cm
      for (var radius = 100; radius < this.realW; radius+=100) {
        this.r.circle(0, 0, radius).attr({stroke: grey, fill: null, transform: "t " + this.center.str + "s" + this.viewportScale, "stroke-opacity": .4});
      }

      // Arraw
      // Unit: pixel
      this.r.path("M-7 10L0 -10L7 10L-7 10").attr({stroke: this.dotColor, fill: this.dotColor, transform: "t " + this.center.str, "fill-opacity": .4});

      this.dots = new Map();
    } else {
      // Init main

      if (oneColor == null
        || twoColor == null) {
          console.error("Main display must have the hokuyos Raphael colors");
          this.isBusy = false;
          return;
        }

        this.oneColor = oneColor;
        this.twoColor = twoColor;

        this.dotColor = Raphael.color("#11B100");

        // Field
        // Paper.rect(x, y, width, height, [r]) in the REAL WORLD dimensions
        // Transform : move to origin and then scale around origin
        this.r.rect(0, 0, this.realW, this.realH).attr({stroke: grey, fill: grey, transform: "t " + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0", "fill-opacity": .1, "stroke-opacity": .4});

        // Towers
        // 8*8 sqare at the corner - 2.2cm of wood board
        let xs = [ -10.2, 302.2 ];
        let ys = [ -10.2, 96,   202.2 ];
        let colors = [ Raphael.color("#2988D8"), Raphael.color("#FAFC08") ];
        let i = 0;

        // for each column (ie left & right)
        for (let col = 0; col<=1; col++){
          // for each line (ie back, middle, front)
          for (let line = 0; line<=2; line++){
            this.r.rect(xs[col], ys[line], 8, 8).attr({stroke: colors[i%2], fill: colors[i%2], transform: "t " + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0", "fill-opacity": .4, "stroke-opacity": .8});
            i++;
          }
        }

        // Hokuyos container
        this.hokuyos = {};

        // Dots container
        this.objects = [];
      }

      this.isBusy = false;
    }

    updatePolarSpots(spots) {
      if (this.mode == this.MAIN) {
        console.error("Main display can't handle polar spot");
        return;
      }

      this.isBusy = true;

      // console.log(spots);
      // For each spots
      spots.forEach(function(newSpot) {
        var existingPlot = this.dots.get(newSpot[0]);

        if (!!existingPlot) {
          // Dot already exist at this angle, let's update it
          existingPlot.attr({cy: newSpot[1] * this.viewportScale });
        } else {
          // This dot doesn't already exist, let's create it
          var dot = this.r.circle(0, newSpot[1] * this.viewportScale, this.dotRadius).attr({
            stroke: this.dotColor,
            fill: this.dotColor,
            transform:  "t," + this.center.str + "r180,0,0" + "r" + -newSpot[0] + ",0,0"});
            //  + " 0 0" +   +
            this.dots.set(newSpot[0], dot);
          }
        }.bind(this));

        this.isBusy = false;
      }

      // updateRobots(robots) {
      // 	if (this.mode != this.MAIN) {
      // 		console.error("Single LiDAR displays can't handle global robot spot");
      // 		return;
      // 	}
      // }

      updateAll(hokuyos, robots, cartesianSpots) {
        if (this.mode != this.MAIN) {
          console.error("Single LiDAR displays can't handle global robot spot");
          return;
        }

        this.isBusy = true;

        if (!!this.clearMainTimeout) {
          clearTimeout(this.clearMainTimeout);
        }

        // console.log(hokuyos);

        // For each hokuyo
        hokuyos.forEach(function(hok) {
          var existingHok = this.hokuyos[hok.name];

          if (!!existingHok) {
            existingHok.remove();
          }

          let color = hok.name == "one" ? this.oneColor : this.twoColor;
          this.hokuyos[hok.name] = this.r.path("M-5 3L5 0L-5 -3L-5 3").attr({
            stroke: color,
            fill: color,
            transform: "t " + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0" + "t " + hok.position.x + "," + hok.position.y + "r" + hok.position.w,
            "fill-opacity": .4});
          }.bind(this));

          // For each object
          for(let obj of this.objects) {
            obj.remove();
          }

          this.objects = [];

          for(let robot of robots) {
            this.objects.push( this.r.circle(robot[0], robot[1], this.dotRadius * 5 ).attr({
              stroke: this.dotColor,
              fill: this.dotColor,
              transform:  "t," + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0",
              "fill-opacity": .4}) );
              this.objects.push( this.r.text(robot[0], robot[1], "["+ parseInt(robot[0]) + "; " + parseInt(robot[1]) +"]").attr({
                fill: this.dotColor,
                "font-size": "6px",
                transform:  "t,50,25" + "t," + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0"}) );

              }

              // console.log(cartesianSpots);
              if (!!cartesianSpots) {
                for(let spot of cartesianSpots) {
                  this.objects.push( this.r.circle(spot[0], spot[1], this.dotRadius).attr({
                    stroke: this.dotColor,
                    fill: this.dotColor,
                    transform:  "t," + this.center.str + "s" + this.viewportScale + "," + this.viewportScale + ",0,0",
                    "fill-opacity": .4}) );
                  }
                }

                this.isBusy = false;

                this.clearMainTimeout = setTimeout(function() {
                  //this.clearMain();
                }.bind(this), 1000);
              }

              clearMain() {
                // For each object
                for(let hokName in this.hokuyos) {
                  this.hokuyos[hokName].remove();
                }

                // For each object
                for(let obj of this.objects) {
                  obj.remove();
                }
              }
            }



            /* eslint-enable */
