const gulp = require('gulp');
const babel = require('gulp-babel');
const concat = require('gulp-concat');

function swallowError(error) {
    console.log(error.toString());

    this.emit('end');
}

// Babel then concat all app js files into roscc.js
gulp.task('js', function () {
    return gulp.src(['app/app.js', 'app/**/*.js'])
        .pipe(babel({presets: ['env']}))
        .pipe(concat('roscc.js'))
        .pipe(gulp.dest('assets/js/'))
        .on('error', swallowError);
});

// Concat all libs into vendor.js
gulp.task('js-vendor', function () {
    return gulp.src([
        'node_modules/underscore/underscore-min.js',
        'node_modules/angular/angular.min.js',
        'node_modules/angular-animate/angular-animate.min.js',
        'node_modules/angular-route/angular-route.min.js',
        'node_modules/angular-local-storage/dist/angular-local-storage.js',
        'node_modules/angular-ui-bootstrap/dist/ui-bootstrap-tpls.js',
        'node_modules/eventemitter2/lib/eventemitter2.js',
        'node_modules/jquery/dist/jquery.min.js',
        'node_modules/smoothie/smoothie.js',
        'node_modules/roslib/build/roslib.min.js'
    ])
        .pipe(concat('vendor.js'))
        .pipe(gulp.dest('assets/js/'));
});

// Changes will be detected automatically
gulp.task('watch', function () {
    gulp.watch('app/**/*.js', ['js']);
});

gulp.task('default', ['js-vendor', 'js', 'watch']);
