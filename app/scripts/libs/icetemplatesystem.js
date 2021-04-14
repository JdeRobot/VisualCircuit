/*jshint unused:false*/
'use strict';
var VzTemplateSystem = function () {
    this.render = function (name, template, view, parse) {
        Mustache.parse(template);
        let r = Mustache.render(template, view);
        return r;
    };
};
