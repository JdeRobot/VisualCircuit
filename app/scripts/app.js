'use strict';

angular
  .module('visualcircuit', [
    'ngRoute',
    'ui.bootstrap',
    'gettext'
  ])
  .config(['$routeProvider',
    function ($routeProvider) {
      $routeProvider
        .when('/', {
          templateUrl: 'views/main.html',
          controller: 'MainCtrl'
        })
        .otherwise({
          redirectTo: '/'
        })
    }
  ])
  .run(function (profile,
                project,
                common,
                utils,
                gettextCatalog,
                $timeout)
  {
    $timeout(function (){
      $('body').addClass('waiting');
    }, 0);
    // Load language
    utils.loadLanguage(profile, function () {
      // Initialize title
      project.updateTitle(gettextCatalog.getString('Untitled'));
      $('body').removeClass('waiting');
    });
  });
