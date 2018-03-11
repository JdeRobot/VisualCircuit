'use strict';

angular.module('visualcircuit')
    .service('files', function ($rootScope, nodeFs) {
        this.getFiles = getFiles;
        this.getDirectories = getDirectories;

        function getFiles (dir){
            var fileList = [];
            
            var files = nodeFs.readdirSync(dir);
            for(var i in files){
                console.log(i);
                if (!files.hasOwnProperty(i)) continue;
                var name = dir+'/'+files[i];
                if (!nodeFs.statSync(name).isDirectory()){
                    fileList.push(name);
                }
            }
            return fileList;
        }

        function getDirectories (dir) {
            var directoriesList = [];
            
            var files = nodeFs.readdirSync(dir);
            for(var i in files){
                if (!files.hasOwnProperty(i)) continue;
                var name = dir+'/'+files[i];
                if (nodeFs.statSync(name).isDirectory()){
                    directoriesList.push(name);
                }
            }
            return directoriesList;
        }
    });