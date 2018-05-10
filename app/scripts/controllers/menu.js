'use strict';

angular.module('visualcircuit')
    .controller('MenuCtrl', function ($rootScope, $scope, $timeout, profile, blocks, compiler,
        project, graph, utils, common, shortcuts, gettextCatalog, gui, _package, nodeFs) {

        // Initialize scope
        $scope.profile = profile;
        $scope.project = project;
        $scope.common = common;
        
        $scope.version = _package.version;
        $scope.workingdir = '';
        $scope.snapshotdir = '';

        var zeroProject = true;  // New project without changes
        var resultAlert = null;

        var buildUndoStack = [];
        var changedUndoStack = [];
        var currentUndoStack = [];


        // Window events
        var win = gui.Window.get();
        win.on('close', function () {
            exit();
        });
        win.on('resize', function () {
            graph.fitContent();
        });

        // Darwin fix for shortcuts
        if (process.platform === 'darwin') {
            var mb = new gui.Menu({ type: 'menubar' });
            mb.createMacBuiltin('visualcircuit');
            win.menu = mb;
        }

        // Load app arguments
        setTimeout(function () {
            var local = false;
            for (var i in gui.App.argv) {
                var arg = gui.App.argv[i];
                processArg(arg);
                local = arg === 'local' || local;
            }
            if (local) {
                project.path = '';
            }
            else {
                updateWorkingdir(project.path);
            }
        }, 0);

        /**
         * function:: processArg(args)
         * Checks the arguments passed to the app.
         * @param arg {string[]} arguments of the app call
         */
        function processArg(arg) {
            // Check whether the arg is a path to a project.
            if (nodeFs.existsSync(arg)) {
                // Open filepath
                var filepath = arg;
                project.open(filepath);
                //zeroProject = false;
            }
            else {
                // Move window
                var data = arg.split('x');
                var offset = {
                    x: parseInt(data[0]),
                    y: parseInt(data[1])
                };
                win.moveTo(offset.x, offset.y);
            }
        }


        //-- File

        /**
         * Starts a new project in a new Window.
         */
        $scope.newProject = function () {
            utils.newWindow();
        };

        /**
         * Loads an existing project.
         * If the current project has been modified the new project will be opened in
         * a new window, else the same window will be used.
         */
        $scope.openProjectDialog = function () {
            utils.openDialog('#input-open-project', common.PROJECT_FILES_EXT, function (filepath) {
                if (zeroProject) {
                    // If this is the first action, open
                    // the projec in the same window
                    updateWorkingdir(filepath);
                    project.open(filepath);
                    //zeroProject = false;
                }
                else if (project.changed || !equalWorkingFilepath(filepath)) {
                    // If this is not the first action, and
                    // the file path is different, open
                    // the project in a new window
                    utils.newWindow(filepath);
                }
            });
        };

        $scope.openProject = function (filepath) {
            if (zeroProject) {
                // If this is the first action, open
                // the projec in the same window
                project.open(filepath, true);
                //zeroProject = false;
            }
            else {
                // If this is not the first action, and
                // the file path is different, open
                // the project in a new window
                utils.newWindow(filepath, true);
            }
        };

        /**
         * Saves the current project to a file.
         */
        $scope.saveProject = function () {
            var filepath = project.path;
            if (filepath) {
                project.save(filepath);
                resetChangedStack();
            }
            else {
                $scope.saveProjectAs();
            }
        };


        /**
         * Saves the current project to a file.
         * @param localCallback {function = null} callback called after saving if set.
         */
        $scope.saveProjectAs = function (localCallback) {
            utils.saveDialog('#input-save-project', common.PROJECT_FILES_EXT, function (filepath) {
                updateWorkingdir(filepath);
                project.save(filepath);
                resetChangedStack();
                if (localCallback) {
                    localCallback();
                }
            });
        };

        $rootScope.$on('saveProjectAs', function (event, callback) {
            $scope.saveProjectAs(callback);
        });

        /**
         * Sets a new path as the current working directory.
         * @param filepath {string} the path
         */
        function updateWorkingdir(filepath) {
            $scope.workingdir = utils.dirname(filepath) + utils.sep;
        }

        /**
         * Checks whether a path is the current working directory
         * @return {Boolean}
         */
        function equalWorkingFilepath(filepath) {
            return $scope.workingdir + project.name + common.PROJECT_FILES_EXT === filepath;
        }

        $scope.generate = function () {
            $scope.compiler();
        };

        /**
         * Exits from the app.
         */
        $scope.quit = function () {
            exit();
        };

        function exit() {
            if (project.changed) {
              alertify.set('confirm', 'labels', {
                'ok': gettextCatalog.getString('Close')
              });
              alertify.set('confirm', 'defaultFocus', 'cancel');
              alertify.confirm(
                utils.bold(gettextCatalog.getString('Do you want to close the application?')) + '<br>' +
                gettextCatalog.getString('Your changes will be lost if you don’t save them'),
                function() {
                  // Close
                  _exit();
                },
                function() {
                  // Cancel
                  setTimeout(function() {
                    alertify.set('confirm', 'labels', {
                      'ok': gettextCatalog.getString('OK')
                    });
                    alertify.set('confirm', 'defaultFocus', 'ok');
                  }, 200);
                }
              );
            }
            else {
              _exit();
            }
            function _exit() {
              //win.hide();
              utils.removeTempBuildDir();
              win.close(true);
            }
          }


        //-- Edit menu

        /**
         * Undo the last action.
         */
        $scope.undoGraph = function () {
            graph.undo();
        };

        /**
         * Redo the last action.
         */
        $scope.redoGraph = function () {
            graph.redo();
        };

        /**
         * Cuts the selected element.
         */
        $scope.cutSelected = function () {
            graph.cutSelected();
        };

        /**
         * Copies the selected element.
         */
        $scope.copySelected = function () {
            graph.copySelected();
        };

        /**
         * Paste a new element to the graph.
         */
        var paste = true;
        $scope.pasteSelected = function () {
            if (paste) {
                paste = false;
                graph.pasteSelected();
                setTimeout(function () {
                    paste = true;
                }, 250);
            }
        };

        /**
         * Selects all the elements of the graph.
         */
        $scope.selectAll = function () {
            checkGraph()
                .then(function () {
                    graph.selectAll();
                })
                .catch(function () { });
        };

        /**
         * Removes the selected elements.
         */
        function removeSelected() {
            project.removeSelected();
        }

        /**
         *  Adjusts the graph so that it fits within the available area.
         */
        $scope.fitContent = function () {
            graph.fitContent();
        };

        /**
         *  Event called when the Project information gets updated.
         */
        $(document).on('infoChanged', function (evt, newValues) {
            var values = getProjectInformation();
            if (!_.isEqual(values, newValues)) {
                graph.setInfo(values, newValues, project);
                alertify.message(gettextCatalog.getString('Project information updated') + '.<br>' + gettextCatalog.getString('Click here to view'), 5)
                    .callback = function (isClicked) {
                        if (isClicked) {
                            $scope.setProjectInformation();
                        }
                    };
            }
        });

        /**
         *  Updates the project information.
         */
        $scope.setProjectInformation = function () {
            var values = getProjectInformation();
            utils.projectinfoprompt(values, function (evt, newValues) {
                if (!_.isEqual(values, newValues)) {
                    graph.setInfo(values, newValues, project);
                    alertify.success(gettextCatalog.getString('Project information updated'));
                }
            });
        };

        /**
         * Gets Project information.
         * @return {string[]} information of the current project.
         */
        function getProjectInformation() {
            var p = project.get('package');
            return [
                p.name,
                p.version,
                p.description,
                p.author,
                p.image
            ];
        }

        /**
         *  Event called when the language changes.
         */
        $(document).on('langChanged', function (evt, lang) {
            $scope.selectLanguage(lang);
        });

        /**
         * Updates the app language.
         * @param language {string} with ISO code of the language to set.
         */
        $scope.selectLanguage = function (language) {
            if (profile.get('language') !== language) {
                profile.set('language', graph.selectLanguage(language));
                // Reload the project
                project.update({ deps: false }, function () {
                    graph.loadDesign(project.get('design'), { disabled: false });
                });
            }
        };

        function checkGraph() {
            return new Promise(function (resolve, reject) {
                if (!graph.isEmpty()) {
                    resolve();
                }
                else {
                    if (resultAlert) {
                        resultAlert.dismiss(true);
                    }
                    resultAlert = alertify.warning(gettextCatalog.getString('Add a block to start'), 5);
                    reject();
                }
            });
        }


        //-- Help menu

        $scope.openUrl = function (url) {
            event.preventDefault();
            gui.Shell.openExternal(url);
        };

        $scope.about = function () {
            var content =
                '<div class="row">' +
                '  <div class="col-sm-4">' +
                '   <img src="resources/images/fpgawars-logo.png">' +
                '  </div>' +
                '  <div class="col-sm-7" style="margin-left: 20px;">' +
                '    <h4>VisualCircuit</h4>' +
                '    <p><i>Graphic editor</i></p>' +
                '    <p>Version: ' + $scope.version + '</p>' +
                '    <p>License: GPL v2</p>' +
                '    <p>Created by Sergio Lorenzo Benayas</p>' +
                '  </div>' +
                '</div>';
            alertify.alert(content);
        };

        // Blocks menu
        $scope.blocks = blocks.getAllBlocks();

        $scope.addBlock = function(block) {
            $scope.project.addBlock(block);
        };

        // Events

        $(document).on('stackChanged', function (evt, undoStack) {
            currentUndoStack = undoStack;
            var undoStackString = JSON.stringify(undoStack);
            project.changed = JSON.stringify(changedUndoStack) !== undoStackString;
            project.updateTitle();
            zeroProject = false;
            common.hasChangesSinceBuild = JSON.stringify(buildUndoStack) !== undoStackString;
            utils.rootScopeSafeApply();
        });

        function resetChangedStack() {
            changedUndoStack = currentUndoStack;
            project.changed = false;
            project.updateTitle();
            zeroProject = false;
        }

        // Detect prompt

        var promptShown = false;

        alertify.prompt().set({
            onshow: function () {
                promptShown = true;
            },
            onclose: function () {
                promptShown = false;
            }
        });

        alertify.confirm().set({
            onshow: function () {
                promptShown = true;
            },
            onclose: function () {
                promptShown = false;
            }
        });

        // Configure all shortcuts

        // -- File
        shortcuts.method('newProject', $scope.newProject);
        shortcuts.method('openProject', $scope.openProjectDialog);
        shortcuts.method('saveProject', $scope.saveProject);
        shortcuts.method('saveProjectAs', $scope.saveProjectAs);
        shortcuts.method('quit', $scope.quit);

        // -- Edit
        shortcuts.method('undoGraph', $scope.undoGraph);
        shortcuts.method('redoGraph', $scope.redoGraph);
        shortcuts.method('redoGraph2', $scope.redoGraph);
        shortcuts.method('cutSelected', $scope.cutSelected);
        shortcuts.method('copySelected', $scope.copySelected);
        shortcuts.method('pasteSelected', $scope.pasteSelected);
        shortcuts.method('selectAll', $scope.selectAll);
        shortcuts.method('fitContent', $scope.fitContent);

        // -- Tools
        shortcuts.method('verifyCode', $scope.verifyCode);
        shortcuts.method('buildCode', $scope.buildCode);
        shortcuts.method('uploadCode', $scope.uploadCode);

        // -- Misc
        shortcuts.method('stepUp', graph.stepUp);
        shortcuts.method('stepDown', graph.stepDown);
        shortcuts.method('stepLeft', graph.stepLeft);
        shortcuts.method('stepRight', graph.stepRight);

        shortcuts.method('removeSelected', removeSelected);
        shortcuts.method('back', function () {
            if (graph.isEnabled()) {
                removeSelected();
            }
            else {
                $rootScope.$broadcast('breadcrumbsBack');
            }
        });

        shortcuts.method('takeSnapshot', takeSnapshot);

        // Detect shortcuts

        $(document).on('keydown', function (event) {
            var opt = {
                prompt: promptShown,
                disabled: !graph.isEnabled()
            };

            var ret = shortcuts.execute(event, opt);
            if (ret.preventDefault) {
                event.preventDefault();
            }
        });

        function takeSnapshot() {
            win.capturePage(function (img) {
                var base64Data = img.replace(/^data:image\/(png|jpg|jpeg);base64,/, '');
                saveSnapshot(base64Data);
            }, 'png');
        }

        function saveSnapshot(base64Data) {
            utils.saveDialog('#input-save-snapshot', '.png', function (filepath) {
                nodeFs.writeFile(filepath, base64Data, 'base64', function (err) {
                    $scope.snapshotdir = utils.dirname(filepath) + utils.sep;
                    $scope.$apply();
                    if (!err) {
                        alertify.success(gettextCatalog.getString('Image {{name}} saved', { name: utils.bold(utils.basename(filepath)) }));
                    }
                    else {
                        throw err;
                    }
                });
            });
        }

        // Show/Hide menu management
        var menu;
        var timerOpen;
        var timerClose;

        // mousedown event
        var mousedown = false;
        $(document).on('mouseup', function () {
            mousedown = false;
        });
        $(document).on('mousedown', '.paper', function () {
            mousedown = true;
            // Close current menu
            $scope.status[menu] = false;
            utils.rootScopeSafeApply();
        });

        // Show menu with delay
        $scope.showMenu = function (newMenu) {
            cancelTimeouts();
            if (!mousedown && !graph.addingDraggableBlock && !$scope.status[newMenu]) {
                timerOpen = $timeout(function () {
                    $scope.fixMenu(newMenu);
                }, 300);
            }
        };

        // Hide menu with delay
        $scope.hideMenu = function () {
            cancelTimeouts();
            timerClose = $timeout(function () {
                $scope.status[menu] = false;
            }, 900);
        };

        // Fix menu
        $scope.fixMenu = function (newMenu) {
            menu = newMenu;
            $scope.status[menu] = true;
        };

        function cancelTimeouts() {
            $timeout.cancel(timerOpen);
            $timeout.cancel(timerClose);
        }

        // Disable click in submenus
        $(document).click('.dropdown-submenu', function (event) {
            if ($(event.target).hasClass('dropdown-toggle')) {
                event.stopImmediatePropagation();
                event.preventDefault();
            }
        });

    });
