'use strict';

angular.module('visualcircuit')
    .service('project', function ($rootScope, graph, compiler, profile, utils, common, 
        gettextCatalog, nodeFs, nodePath) {

        this.name = '';  // Used in File dialogs
        this.path = '';  // Used in Save / Save as
        this.filepath = ''; // Used to find external resources (.v, .vh, .list)
        this.changed = false;

        var project = _default();

        function _default() {
            return {
                version: common.VERSION,
                package: {
                    name: '',
                    version: '',
                    description: '',
                    author: '',
                    image: ''
                },
                design: {
                    board: '',
                    graph: { blocks: [], wires: [] },
                    state: { pan: { x: 0, y: 0 }, zoom: 1.0 }
                },
                dependencies: {}
            };
        }

        /* Dependency format
        {
          package: {
            name: '',
            version: '',
            description: '',
            author: '',
            image: ''
          },
          design: {
            graph: { blocks: [], wires: [] }
            state: { pan: { x: 0, y: 0 }, zoom: 1.0 }
          },
        }
        */

        this.get = function (key) {
            if (key in project) {
                return project[key];
            }
            else {
                return project;
            }
        };

        this.set = function (key, obj) {
            if (key in project) {
                project[key] = obj;
            }
        };

        this.new = function (name) {
            this.path = '';
            project = _default();
            this.updateTitle(name);

            graph.clearAll();
            graph.resetCommandStack();
            graph.setState(project.design.state);

            alertify.success(gettextCatalog.getString('New project {{name}} created', { name: utils.bold(name) }));
        };

        this.open = function (filepath, emptyPath) {
            var self = this;
            this.path = emptyPath ? '' : filepath;
            this.filepath = filepath;
            utils.readFile(filepath)
                .then(function (data) {
                    var name = utils.basename(filepath);
                    self.load(name, data);
                })
                .catch(function (error) {
                    alertify.error(error, 30);
                });
        };

        this.load = function (name, data) {
            var self = this;
            if (data.version !== common.VERSION) {
                alertify.warning(gettextCatalog.getString('Old project format {{version}}', { version: data.version }), 5);
            }
            project = _safeLoad(data);
            _load();

            function _load(reset) {
                common.allDependencies = project.dependencies;
                var opt = { reset: reset || false, disabled: false };
                var ret = graph.loadDesign(project.design, opt, function () {
                    graph.resetCommandStack();
                    graph.fitContent();
                    alertify.success(gettextCatalog.getString('Project {{name}} loaded', { name: utils.bold(name) }));
                    common.hasChangesSinceBuild = true;
                });

                if (ret) {
                    self.updateTitle(name);
                }
                else {
                    alertify.error(gettextCatalog.getString('Wrong project format: {{name}}', { name: utils.bold(name) }), 30);
                }
                setTimeout(function () {
                    alertify.set('confirm', 'labels', {
                        'ok': gettextCatalog.getString('OK'),
                        'cancel': gettextCatalog.getString('Cancel')
                    });
                }, 100);
            }
        };

        function _safeLoad(data) {
            // Backwards compatibility
            var project = {};
            switch (data.version) {
                default:
                    project = data;
                    break;
            }
            return project;
        }

        this.save = function (filepath) {
            var name = utils.basename(filepath);
            this.updateTitle(name);
            sortGraph();
            this.update();

            // Copy included files if the previous filepath
            // is different from the new filepath
            var files = [];
            if (this.filepath !== filepath) {
                var origPath = utils.dirname(this.filepath);
                var destPath = utils.dirname(filepath);
                // 1. Parse and find included files
                // var code = compiler.generate('verilog', project);
                // var files = utils.findIncludedFiles(code);
                // Are there included files?
            }

            if (files.length > 0 && filepath) {
                // 3. Copy the included files
                copyIncludedFiles(files, origPath, destPath, function (success) {
                    if (!success) {
                        alertify.error(error, 30);
                        return;
                    }
                });
            }
            doSaveProject();
            

            this.path = filepath;
            this.filepath = filepath;

            function doSaveProject() {
                utils.saveFile(filepath, pruneProject(project))
                    .then(function () {
                        alertify.success(gettextCatalog.getString('Project {{name}} saved', { name: utils.bold(name) }));
                    })
                    .catch(function (error) {
                        alertify.error(error, 30);
                    });
            }

        };

        function sortGraph() {
            var cells = graph.getCells();

            // Sort Constant cells by x-coordinate
            cells = _.sortBy(cells, function (cell) {
                if (cell.get('type') === 'ice.Constant') {
                    return cell.get('position').x;
                }
            });

            // Sort I/O cells by y-coordinate
            cells = _.sortBy(cells, function (cell) {
                if (cell.get('type') === 'ice.Input' ||
                    cell.get('type') === 'ice.Output') {
                    return cell.get('position').y;
                }
            });

            graph.setCells(cells);
        }

        this.addBlockFile = function (filepath, notification) {
            var self = this;
            utils.readFile(filepath)
                .then(function (data) {
                    if (data.version !== common.VERSION) {
                        alertify.warning(gettextCatalog.getString('Old project format {{version}}', { version: data.version }), 5);
                    }
                    var name = utils.basename(filepath);
                    var block = _safeLoad(data, name);
                    if (block) {
                        var origPath = utils.dirname(filepath);
                        var destPath = utils.dirname(self.path);
                        // 1. Parse and find included files
                        var code = compiler.generate('verilog', block);
                        var files = utils.findIncludedFiles(code);
                        // Are there included files?
                        if (files.length > 0) {
                            // 2. Check project's directory
                            if (self.path) {
                                // 3. Copy the included files
                                copyIncludedFiles(files, origPath, destPath, function (success) {
                                    if (success) {
                                        // 4. Success: import block
                                        doImportBlock();
                                    }
                                });
                            }
                            else {
                                alertify.confirm(gettextCatalog.getString('This import operation requires a project path. You need to save the current project. Do you want to continue?'),
                                    function () {
                                        $rootScope.$emit('saveProjectAs', function () {
                                            setTimeout(function () {
                                                // 3. Copy the included files
                                                copyIncludedFiles(files, origPath, destPath, function (success) {
                                                    if (success) {
                                                        // 4. Success: import block
                                                        doImportBlock();
                                                    }
                                                });
                                            }, 500);
                                        });
                                    });
                            }
                        }
                        else {
                            // No included files to copy
                            // 4. Import block
                            doImportBlock();
                        }
                    }

                    function doImportBlock() {
                        self.addBlock(block);
                        if (notification) {
                            alertify.success(gettextCatalog.getString('Block {{name}} imported', { name: utils.bold(block.package.name) }));
                        }
                    }
                })
                .catch(function (error) {
                    alertify.error(error, 30);
                });
        };

        function copyIncludedFiles(files, origPath, destPath, callback) {
            var success = true;
            async.eachSeries(files, function (filename, next) {
                setTimeout(function () {
                    if (origPath !== destPath) {
                        if (nodeFs.existsSync(nodePath.join(destPath, filename))) {
                            alertify.confirm(gettextCatalog.getString('File {{file}} already exists in the project path. Do you want to replace it?', { file: utils.bold(filename) }),
                                function () {
                                    success = success && doCopySync(origPath, destPath, filename);
                                    if (!success) {
                                        return next(); // break
                                    }
                                    next();
                                },
                                function () {
                                    next();
                                });
                        }
                        else {
                            success = success && doCopySync(origPath, destPath, filename);
                            if (!success) {
                                return next(); // break
                            }
                            next();
                        }
                    }
                    else {
                        return next(); // break
                    }
                }, 0);
            }, function (/*result*/) {
                return callback(success);
            });
        }

        function doCopySync(origPath, destPath, filename) {
            var orig = nodePath.join(origPath, filename);
            var dest = nodePath.join(destPath, filename);
            var success = utils.copySync(orig, dest);
            if (success) {
                alertify.message(gettextCatalog.getString('File {{file}} imported', { file: utils.bold(filename) }), 5);
            }
            else {
                alertify.error(gettextCatalog.getString('Original file {{file}} does not exist', { file: utils.bold(filename) }), 30);
            }
            return success;
        }

        function pruneProject(project) {
            var _project = utils.clone(project);

            _prune(_project);
            for (var d in _project.dependencies) {
                _prune(_project.dependencies[d]);
            }

            function _prune(_project) {
                for (var i in _project.design.graph.blocks) {
                    var block = _project.design.graph.blocks[i];
                    switch (block.type) {
                        case 'basic.input':
                        case 'basic.output':
                        case 'basic.constant':
                            break;
                        case 'basic.info':
                            delete block.data.text;
                            break;
                        case 'basic.code':
                            for (var j in block.data.ports.in) {
                                delete block.data.ports.in[j].default;
                            }
                            break;
                        default:
                            // Generic block
                            delete block.data;
                            break;
                    }
                }
            }

            return _project;
        }

        this.update = function (opt, callback) {
            var graphData = graph.toJSON();
            var p = utils.cellsToProject(graphData.cells, opt);

            project.design.board = p.design.board;
            project.design.graph = p.design.graph;
            project.dependencies = p.dependencies;
            var state = graph.getState();
            project.design.state = {
                pan: {
                    x: parseFloat(state.pan.x.toFixed(4)),
                    y: parseFloat(state.pan.y.toFixed(4))
                },
                zoom: parseFloat(state.zoom.toFixed(4))
            };

            if (callback) {
                callback();
            }
        };

        this.updateTitle = function (name) {
            if (name) {
                this.name = name;
                graph.resetBreadcrumbs(name);
            }
            var title = (this.changed ? '*' : '') + this.name + ' ─ VisualCircuit';
            utils.updateWindowTitle(title);
        };

        this.compile = function (target) {
            this.update();
            var opt = { boardRules: profile.get('boardRules') };
            return compiler.generate(target, project, opt);
        };

        this.addBlock = function(block){
            graph.createBlock(block);
        };

        this.removeSelected = function () {
            graph.removeSelected();
        };

        this.clear = function () {
            project = _default();
            graph.clearAll();
            graph.resetBreadcrumbs();
            graph.resetCommandStack();
        };

    });
