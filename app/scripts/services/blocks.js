'use strict';

// var fs = require('fs');
var path = require('path');
 

angular.module('visualcircuit')
    .service('blocks', function (joint, utils, common, gettextCatalog, nodePath, files, nodeFs) {
        var gridsize = 8;

        this.getAllBlocks = getAllBlocks;
        this.newBlock = newBlock;
        this.editBlockCode = editBlockCode;

        var n = nodePath;

        function getAllBlocks() {
            var blocks = [];

            var blockFolders = files.getDirectories(common.BLOCKS_DIR);
            
            blocks = blockFolders.map(function(blockFolder)  {
                var blockName = blockFolder.split('/').slice([-1])[0];

                var metadata = getBlockMetadata(blockFolder, blockName);
                var code = getBlockCode(blockFolder, blockName);
                if (metadata && code) {
                    metadata.Code = code
                }
                return metadata;
            });

            return blocks;
        }

        function getBlockMetadata(folder, name) {
            var metadata = null;

            var filePath = getMetadataFilePath(folder, name);

            try {
                nodeFs.accessSync(filePath, nodeFs.R_OK);
                var content = nodeFs.readFileSync(filePath, 'utf8');
                // TODO: Error checks
                if (content) {
                    var parsed = JSON.parse(content);
                    if (['Id', 'Name', 'Delay'].some(function (field) { return !(field in parsed); })) {
                        // TODO:

                        //raise Exception();
                    }

                    metadata = {
                        'Id': parsed.Id,
                        'Name': parsed.Name ,
                        'Description': parsed.Description || '',
                        'Clock': parsed.Clock || common.DEFAULT_BLOCK_CLOCK,
                        'Image': parsed.Image || common.DEFAULT_BLOCK_IMG,
                        'Inputs': parsed.Inputs,
                        'Outputs': parsed.Outputs
                    };
                }
            } catch (err) {
                console.error('Cannot read metadata in ' + filePath);
            }
            return metadata;
        
        }

        function getBlockCode(folder, name) {
            var filePath = getCodeFilePath(folder, name);
            var code = null;
            try {
                nodeFs.accessSync(filePath, nodeFs.R_OK);
                code = nodeFs.readFileSync(filePath, 'utf8');
            }
            catch (err) {
                console.error(err);
            }
            return code;
        }

        function getMetadataFilePath(path, block) {
            path = path.trim();
            return './' + path + utils.sep + block + '.json';
        }

        function getCodeFilePath(path, block) {
            path = path.trim();
            return './' + path + utils.sep + block + '.py';
        }

        function newBlock (block, callback) {
            if (!block) {
                return ;
            }
            // Add some drawing attributes.
            block.type = 'basic.code';
            block.position = { x: 40 * gridsize, y: 16 * gridsize };
            block.size = { width: 192, height: 128 };
            block.data = {ports: {in: [], out:[]}};

            // Create ports
            block.data.ports.in = [];
            var inputs = Object.keys(block.Inputs) || [];
            inputs.forEach( function(inputName) {
                if (inputName) {
                    var inputProp = block.Inputs[inputName];
                    block.data.ports.in.push({
                        name: inputName,
                        range: inputProp.range || '',
                        size: inputs.length,
                        default: inputProp.default
                    });
                }
            });
            block.data.ports.out = [];
            var outputs = Object.keys(block.Outputs) || [];
            outputs.forEach(function (outputName) {
                if (outputName) {
                    var outputProp = block.Outputs[outputName];
                    block.data.ports.out.push({
                        name: outputName,
                        range: outputProp.range || '',
                        size: outputs.length,
                        default: outputProp.default
                    });
                }
            });
            block.data.params = [];
            // for (var p in block.Parameters) {
            //     if (p) {
            //         blockInstance.data.params.push({
            //             name: p.name
            //         });
            //     }
            // }

            if (callback) {
                callback(loadBasicCode(block));
            }
        }

        /*function newCustomBlock() {
            var blockInstance = {
                id: null,
                data: {
                    code: '',
                    params: [],
                    ports: { 
                        in: [],
                        out: []
                    }
                },
                type: 'basic.code',
                position: { x: 40 * gridsize, y: 16 * gridsize },
                size: { width: 192, height: 128 }
            };
            var defaultValues = [
                'a , b',
                'c , d',
                ''
            ];
        }*/

        function editBlockCode(cellView, callback) {
            var graph = cellView.paper.model;
            var block = cellView.model.attributes;
            var blockInstance = {
                id: block.id,
                data: utils.clone(block.data),
                type: 'basic.code',
                position: block.position,
                size: block.size
            };
            newBasicCode(function (cells) {
                if (callback) {
                    var cell = cells[0];
                    if (cell) {
                        var connectedWires = graph.getConnectedLinks(cellView.model);
                        graph.startBatch('change');
                        cellView.model.remove();
                        callback(cell);
                        // Restore previous connections
                        for (var w in connectedWires) {
                            var wire = connectedWires[w];
                            var size = wire.get('size');
                            var source = wire.get('source');
                            var target = wire.get('target');
                            if ((source.id === cell.id && containsPort(source.port, size, cell.get('rightPorts'))) ||
                                (target.id === cell.id && containsPort(target.port, size, cell.get('leftPorts')) && source.port !== 'constant-out') ||
                                (target.id === cell.id && containsPort(target.port, size, cell.get('topPorts')) && source.port === 'constant-out')) {
                                graph.addCell(wire);
                            }
                        }
                        graph.stopBatch('change');
                        alertify.success(gettextCatalog.getString('Block updated'));
                    }
                }
            }, blockInstance);
        }


        function loadBasicCode(instance, disabled) {
            var port;
            var leftPorts = [];
            var rightPorts = [];
            var topPorts = [];
    
            for (var i in instance.data.ports.in) {
                port = instance.data.ports.in[i];
                leftPorts.push({
                    id: port.name,
                    name: port.name,
                    label: port.name + (port.range || ''),
                    size: port.size || 1
                });
            }
    
            for (var o in instance.data.ports.out) {
                port = instance.data.ports.out[o];
                rightPorts.push({
                    id: port.name,
                    name: port.name,
                    label: port.name + (port.range || ''),
                    size: port.size || 1
                });
            }
    
            for (var p in instance.data.params) {
                port = instance.data.params[p];
                topPorts.push({
                    id: port.name,
                    name: port.name,
                    label: port.name
                });
            }

            instance.data.code = instance.Code || '';
    
            var cell = new joint.shapes.ice.Code({
                id: instance.id,
                blockType: instance.type,
                data: instance.data,
                position: instance.position,
                size: instance.size,
                image: instance.Image,
                disabled: disabled,
                leftPorts: leftPorts,
                rightPorts: rightPorts,
                topPorts: topPorts
            });

            cell.set('code', instance.Code);
    
            return cell;
        }


        function getPins(portInfo) {
            var pins = [];
            if (portInfo.range) {
                for (var r in portInfo.range) {
                    pins.push({ index: portInfo.range[r].toString(), name: '', value: '0' });
                }
            }
            else {
                pins.push({ index: '0', name: '', value: '0' });
            }
            return pins;
        }
    });