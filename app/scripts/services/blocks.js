'use strict';

// var fs = require('fs');
 

angular.module('visualcircuit')
    .service('blocks', function (joint, utils, common, gettextCatalog, nodePath, files, nodeFs) {
        var gridsize = 8;

        this.getAllBlocks = getAllBlocks;
        this.newBlock = newBlock;

        function getAllBlocks() {
            var blocks = [];

            var blockFolders = files.getDirectories(common.BLOCKS_DIR);
            
            blocks = blockFolders.map(function(blockFolder)  {
                var blockName = blockFolder.split('/').slice([-1])[0];

                var metadata = getBlockMetadata(blockFolder, blockName);
                var code = getBlockCode(blockFolder, blockName);
                if (metadata && code) {
                    Object.assign(metadata,code);
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
                console.log('can read/write');

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
                    debugger;
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
                code = nodeFs.readFile(filePath);
            }
            catch (err) {
                console.error(err);
            }
            return code;
        }

        function getMetadataFilePath(path, block) {
            path = path.trim();
            return '/home/sergio/Desarrollos/visualcircuit/app/' + path + utils.sep + block + '.json';
        }

        function getCodeFilePath(path, block) {
            path = path.trim();
            return '/home/sergio/Desarrollos/visualcircuit/app/' + path + utils.sep + block + '.py';
        }

        function newBlock (block, callback) {
            return;
            if (!block) {
                return ;
            }
            debugger;
            // Add some drawing attributes.
            block.type = 'basic.code';
            block.position = { x: 40 * gridsize, y: 16 * gridsize };
            block.size = { width: 192, height: 128 };
            block.data = {ports: {in: [], out:[]}};
            // Create ports
            block.data.ports.in = [];
            Object.keys(block.Inputs).forEach( function(inputName) {
                if (inputName) {
                    var inputProp = block.Inputs[inputName];
                    block.data.ports.in.push({
                        name: inputName,
                        range: inputProp.range || '',
                        // size: (pins.length > 1) ? pins.length : undefined
                        // size: undefined,
                        default: inputProp.default
                    });
                }
            });
            block.data.ports.out = [];
            Object.keys(block.Outputs).forEach(function (outputName) {
                if (outputName) {
                    var outputProp = block.Outputs[outputName];
                    block.data.ports.out.push({
                        name: outputName,
                        range: outputProp.range || '',
                        // size: (pins.length > 1) ? pins.length : undefined
                        // size: undefined,
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

        function newCustomBlock() {
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
    
            var cell = new joint.shapes.ice.Code({
                id: instance.id,
                blockType: instance.type,
                data: instance.data,
                position: instance.position,
                size: instance.size,
                disabled: disabled,
                leftPorts: leftPorts,
                rightPorts: rightPorts,
                topPorts: topPorts
            });
    
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