'use strict';
 

angular.module('visualcircuit')
    .service('blocks', function (joint, utils, common, gettextCatalog, nodePath, files, nodeFs) {
        var gridsize = 8;

        this.getAllBlocks = getAllBlocks;
        this.newBlock = newBlock;
        this.editBlockCode = editBlockCode;
        this.loadBlock = loadBlock;
        this.loadWire = loadWire;

        function getAllBlocks() {
            var blocks = [];

            var blockFolders = files.getDirectories(common.BLOCKS_DIR);
            
            blocks = blockFolders.map(function(blockFolder)  {
                var blockName = blockFolder.split('/').slice([-1])[0];

                var metadata = getBlockMetadata(blockFolder, blockName);
                var code = getBlockCode(blockFolder, blockName);
                if (metadata && code) {
                    metadata.Code = code;
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
                        size: inputProp.size || 1,
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
                        size: outputProp.size || 1,
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
            block.data.code = block.data.code || block.Code;
            block.data.image = block.data.image || block.Image;

            if (callback) {
                callback(loadBlock(block));
            }
        }

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
            // TODO: 
        }


        function loadBlock(instance, disabled) {
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

            instance.data.code = instance.data.code || instance.Code || '';
            
            var cell = new joint.shapes.ice.Code({
                id: instance.id,
                blockType: instance.type,
                data: instance.data,
                position: instance.position,
                size: instance.size,
                image: instance.data.image,
                disabled: disabled,
                leftPorts: leftPorts,
                rightPorts: rightPorts,
                topPorts: topPorts
            });

            cell.set('code', instance.data.code);
    
            return cell;
        }

        function loadWire(instance, source, target) {

            // Find selectors
            var sourceSelector, targetSelector;
            var leftPorts = target.get('leftPorts');
            var rightPorts = source.get('rightPorts');
            for (var _out = 0; _out < rightPorts.length; _out++) {
                if (rightPorts[_out] === instance.source.port) {
                    sourceSelector = _out;
                    break;
                }
            }
            for (var _in = 0; _in < leftPorts.length; _in++) {
                if (leftPorts[_in] === instance.target.port) {
                    targetSelector = _in;
                    break;
                }
            }

            var _wire = new joint.shapes.ice.Wire({
                source: {
                    id: source.id,
                    selector: sourceSelector,
                    port: instance.source.port
                },
                target: {
                    id: target.id,
                    selector: targetSelector,
                    port: instance.target.port
                },
                vertices: instance.vertices
            });
            return _wire;
        }
    });