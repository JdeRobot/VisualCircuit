import { AbstractModelFactory, Toolkit } from '@projectstorm/react-canvas-core';
import { LinkModel, NodeModel, PortModel } from "@projectstorm/react-diagrams";
import { DefaultPortModel } from "@projectstorm/react-diagrams-defaults";
import { RightAngleLinkModel } from "@projectstorm/react-diagrams-routing";
import { PortTypes, ProjectInfo } from '../../../core/constants';
import { Dependency, ProjectDesign } from '../../../core/serialiser/interfaces';
import createCodeDialog from '../../dialogs/code-block-dialog';
import createConstantDialog from "../../dialogs/constant-block-dialog";
import createIODialog from '../../dialogs/input-output-block-dialog';
import { CodeBlockModel } from "../basic/code/code-model";
import { ConstantBlockModel } from "../basic/constant/constant-model";
import { InputBlockModel } from '../basic/input/input-model';
import { OutputBlockModel } from '../basic/output/output-model';
import { getCollectionBlock } from '../collection/collection-factory';
import { PackageBlockModel } from '../package/package-model';
import { BaseInputPortModel, BaseOutputPortModel, BaseParameterPortModel, BasePortModelOptions } from './base-port/port-model';

/**
 * Port model for wires which bend at 90 degrees. Unused as of now.
 */
export class RightAnglePortModel extends DefaultPortModel {
	createLinkModel(_factory?: AbstractModelFactory<LinkModel>): LinkModel {
		return new RightAngleLinkModel();
	}

    link<T extends LinkModel>(port: PortModel, factory?: AbstractModelFactory<T>): T {
		let link = this.createLinkModel(factory);
		link.setSourcePort(this);
		link.setTargetPort(port);
		return link as T;
	}
}

/**
 * Create port model of either input or output or parameter type.
 * @param options Port options based on which different Port model is created
 * @returns Port model
 */
export const createPortModel = (options: BasePortModelOptions) => {
    switch (options.type) {
        case PortTypes.INPUT:
            return new BaseInputPortModel(options);
        case PortTypes.OUTPUT:
            return new BaseOutputPortModel(options);
        case PortTypes.PARAM:
            return new BaseParameterPortModel(options);
        default:
            return new DefaultPortModel(options);
    }
    
}

/**
 * Helper function to edit a block.
 * @param node Block to be edited
 */
export const editBlock = async (node : NodeModel) => {
    var data;
    try {
        if (node instanceof ConstantBlockModel) {
            data = await createConstantDialog({isOpen: true, name: node.getData().name, local: node.getData().local});
            node.setData(data);
        } else if (node instanceof CodeBlockModel) {
            data = await createCodeDialog({isOpen: true, 
                inputs: node.getInputNames(), outputs: node.getOutputNames(), params: node.getParameterNames()});
            node.setData(data);
        } else if (node instanceof InputBlockModel || node instanceof OutputBlockModel) {
            data = await createIODialog({isOpen: true, name: node.getData().name});
            node.setData(data);
        }
    } catch (error) {
        console.log(error);
    }
}

/**
 * Helper function to create a block of specified type. For constant blocks the ID is modified to
 * make it semi determinate so that first added block gets lower ID
 * @param name Name / type of the block
 * @param blockCount count of blocks placed (Used as unique ID for constant blocks)
 * @returns block model
 */
export const createBlock = async (name: string, blockCount: number) => {
    var block;
    var data;
    try {
        switch (name) {
            case 'basic.constant':
                data = await createConstantDialog({isOpen: true});
                // This is workaround to indicate how blocks should be sorted
                data.id = blockCount.toString().padStart(4, '0') + '-' + Toolkit.UID();
                block = new ConstantBlockModel(data)
                break;
            case 'basic.code':
                data = await createCodeDialog({isOpen: true});
                block = new CodeBlockModel(data);
                break;
            case 'basic.input':
                data = await createIODialog({isOpen: true});
                block = new InputBlockModel(data);
                break;
            case 'basic.output':
                data = await createIODialog({isOpen: true});
                block = new OutputBlockModel(data);
                break;
            default:
                data = await getCollectionBlock(name);
                block = loadPackage(data);
                break;
        }
    } catch (error) {
        console.log(error);
    }
    return block;
}

/**
 * Load a project as Package block
 * @param jsonModel object conforming to the project structure
     * Project Structure: {
     *      "editor": {...},
     *      "version": "3.0",
     *      "package": {...},
     *      "design": {...},
     *      "dependencies": [...]
     * }
 * @returns Package block
 */
export const loadPackage = (jsonModel: any) => {
    const model = jsonModel.editor;
    const design = jsonModel.design as ProjectDesign;
    const info = jsonModel.package as ProjectInfo;
    let dependencies = [];

    // Check if jsonModel.dependencies exists and is an object
    if (jsonModel.dependencies && typeof jsonModel.dependencies === 'object') {
        
        dependencies.push({
            package: jsonModel.dependencies.package as ProjectInfo,
            design: jsonModel.dependencies.design as ProjectDesign,
            dependencies: jsonModel.dependencies.dependencies as Dependency[]
        });
    }
    
    return new PackageBlockModel({
        model: model,
        design: design,
        info: info,
        dependencies: dependencies
    });
}

/**
 * Fixed initial position for all blocks.
 * TODO: Better way to pick a position dynamically.
 * @returns Position x, y
 */
export const getInitialPosition = (): [number, number] => {
    return [100, 100]
}

