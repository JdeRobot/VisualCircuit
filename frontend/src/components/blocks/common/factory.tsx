import { AbstractModelFactory, Toolkit } from '@projectstorm/react-canvas-core';
import { LinkModel, PortModel } from "@projectstorm/react-diagrams";
import { DefaultPortModel } from "@projectstorm/react-diagrams-defaults";
import { RightAngleLinkModel } from "@projectstorm/react-diagrams-routing";
import { PortTypes, ProjectInfo } from '../../../core/constants';
import { ProjectDesign } from '../../../core/serialiser/interfaces';
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

export const createPortModel = (options: BasePortModelOptions) => {
    // return new RightAnglePortModel(options);
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

export const loadPackage = (jsonModel: any) => {
    const model = jsonModel.editor;
    const design = jsonModel.design as ProjectDesign;
    const info = jsonModel.package as ProjectInfo;
    return new PackageBlockModel({
        model: model,
        design: design,
        info: info
    });
}


export const getInitialPosition = (): [number, number] => {
    return [100, 100]
}

