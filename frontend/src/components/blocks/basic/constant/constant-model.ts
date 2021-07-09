import { NodeModelGenerics } from "@projectstorm/react-diagrams";
import { PortModel, PortModelAlignment } from "@projectstorm/react-diagrams-core";
import { DeserializeEvent } from '@projectstorm/react-canvas-core';
import { BaseModelOptions } from '@projectstorm/react-canvas-core';
import { createPortModel } from "../../common/factory";
import BaseModel from "../../common/base-model";
import { PortTypes } from "../../../../core/constants";

export interface ConstantBlockModelOptions extends BaseModelOptions {
    name: string;
    local: boolean;
}

interface ConstantBlockData {
    name: string;
    value: string;
    local: boolean;
}

export class ConstantBlockModel extends BaseModel<ConstantBlockData, NodeModelGenerics & ConstantBlockModelOptions> {

    constructor(options: ConstantBlockModelOptions) {
		super({
			...options,
			type: 'basic.constant'
		});

        this.data = {
            name: options.name,
            value: '',
            local: true
        }

        this.addPort(
			createPortModel({
				in: false,
				name: 'constant-out',
                alignment: PortModelAlignment.BOTTOM,
                type: PortTypes.OUTPUT,
                hideLabel: true
			})
		);
    }

    getPort(): PortModel {
        return super.getPort('constant-out')!;
    }

    serialize() {
        return {
            ...super.serialize(),
            data: this.getData()
        }
    }

    deserialize(event: DeserializeEvent<this>): void {
        super.deserialize(event);
        this.data = event.data.data;
    }

}