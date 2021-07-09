import { BaseModelOptions, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { NodeModelGenerics, PortModel, PortModelAlignment } from '@projectstorm/react-diagrams-core';
import { PortTypes } from '../../../../core/constants';
import BaseModel from '../../common/base-model';
import { createPortModel } from '../../common/factory';

export interface OutputBlockModelOptions extends BaseModelOptions {
    name: string;
} 

interface OutputBlockData {
    name: string;
}

export class OutputBlockModel extends BaseModel<OutputBlockData, NodeModelGenerics & OutputBlockModelOptions> {

    constructor(options: OutputBlockModelOptions) {
		super({
			...options,
			type: 'basic.output'
		});

        this.data = {
            name: options.name
        }
        this.addPort(
			createPortModel({
				in: true,
				name: 'output-in',
                alignment: PortModelAlignment.LEFT,
                hideLabel: true,
                type: PortTypes.INPUT
			})
		);
    }

    getPort(): PortModel {
        return super.getPort('output-in')!;
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