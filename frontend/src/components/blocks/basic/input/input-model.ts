import { BaseModelOptions, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { NodeModelGenerics, PortModel, PortModelAlignment } from '@projectstorm/react-diagrams-core';
import { PortTypes } from '../../../../core/constants';
import BaseModel from '../../common/base-model';
import { createPortModel } from '../../common/factory';

export interface InputBlockModelOptions extends BaseModelOptions {
    name: string;
}

interface InputBlockData {
    name: string;
}

export class InputBlockModel extends BaseModel<InputBlockData, NodeModelGenerics & InputBlockModelOptions> {

    constructor(options: InputBlockModelOptions) {
        super({
            ...options,
            type: 'basic.input'
        });

        this.data = {
            name: options.name
        }
        this.addPort(
            createPortModel({
                in: false,
                name: 'input-out',
                alignment: PortModelAlignment.RIGHT,
                hideLabel: true,
                type: PortTypes.OUTPUT
            })
        );
    }

    getPort(): PortModel {
        return super.getPort('input-out')!;
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