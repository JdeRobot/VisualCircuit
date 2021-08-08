import { BaseModelOptions, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { NodeModelGenerics, PortModel, PortModelAlignment } from '@projectstorm/react-diagrams-core';
import { PortTypes } from '../../../../core/constants';
import BaseModel from '../../common/base-model';
import { createPortModel } from '../../common/factory';

/**
 * Options for Input block
 */
export interface InputBlockModelOptions extends BaseModelOptions {
    name: string;
}

/**
 * Interface for Input block data
 */
interface InputBlockData {
    name: string;
}

/**
 * Data model for Input block
 */
export class InputBlockModel extends BaseModel<InputBlockData, NodeModelGenerics & InputBlockModelOptions> {

    constructor(options: InputBlockModelOptions) {
        super({
            ...options,
            type: 'basic.input'
        });

        // Initialise data
        this.data = {
            name: options.name
        }
        // Create an output port
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

    /**
     * Getter for the default port of input block
     * @returns Input block output port
     */
    getPort(): PortModel {
        return super.getPort('input-out')!;
    }

    /**
     * Serialise data and model
     * @returns Serialised model and data
     */
    serialize() {
        return {
            ...super.serialize(),
            data: this.getData()
        }
    }

    /**
     * Deserialise model and data
     * @param event Event which indicates model to deserialise data
     */
    deserialize(event: DeserializeEvent<this>): void {
        super.deserialize(event);
        this.data = event.data.data;
    }
}