import { BaseModelOptions, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { NodeModelGenerics, PortModel, PortModelAlignment } from '@projectstorm/react-diagrams-core';
import { PortTypes } from '../../../../core/constants';
import BaseModel from '../../common/base-model';
import { createPortModel } from '../../common/factory';

/**
 * Options for Output block
 */
export interface OutputBlockModelOptions extends BaseModelOptions {
    name: string;
}

/**
 * Interface for Output block data
 */
interface OutputBlockData {
    name: string;
}

/**
 * Data model for Output block
 */
export class OutputBlockModel extends BaseModel<OutputBlockData, NodeModelGenerics & OutputBlockModelOptions> {

    constructor(options: OutputBlockModelOptions) {
        super({
            ...options,
            type: 'basic.output'
        });

        // Initialise data
        this.data = {
            name: options.name
        }
        // Create an input port
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

    /**
     * Getter for the default port of output block
     * @returns Output block input port
     */
    getPort(): PortModel {
        return super.getPort('output-in')!;
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