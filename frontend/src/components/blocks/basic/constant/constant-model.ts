import { NodeModelGenerics } from "@projectstorm/react-diagrams";
import { PortModel, PortModelAlignment } from "@projectstorm/react-diagrams-core";
import { DeserializeEvent } from '@projectstorm/react-canvas-core';
import { BaseModelOptions } from '@projectstorm/react-canvas-core';
import { createPortModel } from "../../common/factory";
import BaseModel from "../../common/base-model";
import { PortTypes } from "../../../../core/constants";

/**
 * Options for Constant block
 */
export interface ConstantBlockModelOptions extends BaseModelOptions {
    name: string;
    local: boolean;
}

/**
 * Interface for Constant block data
 */
interface ConstantBlockData {
    name: string;
    value: string;
    local: boolean;
}

/**
 * Data model for Constant block
 */
export class ConstantBlockModel extends BaseModel<ConstantBlockData, NodeModelGenerics & ConstantBlockModelOptions> {

    constructor(options: ConstantBlockModelOptions) {
		super({
			...options,
			type: 'basic.constant'
		});

        // Initialise data
        this.data = {
            name: options.name,
            value: '',
            local: true
        }

        // Create an output port
        this.addPort(
			createPortModel({
				in: false,
				name: 'constant-out',
                alignment: PortModelAlignment.BOTTOM,
                type: PortTypes.OUTPUT,
                hideLabel: true,
                label: options.name
			})
		);
    }

    /**
     * Getter for the default port of constant
     * @returns Constant output port
     */
    getPort(): PortModel {
        return super.getPort('constant-out')!;
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