import { BaseModelOptions, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { NodeModelGenerics } from "@projectstorm/react-diagrams";
import { PortModelAlignment } from "@projectstorm/react-diagrams-core";
import { PortTypes } from "../../../../core/constants";
import { PortName } from "../../../../core/serialiser/interfaces";
import BaseModel from "../../common/base-model";
import { createPortModel } from "../../common/factory";

/**
 * Options for Code model
 */
export interface CodeBlockModelOptions extends BaseModelOptions {
    inputs?: string[];
    outputs?: string[];
    params?: string[];
}

/**
 * Interface for code block data
 */
interface CodeBlockData {
    code: string;
    frequency: string;
    params?: PortName[],
    ports: {
        in: PortName[],
        out: PortName[]
    }
}

/**
 * Data model for Code block
 */
export class CodeBlockModel extends BaseModel<CodeBlockData, NodeModelGenerics & CodeBlockModelOptions> {


    constructor(options: CodeBlockModelOptions) {
        super({
            ...options,
            type: 'basic.code'
        });
        // default code shown on editor
        const code = (
`def main(inputs, outputs, parameters, synchronise):
    pass`);
        // Initialise data
        this.data = {
            code: code,
            frequency: '30',
            params: options.params?.map((port) => {
                return { name: port }
            }) || [],
            ports: {
                in: options.inputs?.map((port) => {
                    return { name: port }
                }) || [],
                out: options.outputs?.map((port) => {
                    return { name: port }
                }) || []
            },
            size: {
                width: '',
                height: ''
            }
        }

        // Create Input ports for each input option
        options.inputs?.forEach((port) => {
            this.addPort(
                createPortModel({
                    in: true,
                    name: port,
                    alignment: PortModelAlignment.LEFT,
                    type: PortTypes.INPUT,
                    label: port
                })
            );
        });

        // Create Output ports for each output option
        options.outputs?.forEach((port) => {
            this.addPort(
                createPortModel({
                    in: false,
                    name: port,
                    alignment: PortModelAlignment.RIGHT,
                    type: PortTypes.OUTPUT,
                    label: port
                })
            )
        });

        // Create Parameter ports for each parameter option
        options.params?.forEach((port) => {
            this.addPort(
                createPortModel({
                    in: true,
                    name: port,
                    alignment: PortModelAlignment.TOP,
                    type: PortTypes.PARAM,
                    label: port
                })
            )
        });
    }

    /**
     * Generate inputs from list of output port names.
     * @returns List of input ports
     */
    getInputs() {
        return this.getData().ports.in?.map((port) => this.getPort(port.name)) || [];
    }

    getInputNames() {
        return this.getData().ports.in?.map((port) => port.name) || [];
    }

    /**
     * Generate outputs from list of output port names.
     * @returns List of output ports
     */
    getOutputs() {
        return this.getData().ports.out?.map((port) => this.getPort(port.name)) || [];
    }

    getOutputNames() {
        return this.getData().ports.out?.map((port) => port.name) || [];
    }

    /**
     * Generate outputs from list of parameter port names.
     * @returns List of parameter ports
     */
    getParameters() {
        return this.getData().params?.map((port) => this.getPort(port.name)) || [];
    }

    getParameterNames() {
        return this.getData().params?.map((port) => port.name) || [];
    }

    /**
     * Getter for data object
     * @returns Data object
     */
    getData(): CodeBlockData {
        return this.data;
    }

    /**
     * Set the width and height of block
     * @param width Width of block
     * @param height Height of block
     */
    setSize(width: number, height: number): void {
        const size = {
            width: width.toString() + 'px',
            height: height.toString() + 'px'
        }
        this.data.size = size;

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