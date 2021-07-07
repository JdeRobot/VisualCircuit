import { BaseModelOptions, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { NodeModelGenerics } from "@projectstorm/react-diagrams";
import { PortModelAlignment } from "@projectstorm/react-diagrams-core";
import { PortTypes } from "../../../../core/constants";
import { PortName } from "../../../../core/serialiser/interfaces";
import BaseModel from "../../common/base-model";
import { createPortModel } from "../../common/factory";

export interface CodeBlockModelOptions extends BaseModelOptions {
    inputs?: string[];
    outputs?: string[];
    params?: string[];
}

interface CodeBlockData {
    code: string;
    params?: PortName[],
    ports: {
        in: PortName[],
        out: PortName[]
    }
}

export class CodeBlockModel extends BaseModel<CodeBlockData, NodeModelGenerics & CodeBlockModelOptions> {


    constructor(options: CodeBlockModelOptions) {
        super({
            ...options,
            type: 'basic.code'
        });

        this.data = {
            code: '',
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


        options.inputs?.forEach((port) => {
            this.addPort(
                createPortModel({
                    in: true,
                    name: port,
                    alignment: PortModelAlignment.LEFT,
                    type: PortTypes.INPUT
                })
            );
        });

        options.outputs?.forEach((port) => {
            this.addPort(
                createPortModel({
                    in: false,
                    name: port,
                    alignment: PortModelAlignment.RIGHT,
                    type: PortTypes.OUTPUT
                })
            )
        });

        options.params?.forEach((port) => {
            this.addPort(
                createPortModel({
                    in: true,
                    name: port,
                    alignment: PortModelAlignment.TOP,
                    type: PortTypes.PARAM
                })
            )
        });
    }

    getInputs() {
        return this.getData().ports.in?.map((port) => this.getPort(port.name)) || [];
    }

    getOutputs() {
        return this.getData().ports.out?.map((port) => this.getPort(port.name)) || [];
    }

    getParameters() {
        return this.getData().params?.map((port) => this.getPort(port.name)) || [];
    }

    getData(): CodeBlockData {
        return this.data;
    }

    setSize(width: number, height: number): void {
        const size = {
            width: width.toString() + 'px',
            height: height.toString() + 'px'
        }
        this.data.size = size;

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