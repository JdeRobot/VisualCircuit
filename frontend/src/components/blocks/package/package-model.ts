import BaseModel from "../common/base-model";
import { NodeModelGenerics, PortModelAlignment } from "@projectstorm/react-diagrams";
import { BaseModelOptions, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { ProjectDesign } from "../../../core/serialiser/interfaces";
import { createPortModel } from "../common/factory";
import { PortTypes, ProjectInfo } from "../../../core/constants";

export interface PackageBlockData {
    name: string;
}

export interface PackageBlockModelOptions extends BaseModelOptions {
    design: ProjectDesign;
    model: any;
    info: ProjectInfo;
}

export class PackageBlockModel extends BaseModel<PackageBlockData, NodeModelGenerics & PackageBlockModelOptions> {

    public model: any;
    public info: ProjectInfo;
    public design: ProjectDesign;
    private inputs: string[] = [];
    private outputs: string[] = [];

    constructor(options: PackageBlockModelOptions) {
		super({
			...options,
			type: 'block.package'
		});
        this.model = options.model;
        this.info = options.info;
        this.design = options.design;
        options.design.graph.blocks.forEach((block) => {
            if (block.type === 'basic.input') {
                this.addPort(
                    createPortModel({
                        in: true,
                        name: block.id,
                        alignment: PortModelAlignment.LEFT,
                        label: block.data.name || block.id,
                        type: PortTypes.INPUT
                    })
                );
                this.inputs.push(block.id);
            } else if (block.type === 'basic.output') {
                this.addPort(
                    createPortModel({
                        in: false,
                        name: block.id,
                        alignment: PortModelAlignment.RIGHT,
                        label: block.data.name || block.id,
                        type: PortTypes.OUTPUT
                    })
                );
                this.outputs.push(block.id);
            }
        })
    }

    getInputs() {
        return this.inputs.map((port) => this.getPort(port));
    }

    getOutputs() {
        return this.outputs.map((port) => this.getPort(port));
    }

    getImage() {
        return this.info.image;
    }

    serialize() {
        return {
            ...super.serialize(),
            data: this.getData(),
            model: this.model,
            info: this.info,
            design: this.design
        }
    }

    deserialize(event: DeserializeEvent<this>): void {
        super.deserialize(event);
        this.data = event.data.data;
        this.model = event.data.model;
        this.info = event.data.info;
        this.design = event.data.design;
    }

}