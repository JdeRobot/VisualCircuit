import { DefaultLinkModel, DefaultPortModel, DefaultPortModelOptions, RightAngleLinkModel } from "@projectstorm/react-diagrams";
import { AbstractModelFactory, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { LinkModel, PortModel } from "@projectstorm/react-diagrams";



export class BasePortModel extends DefaultPortModel {

    public hideLabel: boolean;

    constructor(options: BasePortModelOptions) {
        super(options);
        this.hideLabel = options.hideLabel || false;
    }

	createLinkModel(_factory?: AbstractModelFactory<LinkModel>): LinkModel {
		return new DefaultLinkModel();
	}

    link<T extends LinkModel>(port: PortModel, factory?: AbstractModelFactory<T>): T {
		let link = this.createLinkModel(factory);
		link.setSourcePort(this);
		link.setTargetPort(port);
		return link as T;
	}

    getOptions(): BasePortModelOptions {
        return this.options;
    }

    serialize() {
        return {
            ...super.serialize(),
            hideLabel: this.hideLabel
        };
    }

    deserialize(event: DeserializeEvent<this>): void {
        super.deserialize(event);
        this.hideLabel = event.data.hideLabel
    }
}


export interface BasePortModelOptions extends DefaultPortModelOptions {
    hideLabel?: boolean
}


export class BaseInputPortModel extends BasePortModel {
    constructor(options: BasePortModelOptions) {
        super({...options, type: 'port.input'});
    }
}

export class BaseOutputPortModel extends BasePortModel {
    constructor(options: BasePortModelOptions) {
        super({...options, type: 'port.output'});
    }
}

export class BaseParameterPortModel extends BasePortModel {
    constructor(options: BasePortModelOptions) {
        super({...options, type: 'port.parameter'});
    }
}