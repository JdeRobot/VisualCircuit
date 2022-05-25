import { DefaultLinkModel, DefaultPortModel, DefaultPortModelOptions, RightAngleLinkModel } from "@projectstorm/react-diagrams";
import { AbstractModelFactory, DeserializeEvent } from '@projectstorm/react-canvas-core';
import { LinkModel, PortModel } from "@projectstorm/react-diagrams";


/**
 * Abstract base port model
 */
export class BasePortModel extends DefaultPortModel {

    /** If true, does not show any name against the port **/
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

    getLabel(): string {
        return this.options.label || '';
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

/**
 * Options for port model
 */
export interface BasePortModelOptions extends DefaultPortModelOptions {
    /**
     * If true, does not show any name against the port
     */
    hideLabel?: boolean
}

/**
 * Data model Input port 
 */
export class BaseInputPortModel extends BasePortModel {
    constructor(options: BasePortModelOptions) {
        super({...options, type: 'port.input'});
    }
}

/**
 * Data model Output port 
 */
export class BaseOutputPortModel extends BasePortModel {
    constructor(options: BasePortModelOptions) {
        super({...options, type: 'port.output'});
    }
}

/**
 * Data model Parameter port 
 */
export class BaseParameterPortModel extends BasePortModel {
    constructor(options: BasePortModelOptions) {
        super({...options, type: 'port.parameter'});
    }
}