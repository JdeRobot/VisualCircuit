import { DiagramEngine } from "@projectstorm/react-diagrams";
import { AbstractModelFactory } from '@projectstorm/react-canvas-core';
import { BaseInputPortModel, BaseOutputPortModel, BaseParameterPortModel } from "./port-model";

export class BaseInputPortFactory extends AbstractModelFactory<BaseInputPortModel, DiagramEngine> {
    constructor() {
		super('port.input');
	}

	generateModel(): BaseInputPortModel {
		return new BaseInputPortModel({
			name: 'unknown'
		});
	}
}

export class BaseOutputPortFactory extends AbstractModelFactory<BaseOutputPortModel, DiagramEngine> {
    constructor() {
		super('port.output');
	}

	generateModel(): BaseOutputPortModel {
		return new BaseOutputPortModel({
			name: 'unknown'
		});
	}
}

export class BaseParameterPortFactory extends AbstractModelFactory<BaseParameterPortModel, DiagramEngine> {
    constructor() {
		super('port.parameter');
	}

	generateModel(): BaseParameterPortModel {
		return new BaseParameterPortModel({
			name: 'unknown'
		});
	}
}