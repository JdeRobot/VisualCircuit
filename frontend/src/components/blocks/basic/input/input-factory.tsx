import React from 'react';
import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import { InputBlockModel, InputBlockModelOptions } from './input-model';
import { InputBlockWidget } from './input-widget';


export class InputBlockFactory extends AbstractReactFactory<InputBlockModel, DiagramEngine> {

    constructor() {
        super('basic.input');
    }

    generateModel(event: GenerateModelEvent): InputBlockModel {
        const options = event.initialConfig as InputBlockModelOptions
        
        return new InputBlockModel(options);
    }

    generateReactWidget(event: GenerateWidgetEvent<InputBlockModel>): JSX.Element {
        return <InputBlockWidget engine={this.engine} node={event.model} />;
    }
}