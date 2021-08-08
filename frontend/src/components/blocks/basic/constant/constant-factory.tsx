import React from 'react';
import { ConstantBlockModel, ConstantBlockModelOptions } from './constant-model';
import { ConstantBlockWidget } from './constant-widget';
import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';


/**
 * Factory for Constant block
 */
export class ConstantBlockFactory extends AbstractReactFactory<ConstantBlockModel, DiagramEngine> {

    constructor() {
        super('basic.constant');
    }

    generateModel(event: GenerateModelEvent): ConstantBlockModel {
        const options = event.initialConfig as ConstantBlockModelOptions
        
        return new ConstantBlockModel(options);
    }

    generateReactWidget(event: GenerateWidgetEvent<ConstantBlockModel>): JSX.Element {
        return <ConstantBlockWidget engine={this.engine} node={event.model} />;
    }
}