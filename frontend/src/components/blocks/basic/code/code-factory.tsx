import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import React from 'react';
import { CodeBlockModel, CodeBlockModelOptions } from './code-model';
import { CodeBlockWidget } from './code-widget';


/**
 * Factory for Code block
 */
export class CodeBlockFactory extends AbstractReactFactory<CodeBlockModel, DiagramEngine> {

    constructor() {
        super('basic.code');
    }

    generateModel(event: GenerateModelEvent): CodeBlockModel {
        const options = event.initialConfig as CodeBlockModelOptions
        return new CodeBlockModel(options);
    }

    generateReactWidget(event: GenerateWidgetEvent<CodeBlockModel>): JSX.Element {
        return <CodeBlockWidget engine={this.engine} node={event.model} />;
    }
}