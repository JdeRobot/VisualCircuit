import React from 'react';
import { ConstantBlockModel, ConstantBlockModelOptions } from './constant-model';
import { ConstantBlockWidget } from './constant-widget';
import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import Editor from '../../../../core/editor';


/**
 * Factory for Constant block
 */
export class ConstantBlockFactory extends AbstractReactFactory<ConstantBlockModel, DiagramEngine> {

    private editor: Editor;
    constructor(editor: Editor) {
        super('basic.constant');
        this.editor = editor;
    }

    generateModel(event: GenerateModelEvent): ConstantBlockModel {
        const options = event.initialConfig as ConstantBlockModelOptions
        
        return new ConstantBlockModel(options);
    }

    generateReactWidget(event: GenerateWidgetEvent<ConstantBlockModel>): JSX.Element {
        return <ConstantBlockWidget engine={this.engine} node={event.model} editor={this.editor}/>;
    }
}