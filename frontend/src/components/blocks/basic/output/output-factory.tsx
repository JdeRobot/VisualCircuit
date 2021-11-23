import React from 'react';
import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import { OutputBlockModel, OutputBlockModelOptions } from './output-model';
import { OutputBlockWidget } from './output-widget';
import Editor from '../../../../core/editor';


/**
 * Factory for Output block
 */
export class OutputBlockFactory extends AbstractReactFactory<OutputBlockModel, DiagramEngine> {

    private editor: Editor;
    constructor(editor: Editor) {
        super('basic.output');
        this.editor = editor;
    }

    generateModel(event: GenerateModelEvent): OutputBlockModel {
        const options = event.initialConfig as OutputBlockModelOptions
        return new OutputBlockModel(options);
    }

    generateReactWidget(event: GenerateWidgetEvent<OutputBlockModel>): JSX.Element {
        return <OutputBlockWidget engine={this.engine} node={event.model} editor={this.editor}/>;
    }
}