import React from 'react';

import { AbstractReactFactory, GenerateModelEvent, GenerateWidgetEvent } from '@projectstorm/react-canvas-core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import { PackageBlockModel, PackageBlockModelOptions } from './package-model';
import { PackageBlockWidget } from './package-widget';
import Editor from '../../../core/editor';


export class PackageBlockFactory extends AbstractReactFactory<PackageBlockModel, DiagramEngine> {

    private editor: Editor;
    constructor(editor: Editor) {
        super('block.package');
        this.editor = editor;
    }

    generateModel(event: GenerateModelEvent): PackageBlockModel {
        const options = event.initialConfig as PackageBlockModelOptions
        return new PackageBlockModel(options);
    }

    generateReactWidget(event: GenerateWidgetEvent<PackageBlockModel>): JSX.Element {
        return <PackageBlockWidget engine={this.engine} node={event.model} editor={this.editor}/>;
    }
}