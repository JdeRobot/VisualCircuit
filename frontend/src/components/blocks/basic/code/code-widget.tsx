import { Card, CardContent, CardHeader, TextField, InputAdornment } from '@material-ui/core';
import MonacoEditor from "@monaco-editor/react";
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import CSS from 'csstype';
import React, { ChangeEvent, Fragment, MouseEventHandler, WheelEventHandler } from 'react';
import Editor from '../../../../core/editor';
import { GlobalState } from '../../../../core/store';
import BaseBlock, { ContextOption } from '../../common/base-block';
import BasePort from '../../common/base-port';
import { CodeBlockModel } from './code-model';
import {unitConversion} from '../../../utils/tooltip/index'
import './styles.scss';


/**
 * Interface for code block widget props
 */
export interface CodeBlockWidgetProps {
    node: CodeBlockModel;
    engine: DiagramEngine;
    editor: Editor;
}

/**
 * Interface for code block widget state
 */
export interface CodeBlockWidgetState {
    // For code text
    code: string,
    width: string,
    height: string,
    frequency: string
}

/**
 * Widget for the code block
 */
export class CodeBlockWidget extends React.Component<CodeBlockWidgetProps, CodeBlockWidgetState> {

    static contextType = GlobalState;
    readonly contextOptions: ContextOption[] = [{key: 'delete', label: 'Delete'}];

    constructor(props: CodeBlockWidgetProps) {
        super(props);
        this.state = {
            code: this.props.node.data.code || '',
            width: this.props.node.data.size?.width || '300px',
            height: this.props.node.data.size?.height || '300px',
            frequency: this.props.node.data.frequency
        };
    }

    /**
     * Handler for context menu
     * @param key Key cooresponding to the context menu clicked
     */
    onContextMenu(key: string) {
        switch (key) {
            case 'delete':
                this.props.editor.removeNode(this.props.node);
                break;
            default:
                break;
        }
        
    }

    render() {
        const { state } = this.context;
        const textAreaStyle: CSS.Properties = {
            width: this.state.width,
            height: this.state.height
        };

        return (
            <BaseBlock selected={this.props.node.isSelected()} contextOptions={this.contextOptions} 
            contextHandler={this.onContextMenu.bind(this)}>
                <div>
                    <Card variant='outlined' className="block-basic-code" raised>
                        <CardHeader 
                            className='block-basic-code-frequency'
                            title={
                            <Fragment>
                                <span className='block-basic-code-frequency-text'>Freq:</span>
                                <TextField variant="outlined" 
                                    defaultValue={this.state.frequency} 
                                    size="small"
                                    InputProps={{
                                        endAdornment: <InputAdornment position="start">Hz</InputAdornment>,
                                    }}
                                    type="text"
                                    margin="dense"
                                    value={this.state.frequency}
                                    onChange={this.handleFrequencyInput}
                                    onWheel={this.blockScrollEvents}
                                    className='block-basic-code-frequency-input'
                            />
                            </Fragment>
                        }>
                        </CardHeader>
                        <CardContent className='p-0'>
                            <div className='block-basic-code-parameters'>
                                {this.props.node.getParameters().map((port) => {
                                    return (
                                        <BasePort className='code-parameter-port'
                                            port={port!}
                                            engine={this.props.engine}
                                            isInput={true}
                                            key={port?.getID()}>
                                        </BasePort>
                                    );
                                })}
                            </div>
                            <div className='grid-container'>
                                <div className='block-basic-code-inputs'>
                                    {this.props.node.getInputs().map((port, index) => {
                                        return (
                                            <BasePort className='code-input-port'
                                                port={port!}
                                                engine={this.props.engine}
                                                isInput={true}
                                                key={port?.getID()}>
                                            </BasePort>
                                        );
                                    })}
                                </div>
                                <div className='block-basic-code-textarea-container' 
                                    onMouseDown={this.blockMouseEvents}
                                    onMouseUp={this.handleResize}
                                    onWheel={this.blockScrollEvents}
                                    style={textAreaStyle}>
                                    <MonacoEditor
                                        height={state.height}
                                        width={state.width}
                                        language="python"
                                        defaultValue={this.state.code}
                                        onChange={this.handleInput}
                                        theme="vs-dark"
                                        options={{
                                            "readOnly": state.locked,
                                            "minimap": {
                                                "enabled": false
                                            }
                                        }}
                                    />
                                </div>
                                <div className='block-basic-code-outputs'>
                                    {this.props.node.getOutputs().map((port) => {
                                        return (
                                            <BasePort className='code-output-port'
                                                port={port!}
                                                engine={this.props.engine}
                                                isInput={false}
                                                key={port?.getID()}>
                                            </BasePort>
                                        );
                                    })}
                                </div>
                            </div>
                        </CardContent>
                    </Card>
                </div>
            </BaseBlock>
        );
    }

    /**
     * Callback when code input field changes
     * @param event Change event from Code text area
     */
    handleInput = (value: string | undefined) => {
        this.setState({ code: value || '' });
        this.props.node.data.code = value || '';
    }

    handleFrequencyInput = (event: ChangeEvent<HTMLInputElement>) => {
        const actual_val = parseInt(unitConversion(event.target.value), 10);
        this.setState({frequency: event.target.value});
        this.props.node.data.frequency = actual_val;
    }

    /**
     * Block all mouse click events, so that its not handled by the editor.
     * This is to make sure that dragging or selecting inside the text area is handled within the text area.
     * @param event Mouse click events
     */
    blockMouseEvents: MouseEventHandler<HTMLDivElement> = (event) => {
        event.stopPropagation();
    }

    /**
     * Block all mouse scroll events from code text area, so that its not handled by the editor.
     * This is to make sure that scrolling on the text area doesnt scroll the whole window.
     * @param event Mouse scroll events
     */
    blockScrollEvents: WheelEventHandler<HTMLDivElement> = (event) => {
        event.stopPropagation();
    }

    /**
     * Callback for text area resize. When the size of block changes, store it so that it can be serialised.
     * @param event Text area resize event
     */
    handleResize: MouseEventHandler<HTMLDivElement> = (event) => {
        const element = event.target as HTMLDivElement;
        this.props.node.setSize(Math.max(element.clientWidth, 300), Math.max(element.clientHeight, 300));
    }


}