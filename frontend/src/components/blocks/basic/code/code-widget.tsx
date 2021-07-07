import { Card, CardContent } from '@material-ui/core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import CSS from 'csstype';
import React, { ChangeEvent, MouseEventHandler, WheelEventHandler } from 'react';
import { GlobalState } from '../../../../core/store';
import BaseBlock from '../../common/base-block';
import BasePort from '../../common/base-port';
import { CodeBlockModel } from './code-model';
import './styles.scss';


export interface CodeBlockWidgetProps {
    node: CodeBlockModel;
    engine: DiagramEngine;
}

export interface CodeBlockWidgetState {
    code: string
}

export class CodeBlockWidget extends React.Component<CodeBlockWidgetProps, CodeBlockWidgetState> {

    static contextType = GlobalState;

    constructor(props: CodeBlockWidgetProps) {
        super(props);
        this.state = {
            code: this.props.node.data.code || ''
        };
    }

    render() {
        const { state } = this.context;
        const textAreaStyle: CSS.Properties = {};
        const width = this.props.node.data.size?.width;
        const height = this.props.node.data.size?.height;
        if (width && height) {
            textAreaStyle.width = width;
            textAreaStyle.height = height;
        }

        return (
            <BaseBlock selected={this.props.node.isSelected()}>
                <div>
                    <Card variant='outlined' className="block-basic-code" raised>
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
                                <div className='block-basic-code-textarea-container'>
                                    <textarea
                                        aria-label='code-block'
                                        className='block-basic-code-textarea'
                                        value={this.state.code}
                                        onChange={this.handleInput}
                                        onMouseDown={this.blockMouseEvents}
                                        onMouseUp={this.handleResize}
                                        onWheel={this.blockScrollEvents}
                                        readOnly={state.locked}
                                        style={textAreaStyle}
                                        spellCheck="false"
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

    handleInput = (event: ChangeEvent<HTMLTextAreaElement>) => {
        this.setState({ code: event.target.value });
        this.props.node.data.code = event.target.value;
    }

    blockMouseEvents: MouseEventHandler<HTMLTextAreaElement> = (event) => {
        event.stopPropagation();
    }

    blockScrollEvents: WheelEventHandler<HTMLTextAreaElement> = (event) => {
        event.stopPropagation();
    }

    handleResize: MouseEventHandler<HTMLTextAreaElement> = (event) => {
        const element = event.target as HTMLTextAreaElement;
        this.props.node.setSize(element.clientWidth, element.clientHeight);
    }


}