import { Card, CardContent, TextField } from '@material-ui/core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import React, { ChangeEvent } from 'react';
import Editor from '../../../../core/editor';
import { GlobalState } from '../../../../core/store';
import BaseBlock, { ContextOption } from '../../common/base-block';
import BasePort from '../../common/base-port';
import { ConstantBlockModel } from './constant-model';
import {unitConversion} from '../../../utils/tooltip/index'
import './styles.scss';

/**
 * Interface for constant block widget props
 */
export interface ConstantBlockWidgetProps {
    node: ConstantBlockModel;
    engine: DiagramEngine;
    editor: Editor;
}

/**
 * Interface for constant block widget state
 */
export interface ConstantBlockWidgetState {
    // Text field value of constant block
    value: any
}

/**
 * Widget for the constant block
 */
export class ConstantBlockWidget extends React.Component<ConstantBlockWidgetProps, ConstantBlockWidgetState> {

    static contextType = GlobalState;
    readonly contextOptions: ContextOption[] = [{key: 'rename', label: 'Rename'}, {key: 'delete', label: 'Delete'}];

    constructor(props: ConstantBlockWidgetProps) {
        super(props);
        this.state = {
            value: this.props.node.data.value || ''
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
            case 'rename':
                this.props.editor.editNode(this.props.node);
                break;
            default:
                break;
        }
    }

    render() {
        const { state } = this.context;
        return (
            <BaseBlock selected={this.props.node.isSelected()} contextOptions={this.contextOptions}
                contextHandler={this.onContextMenu.bind(this)}>
                <div>
                    <Card variant='outlined' className="block-basic-constant" raised>
                        <CardContent className='p-0'>
                            <p className='text-center'>{this.props.node.data.name}</p>
                            <div className='block-basic-constant-input'>
                                <TextField
                                    variant="outlined"
                                    onChange={this.handleInput}
                                    value={this.state.value}
                                    size='small'
                                    InputProps={{
                                        readOnly: state.locked,
                                    }}
                                />
                            </div>
                            <div style={{ display: 'flex', justifyContent: 'center' }}>
                                <BasePort className='constant-output-port'
                                    port={this.props.node.getPort()}
                                    engine={this.props.engine}
                                    isInput={false}>
                                </BasePort>
                            </div>

                        </CardContent>
                    </Card>

                </div>
            </BaseBlock>
        );
    }

    /**
     * Callback when constant input field changes
     * @param event Change event from constant input field
     */
    handleInput = (event: ChangeEvent<HTMLInputElement>) => {
        const actual_val = unitConversion(event.target.value);
        this.setState({ value: event.target.value});
        if (this.props.node.data) {
            this.props.node.data.value = actual_val
        }
    }
}