import { Card, CardContent, TextField } from '@material-ui/core';
import { DiagramEngine } from '@projectstorm/react-diagrams-core';
import React, { ChangeEvent } from 'react';
import { GlobalState } from '../../../../core/store';
import BaseBlock from '../../common/base-block';
import BasePort from '../../common/base-port';
import { ConstantBlockModel } from './constant-model';
import './styles.scss';


export interface ConstantBlockWidgetProps {
    node: ConstantBlockModel;
    engine: DiagramEngine;
}

export interface ConstantBlockWidgetState {
    value: any
}

export class ConstantBlockWidget extends React.Component<ConstantBlockWidgetProps, ConstantBlockWidgetState> {

    static contextType = GlobalState;

    constructor(props: ConstantBlockWidgetProps) {
        super(props);
        this.state = {
            value: this.props.node.data.value || ''
        };
    }

    render() {
        const { state } = this.context;
        return (
            <BaseBlock selected={this.props.node.isSelected()}>
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

    handleInput = (event: ChangeEvent<HTMLInputElement>) => {
        this.setState({ value: event.target.value });
        if (this.props.node.data) {
            this.props.node.data.value = event.target.value
        }
    }
}