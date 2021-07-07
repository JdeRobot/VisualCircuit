import Card from "@material-ui/core/Card";
import CardContent from "@material-ui/core/CardContent";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import React from "react";
import BaseBlock from "../../common/base-block";
import BasePort from "../../common/base-port";
import { InputBlockModel } from "./input-model";
import './styles.scss';




export interface InputBlockWidgetProps {
    node: InputBlockModel;
    engine: DiagramEngine;
}

export class InputBlockWidget extends React.Component<InputBlockWidgetProps> {

    render() {
        return (
            <BaseBlock selected={this.props.node.isSelected()}>
                <div>
                    <Card variant='outlined' className="block-basic-input" raised>
                        <CardContent className='p-0'>
                            <div style={{ display: 'flex', alignItems: 'center' }}>
                                <p className='text-center' style={{ flex: 1 }}>{this.props.node.data.name}</p>
                                <BasePort className='input-output-port'
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
}