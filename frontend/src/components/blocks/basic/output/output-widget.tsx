import Card from "@material-ui/core/Card";
import CardContent from "@material-ui/core/CardContent";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import React from "react";
import BaseBlock from "../../common/base-block";
import BasePort from "../../common/base-port";
import { OutputBlockModel } from "./output-model";
import './styles.scss';


export interface OutputBlockWidgetProps {
    node: OutputBlockModel;
    engine: DiagramEngine;
}

export class OutputBlockWidget extends React.Component<OutputBlockWidgetProps> {

    render() {
        return (
            <BaseBlock selected={this.props.node.isSelected()}>
                <div>
                    <Card variant='outlined' className="block-basic-output" raised>
                        <CardContent className='p-0'>
                            <div style={{ display: 'flex', alignItems: 'center' }}>
                                <BasePort className='output-input-port'
                                    port={this.props.node.getPort()}
                                    engine={this.props.engine}
                                    isInput={true}>
                                </BasePort>
                                <p className='text-center' style={{ flex: 1 }}>{this.props.node.data.name}</p>
                            </div>

                        </CardContent>
                    </Card>

                </div>
            </BaseBlock>
        );
    }
}