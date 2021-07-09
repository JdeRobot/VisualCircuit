import { DiagramEngine, PortModel, PortModelAlignment, PortModelGenerics, PortWidget } from "@projectstorm/react-diagrams";
import React, { Key } from "react";
import { PortTypes } from "../../../../core/constants";
import { BasePortModel } from "./port-model";

import './styles.scss';

export type BasePortProps = {
    port: PortModel<PortModelGenerics>;
    engine: DiagramEngine;
    isInput: boolean;
    className: string;
    key?: Key
}

const BasePort: React.FC<BasePortProps> = (props) => {

    const port: BasePortModel = props.port as BasePortModel;
    const alignment = port.getOptions().alignment;
    let alignmentClass = '';
    let portClass = ''

    switch (port.getType()) {
        case PortTypes.INPUT:
            portClass = 'custom-input-port';
            break;
        case PortTypes.OUTPUT:
            portClass = 'custom-output-port';
            break;
        case PortTypes.PARAM:
            portClass = 'custom-param-port';
            break;
        default:
            portClass = props.isInput ? 'custom-input-port': 'custom-output-port';
            break;
    }

    switch (alignment) {
        case PortModelAlignment.RIGHT:
            alignmentClass = 'right-aligned-port'
            break;
        case PortModelAlignment.BOTTOM:
            alignmentClass = 'bottom-aligned-port'
            break;
        case PortModelAlignment.LEFT:
            alignmentClass = 'left-aligned-port'
            break;
        case PortModelAlignment.TOP:
            alignmentClass = 'top-aligned-port'
            break;
        default:
            break;
    }

    return (<div className={`port-container ${alignmentClass}`}>
                <PortWidget {...props}>
                    <div className={`custom-port ${portClass}`}></div>
                </PortWidget>
                {!port.hideLabel && port.getOptions().label && <div>{port.getOptions().label}</div>}
            </div>);
}

export default BasePort;