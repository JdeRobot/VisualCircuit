import Card from "@material-ui/core/Card";
import CardContent from "@material-ui/core/CardContent";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import React from "react";
import Editor from "../../../core/editor";
import { GlobalState } from "../../../core/store";
import ArrowedTooltip from "../../utils/tooltip";
import BaseBlock from "../common/base-block";
import BasePort from "../common/base-port";
import { PackageBlockModel } from "./package-model";
import './styles.scss';


/**
 * Interface for Package block widget props
 */
export interface PackageBlockWidgetProps {
    node: PackageBlockModel;
    engine: DiagramEngine;
    editor: Editor;
}

/**
 * Widget for the Package block
 */
export class PackageBlockWidget extends React.Component<PackageBlockWidgetProps> {

    static contextType = GlobalState;

    /**
     * Callback for when a package block is double clicked.
     * It opens the package as the model in current editor.
     * And stores the lockstate and showing package in the widget state.
     */
    openPackage() {
        const {setState} = this.context;
        this.props.editor.openPackage(this.props.node);
        setState({
            locked: this.props.editor.locked(),
            showingPackage: true
        })
    }

    render() {
        return (
            <BaseBlock 
                selected = {this.props.node.isSelected()}>
                <div onDoubleClick={() => this.openPackage()}>
                    <Card variant='outlined' className="block-package" raised>
                        <CardContent className='p-0 h-100'>
                            <div className='grid-container h-100'>
                                <div className='block-package-inputs'>
                                    {this.props.node.getInputs().map((port, index) => {
                                        return (
                                        <BasePort className='package-input-port'
                                            port={port!} 
                                            engine={this.props.engine} 
                                            isInput={true}
                                            key={port?.getID()}>
                                        </BasePort>
                                        );
                                    })}
                                </div>
                                <div className='block-package-image-container'>
                                    <ArrowedTooltip 
                                        title={this.props.node.info.description} 
                                        aria-label={this.props.node.info.description}
                                        enterDelay={1000}>
                                    <img 
                                        src={this.props.node.getImage()} 
                                        className='block-package-image' 
                                        draggable={false}
                                        alt={this.props.node.info.name}/>
                                    </ArrowedTooltip>
                                </div>
                                <div className='block-package-outputs'>
                                    {this.props.node.getOutputs().map((port) => {
                                        return (
                                        <BasePort className='package-output-port'
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

}