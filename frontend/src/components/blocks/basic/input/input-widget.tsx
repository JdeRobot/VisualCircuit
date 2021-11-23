import Card from "@material-ui/core/Card";
import CardContent from "@material-ui/core/CardContent";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import React from "react";
import Editor from "../../../../core/editor";
import BaseBlock, { ContextOption } from "../../common/base-block";
import BasePort from "../../common/base-port";
import { InputBlockModel } from "./input-model";
import './styles.scss';



/**
 * Interface for Input block widget props
 */
export interface InputBlockWidgetProps {
    node: InputBlockModel;
    engine: DiagramEngine;
    editor: Editor;
}

/**
 * Widget for the Input block
 */
export class InputBlockWidget extends React.Component<InputBlockWidgetProps> {

    readonly contextOptions: ContextOption[] = [{key: 'rename', label: 'Rename'}, {key: 'delete', label: 'Delete'}];

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
        return (
            <BaseBlock selected={this.props.node.isSelected()} contextOptions={this.contextOptions}
                 contextHandler={this.onContextMenu.bind(this)}>
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