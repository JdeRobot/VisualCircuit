import Card from "@material-ui/core/Card";
import CardContent from "@material-ui/core/CardContent";
import { DiagramEngine } from "@projectstorm/react-diagrams";
import React from "react";
import Editor from "../../../../core/editor";
import BaseBlock, { ContextOption } from "../../common/base-block";
import BasePort from "../../common/base-port";
import { OutputBlockModel } from "./output-model";
import './styles.scss';


/**
 * Interface for Output block widget props
 */
export interface OutputBlockWidgetProps {
    node: OutputBlockModel;
    engine: DiagramEngine;
    editor: Editor;
}

/**
 * Widget for the Output block
 */
export class OutputBlockWidget extends React.Component<OutputBlockWidgetProps> {

    readonly contextOptions: ContextOption[] = [{key: 'rename', label: 'Rename'}, {key: 'delete', label: 'Delete'}];

    componentDidMount() {
        document.addEventListener('keydown', this.handleKeyDown); // Adding keydown event listener when component mounts
    }

    componentWillUnmount() {
        document.removeEventListener('keydown', this.handleKeyDown); // Removing keydown event listener when component unmounts
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
        return (
            <BaseBlock selected={this.props.node.isSelected()} contextOptions={this.contextOptions} 
                contextHandler={this.onContextMenu.bind(this)}>
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

       /**
     * Keydown event handler to listen for Alt+R key combination
     * @param event Keydown event
     */
       handleKeyDown = (event: KeyboardEvent) => {
        const { node } = this.props;
        if (event.altKey && (event.key === 'r' || event.key === 'R') && node.isSelected()) {
            this.props.editor.editNode(node); // Trigger rename action
        }
    }
}