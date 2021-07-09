import { DiagramModel } from "@projectstorm/react-diagrams-core";
import BaseModel from "../../components/blocks/common/base-model";
import { PackageBlockModel } from "../../components/blocks/package/package-model";
import { PortTypes, ProjectInfo, PROJECT_BOARD_NAME, VERSION } from "../constants";
import { makeid } from "../utils";
import { Block, Dependency, Wire } from "./interfaces";




export function convertToOld(model: DiagramModel, projectInfo: ProjectInfo) {
    const { blocks, dependencies } = getBlocksAndDependencies(model);
    const data = {
        version: VERSION,
        package: projectInfo,
        design: {
            board: PROJECT_BOARD_NAME,
            graph: {
                blocks: blocks,
                wires: getWires(model)
            }
        },
        dependencies: dependencies
    }

    console.log(data);
    return data;
}

function getWires(model: DiagramModel): Wire[] {

    const wires: Wire[] = [];
    model.getLinks().forEach((link) => {
        if (link.getSourcePort() && link.getTargetPort()) {
            const port1 = link.getSourcePort();
            const port2 = link.getTargetPort();

            const source = port1.getType() === PortTypes.OUTPUT ? port1 : port2;
            const target = port1.getType() !== PortTypes.OUTPUT ? port1 : port2;
            wires.push(
                {
                    source: {
                        block: source.getParent().getID(),
                        port: source.getName()
                    },
                    target: {
                        block: target.getParent().getID(),
                        port: target.getName()
                    }
                }
            );
        }
    })

    return wires;
}

function getBlocksAndDependencies(model: DiagramModel) {
    const blocks: Block[] = [];
    const dependencies: { [k: string]: Dependency } = {};
    model.getNodes().forEach((node) => {

        if (node instanceof BaseModel) {
            const block = {
                id: node.getID(),
                type: node.getType(),
                data: node.getData(),
                position: {
                    x: node.getPosition().x,
                    y: node.getPosition().y
                }
            }

            if (node instanceof PackageBlockModel) {
                block.type = makeid(40);
                dependencies[block.type] = {
                    package: node.info,
                    design: node.design
                }

            }
            blocks.push(block);
        }
    })

    return { blocks: blocks, dependencies: dependencies };
}