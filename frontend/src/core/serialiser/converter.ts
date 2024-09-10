import { DiagramModel } from "@projectstorm/react-diagrams-core";
import BaseModel from "../../components/blocks/common/base-model";
import { BasePortModel } from "../../components/blocks/common/base-port/port-model";
import { PackageBlockModel } from "../../components/blocks/package/package-model";
import { PortTypes, ProjectInfo, PROJECT_BOARD_NAME, VERSION } from "../constants";
import { makeid } from "../utils";
import { Block, Dependency, Wire } from "./interfaces";



/**
 * Convert the project (model) into VisualCircuit2 backend compatible project data structure.
 * @param model Project (model) for which data has to be generated for VisualCircuit2 backend
 * @param projectInfo Meta information about project
 * @returns VisualCircuit2 backend compatible project data structure
 */
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

    return data;
}

/**
 * Get list of connections between the blocks of the project (model)
 * @param model Project (model) for which wires (connections) list has to be created
 * @returns List of connections
 */
function getWires(model: DiagramModel): Wire[] {

    const wires: Wire[] = [];
    model.getLinks().forEach((link) => {
        if (link.getSourcePort() && link.getTargetPort()) {
            const port1 = link.getSourcePort();
            const port2 = link.getTargetPort();

            if (port1 instanceof BasePortModel && port2 instanceof BasePortModel) {
                const source = port1.getType() === PortTypes.OUTPUT ? port1 : port2;
                const target = port1.getType() !== PortTypes.OUTPUT ? port1 : port2;
                wires.push(
                    {
                        source: {
                            block: source.getParent().getID(),
                            port: source.getName(),
                            name: source.getLabel()
                        },
                        target: {
                            block: target.getParent().getID(),
                            port: target.getName(),
                            name: target.getLabel()
                        }
                    }
                );
            }
        }
    })

    return wires;
}

/**
 * Get list of blocks and dependency blocks (package blocks) present in project (model)
 * @param model Project (model) for which blocks and dependency list has to be created
 * @returns List of blocks and dependencies (package blocks)
 */
function getBlocksAndDependencies(model: DiagramModel) {
    const blocks: Block[] = [];
    const dependencies: { [k: string]: Dependency } = {};

    model.getNodes().forEach((node) => {
        if (node instanceof BaseModel) {
            const block: Block = {
                id: node.getID(),
                type: node.getType(),
                data: node.getData(),
                position: {
                    x: node.getPosition().x,
                    y: node.getPosition().y
                }
            };

            if (node instanceof PackageBlockModel) {
                const packageBlockModel = node as PackageBlockModel;
                block.type = makeid(40); // Generate a unique identifier for the dependency

                // Add the package information, design, and nested dependencies to the dependencies object
                dependencies[block.type] = {
                    package: node.info,
                    design: node.design,
                    dependencies: node.dependencies
                }
            }

            blocks.push(block);
        }
    });
    // Return the blocks and dependencies as an object
    return { blocks: blocks, dependencies: dependencies };
}
