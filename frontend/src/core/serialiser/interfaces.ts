import { ProjectInfo } from "../constants";

/**
 * Interface for connections between blocks
 */
export interface Wire {
    // Block providing the output
    source: {
        // Block ID
        block: string;
        // Port ID
        port: string;
        name: string;
    },
    // Block where input is received
    target: {
        // Block ID
        block: string;
        // Port ID
        port: string;
        name: string;
    }
}

/**
 * Interface for port names
 */
export interface PortName {
    name: string;
}

/**
 * Interface for Block (node)
 * These are mandatory fields, some of them might have extra fields.
 */
export interface Block {
    // Unique ID
    id: string;
    // Type of block. Eg. Constant, Code, blocks.opencv.blur
    type: string;
    // Current position in editor
    position: {
        x: number,
        y: number
    },
    // Any map object for block data
    data: any
}

/**
 * Interface for model data to be stored for backend.
 */
export interface ProjectDesign {
    // Meta info about how project is built. Currently Python3-noetic
    board: string;
    // Graph structure containing blocks array. (Nodes of this graph)
    // And wires represent the connection between blocks.
    graph: {
        blocks: Block[];
        wires: Wire[];
    }
}

/**
 * Interface for package blocks included as dependency in a project.
 */
export interface Dependency {
    // Meta info about project (model)
    package: ProjectInfo;
    // Data about blocks, connections and nested dependency.
    design: ProjectDesign;
    dependencies: Dependency;
}