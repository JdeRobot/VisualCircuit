import { ProjectInfo } from "../constants";

export interface Wire {
    source: {
        block: string;
        port: string;
    },
    target: {
        block: string;
        port: string;
    }
}

export interface PortName {
    name: string;
}

export interface Block {
    id: string;
    type: string;
    position: {
        x: number,
        y: number
    },
    data: any
}

export interface ProjectDesign {
    board: string;
    graph: {
        blocks: Block[];
        wires: Wire[];
    }
}

export interface Dependency {
    package: ProjectInfo;
    design: ProjectDesign;
}