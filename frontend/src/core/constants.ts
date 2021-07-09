export const PROJECT_FILE_EXTENSION = '.vc3';
export const VERSION = '3.0';
export const PROJECT_BOARD_NAME = 'Python3-Noetic';


export enum PortTypes {
    INPUT = 'port.input',
    OUTPUT = 'port.output',
    PARAM = 'port.parameter' 
}

export interface ProjectInfo {
    name: string;
    version: string;
    description: string;
    author: string;
    image: string;
}