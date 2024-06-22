import { DeleteItemsAction } from "@projectstorm/react-canvas-core";
import createEngine, { DiagramEngine, DiagramModel, NodeModel, RightAngleLinkFactory } from "@projectstorm/react-diagrams";
import { CodeBlockFactory } from "../components/blocks/basic/code/code-factory";
import { ConstantBlockFactory } from "../components/blocks/basic/constant/constant-factory";
import { InputBlockFactory } from "../components/blocks/basic/input/input-factory";
import { OutputBlockFactory } from "../components/blocks/basic/output/output-factory";
import { BaseInputPortFactory, BaseOutputPortFactory, BaseParameterPortFactory } from "../components/blocks/common/base-port/port-factory";
import { createBlock, editBlock, getInitialPosition, loadPackage,createComposedBlock } from "../components/blocks/common/factory";
import { PackageBlockFactory } from "../components/blocks/package/package-factory";
import { PackageBlockModel } from "../components/blocks/package/package-model";
import createProjectInfoDialog from "../components/dialogs/project-info-dialog";
import { ProjectInfo,BlockData } from "./constants";
import { convertToOld } from "./serialiser/converter";
import BaseModel from "../components/blocks/common/base-model";
import { count } from "console";
import createBlockDialog from "../components/dialogs/blocks-dialog";


class Editor {

    private static instance: Editor;
    private currentProjectName: string;
    private projectInfo: ProjectInfo;
    private BlockData: BlockData;
    
    
    private stack: { model: DiagramModel, info: ProjectInfo, node: PackageBlockModel }[];
    private stackOfBlock: { model: DiagramModel, info: ProjectInfo}[];
    private activeModel: DiagramModel;
    private blockCount: number = 0;

    public engine: DiagramEngine;


    private constructor() {
        // Name of the project. Used as file name while downloading.
        this.currentProjectName = 'Untitled';
        // Do not register default delete action keyboard keys, because Backspace is also included in it.
        // Only Delete button is registered under register factories method.
        this.engine = createEngine({ registerDefaultDeleteItemsAction: false });
        this.activeModel = new DiagramModel();
        // Use an array as stack, to keep track of levels of circuit model.
        this.stack = [];
        this.stackOfBlock = [];
        this.engine.setModel(this.activeModel);
        this.registerFactories();
        this.projectInfo = {
            'name': '',
            'version': '',
            'description': '',
            'author': '',
            'image': ''
        };
        this.BlockData = {
            'selectedInputIds': [],
            'selectedOutputIds': []
        };
    }

    /**
     * Register factories for different blocks and links
     */
    private registerFactories() {
        // RightAngle links is not used as of now. Its for future when links might be converted to straight wires
        this.engine.getLinkFactories().registerFactory(new RightAngleLinkFactory());
        this.engine.getPortFactories().registerFactory(new BaseInputPortFactory());
        this.engine.getPortFactories().registerFactory(new BaseOutputPortFactory());
        this.engine.getPortFactories().registerFactory(new BaseParameterPortFactory());
        this.engine.getNodeFactories().registerFactory(new ConstantBlockFactory(this));
        this.engine.getNodeFactories().registerFactory(new CodeBlockFactory(this));
        this.engine.getNodeFactories().registerFactory(new InputBlockFactory(this));
        this.engine.getNodeFactories().registerFactory(new OutputBlockFactory(this));
        this.engine.getNodeFactories().registerFactory(new PackageBlockFactory(this));

        // register an DeleteItemsAction with custom keyCodes (in this case, only Delete key)
	    this.engine.getActionEventBus().registerAction(new DeleteItemsAction({ keyCodes: [46] }));
    }

    /**
     * Main entry point to get Editor object, since constructor is private.
     * @returns instance of Editor object
     */
    public static getInstance() {
        // Editor is used as a singleton across the whole application.
        if (!Editor.instance) {
            Editor.instance = new Editor();
        }
        return Editor.instance;
    }

    /**
     * Deserialise the JSON object into model instance and open the project circuit.
     * @param jsonModel : JSON object conforming to the project structure
     * Project Structure: {
     *      "editor": {...},
     *      "version": "3.0",
     *      "package": {...},
     *      "design": {...},
     *      "dependencies": {...}
     * }
     */
    public loadProject(jsonModel: any) {
        const model = new DiagramModel();
        const editor = jsonModel.editor;
        if (editor) {
            model.deserializeModel(editor, this.engine);
            this.activeModel = model;
            this.projectInfo = jsonModel.package;
            this.engine.setModel(model)
        }
    }

    /**
     * Load an empty instance of DiagramModel as the current project.
     */
    public clearProject(): void {
        this.activeModel = new DiagramModel();
        this.engine.setModel(this.activeModel);
    }

    /**
     * Serialise the model data and also VisualCircuit data as required by backend.
     * Project Structure: {
     *      "editor": {...},
     *      "version": "3.0",
     *      "package": {...},
     *      "design": {...},
     *      "dependencies": {...}
     * }
     * @returns Serialised data of the project (model) and VisualCircuit (old) format data
     */
    public serialise() {
        const data = convertToOld(this.activeModel, this.projectInfo);
        return { editor : this.activeModel.serialize(), ...data};
    }

    /**
 * Get list of blocks and dependency blocks (package blocks) present in project (model)
 * @param model Project (model) for which blocks and dependency list has to be created
 * @returns List of blocks and dependencies (package blocks)
 */
    public async processBlock(model: DiagramModel, data: BlockData): Promise<void> {
        console.log("data", data.selectedInputIds);
        // Process selected input IDs
        await Promise.all(data.selectedInputIds.map(async (id: string) => {
            console.log("selectedInputIds", id);
            const [blockid, name] = id.split(":");
            await this.addComposedBlock('basic.input', name); 
        }));
        
        // Process selected output IDs
        await Promise.all(data.selectedOutputIds.map(async (id: string) => {
            console.log("selectedOutputIds", id);
            const [blockid, name] = id.split(":");
            await this.addComposedBlock('basic.output', name); 
        }));
    }
    



    public getGInputsOutput(): [{ indexOne: number, label: string, id:string }[], { indexTwo: number, label: string,id:string }[]] {
        let counter = 0;
        let indexOne = 0;
        let indexTwo = 0;
        let valueOne: { indexOne: number, label: string, id:string }[] = [];
        let valueTwo: { indexTwo: number, label: string, id:string }[] = [];
        this.activeModel.getNodes().forEach((node) => {
            
            counter++;
            if (node instanceof BaseModel) {
                if (node.getType()==="basic.code"){
                    var data = node.getData();
                    var options = node.getOptions();
                    
                    if (data.ports && data.ports.in) {
                        data.ports.in.forEach((port: { name: string }) => {
                            indexOne++;
                            let label = `basic.code -> ${counter} : ${port.name}`;
                            var id = `${options.id}:${port.name}`;
                            valueOne.push({ indexOne, label, id });
                        });
                    }
                    if (data.ports && data.ports.out) {
                        data.ports.out.forEach((port: { name: string }) => {
                            indexTwo++;
                            let label = `basic.code -> ${counter} : ${port.name}`;
                            var id = `${options.id}:${port.name}`;
                            valueTwo.push({ indexTwo, label,id });
                        });
                    }
                }
                
                
            }
           
        })
        console.log("valueOne", valueOne);
        console.log("valueTwo", valueTwo);
        return [valueOne, valueTwo];
    }

    /**
     * Callback for the 'Edit Block' button in menu.
     * Opens a dialog box and saves the data entered to projectInfo variable.
     */
    public async editBlock(): Promise<void> {
        try {
            const data = await createBlockDialog({ isOpen: true, getGInputsOutput: this.getGInputsOutput.bind(this) });
            this.BlockData = data;
            console.log("this.BlockData", this.BlockData);
            this.stackOfBlock.push({ model: this.activeModel, info: this.projectInfo });
            await this.processBlock(this.activeModel, data); // Await the processBlock call
        } catch (error) {
            console.log('Block dialog closed');
        }
    }
    
    

    /**
     * Getter for Project Name
     * @returns Project name
     */
    public getName(): string {
        if (this.projectInfo.name) {
            this.currentProjectName = this.projectInfo.name;
        }
        return this.currentProjectName;
    }

    /**
     * Add the given type of block to the current project / model.
     * @param name : Name / type of the block to add to model.
     */
    public async addBlock(name: string): Promise<void> {
        this.blockCount += 1;
        const block = await createBlock(name, this.blockCount);
        if (block) {
            // Get a default position and set it as blocks position
            // TODO: Better way would be to get an empty position dynamically or track mouse's current position.
            block.setPosition(...getInitialPosition())
            this.activeModel.addNode(block);
            // Once the block is added, the page has to rendered again, this is done by repainting the canvas.
            this.engine.repaintCanvas();
        }
    }

    /**
     * Add the given type of block for composed.
     * @param name : Name / type of the block to add to model.
     */
    public async addComposedBlock(type: string,name: string): Promise<void> {
        this.blockCount += 1;
        const block = await createComposedBlock(type,name);
        if (block) {
            // Get a default position and set it as blocks position
            // TODO: Better way would be to get an empty position dynamically or track mouse's current position.
            block.setPosition(...getInitialPosition())
            this.activeModel.addNode(block);

        }
    }

    /**
     * Callback for the 'Edit Project Information' button in menu.
     * Opens a dialog box and saves the data entered to projectInfo variable.
     */
    public async editProjectInfo(): Promise<void> {
        // Helper to open Project Info dialog box
        createProjectInfoDialog({isOpen: true, ...this.projectInfo})
        .then((data) => {
            this.projectInfo = data; 
        })
        .catch(() => {
            console.log('Project Info dialog closed');
        });
    }

    /**
     * Adds a project as a block to the current project
     * @param jsonModel JSON object conforming to the project structure
     * Project Structure: {
     *      "editor": {...},
     *      "version": "3.0",
     *      "package": {...},
     *      "design": {...},
     *      "dependencies": {...}
     * }
     */
    public addAsBlock(jsonModel: any) {
        // Helper to convert JSON object to block.
        const block = loadPackage(jsonModel);
        // Get a default position and set it as blocks position
        // TODO: Better way would be to get an empty position dynamically or track mouse's current position.
        if (block) {
            block.setPosition(...getInitialPosition())
            this.activeModel.addNode(block);
            // Once the block is added, the page has to rendered again, this is done by repainting the canvas.
            this.engine.repaintCanvas();
        }
    }

    /**
     * Open a block as current project (model)
     * @param node Block to be opened
     */
    public openPackage(node: PackageBlockModel) {
        // Store the current project (model), project info and the block asked to open in stack, 
        // so that it can be restored later. 
        this.stack.push({model: this.activeModel, info: this.projectInfo, node: node});
        // Create a new model and deserialise the block into it.
        const model = new DiagramModel();
        const editor = node.model;
        if (editor) {
            model.deserializeModel(editor, this.engine);
            this.activeModel = model;
            this.projectInfo = node.info;
            // Set the block as the current project (model)
            this.engine.setModel(model)
            // By default lock the project
            this.setLock(true);
        }
    }


    /**
     * Check whether currently any package block is opened.
     * @returns True if there is a model in the stack.
     */
    public showingPackage() {
        return this.stack.length > 0;
    }


    /**
     * Get status of model lock 
     * @returns Whether the current project (model) is locked from any editing
     */
    public locked(): boolean {
        return this.activeModel.isLocked();
    }

    /**
     * Set the status of model lock
     * @param lock True if project (model) has to be locked.
     */
    public setLock(lock: boolean) {
        this.activeModel.setLocked(lock);
    }

    /**
     * Go one level higher in the model stack.
     * When back button is pressed while viewing/editing a model, the current project is changed to its
     * parent model.
     */
    public goToPreviousModel() {
        // Check if there is anything in the stack
        if (this.stack.length) {
            // Since the model could have changed, get new data for backend.
            const data = convertToOld(this.activeModel, this.projectInfo);
            // Get the parent model, project info and the current block being modified from the stack
            const {model, info, node} = this.stack.pop()!;
            // Assign the modified data to the stored block object, as this is the one used in project.
            node.design = data.design;
            node.model = this.activeModel.serialize();
            this.activeModel = model;
            this.projectInfo = info;
            // Change the current model to the model got from the stack.
            this.engine.setModel(this.activeModel);

            this.setLock(false);
        }
    }

    /**
     * Delete a block node, if current model is not locked.
     * @param node 
     */
    public removeNode(node: NodeModel) {
        if (!this.locked()) {
            node.remove()
            this.engine.repaintCanvas();
        }
    }

    /**
     * Edit a block node, if current model is not locked.
     * @param node 
     */
    public async editNode<T extends NodeModel>(node: T) {
        if (!this.locked()) {
            await editBlock(node);
            this.engine.repaintCanvas();
        }
    }
}

export default Editor;

