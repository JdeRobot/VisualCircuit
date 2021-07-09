import { DeleteItemsAction } from "@projectstorm/react-canvas-core";
import createEngine, { DiagramEngine, DiagramModel, RightAngleLinkFactory } from "@projectstorm/react-diagrams";
import { CodeBlockFactory } from "../components/blocks/basic/code/code-factory";
import { ConstantBlockFactory } from "../components/blocks/basic/constant/constant-factory";
import { InputBlockFactory } from "../components/blocks/basic/input/input-factory";
import { OutputBlockFactory } from "../components/blocks/basic/output/output-factory";
import { BaseInputPortFactory, BaseOutputPortFactory, BaseParameterPortFactory } from "../components/blocks/common/base-port/port-factory";
import { createBlock, getInitialPosition, loadPackage } from "../components/blocks/common/factory";
import { PackageBlockFactory } from "../components/blocks/package/package-factory";
import { PackageBlockModel } from "../components/blocks/package/package-model";
import createProjectInfoDialog from "../components/dialogs/project-info-dialog";
import { ProjectInfo } from "./constants";
import { convertToOld } from "./serialiser/converter";


class Editor {
 
    private static instance: Editor;
    private currentProjectName: string;
    private projectInfo: ProjectInfo;
    
    private stack: {model: DiagramModel, info: ProjectInfo, node: PackageBlockModel}[];
    private activeModel: DiagramModel;
    private blockCount: number = 0;

    public engine: DiagramEngine;


    private constructor() {
        this.currentProjectName = 'Untitled';
        this.engine = createEngine({ registerDefaultDeleteItemsAction: false });
        this.activeModel = new DiagramModel();
        this.stack = [];
        this.engine.setModel(this.activeModel);
        this.registerFactories();
        this.projectInfo = {
            'name': '',
            'version': '',
            'description': '',
            'author': '',
            'image': ''
        };
    }

    private registerFactories() {
        this.engine.getLinkFactories().registerFactory(new RightAngleLinkFactory());
        this.engine.getPortFactories().registerFactory(new BaseInputPortFactory());
        this.engine.getPortFactories().registerFactory(new BaseOutputPortFactory());
        this.engine.getPortFactories().registerFactory(new BaseParameterPortFactory());
        this.engine.getNodeFactories().registerFactory(new ConstantBlockFactory());
        this.engine.getNodeFactories().registerFactory(new CodeBlockFactory());
        this.engine.getNodeFactories().registerFactory(new InputBlockFactory());
        this.engine.getNodeFactories().registerFactory(new OutputBlockFactory());
        this.engine.getNodeFactories().registerFactory(new PackageBlockFactory(this));

        // register an DeleteItemsAction with custom keyCodes (in this case, only Delete key)
	    this.engine.getActionEventBus().registerAction(new DeleteItemsAction({ keyCodes: [46] }));
    }

    public static getInstance() {
        if (!Editor.instance) {
            Editor.instance = new Editor();
        }
        return Editor.instance;
    }

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

    public clearProject(): void {
        this.activeModel = new DiagramModel();
        this.engine.setModel(this.activeModel);
    }

    public serialise(): {[k: string]: any} {
        const data = convertToOld(this.activeModel, this.projectInfo);
        return { editor : this.activeModel.serialize(), ...data};
    }

    public getName(): string {
        return this.currentProjectName;
    }

    public async addBlock(name: string): Promise<void> {
        this.blockCount += 1;
        const block = await createBlock(name, this.blockCount);
        if (block) {
            block.setPosition(...getInitialPosition())
            this.activeModel.addNode(block);
            this.engine.repaintCanvas();
        }
    }

    public async editProjectInfo(): Promise<void> {
        createProjectInfoDialog({isOpen: true, ...this.projectInfo})
        .then((data) => {
            this.projectInfo = data; 
        })
        .catch(() => {
            console.log('Project Info dialog closed');
        });
    }

    public addAsBlock(jsonModel: any) {
        const block = loadPackage(jsonModel);
        if (block) {
            block.setPosition(...getInitialPosition())
            this.activeModel.addNode(block);
            this.engine.repaintCanvas();
        }
    }

    public openPackage(node: PackageBlockModel) {
        this.stack.push({model: this.activeModel, info: this.projectInfo, node: node});
        const model = new DiagramModel();
        const editor = node.model;
        if (editor) {
            model.deserializeModel(editor, this.engine);
            this.activeModel = model;
            this.projectInfo = node.info;
            this.engine.setModel(model)
            this.setLock(true);
        }
    }


    public showingPackage() {
        return this.stack.length > 0;
    }


    public locked(): boolean {
        return this.activeModel.isLocked();
    }

    public setLock(lock: boolean) {
        this.activeModel.setLocked(lock);
    }

    public goToPreviousModel() {
        if (this.stack.length) {
            const data = convertToOld(this.activeModel, this.projectInfo);
            const {model, info, node} = this.stack.pop()!;
            node.design = data.design;
            this.activeModel = model;
            this.projectInfo = info;
            this.engine.setModel(this.activeModel);
        }
    }
}

export default Editor;

