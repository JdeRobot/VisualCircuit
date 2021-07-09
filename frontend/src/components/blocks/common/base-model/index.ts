import { NodeModelGenerics } from "@projectstorm/react-diagrams";
import { NodeModel } from "@projectstorm/react-diagrams-core";

class BaseModel<D, G extends NodeModelGenerics = NodeModelGenerics> extends NodeModel<G> {

    public data: D | any = {};

    // constructor(options: G['OPTIONS']) {
    //     super(options);
    //     this.data = {:};
    // }

    public getData(): D | any {
        return this.data;
    }

}

export default BaseModel;