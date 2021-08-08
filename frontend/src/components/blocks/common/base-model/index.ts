import { NodeModelGenerics } from "@projectstorm/react-diagrams";
import { NodeModel } from "@projectstorm/react-diagrams-core";

/**
 * Abstract base model. It exposes getter for data, since all models store data in this object.
 */
class BaseModel<D, G extends NodeModelGenerics = NodeModelGenerics> extends NodeModel<G> {

    public data: D | any = {};

    public getData(): D | any {
        return this.data;
    }

}

export default BaseModel;