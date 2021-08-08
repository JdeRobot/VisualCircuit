import React from 'react';

import './styles.scss';

/**
 * Container for the editor component
 * @param props React component props
 */
const CanvasContainer: React.FC = (props) => {
    return <div id='canvas-container'>{props.children}</div>;
}

export default CanvasContainer;