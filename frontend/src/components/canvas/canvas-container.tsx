import React from 'react';

import './styles.scss';

const CanvasContainer: React.FC = (props) => {
    return <div id='canvas-container'>{props.children}</div>;
}

export default CanvasContainer;