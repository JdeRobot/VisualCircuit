import React from 'react';

import './styles.scss';

interface BaseBlockProps{
    selected: boolean;
}

const BaseBlock : React.FC<BaseBlockProps> = (props) => {
    const selectedClass = props.selected ? 'selected': '';
    return <div className={`block-container ${selectedClass}`}>{props.children}</div>;
}

export default BaseBlock;