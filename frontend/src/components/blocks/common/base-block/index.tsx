import React from 'react';

import './styles.scss';

/**
 * Base widget properties
 */
interface BaseBlockProps{
    selected: boolean;
}

/**
 * Base block widget. Used in all other blocks.
 */
const BaseBlock : React.FC<BaseBlockProps> = (props) => {
    // If selected is true, show a border.
    const selectedClass = props.selected ? 'selected': '';
    return <div className={`block-container ${selectedClass}`}>{props.children}</div>;
}

export default BaseBlock;