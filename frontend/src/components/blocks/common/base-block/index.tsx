import useTheme from '@material-ui/core/styles/useTheme';
import { ControlledMenu, MenuItem, useMenuState } from '@szhsin/react-menu';
import React, { MouseEvent, useState } from 'react';

import './styles.scss';


type ContextHandlerFunction = (key: string) => void;

export interface ContextOption {
    key: string;
    label: string;
}

/**
 * Base widget properties
 */
interface BaseBlockProps{
    selected: boolean;
    contextOptions?: ContextOption[];
    contextHandler?: ContextHandlerFunction
}

/**
 * Base block widget. Used in all other blocks.
 */
const BaseBlock : React.FC<BaseBlockProps> = (props) => {
    // If selected is true, show a border.
    const selectedClass = props.selected ? 'selected': '';
    const options = props.contextOptions ? props.contextOptions : [];
    const contextHandler = options.length > 0 && props.contextHandler ? props.contextHandler : (_: string) => {}
    const theme = useTheme();
    const isDark = theme.palette.type === 'dark';
    const { toggleMenu, ...menuProps } = useMenuState();
    const [anchorPoint, setAnchorPoint] = useState({ x: 0, y: 0 });

    /**
     * Handler for right mouse click on a block.
     * @param event Mouse click event
     */
    const openContextMenu = (event: MouseEvent<HTMLDivElement>) => {
            event.preventDefault();
            setAnchorPoint({ x: event.clientX, y: event.clientY });
            toggleMenu('initial');
    }

    return <div className={`block-container ${selectedClass}`} onContextMenu={openContextMenu}>
            {props.children}
            {options.length > 0 &&
            <ControlledMenu {...menuProps} anchorPoint={anchorPoint} theming={isDark ? 'dark' : undefined}
                onClose={() => toggleMenu('initial')}>
                {options.map((option, index) => <MenuItem key={index} onClick={() => {contextHandler(option.key)}}>{option.label}</MenuItem>)}
            </ControlledMenu>
            }
        </div>;
}

export default BaseBlock;