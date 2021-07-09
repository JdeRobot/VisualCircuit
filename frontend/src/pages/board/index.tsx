import Fab from '@material-ui/core/Fab';
import ArrowBackIcon from '@material-ui/icons/ArrowBack';
import LockIcon from '@material-ui/icons/Lock';
import LockOpenIcon from '@material-ui/icons/LockOpen';
import {
    CanvasWidget
} from '@projectstorm/react-canvas-core';
import React from 'react';
import CanvasContainer from '../../components/canvas/canvas-container';
import Editor from '../../core/editor';
import { useGlobalState } from '../../core/store';
import './styles.scss';


interface BoardProps {
    editor: Editor;
}


function Board(props: BoardProps) {

    const { editor } = props;
    const { state } = useGlobalState();

    return <div id='board'>
        {state.showingPackage && <Toolbar editor={editor} />}
        <CanvasContainer>
            <CanvasWidget engine={editor.engine} className='canvas' />
        </CanvasContainer>
    </div>
}

const Toolbar: React.FC<{ editor: Editor }> = (props) => {

    const { state, setState } = useGlobalState();

    const setLock = (lock: boolean) => {
        props.editor.setLock(lock);
        setState({ ...state, locked: lock });
    }

    const goBack = () => {
        props.editor.goToPreviousModel()
        setState({...state, showingPackage: props.editor.showingPackage()});
    }

    return (
        <div id='toolbar'>
            <Fab
                variant="extended"
                size='small'
                className='toolbar-button'
                onClick={() => goBack()}>
                <ArrowBackIcon />
                Back
            </Fab>
            <div className='flex-spacer'></div>
            {state.locked &&
                <Fab
                    size='small'
                    className='toolbar-button'
                    onClick={() => setLock(false)}>
                    <LockIcon />
                </Fab>
            }

            {!state.locked &&
                <Fab
                    size='small'
                    className='toolbar-button'
                    onClick={() => setLock(true)}>
                    <LockOpenIcon />
                </Fab>
            }

        </div>
    );
}

export default Board;