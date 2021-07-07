import { AppBar, Button, Toolbar, useTheme } from '@material-ui/core';
import { ClickEvent, Menu, MenuItem, SubMenu } from '@szhsin/react-menu';
import React, { ChangeEvent } from 'react';
import logo from '../../assets/images/logo.png';
import { PROJECT_FILE_EXTENSION } from '../../core/constants';
import Editor from '../../core/editor';
import { textFile2DataURL } from '../../core/utils';
import { collectionBlocks, CollectionBlockType } from '../blocks/collection/collection-factory';
import './styles.scss';



export interface MenuBarProps {
    editor: Editor;
}

function MenuBar(props: MenuBarProps) {

    const theme = useTheme();
    const isDark = theme.palette.type === 'dark';
    const projectReader = new FileReader();
    const blockReader = new FileReader();
    const { editor } = props;

    const setBlock = (type: string) => {
        editor.addBlock(type);
    }


    const newProject = (_event: ClickEvent) => {
        editor.clearProject();
    }

    const openProject = (_event: ClickEvent) => {
        document.getElementById('openProjectInput')?.click();
        projectReader.onload = (event) => {
            if (event.target?.result) {
                editor.loadProject(JSON.parse(event.target.result.toString()))
            }
        };
    }

    const saveProject = (_event: ClickEvent) => {
        const model = editor.serialise();
        const url = textFile2DataURL(JSON.stringify(model), 'text/json');
        const link = document.getElementById('saveProjectLink');
        link?.setAttribute('href', url);
        link?.setAttribute('download', editor.getName() + PROJECT_FILE_EXTENSION);
        link?.click();
    }

    const onFileUpload = (event: ChangeEvent<HTMLInputElement>, reader: FileReader) => {
        const file = event.target.files?.length ? event.target.files[0] : null;
        event.target.value = '';
        if (file) {
            reader.readAsText(file);
        }
    }

    const editProjectInfo = (_event: ClickEvent) => {
        editor.editProjectInfo();
    }

    const addAsBlock = (_event: ClickEvent) => {
        document.getElementById('addAsBlockInput')?.click();
        blockReader.onload = (event) => {
            if (event.target?.result) {
                editor.addAsBlock(JSON.parse(event.target.result.toString()));
            }
        };
    }

    const blocksEntries = (blocks: CollectionBlockType, key: string = '') => {
        return Object.entries(blocks).map(([name, block]) => {
            var items;
            const newKey = `${key}.${name}`;
            if (block.children) {
                items = <SubMenu label={block.label} key={newKey}>
                    {blocksEntries(block.children, newKey)}
                </SubMenu>;
            } else {
                items = <MenuItem onClick={() => setBlock(newKey)} key={newKey}>{block.label}</MenuItem>
            }
            return items;
        })
    }

    const blocks = blocksEntries(collectionBlocks.blocks, 'blocks');

    return (
        <AppBar position="static" className='menu-bar' id='menu-bar'>
            <Toolbar >
                <img src={logo} alt="Visual Circuit" width='50px' style={{ marginRight: '1em' }} />
                <Menu
                    menuButton={<Button className='menu-button'>File</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    <MenuItem onClick={newProject}>New File</MenuItem>
                    <MenuItem onClick={openProject}>Open</MenuItem>
                    <MenuItem onClick={saveProject}>Save as..</MenuItem>
                    <MenuItem onClick={addAsBlock}>Add as block</MenuItem>
                </Menu>
                <Menu
                    menuButton={<Button className='menu-button'>Edit</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    <MenuItem onClick={editProjectInfo}>Edit Project Information</MenuItem>
                </Menu>
                <div style={{ flex: 1 }} />
                <Menu
                    menuButton={<Button className='menu-button'>Basic</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    <MenuItem onClick={() => setBlock('basic.constant')}>Constant</MenuItem>
                    <MenuItem onClick={() => setBlock('basic.code')}>Code</MenuItem>
                    <MenuItem onClick={() => setBlock('basic.input')}>Input</MenuItem>
                    <MenuItem onClick={() => setBlock('basic.output')}>Output</MenuItem>
                    <MenuItem onClick={() => setBlock('basic.information')}>Information</MenuItem>
                </Menu>

                <Menu
                    menuButton={<Button className='menu-button'>Blocks</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    {blocks}
                </Menu>
            </Toolbar>

            <input type='file' id='openProjectInput' accept={PROJECT_FILE_EXTENSION}
                onChange={(event) => onFileUpload(event, projectReader)} hidden />
            <input type='file' id='addAsBlockInput' accept={PROJECT_FILE_EXTENSION}
                onChange={(event) => onFileUpload(event, blockReader)} hidden />
            <a href='/' id='saveProjectLink' hidden download>Download Project</a>
        </AppBar>
    )
}

export default MenuBar;