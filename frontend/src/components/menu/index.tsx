import { AppBar, Button, Toolbar, useTheme } from '@material-ui/core';
import { ClickEvent, Menu, MenuItem, SubMenu } from '@szhsin/react-menu';
import html2canvas from 'html2canvas';
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


interface FileHelper {
    fileName: string;
    reader: FileReader;
}
/**
 * 
 * MenuBar component
 * It offers 'File', 'Edit', 'Basic' and 'Blocks' menus
 */
function MenuBar(props: MenuBarProps) {

    const theme = useTheme();
    const isDark = theme.palette.type === 'dark';
    const projectReader : FileHelper = {'fileName': '', 'reader': new FileReader()};
    const blockReader: FileHelper = {'fileName': '', 'reader': new FileReader()};
    const { editor } = props;

    /**
     * Callback for when a block is selected.
     * It adds the block to the current project through the editor.
     * @param type Name / type of block selected
     */
    const setBlock = (type: string) => {
        editor.addBlock(type);
    }


    /**
     * Callback for 'New File' option under 'File' menu.
     * It clears the current project and shows an empty project.
     * @param _event Mouse click event. Unused
     */
    const newProject = (_event: ClickEvent) => {
        editor.clearProject();
    }

    /**
     * Callback for 'Open' option under 'File' menu.
     * It simulates clicking on an File input field to open a file selection dialog box.
     * Once the file is selected, contents of the file is read and is passed to editor to
     * load it as a project.
     * @param _event Mouse click event. Unused
     */
    const openProject = (_event: ClickEvent) => {
        projectReader.fileName = '';
        // Simulate click to open file selection dialog.
        document.getElementById('openProjectInput')?.click();
        projectReader.reader.onload = (event) => {
            if (event.target?.result) {
                // Parse file as JSON
                editor.loadProject(JSON.parse(event.target.result.toString()), projectReader.fileName);
            }
        };
    }

    /**
     * Callback for 'Save as...' option under 'File' menu.
     * The serialised data of project is converted to text blob and a data URI to it is obtianed.
     * This is attached as hidden link to the document. And then a click on link is simulated to
     * start downloading of file. 
     * @param _event Mouse click event. Unused
     */
    const saveProject = (_event: ClickEvent) => {
        const model = editor.serialise();
        // Get data URI for the blob created using text 
        const url = textFile2DataURL(JSON.stringify(model), 'text/json');
        // Attach the URI to the hidden link and then simulate a click on it.
        const link = document.getElementById('saveProjectLink');
        link?.setAttribute('href', url);
        link?.setAttribute('download', editor.getName() + PROJECT_FILE_EXTENSION);
        link?.click();
    }

    /**
     * Callback for 'Save Block' option under 'File' menu.
     * @param _event Mouse click event. Unused
     */
    const saveBlockAsync = async (_event: ClickEvent) => {
        const completed = await editor.editBlock(); 
        if(completed){
            const model = editor.serialise();
            const url = textFile2DataURL(JSON.stringify(model), 'text/json');
            const link = document.getElementById('saveProjectLink');
            if (link) {
                link.setAttribute('href', url);
                link.setAttribute('download', editor.getName() + PROJECT_FILE_EXTENSION);
                link.click();
                editor.retriveCircuit();
            } else {
                console.error('Save project link element not found.');
            }
        }
    };
    
    const saveBlock = (_event: ClickEvent) => {
        saveBlockAsync(_event).catch(error => {
            console.error('Error saving block:', error);
        });
    };

    /**
     * Callback when file is uploaded.
     * @param event File field change event.
     * @param reader Reader to open the uploaded file as text
     */
    const onFileUpload = (event: ChangeEvent<HTMLInputElement>, fileHelper: FileHelper) => {
        const file = event.target.files?.length ? event.target.files[0] : null;
        event.target.value = '';
        if (file) {
            fileHelper.fileName = file.name;
            fileHelper.reader.readAsText(file);
        }
    }

    /**
     * Callback for 'Edit Project Information' under 'Edit' menu.
     * @param _event Mouse click event. Unused
     */
    const editProjectInfo = (_event: ClickEvent) => {
        editor.editProjectInfo();
    }

    /**
     * Callback for 'Add as block' under 'File' menu.
     * It simulates clicking on an File input field to open a file selection dialog box.
     * Once the file is selected, contents of the file is read and is passed to editor to
     * add it as a Package block.
     * @param _event Mouse click event. Unused
     */
    const addAsBlock = (_event: ClickEvent) => {
        blockReader.fileName = '';
        document.getElementById('addAsBlockInput')?.click();
        blockReader.reader.onload = (event) => {
            if (event.target?.result) {
                editor.addAsBlock(JSON.parse(event.target.result.toString()), blockReader.fileName);
            }
        };
    }

    /**
     * Callback for 'Build and Download' under 'File' menu.
     * It sends the project as json to the backend for Python synthesis. Once response is received,
     * this is attached as data URI to a hidden link. And then a click on link is simulated to
     * start downloading of file. 
     * @param _event Mouse click event. Unused
     */
    const buildAndDownload = (_event: ClickEvent) => {
        const model = editor.serialise();
        let filename = editor.getName();
        if (process.env.REACT_APP_BACKEND_HOST && model) {
            const url = process.env.REACT_APP_BACKEND_HOST + 'build'
            const headers: HeadersInit = {'Content-Type': 'application/json'};
            fetch(url, {
                method: 'POST',
                body: JSON.stringify(model),
                headers:  headers}
            ).then((response) => {
                if (response.ok) {
                    // Get the filename
                    const header = response.headers.get('Content-Disposition');
                    filename = header?.split(';')[1]?.split('=')[1] || filename;
                    return response.blob()
                }
                throw Error('Something went wrong!')
            }).then((blob) => {
                // Convert the blob to a data URI
                const url = URL.createObjectURL(blob)
                // Attach the URI to the hidden link and then simulate a click on it.
                const link = document.getElementById('buildProjectLink');
                link?.setAttribute('href', url);
                // Remove extra quotes around the name. This is hack
                // TODO: Find why quotes are getting added to project name
                link?.setAttribute('download', filename.replace(/^"(.+(?="$))"$/, '$1'));
                link?.click();
            }).catch((reason) => {
                // If there's an error show the message in an alert.
                alert(reason);
            });
        }
    }

    /**
     * Export the current canvas as a PNG image.
     * Captures the element with id 'canvas-container' using html2canvas and triggers a download.
     */
    const exportAsPNG = (_event: ClickEvent) => {
        const container = document.getElementById('canvas-container');
        if (!container) {
            alert('Canvas area not found.');
            return;
        }
       // The promise chain ensures the handler executes synchronously within the menu component.

        html2canvas(container as HTMLElement, { backgroundColor: null })
            .then((canvas: HTMLCanvasElement) => {
                const url = canvas.toDataURL('image/png');
                const link = document.createElement('a');
                link.href = url;
                link.download = editor.getName() + '.png';
                document.body.appendChild(link);
                link.click();
                document.body.removeChild(link);
            })
            .catch((err: any) => {
                console.error('Error exporting canvas as PNG:', err);
                alert('Could not export canvas as PNG. See console for details.');
            });
    }

    /**
     * Recursive helper function to generate menu options for the Blocks menu.
     * @param blocks Map containing Blocks menu structure
     * @param key Prefix to be used while constructing block name.
     * @returns Menu / submenu components in order similar to blocks structure
     */
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
    const processingBlocks = blocksEntries(collectionBlocks.processing, 'processing');
    const driverBlocks = blocksEntries(collectionBlocks.drivers, 'drivers');

    // TODO: Localise string instead of hardcoding it. 
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
                    <MenuItem onClick={saveBlock}>Save Block</MenuItem>
                    <MenuItem onClick={addAsBlock}>Add as block</MenuItem>
                    <MenuItem onClick={buildAndDownload}>Build and Download</MenuItem>
                    <MenuItem onClick={exportAsPNG}>Export as PNG</MenuItem>
                </Menu>
                <Menu
                    menuButton={<Button className='menu-button'>Edit</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    <MenuItem onClick={editProjectInfo}>Edit Project Information</MenuItem>
                </Menu>
                <Menu
                    menuButton={<Button className='menu-button'>Help</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    <MenuItem href='https://jderobot.github.io/VisualCircuit/' target='_blank'>Docs</MenuItem>
                    <MenuItem href='https://github.com/JdeRobot/VisualCircuit' target='_blank'>Github</MenuItem>
                    <MenuItem href='https://github.com/JdeRobot/VisualCircuit/releases' target='_blank'>Releases</MenuItem>
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

                {/* <Menu
                    menuButton={<Button className='menu-button'>Blocks</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    {blocks}
                </Menu> */}
                <Menu
                    menuButton={<Button className='menu-button'>Processing</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    {processingBlocks}
                </Menu>
                <Menu
                    menuButton={<Button className='menu-button'>Drivers</Button>}
                    theming={isDark ? 'dark' : undefined}>
                    {driverBlocks}
                </Menu>
            </Toolbar>
            {/* Hidden file input field for opening project file selection dialog. */}
            <input type='file' id='openProjectInput' accept={PROJECT_FILE_EXTENSION}
                onChange={(event) => onFileUpload(event, projectReader)} hidden />
            {/* Hidden file input field for opening file selection dialog to be added as a block. */}
            <input type='file' id='addAsBlockInput' accept={PROJECT_FILE_EXTENSION}
                onChange={(event) => onFileUpload(event, blockReader)} hidden />
            <a href='/' id='saveProjectLink' hidden download>Download Project</a>
            <a href='/' id='buildProjectLink' hidden download>Build Project</a>
        </AppBar>
    )
}

export default MenuBar;