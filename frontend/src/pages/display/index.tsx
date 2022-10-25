import MaximizeIcon from '@material-ui/icons/Maximize';
import MinimizeIcon from '@material-ui/icons/Minimize';
import React, { FunctionComponent } from 'react';
import { Socket } from "socket.io-client";


import { Box, IconButton, Tab, Tabs, Typography } from '@material-ui/core';
import { Add } from '@material-ui/icons';
import { Terminal } from 'xterm';
import { FitAddon } from 'xterm-addon-fit';
import Connect from '../../core/connect';
import './styles.scss';
import TerminalTab from './terminal';

type TerminalDisplayProps = {
    io: Connect
}

type TerminalDisplayState = {
    tabs: number[];
    currentTab: number;
    minimized: boolean;
}

type DisplayProps = {
    io: Connect
}


function fitToscreen(socket: Socket, fit: FitAddon, term: Terminal) {
    fit.fit();
    const dims = { cols: term.cols, rows: term.rows };
    console.log("sending new dimensions to server's pty", dims);
    socket.emit("resize", dims);
}


interface TabPanelProps {
    children?: React.ReactNode;
    index: number;
    value: number;
}

function TabPanel(props: TabPanelProps) {
    const { children, value, index, ...other } = props;

    return (
        <div
            role="tabpanel"
            hidden={value !== index}
            id={`simple-tabpanel-${index}`}
            aria-labelledby={`simple-tab-${index}`}
            {...other}
        >
            <Box>
                <Typography>{children}</Typography>
            </Box>
        </div>
    );
}

function a11yProps(index: number) {
    return {
        id: `simple-tab-${index}`,
        'aria-controls': `simple-tabpanel-${index}`,
    };
}




class TerminalDisplay extends React.Component<TerminalDisplayProps, TerminalDisplayState> {
    constructor(props: TerminalDisplayProps) {
        super(props);

        this.state = { tabs: [], currentTab: 0, minimized: true };
    }

    componentDidMount(): void {
        this.props.io.connectToLocalDocker();
    }

    toggleTerminal(minimize: boolean) {
        const terminal = document.getElementById('terminal');
        if (terminal) {
            if (minimize) {
                terminal.classList.add('hidden');
            } else {
                terminal.classList.remove('hidden');
            }
            this.setState({ minimized: minimize })
        }
    }

    render(): React.ReactNode {
        return <div className='terminal-container'>
            <div className='header'>
                <div>Terminal</div>
                {this.state.minimized &&
                    <IconButton onClick={() => this.toggleTerminal(false)}>
                        <MaximizeIcon />
                    </IconButton>
                }
                {!this.state.minimized &&
                    <IconButton onClick={() => this.toggleTerminal(true)}>
                        <MinimizeIcon />
                    </IconButton>
                }
            </div>
            <div id='terminal' className='hidden'>

                <Tabs
                    value={this.state.currentTab}
                    onChange={(_, tab) => this.setState({ currentTab: tab })}
                    className="tabs"
                    aria-label="terminal tabs">
                    {this.state.tabs.map((tabIndex) => <Tab label={`Tab ${tabIndex}`} className="tab" key={tabIndex} {...a11yProps(tabIndex)} />)}
                    <IconButton onClick={() => { this.setState({ tabs: [...this.state.tabs, this.state.tabs.length], currentTab: this.state.tabs.length }) }}>
                        <Add />
                    </IconButton>
                </Tabs>

                {this.state.tabs.map((tabIndex) => {
                    return (
                        <TabPanel value={this.state.currentTab} index={tabIndex} key={tabIndex}>
                            <TerminalTab tid={tabIndex} socket={this.props.io.dockerSocket!} key={tabIndex} />
                        </TabPanel>
                    )
                })}
            </div>
        </div>
    }
}


const Display: FunctionComponent<DisplayProps> = (props) => {
    props.io.connectToLocalDocker();
    return (
        <div className='display'>
            <TerminalDisplay io={props.io} />
            <iframe src={process.env.REACT_APP_VNC} className='vnc' title='vnc' />
        </div>
    );
}

export default Display;