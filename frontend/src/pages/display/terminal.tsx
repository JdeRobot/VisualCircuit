import React from "react";
import { XTerm } from "xterm-for-react";
import { Socket } from "socket.io-client";
import { FitAddon } from 'xterm-addon-fit';
import { Terminal } from 'xterm';

type TerminalProps = {
    tid: number,
    socket: Socket
}

class TerminalTab extends React.Component<TerminalProps> {
    fit: FitAddon;
    xtermRef: React.RefObject<XTerm>;

    constructor(props: TerminalProps) {
        super(props);

        this.fit = new FitAddon();
        this.xtermRef = React.createRef<XTerm>();

        this.props.socket.on("pty-output", (data: any) => this.onReceiveData(data));
    }

    componentDidMount(): void {
        console.log("Did mount");
        this.props.socket.emit("start_bash", {
            rows: this.xtermRef.current?.terminal.rows,
            cols: this.xtermRef.current?.terminal.cols,
            tid: this.props.tid
        })
        this.xtermRef?.current?.terminal.resize(100, 20);
    }

    componentDidUpdate(prevProps: Readonly<{}>, prevState: Readonly<{}>, snapshot?: any): void {
        console.log("Updated");
    }

    componentWillUnmount(): void {
        console.log("Will unmount");
    }

    onReceiveData(data: any) {
        if (data.tid == this.props.tid) {
            this.xtermRef.current?.terminal.write(data.output);
        }
    }

    onSendData(data: string) {
        this.props.socket.emit("pty-input", { input: data, tid: this.props.tid });
    }

    render(): React.ReactNode {
        return (
            <div>
                <XTerm
                    ref={this.xtermRef}
                    options={{
                        cursorBlink: true,
                        macOptionIsMeta: true
                    }}
                    addons={[this.fit]}
                    onData={(data) => this.onSendData(data)} />
            </div>
        )
    }
}

export default TerminalTab;