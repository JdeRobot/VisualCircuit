import { io, Socket } from "socket.io-client";

class Connect {

    private static instance: Connect;
    public dockerSocket: Socket | null = null;
    private constructor() {

    }

    public static getInstance() {
        if (!Connect.instance) {
            Connect.instance = new Connect();
        }
        return Connect.instance;
    }

    public connectToLocalDocker() {
        if (this.dockerSocket === null) {
            this.dockerSocket = io(process.env.REACT_APP_TERMINAL_SOCKET || "");
            this.dockerSocket.connect()
        }
    }
}

export default Connect;