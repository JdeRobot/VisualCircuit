import { Button, Dialog, DialogActions, DialogContent, DialogContentText, TextField } from '@material-ui/core';
import React, { useState } from 'react';
import { create, InstanceProps } from 'react-modal-promise';
import { CodeBlockModelOptions } from '../blocks/basic/code/code-model';



const CodeBlockDialog = ({ isOpen, onResolve, onReject }: InstanceProps<CodeBlockModelOptions>) => {


    const [inputPorts, setInputPorts] = useState('');
    const [outputPorts, setOutputPorts] = useState('');
    const [parameters, setParameters] = useState('');
    const [error, setError] = useState('');


    const handleSubmit = () => {
        if (inputPorts.length > 0 || outputPorts.length > 0) {
            setError('')
            const inputs = inputPorts.split(',').filter((port) => Boolean(port)).map((port) => port.trim());
            const outputs = outputPorts.split(',').filter((port) => Boolean(port)).map((port) => port.trim());
            const params = parameters.split(',').filter((port) => Boolean(port)).map((port) => port.trim());
            onResolve({ inputs: inputs, outputs: outputs, params: params });
        } else {
            setError('Code block needs atleast one Input or one Output')
        }
    }

    return (
        <Dialog open={isOpen} aria-labelledby="form-dialog-title" fullWidth>

            <DialogContent>
                <DialogContentText>
                    Enter the input ports
                </DialogContentText>
                <TextField
                    autoFocus
                    margin="dense"
                    type="text"
                    variant='outlined'
                    value={inputPorts}
                    onChange={(event) => setInputPorts(event.target.value)}
                    error={Boolean(error)}
                    helperText={error}
                    fullWidth
                />

                <DialogContentText>
                    Enter the output ports
                </DialogContentText>
                <TextField
                    autoFocus
                    margin="dense"
                    type="text"
                    variant='outlined'
                    value={outputPorts}
                    onChange={(event) => setOutputPorts(event.target.value)}
                    error={Boolean(error)}
                    helperText={error}
                    fullWidth
                />

                <DialogContentText>
                    Enter the parameters
                </DialogContentText>
                <TextField
                    autoFocus
                    margin="dense"
                    type="text"
                    variant='outlined'
                    value={parameters}
                    onChange={(event) => setParameters(event.target.value)}
                    fullWidth
                />

            </DialogContent>
            <DialogActions>
                <Button onClick={() => onReject()}>
                    Cancel
                </Button>
                <Button onClick={() => handleSubmit()}>
                    Ok
                </Button>
            </DialogActions>
        </Dialog>
    )
}

const createCodeDialog = create(CodeBlockDialog);

export default createCodeDialog;