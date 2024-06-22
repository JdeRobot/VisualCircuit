import { Button, Checkbox, Dialog, DialogActions, DialogContent, DialogContentText } from '@material-ui/core';
import React, { useEffect, useState } from 'react';
import { create, InstanceProps } from 'react-modal-promise';
import { BlockData } from '../../core/constants';

interface BlockDialogProps extends InstanceProps<BlockData>, Partial<BlockData> { 
    getGInputsOutput: () => [{ indexOne: number, label: string, id: string }[], { indexTwo: number, label: string, id: string }[]];
};

/**
 * 
 * @param {
 *          isOpen: True if modal needs to be opened.
 *          onResolve: Will be called to indicate success / completion.
 *          onReject: Will be called to indicate failure.
 *        }
 */
const BlockDialog = ({ isOpen, onResolve, onReject, getGInputsOutput, selectedInputIds, selectedOutputIds }: BlockDialogProps) => {

    const [inputs, outputs] = getGInputsOutput();
    const [checkedState, setCheckedState] = useState<{ [key: string]: boolean }>({});

    useEffect(() => {
        const initialCheckedState: { [key: string]: boolean } = {};

        selectedInputIds?.forEach(id => {
            initialCheckedState[id] = true;
        });

        selectedOutputIds?.forEach(id => {
            initialCheckedState[id] = true;
        });

        setCheckedState(initialCheckedState);
    }, [selectedInputIds, selectedOutputIds]);

    const handleSubmit = () => {
        const selectedInputIds = inputs.filter(item => checkedState[item.id]).map(item => item.id);
        const selectedOutputIds = outputs.filter(item => checkedState[item.id]).map(item => item.id);
        onResolve({ selectedInputIds, selectedOutputIds });
    }

    const handleChange = (event: React.ChangeEvent<HTMLInputElement>) => {
        const { name, checked } = event.target;
        setCheckedState(prevState => ({
            ...prevState,
            [name]: checked,
        }));
    };

    return (
        <Dialog
            open={isOpen}
            fullWidth={true}
            maxWidth='md'
            aria-labelledby="form-dialog-title">
            <DialogContent>
                <DialogContentText>
                    Edit Block
                </DialogContentText>
                <div style={{ display: 'flex', alignItems: 'center' }}>
                    <div style={{ flex: 1 }}>
                        Global Input
                        {inputs.map((item: { indexOne: number, label: string, id: string }) => (
                            <div key={item.id}>
                                <Checkbox
                                    checked={checkedState[item.id] || false}
                                    onChange={handleChange}
                                    name={item.id}
                                    color="primary"
                                />
                                <label htmlFor={item.id}>{item.label}</label>
                            </div>
                        ))}
                    </div>
                    <div style={{ flex: 1 }}>
                        Global Output
                        {outputs.map((item: { indexTwo: number, label: string, id: string }) => (
                            <div key={item.id}>
                                <Checkbox
                                    checked={checkedState[item.id] || false}
                                    onChange={handleChange}
                                    name={item.id}
                                    color="primary"
                                />
                                <label htmlFor={item.id}>{item.label}</label>
                            </div>
                        ))}
                    </div>
                </div>   
            </DialogContent>
            <DialogActions>
                <Button onClick={() => onReject()}>
                    Cancel
                </Button>
                <Button onClick={handleSubmit}>
                    Ok
                </Button>
            </DialogActions>
        </Dialog>
    )
}

const createBlockDialog = create(BlockDialog);

export default createBlockDialog;