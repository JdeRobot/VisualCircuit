import { Button, Checkbox, Dialog, DialogActions, DialogContent, DialogContentText, TextField } from '@material-ui/core';
import React, { ChangeEvent, useState } from 'react';
import { create, InstanceProps } from 'react-modal-promise';
import { ProjectInfo } from '../../core/constants';
import { PackageBlockModel } from "../../components/blocks/package/package-model";

interface BlockDialogProps extends InstanceProps<ProjectInfo>, Partial<ProjectInfo> { 
    getGInputsOutput: () => [{ indexOne: number, label: string }[], { indexTwo: number, label: string }[]];
};

/**
 * 
 * @param {
 *          isOpen: True if modal needs to be opened.
 *          onResolve: Will be called to indicate success / completion.
 *          onReject: Will be called to indicate failure.
 *        }
 */
const BlockDialog = ({ isOpen, onResolve, onReject, getGInputsOutput
     }: BlockDialogProps) => {

    const [inputs, outputs] = getGInputsOutput();
    const [checkedState, setCheckedState] = useState();
    
    const handleSubmit = () => {
        // Send the filled data back
        // onResolve({
            
        // });
    }

    // const handleCheckGInputs = () => {
       
    // }
    function handleChange(event: ChangeEvent<HTMLInputElement>, checked: boolean): void {
        throw new Error('Function not implemented.');
    }

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
                       {inputs.map((item: { indexOne: number, label: string }) => (
                            <div key={item.indexOne}>
                                <Checkbox
                                    // checked={checkedState[item.index]}
                                    onChange={handleChange}
                                    name={item.indexOne.toString()}
                                    color="primary"
                                />
                                <label htmlFor={item.indexOne.toString()}>{item.label}</label>
                            </div>
                        ))}
                    </div>
                    <div style={{ flex: 1 }}>
                       Global Output
                       {outputs.map((item: { indexTwo: number, label: string }) => (
                            <div key={item.indexTwo}>
                                <Checkbox
                                    // checked={checkedState[item.index]}
                                    onChange={handleChange}
                                    name={item.indexTwo.toString()}
                                    color="primary"
                                />
                                <label htmlFor={item.indexTwo.toString()}>{item.label}</label>
                            </div>
                        ))}
                    </div>
                 </div>   
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

const createBlockDialog = create(BlockDialog);

export default createBlockDialog;