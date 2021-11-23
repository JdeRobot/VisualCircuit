import { Button, Dialog, DialogActions, DialogContent, DialogContentText, TextField } from '@material-ui/core';
import React, { ChangeEvent, useState } from 'react';
import { create, InstanceProps } from 'react-modal-promise';
import { InputBlockModelOptions } from '../blocks/basic/input/input-model';


/**
 * 
 * @param {
 *          isOpen: True if modal needs to be opened.
 *          onResolve: Will be called to indicate success / completion.
 *          onReject: Will be called to indicate failure.
 *        }
 */
const IOBlockDialog = ({ isOpen, onResolve, onReject, name: _name }: InstanceProps<InputBlockModelOptions> & Partial<InputBlockModelOptions>) => {


  /**
   * Name for the input or output block
   */
  const [name, setName] = useState(_name || '');
  /**
   * Error message shown when 'Ok' is pressed without giving any name.
   * By default it is empty. And will be populated upon validation.
   */
  const [errorMsg, setErrorMsg] = useState('');

  /**
   * Callback for when Name field changes
   * @param event Input field change event
   */
  const handleInput = (event: ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    // If previously there was an error clear it.
    if (errorMsg) {
      setErrorMsg('')
    }
    setName(event.target.value);
  }

  /**
   * Callback for 'Ok' button of the dialog
   */
  const handleSubmit = () => {
    // If name is not undefined or empty, indicate success by calling onResolve with the name as data
    if (name) {
      onResolve({ name: name })
    } else {
      // If name is not present show an error message.
      setErrorMsg('Block name is mandatory');
    }
  }

  return (
    <Dialog open={isOpen} aria-labelledby="form-dialog-title">

      <DialogContent>
        <DialogContentText>
          Enter the name
          </DialogContentText>
        <TextField
          autoFocus
          margin="dense"
          type="text"
          variant='outlined'
          value={name}
          onChange={handleInput}
          error={Boolean(errorMsg)}
          helperText={errorMsg}
          fullWidth
        />
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

const createIODialog = create(IOBlockDialog);

export default createIODialog;