import { Button, Dialog, DialogActions, DialogContent, DialogContentText, TextField } from '@material-ui/core';
import Checkbox from '@material-ui/core/Checkbox/Checkbox';
import FormControlLabel from '@material-ui/core/FormControlLabel/FormControlLabel';
import React, { ChangeEvent, useState } from 'react';
import { create, InstanceProps } from 'react-modal-promise';
import { ConstantBlockModelOptions } from '../blocks/basic/constant/constant-model';


/**
 * 
 * @param {
 *          isOpen: True if modal needs to be opened.
 *          onResolve: Will be called to indicate success / completion.
 *          onReject: Will be called to indicate failure.
 *        }
 */
const ConstantBlockDialog = ({ isOpen, onResolve, onReject, name: _name, local: _local}: InstanceProps<ConstantBlockModelOptions> & Partial<ConstantBlockModelOptions>) => {


  const [name, setName] = useState(_name || '');
  const [local, setLocal] = useState<boolean>(_local || true);

  const [errorMsg, setErrorMsg] = useState('');


  /**
   * Callback when constant input field changes.
   * @param event Constant input field change event
   */
  const handleInput = (event: ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    // If there was error previously, clear the error message.
    if (errorMsg) {
      setErrorMsg('')
    }
    setName(event.target.value)
  }

  /**
   * Callback for 'Ok' button of the dialog
   */
  const handleSubmit = () => {
    // If name is defined, send the data back indicating success.
    if (name) {
      onResolve({ name: name, local: local })
    } else {
      // If name is not defined, show an error message.
      setErrorMsg('Block name is mandatory')
    }
  }

  return (
    <Dialog open={isOpen} aria-labelledby="form-dialog-title">

      <DialogContent>
        <DialogContentText>
          Enter the name of constant block
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
        <FormControlLabel
          control={
            <Checkbox
              color='default'
              checked={local}
              onChange={(event) => setLocal(event.target.checked)}
            />
          }
          label="Local Parameter"
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

const createConstantDialog = create(ConstantBlockDialog);

export default createConstantDialog;