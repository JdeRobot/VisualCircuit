import { Button, Dialog, DialogActions, DialogContent, DialogContentText, TextField } from '@material-ui/core';
import React, { ChangeEvent, useState } from 'react';
import { create, InstanceProps } from 'react-modal-promise';
import { InputBlockModelOptions } from '../blocks/basic/input/input-model';



const IOBlockDialog = ({ isOpen, onResolve, onReject }: InstanceProps<InputBlockModelOptions>) => {


  const [name, setName] = useState('');
  const [errorMsg, setErrorMsg] = useState('');

  const handleInput = (event: ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    if (errorMsg) {
      setErrorMsg('')
    }
    setName(event.target.value);
  }

  const handleSubmit = () => {
    if (name) {
      onResolve({ name: name })
    } else {
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