import { Button, Dialog, DialogActions, DialogContent, DialogContentText, TextField } from '@material-ui/core';
import Checkbox from '@material-ui/core/Checkbox/Checkbox';
import FormControlLabel from '@material-ui/core/FormControlLabel/FormControlLabel';
import React, { ChangeEvent, useState } from 'react';
import { create, InstanceProps } from 'react-modal-promise';
import { ConstantBlockModelOptions } from '../blocks/basic/constant/constant-model';



const ConstantBlockDialog = ({ isOpen, onResolve, onReject }: InstanceProps<ConstantBlockModelOptions>) => {


  const [name, setName] = useState('');
  const [local, setLocal] = useState(true);

  const [errorMsg, setErrorMsg] = useState('');


  const handleInput = (event: ChangeEvent<HTMLInputElement | HTMLTextAreaElement>) => {
    if (errorMsg) {
      setErrorMsg('')
    }
    setName(event.target.value)
  }

  const handleSubmit = () => {
    if (name) {
      onResolve({ name: name, local: local })
    } else {
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